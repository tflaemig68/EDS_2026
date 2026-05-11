# Balancer -- OOP Refactoring & Cascaded Distance Control

Studienarbeit **T3200** -- *Erweiterung des Balancer Roboters durch eine Abstandsregelung*
DHBW Stuttgart, Studiengang Mechatronik (TMT23GR1), 6. Semester.

This software is licensed based on [CC BY-NC-SA 4.0](https://creativecommons.org/licenses/by-nc-sa/4.0/deed.de).


## Table of Contents

  - [Introduction](#Intro)
  - [Part 1 - OOP Refactoring (balancer_t)](#Refac)
  - [Part 2 - Cascaded Distance Control (M_DistCtrl)](#DistCtrl)
  - [Diagrams](#Diagrams)
  - [Authors](#Authoren)


<a id="Intro"></a>
## Introduction

The Balancer is a self-balancing two-wheel robot built on an STM32F4 microcontroller. Before this project, the codebase already provided a working pitch (tilt) controller -- the robot could hold the upright position and follow manually defined turn routines -- together with a set of OOP-style C drivers (`MPU6050_t`, `TOFSensor_t`, `Stepper_t`, `PIDContr_t`, `MeanVal_t`, `analogCh_t`).

This contribution to the project is split into two coupled parts:

1. **Refactoring.** The top-level program (old `main.c`) is reorganised into an OOP-inspired `Balancer_t` that owns all sensors, actuators, controllers and state. The new `main.c` is reduced to hardware instantiation, peripheral init, and a thin scheduler that calls the object's methods.
2. **Cascaded distance control.** A new task mode `M_DistCtrl` is added. An outer distance loop (TOF sensor &rarr; target velocity) and a middle velocity loop (wheel-derived velocity &rarr; target acceleration &rarr; pitch offset) are wrapped around the existing inner pitch loop. The robot is meant to demonstrate the *distance control*-function on a treadmill.

Function- and field-level documentation is placed directly as Doxygen comments in the source files. This page introduces the two parts and the diagrams that explain the architecture.


<a id="Refac"></a>
## Part 1 -- OOP Refactoring (`balancer_t`)

### Goal of the refactoring

The pre-existing `main.c` mixed multiple concerns in one file: hardware instantiation, peripheral init, mode/state machine, control law, and parameter editing. Adding distance control on top of that flat structure would have made the code even more overloaded. The refactoring introduces one *object* that holds all subsystems, exposes a small set of public method-pointers, and keeps `main.c` focused on scheduling.

### The C-with-structs OOP pattern

[Quantum  Leaps' guide ](https://www.state-machine.com/oop) can be a good starting point to understanding and applying the use of object oriented programming in C. In the following it will be exlpained to what extend and how those conceps have been incorporated into the balancer refactoring.

The pattern follows the convention already used by `PIDContr_t` in `regler.h` and the hardware drivers: a struct stores both data fields and **function pointers as members**. The pointer assignments are made once, at file scope, in a single `const` *prototype instance*:

```c
/* in balancer_t.c - the prototype instance */
const Balancer_t Balancer = {
    .init               = BalancerInit,
    .updatePitch        = BalancerUpdatePitch,
    .updateDist         = BalancerUpdateDist,
    .updateDisplay      = BalancerUpdateDisplay,
    .DispAlphaNumMPU    = BalancerDispAlphaNumMPU,
    .paramEdit          = BalancerParamEdit,
};
```

1. **Encapsulation:** A class is a struct of attributes plus functions that take `Balancer_t *b` as first argument; a `BalancerCreate()` initialises the instance; method implementations live in the .c file &rarr; `balancer_t.h` declares, `balancer_t.c` defines.

```c
/* Attributes — balancer_t.h */
struct Balancer {
    MPU6050_t   *pIMU;
    TOFSensor_t *pTOF;
    PIDContr_t   PID_phi;
    /* ...state, params, timers... */
};

/* Operations — balancer_t.c, all static (file-private) */
static void BalancerInit       (Balancer_t *b) { ... }
static void BalancerUpdatePitch(Balancer_t *b) { ... }
static void BalancerUpdateDist (Balancer_t *b) { ... }

/* Constructor */
void BalancerCreate(Balancer_t *b, MPU6050_t *imu, /* ... */) {
    *b = Balancer;
    b->pIMU = imu;
    /* ... */
    Balancer.init(b);
}
```

2. **Inheritance:** Not used in this project -- `balancer_t` has no __super__ member anywhere. It expresses relationships via composition and aggregation (see [class diagram](#Diagrams)).

```c
/* balancer_t.h — Balancer "has a" IMU, "has a" pitch PID */
struct Balancer {
    MPU6050_t   *pIMU;       /* aggregation: pointer, lifetime external */
    TOFSensor_t *pTOF;       /* aggregation */
    Stepper_t   *pStepL;     /* aggregation */
    PIDContr_t   PID_phi;    /* composition: owned by value */
    PIDContr_t   PID_dist;   /* composition */
    MeanVal_t    LPF_dist;   /* composition */
    /* ... */
};
```

3. **Polymorphism:** The function pointers live directly inside the instance struct.

```c
/* balancer_t.h */
struct Balancer {
    /* ...attributes... */
    void (*init)         (Balancer_t *b);
    void (*updatePitch)  (Balancer_t *b);
    void (*updateDist)   (Balancer_t *b);
    void (*updateDisplay)(Balancer_t *b);
    void (*paramEdit)    (Balancer_t *b);
};
extern const Balancer_t Balancer;    /* prototype object */
```

The pointers are seeded from a single const prototype, copied wholesale into each instance by the constructor:

```c
/* balancer_t.c */
const Balancer_t Balancer = {
    .init          = BalancerInit,
    .updatePitch   = BalancerUpdatePitch,
    .updateDist    = BalancerUpdateDist,
    .updateDisplay = BalancerUpdateDisplay,
    .paramEdit     = BalancerParamEdit,
};

void BalancerCreate(Balancer_t *b, /* ...HW ptrs... */) {
    *b = Balancer;            /* <== copy prototype, including method ptrs */
    b->pIMU = imu;
    /* ... */
    Balancer.init(b);
};
```

Calls then use the instance's own copy of the pointer -- `bala.updatePitch(&bala)` in `main.c`.


The actual method implementations are `static` (file-private), so the only public entry points are the function pointers carried by the struct itself.

### What `Balancer_t` aggregates

`Balancer_t` (defined in `balancer_t.h`) groups its fields by responsibility, exactly as visualised in the [class diagram](#Diagrams):

- **HW pointers** -- `*pIMU`, `*pTOF`, `*pStepL`, `*pStepR`, `*pBatADC`. The hardware objects themselves are `static` globals in `main.c`; their addresses are *injected* via `BalancerCreate()`. This is dependency injection, not ownership.
- **Control objects (owned by value)** -- `PID_phi` (inner pitch loop), `PID_dist` (outer distance loop), `PID_velo` (middle velocity loop), `LPF_dist` (TOF low-pass filter `MeanVal_t`).
- **Operating state** -- pitch offset, distance setpoint, target velocity, measured velocity, motor positions, route variables, fall/active flags, current `TaskMode`, etc.
- **Parameters** -- one flat `ParamValue[PARAM_COUNT]` array indexed by the `ParamIdx` enum (`a_piKP`, `a_dKP`, `a_dSP`, ...), kept in lock-step with `defaultParam[]`, `ParamTitle[]` and `ParamScale[]` in `balancer_t.c`.
- **SysTick timers** -- `StepTaskTimer` (inner loop, 7 ms), `DistCtrlTaskTimer` (outer loop, 49 ms), `DispTaskTimer` (display, 700 ms).
- **Methods (function pointers)** -- `init`, `updatePitch`, `updateDist`, `updateDisplay`, `paramEdit`, `DispAlphaNumMPU`.

### Constructor and lifecycle

`BalancerCreate(b, imu, tof, stepL, stepR, bat)` is the constructor in `balancer_t.c`. It (1) copies the `const Balancer_t Balancer` prototype into `*b` to seed the six method function pointers, (2) wires the five HW pointers (`pIMU`, `pTOF`, `pStepL`, `pStepR`, `pBatADC`) to the `static` hardware instances declared in `main.c` -- dependency injection, lifetime stays with the caller -- and (3) calls `Balancer.init(b)`, which loads `defaultParam[]` into `ParamValue[]`, initialises the three PIDs and the TOF low-pass filter `LPF_dist` with their parameter values and sample times (`TA_INNER` / `TA_OUTER`), and resets every operating-state field. After the call, the object is ready for use:

```c
/* in main.c */
static MPU6050_t   MPU1;
static TOFSensor_t TOF1;
static Stepper_t   StepL, StepR;
static analogCh_t  adChn;
static Balancer_t  bala;

BalancerCreate(&bala, &MPU1, &TOF1, &StepL, &StepR, &adChn);
```

### Scheduling in `main.c`

The main loop is reduced to four parts: a SysTick tick handler, the parameter editor `bala.paramEdit(&bala)`, a `StepTask` switch over `bala.TaskMode`, and a 700 ms `DispTask`. The mode-FSM transitions (`M_InitBat` &rarr; `M_CheckI2cSlaves` &rarr; `M_Bala / M_DispMpuData / M_DispTofData / M_DistCtrl`) are visualised in the [program schedule diagram](#Diagrams).

### Files (Doxygen-commented in source)

- `balancer_t.h` -- type definitions, `TaskModus` and `ParamIdx` enums, `Balancer_t` struct, constructor prototype.
- `balancer_t.c` -- default parameter table, all method implementations (`BalancerInit`, `BalancerUpdatePitch`, `BalancerUpdateDist`, `BalancerUpdateDisplay`, `BalancerParamEdit`, `BalancerDispAlphaNumMPU`), helper `applyParams()` plus three private helpers (`StepperIHold`, `dispMPUBat`, `visualisationTOF`) used internally by the methods above, and the public prototype instance `Balancer`.
- `main.c` -- HW object instances, peripheral init, `CheckAndInitI2cSlaves()`, main loop with the `TaskMode` switch.


<a id="DistCtrl"></a>
## Part 2 -- Cascaded Distance Control (`M_DistCtrl`)

### Goal

When entering `M_DistCtrl` the robot must hold a target distance to an obstacle in front of it (typical demo: stationary on a treadmill, the belt moving the obstacle relative to the robot). Pitch control must keep working unchanged -- distance control sits *on top* of it.

### Cascade architecture

A direct distance-to-pitch PID would fight the inner balance loop. The robust solution is the classical **cascade** with three nested loops, each running at the rate that fits its dynamics:

| Loop | Input | Output | Rate | Object |
| --- | --- | --- | --- | --- |
| Outer (distance) | filtered TOF distance vs. setpoint | target velocity `tarVelo` [mm/s] | 49 ms | `PID_dist` |
| Middle (velocity) | `tarVelo` vs. measured velocity | target acceleration &rarr; `pitchOffset` [rad] | 49 ms | `PID_velo`, factor `a_a2p` |
| Inner (pitch) | pitch IMU vs. `pitchOffset` | stepper step command (alternating left and right side) | 7 ms | `PID_phi` (existing) |

The output of the cascade is `b->pitchOffset`. Inside the existing `BalancerUpdatePitch()`, it is added to the pitch reference and clamped by `a_pClamp` so that the outer loops can never destabilise the inner one:

```c
/* inside BalancerUpdatePitch() */
float offset = b->pitchOffset;
if (offset >  pClamp) offset =  pClamp;
if (offset < -pClamp) offset = -pClamp;
float error_phi = pitch - offset;
float setPitch  = (float)rad2step * PID.run(&b->PID_phi, error_phi);
```

This is the single integration point between the two halves of the project: `pitchOffset` is written by `updateDist()` and read by `updatePitch()`. Everything else stays decoupled.

### TOF handling and velocity fusion

`BalancerUpdateDist()` (in `balancer_t.c`) executes the outer two loops. Two practical details are worth highlighting:

1. **TOF dropouts.** When the sensor returns `TOF_VL53L0X_OUT_OF_RANGE` or a value below `a_distBlind`, the controller freezes the last filtered distance, sets `distanceVelo = 0`, and clears the velocity-PID integral. This avoids spikes from blind-spot readings driving the robot off.
2. **Velocity measurement.** Two independent estimates exist: `distanceVelo` derived from the filtered TOF, and `translationVeloMean` derived from stepper-position differences (`STEP_TO_MM_S` conversion, mean of left and right wheel). The current implementation picks whichever has the larger magnitude as `veloMeas` -- a simple "trust whichever shows movement" heuristic. This is a candidate for a proper sensor-fusion later.

### New parameters

The `ParamIdx` enum and `defaultParam[]` are extended with the cascade tunables -- every value is editable live via the rotary encoder + push button (`BalancerParamEdit()`):

`a_dKP`, `a_dKI`, `a_dKD`, `a_dSP` (setpoint mm), `a_dLPF` (TOF MeanVal weight), `a_distBlind`, `a_vKP`, `a_vKI`, `a_vKD`, `a_vMax`, `a_a2p` (acceleration&rarr;pitch gain), `a_pClamp`.

### Mode integration in `main.c`

The new mode `M_DistCtrl` runs the inner pitch loop on every `StepTask` tick (7 ms) and the outer/middle loops only when `DistCtrlTaskTimer` expires (49 ms):

```c
case M_DistCtrl:
    bala.updatePitch(&bala);                         /* every 7 ms */
    if (isSystickExpired(bala.DistCtrlTaskTimer)) {
        systickSetTicktime(&bala.DistCtrlTaskTimer, DistCtrlTaskTimeMs);
        bala.updateDist(&bala);                      /* every 49 ms */
    }
    break;
```

`M_Bala` is unchanged -- it simply does not call `updateDist()`, so `pitchOffset` stays zero and the inner loop behaves exactly as before. This makes `M_Bala` the safe fallback during distance-loop tuning.

### Files (Doxygen-commented in source)

- `balancer_t.h` -- added `M_DistCtrl` to `TaskModus`, distance-control fields in `Balancer_t`, distance-control entries in `ParamIdx`.
- `balancer_t.c` -- `BalancerUpdateDist()`, distance-control branches inside `BalancerUpdatePitch()` (offset clamp, integral reset on fall), distance entries in `defaultParam[]` / `ParamScale[]` / `ParamTitle[]`, distance branches in `applyParams()`.
- `main.c` -- `M_DistCtrl` case in the `TaskMode` switch and `DistCtrlTaskTimer` registration in the SysTick timer list.


<a id="Diagrams"></a>
## Diagrams

The three diagrams referenced from the sections above sit alongside this page in the Doxygen output.

**Class diagram -- `Balancer_t` aggregate** *(Part 1)*
Shows the root aggregate `Balancer_t` with its six sub-sections (HW pointers, control objects, operating state, parameters, SysTick timers, method-pointers). HW instances live in `main.c` and are aggregated by pointer; PIDs and the LPF are owned by value (composition). The diagram colour-codes what is new (distance-control fields and methods) versus what existed.

@image html  Balancer_OOP_structure.png  "Balancer_t -- OOP-style class diagram" width=100%


**Program schedule -- `TaskMode` FSM and tick scheduling** *(Part 1 &rarr; Part 2 entry point)*
Shows the mode transitions (`M_InitBat` &rarr; `M_CheckI2cSlaves` &rarr; `M_Bala / M_DispMpuData / M_DispTofData / M_DistCtrl`) driven by the rotary parameter `a_MODE`, and how the three SysTick timers (`StepTaskTimer` 7 ms, `DistCtrlTaskTimer` 49 ms, `DispTaskTimer` 700 ms) trigger which methods inside the main loop.

@image html  PAP_TopLevel.png  "Program schedule -- TaskMode FSM and SysTick task scheduling" width=75%


**Cascaded control loops -- `M_DistCtrl`** *(Part 2)*
Signal-flow diagram of the three nested loops. From left to right: distance setpoint `a_dSP` &rarr; outer PID &rarr; `tarVelo` (clamped by `a_vMax`) &rarr; middle PID &rarr; target acceleration &rarr; `* a_a2p` &rarr; `pitchOffset` (clamped by `a_pClamp`) &rarr; inner pitch PID &rarr; stepper command. Sensor returns: TOF + MeanVal LPF for distance, stepper-position derivative + TOF derivative for velocity (selected by max-magnitude switch).

@image html Balancer_control_loop_outer_cascade.png "Cascaded outer control loops - distance / velocity / pitch" width=100%

@image html Balancer_control_loop_pitch.png "Inner pitch control loop" width=100%



<a id="Authoren"></a>
## Authors

- [Luis Brunn](mailto:mt23009@lehre.dhbw-stuttgart.de) -- Studienarbeit T3200, TMT23GR1 *(refactoring + distance control)*

Supervisor: Prof. Dr.-Ing. [Tobias Gerhard Flaemig](mailto:flaemig@dhbw-stuttgart.de).
Built on the existing Balancer codebase developed at DHBW Stuttgart.
