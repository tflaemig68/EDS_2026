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

Function- and field-level documentation is placed directly as Doxygen comments in the source files. This page introduces the two parts and the [diagrams](#Diagrams) that explain the architecture.


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

- **HW pointers** -- `*pIMU`, `*pTOF`, `*pStepL`, `*pStepR`, `*pBatADC`. The hardware objects themselves are `static` globals in `main.c`; their addresses are *injected* via `BalancerCreate()`.
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

A direct distance-to-pitch PID could fight the inner balance loop an is more intransparent in terms of debugging and finding the control parameters. The robust solution is the classical **cascade** with three nested loops, each running at the rate that fits its dynamics:

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

The whole control architecture is displayed in [control diagrams](#Diagrams2) for both inner and outer loops as well as a detalied [viausisation](#Diagrams2) of the `updateDist()`-routine.

### TOF handling and velocity fusion

`BalancerUpdateDist()` (in `balancer_t.c`) executes the outer two loops. Two practical details are worth highlighting:

1. **TOF dropouts.** When the sensor returns `TOF_VL53L0X_OUT_OF_RANGE` or a value below `a_distBlind`, the controller freezes the last filtered distance, sets `distanceVelo = 0`, and clears the velocity-PID integral. This avoids spikes from blind-spot readings driving the robot off.
2. **Velocity measurement.** Two independent estimates exist: `distanceVelo` derived from the filtered TOF, and `translationVeloMean` derived from stepper-position differences (`STEP_TO_MM_S` conversion, mean of left and right wheel). The current implementation picks whichever has the larger magnitude as `veloMeas`.

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


### Distance Controller Tuning

#### Final Configuration

| Parameter | Value | Description |
|-----------|-------|-------------|
| `a_dKP`   | 8.0   | Distance loop proportional gain |
| `a_dKI`   | 0.002 | Distance loop integral gain |
| `a_dKD`   | 0.05  | Distance loop derivative gain |
| `a_dSP`   | 150 mm | Distance setpoint |
| `a_dLPF`  | 0.2   | TOF low-pass filter weight |
| `a_dBli`  | 10 mm | TOF blind-zone fallback distance |
| `a_vKP`   | 0.5   | Velocity loop proportional gain |
| `a_vKI`   | 0     | Velocity loop integral gain |
| `a_vKD`   | 0     | Velocity loop derivative gain |
| `a_vMax`  | 1000 mm/s | Maximum commanded velocity |
| `a_a2p`   | 0.0003 | Acceleration-to-pitch conversion gain |
| `a_pClp`  | 0.15 rad | Pitch clamp limit |

The balancer holds distance to a static wall and tracks a moving target (e.g. a hand). The cascaded control structure is fully functional.

### Tuning Procedure

The cascade was tuned inside-out: each stage was made stable before the next outer stage was enabled. Gains were changed one at a time, the physical response was observed, and adjustments were made based on a small set of recognisable failure modes.

#### Stage 1 — Operating environment

The physical limits of the stepper actuator were characterised before any cascade tuning. Because the wheels are position-controlled, the textbook coupling between commanded chassis lean and resulting translation does not hold cleanly: lean angles below approximately 0.05 rad produce no visible translation, while lean angles above approximately 0.125 to 0.15 rad cause the chassis to fall forward. The working range for the cascade output is therefore narrow. `a_pClp` was set to 0.15 rad so the cascade has access to the upper edge of the working band while the inner-loop fall detection at 0.35 rad remains as a final safety net.

#### Stage 2 — Static cascade gain

The cascade output at zero measured velocity is `pitchOffset = dKP × vKP × a2p × error_dist`. This product was tuned so that the cascade reaches the upper end of the working envelope at a typical distance error of 100 mm. Several gain combinations satisfy the required product, but the ratio between `dKP` and `vKP` matters. Initial attempts with `dKP = 2, vKP = 2` and `dKP = 4, vKP = 1` produced violent oscillation between the positive and negative pitchClamp. The cause was excessive authority of the velocity feedback path: the inner balance loop's wheel corrections produce velocity swings of several hundred mm/s, which at high `vKP` are sufficient to drive `pitchOffset` between its clamps each cycle.

The fix was to shift gain from `vKP` to `dKP` while preserving the product. The configuration `dKP = 8, vKP = 0.5` reduced velocity-loop authority by a factor of four and eliminated the oscillation while keeping the static cascade strength unchanged.

#### Stage 3 — Derivative damping on the distance loop

With proportional behaviour working, the robot accelerated toward the setpoint and tended to overshoot. The distance derivative term `a_dKD = 0.05` was introduced to provide anticipatory braking as the distance shrinks. This produced clean approach behaviour with minimal overshoot. The velocity derivative `a_vKD` was tried but proved unnecessary at the current velocity-loop gain.

#### Stage 4 — Integral correction on the distance loop

With proportional and derivative active, the robot reliably approached the setpoint but settled with a steady-state offset. This is the expected behaviour of a purely proportional cascade: at the setpoint `pitchOffset = 0`, but a small lean is physically required to overcome stepper detent torque and rolling friction. A small distance integral gain `a_dKI = 0.002` accumulates this offset and corrects the residual error. Larger values caused integral windup during the long approach phase and were rejected. The velocity integral `a_vKI` was deliberately kept at zero.

#### Stage 5 — Validation

The tuned cascade was validated in three scenarios:

1. **Static wall:** the robot approaches and stops within the distance of the setpoint without sustained oscillation.
2. **Manual disturbance:** pushing the robot away from or toward the wall produces a brief lean response, and the robot returns to the setpoint within a few seconds.
3. **Moving target:** holding a hand at varying distances within the TOF range causes the robot to translate in the corresponding direction, demonstrating closed-loop tracking.

### Recommendations for future work

Two hardware-level effects significantly influenced the tuning and should be characterised before any future tuning effort.

**Variation between balancer units.** During development two different balancer assemblies were used, with noticeably different behaviour. The pitch angle at which translation begins and the angle at which the chassis falls forward vary substantially between units. The 0.15 rad value used here is close to the fall threshold of the weaker-stepper unit but was insufficient to produce visible translation on the stronger-stepper unit. Future tuning should first measure these two thresholds (translation onset and fall limit) experimentally for the specific hardware in use, then size `a_pClp` and the static cascade gain to fit within the resulting working envelope.

**TOF sensor mounting angle.** On the units used here, the TOF sensor is mounted at a slight downward angle. When the robot leans forward to translate, the sensor's beam can intercept the floor rather than the intended target. The measured distance then drops rapidly as a side effect of the lean itself, not as a consequence of actual translation. This couples the inner pitch loop directly into the distance measurement and creates a false-feedback path that the controller cannot distinguish from real motion. The selection of the velocity value that has a larger magnitude between the translation velocity obtaianed from the weel positions and the one from the TOF distance reading and the usage of the blind distance are approaches to cancel those negative effects. For future balancer revisions and especially for treadmill operation, it could be a possibility to mount the TOF sensor so that its optical axis is horizontal when the chassis is upright.



<a id="Diagrams"></a>
## Diagrams

This section groups the diagrams by purpose. The first diagram explains the OOP-style aggregate structure introduced by the refactoring. The next two diagrams describe the top-level program execution and the internal sequence of the new distance-control routine. The last two diagrams show the control architecture itself: the cascaded outer loops and the existing inner pitch loop.

Together, these figures provide a compact visual explanation of how the structural refactoring and the new control feature fit together.

**Class diagram -- `Balancer_t` aggregate structure**
This diagram shows the `Balancer_t` aggregate introduced by the refactoring. It visualises which subsystems are aggregated by pointer from `main.c` and which control objects are owned directly by value inside the aggregate.

The diagram puts the new additions (distance-control fields and methods) into context with the existing infrastructure. It is meant to clarify the main architectural change of the project: the previous flat top-level implementation is replaced by one central object that encapsulates state, parameters, control objects and behaviour. 

@image html  Balancer_OOP_structure.png  "Balancer_t -  OOP style class / aggregate diagram" width=75%


**Top-level scheduling and distance-control routine**
The first diagram shows the top-level program flow of the refactored firmware. It covers static hardware-object creation, operating-parameter setup, entry into `main()`, peripheral initialisation, construction of the `Balancer` object, and the runtime `while(1)` loop containing the `TaskMode` state machine.

The second diagram focuses on the `BalancerUpdateDist()` routine itself. It visualises the sequence used by the new distance-control implementation: TOF acquisition, validity check, low-pass filtering, outer distance control, velocity estimation, middle velocity control, and generation of the `pitchOffset` signal for the inner loop.

Together, these two diagrams explain not only where the new distance control is called from, but also what happens internally during one execution of the outer control path.

@image html  PAP_TopLevel.png  "Top-level program schedule and task triggering" width=75%

@image html  PAP_UpdateDist.png  "BalancerUpdateDist() routine flowchart (distance control)" width=75%

<a id="Diagrams2"></a>

**Cascaded control architecture** 
These two control diagrams explain the regulation concept used for the new distance feature. The first figure shows the cascaded outer control structure, where distance error is converted into a target velocity, then into a target acceleration, and finally into a `pitchOffset` for the balancing loop.

The second figure shows the inner pitch loop and motor-command path. It makes clear that the existing balance controller remains the innermost stabilising loop, while the new distance-control feature only acts through the `pitchOffset` coupling point. This preserves the proven balancing behaviour and keeps the new feature modular.

Taken together, the two figures show how the new outer control loops are layered around the existing pitch controller without changing the basic motor actuation principle.

@image html Balancer_control_loop_outer_cascade.png "Cascaded distance–velocity–pitch control structure" width=100%

@image html Balancer_control_loop_pitch.png "Inner pitch loop and motor-command path" width=100%



<a id="Authoren"></a>
## Authors

- [Luis Brunn](mailto:mt23009@lehre.dhbw-stuttgart.de) -- Studienarbeit T3200, TMT23GR1 *(refactoring + distance control)*

Supervisor: Prof. Dr.-Ing. [Tobias Gerhard Flaemig](mailto:flaemig@dhbw-stuttgart.de).
Built on the existing Balancer codebase developed at DHBW Stuttgart.
