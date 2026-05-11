/**
 ******************************************************************************
 * @file         balancer_t.h
 * @author       Luis Brunn <https://github.com/LuBru70>
 * @brief        Header of the Balancer_t aggregate.
 *
 * @details      Defines the central Balancer_t type, including hardware-object
 *               pointers, control objects, operating state, editable parameters,
 *               SysTick timers, and method pointers. It also declares the
 *               TaskModus and ParamIdx enumerations, the external Balancer
 *               prototype instance, and the BalancerCreate() constructor.
 *
 * @par Architecture
 * - Balancer_t is an aggregate following the OOP-in-C pattern:
 *   encapsulation, composition, and prototype-copy construction with
 *   function-pointer based method.
 * - Hardware objects (MPU6050_t, TOFSensor_t, Stepper_t, analogCh_t)
 *   are aggregated by pointer. Their lifetime is owned by main.c.
 * - Control objects (PIDContr_t, MeanVal_t) are composed by value inside
 *   Balancer_t.
 *
 * @par Scope of this project (Studienarbeit T3200)
 * - Refactoring of the previous main.c functionality into balancer_t.h and
 *   balancer_t.c.
 * - Addition of the cascaded distance-control mode M_DistCtrl on top of the
 *   existing pitch-control inner loop.
 *
 * @date         March 2026
 ******************************************************************************
 * @attention This software is licensed based on CC BY-NC-SA 4.0
 ******************************************************************************
 */

#ifndef BALO_BALANCER_T_H_
#define BALO_BALANCER_T_H_

/* ----------- Standard and project includes ----------- */
/**
 *  Includes to enable access to basic types and the underlying
 *  hardware components that are aggregated inside Balancer_t.
 */
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx.h"
#include <i2cMPU.h>     // MPU6050_t  (IMU: pitch / accel / gyro)
#include <i2cAMIS.h>    // Stepper_t  (AMIS-30543 stepper drivers)
#include <i2cTOF.h>     // TOFSensor_t (VL53L0X distance sensor)
#include <adcBAT.h>     // analogCh_t (battery voltage measurement)
#include <regler.h>     // PIDContr_t, MeanVal_t (control + filter)

/* ============================================================================
 *  Device mask — which I2C slaves were found during the boot scan
 * ==========================================================================*/

/**
 * @name    Device mask bits
 * @brief   Bit positions for the devMask byte. Each bit is set by
 *          CheckAndInitI2cSlaves() in main.c once the corresponding
 *          slave answers on its I2C address.
 * @note    Identical encoding to the original reference_old_main.c so
 *          existing logic (and printout positions) keep working.
 * @{
 */
#define DevStepR  0x01   //!< Right stepper motor (AMIS) detected on I2C
#define DevStepL  0x02   //!< Left  stepper motor (AMIS) detected on I2C
#define DevMPU1   0x04   //!< MPU6050 IMU detected on I2C
#define DevTOF1   0x08   //!< VL53L0X TOF sensor detected on I2C
/** @} */

/* ============================================================================
 *  Cycle-time tables (declaration here, definition in balancer_t.c)
 * ==========================================================================*/

/**
 * @brief    StepTask cycle time table indexed by TaskModus.
 * @details  Defined in balancer_t.c and read by main.c when reloading
 *           the StepTaskTimer. The named period constants used to
 *           initialise this table are local to balancer_t.c.
 */
extern uint32_t StepTaskTime[];

extern uint32_t DistCtrlTaskTimeMs;	//!< Outer (distance) control loop sample time in ms.

/* ============================================================================
 *  Task-Mode enumeration
 * ==========================================================================*/

/**
 * @brief   Different task modes of the Balancer state machine.
 * @details Drives the switch in main.c. The order is preserved from
 *          reference main.c so that the rotary-encoder mode-select
 *          mechanism (writing into ParamValue[a_MODE]) keeps producing
 *          the same physical behaviour as before.
 */
typedef enum
{
    M_InitBat = 0,   	//!< Initial battery / startup screen
    M_DispMpuData,      //!< Live numeric display of MPU accel data (1)
    M_DispTofData,      //!< Live display of TOF distance (2)
    M_CheckI2cSlaves,   //!< Scan I2C buses, init found slaves (3)
    M_StepFollowPitch,  //!< Open-loop: stepper follows pitch (4)
    M_3DGinit,          //!< Reserved / 3D-gyro init placeholder (5)
    M_Bala,             //!< Pitch closed-loop balancing (6)
    M_DistCtrl          //!< NEW cascaded distance control mode (7)
} TaskModus;

/* ============================================================================
 *  Parameter index enumeration (rotary-encoder editable parameters)
 * ==========================================================================*/
/**
 * @brief 	Number of rotary-editable parameters.
 * @note  	Must match the number of entries in ParamIdx, defaultParam[],
 *        	ParamTitle[] and ParamScale[].
 */
#define PARAM_COUNT  25

/**
 * @brief   Index for Balancer_t ParamValue[].
 * @details The order MUST match defaultParam[] / ParamTitle[] /
 *          ParamScale[] in balancer_t.c. The first 13 entries (a_MODE
 *          through a_raTr) are inherited from the original
 *          main.c reference argParam mechanism, so existing tuning
 *          values keep their position. Entries a_dKP through a_pClamp
 *          are the new additions for the cascaded distance loop.
 */
typedef enum {
    /* ----- legacy params (same order as original ParamValue[]) ----- */
    a_MODE   = 0,   //!< Task-mode selector (write to switch mode)
    a_Cour,         //!< Route number (preselected motion route)
    a_posTol,       //!< Position tolerance [steps]
    a_phiZ,         //!< pitchZero offset [rad] (because of sensor mounting)
    a_GyAc,         //!< pitchFilt: gyro vs. accel complementary blend
    a_HwLP,         //!< Hardware DLPF index for MPU6050 (0..6)
    a_LP,           //!< Software accel low-pass weight
    a_piKP,         //!< PID_phi proportional gain
    a_piKI,         //!< PID_phi integral gain
    a_piKD,         //!< PID_phi derivative gain
    a_raRo,         //!< Rotation ramp increment per cycle
    a_maRo,         //!< Max rotation increment (clamp)
    a_raTr,         //!< Translation ramp
    /* ----- NEW: distance / velocity control parameters ----- */
    a_dKP,          //!< PID_dist proportional gain
    a_dKI,          //!< PID_dist integral gain
    a_dKD,          //!< PID_dist derivative gain
    a_dSP,          //!< Distance setpoint [mm]
    a_dLPF,         //!< MeanVal weight for TOF low-pass filter
    a_distBlind,    //!< TOF blind-zone threshold [mm]; smaller readings are rejected
    a_vKP,          //!< PID_velo proportional gain
    a_vKI,          //!< PID_velo integral gain
    a_vKD,          //!< PID_velo derivative gain
    a_vMax,         //!< Velocity clamp [mm/s]
    a_a2p,          //!< Accel-to-pitch gain (translation -> pitch ref)
    a_pClamp        //!< Variable pitch clamp [rad]; replaces fixed PITCH_CLAMP
} ParamIdx;

/* ============================================================================
 *  Balancer struct + prototype + constructor
 * ==========================================================================*/

/* Forward declaration so member function pointers can take Balancer_t* */
typedef struct Balancer Balancer_t;

/**
 * @brief   Central aggregate object of the balancer features.
 *
 * @details The Balancer_t aggregates:
 *          - hardware abstraction objects (by pointer, owned by main.c),
 *          - control objects (by value, owned by Balancer_t),
 *          - operating state (mode, flags, timers),
 *          - the editable parameters,
 *          - the public method table (function pointers).
 *
 *          Method dispatch follows the prototype-copy pattern: a const
 *          instance "Balancer" in balancer_t.c holds the function pointers,
 *          BalancerCreate() copies it into the user-supplied Balancer_t,
 *          then overwrites the HW pointers and calls init().
 *
 *          Lifetime: a single static Balancer_t named "bala" lives in
 *          main.c.
 */
struct Balancer {
    /* ---------- HW components (aggregated by pointer) ----------
     *  Created as globals in main.c; only their addresses are stored
     *  here so that main.c remains the single owner of HW.
     */
    MPU6050_t *pIMU;      	//!< IMU (MPU6050) — pitch/accel/gyro
    TOFSensor_t *pTOF;      //!< Time-of-Flight sensor (VL53L0X)
    Stepper_t *pStepL;    	//!< Left  stepper (AMIS-30543)
    Stepper_t *pStepR;    	//!< Right stepper (AMIS-30543)
    analogCh_t *pBatADC;   	//!< ADC channel for battery voltage

	/* ---------- Control Objects (composed by value) ---------- */
    PIDContr_t PID_phi;    	//!< Inner loop: pitch -> stepper position
    PIDContr_t PID_dist;  	//!< Outer loop: distance -> target velocity
    PIDContr_t PID_velo;   	//!< Middle loop: velocity -> target accel
    MeanVal_t LPF_dist;   	//!< Low-pass filter on TOF distance

	/* ---------- Operating variables/stated/modes ---------- */
	float pitchOffset;		//!< Output of velocity loop [rad]; subtracted from measured pitch to form error_phi (i.e. acts as the pitch setpoint)
	float distSetpoint;		//!< Target distance [mm]
	float tarVelo;          //!< Clamped target velocity [mm/s] (PID_dist out)
	float veloMeas;         //!< Selected velocity measurement [mm/s]
	float prevDist;         //!< Previous filtered dist for derivative
	float veloL;            //!< Left wheel velocity [mm/s]
	float veloR;            //!< Right wheel velocity [mm/s]
	int16_t prevPosL;       //!< Previous left  motor position [steps]
	int16_t prevPosR;       //!< Previous right motor position [steps]
	float accel2pitchK;     //!< Conversion gain accel -> pitch ref
	float incRot;
	float rampRot;
	float tarPosL;
	float tarPosR;
	float targetTra;
	float targetRot;
	int16_t curMotL;
	int16_t curMotR;
	bool activeMove;
	bool resetStepL;
	bool resetStepR;
	bool stepLenable;
	bool stepRenable;
	uint8_t  routeNum;
	uint8_t  routeStep;
	uint8_t  motionVar;
	int16_t  posMotL;
	int16_t  posMotR;


	TaskModus TaskMode;		//!< Current top-level task mode
	uint8_t devMask;		//!< Bit field of detected I2C devices
	bool RunInit;

	/* ---------- Params ---------- */
	float ParamValue[PARAM_COUNT];	//!< values, edited via rotary encoder

	/* ---------- SysTick timers ---------- */
	uint32_t StepTaskTimer;			//!< Inner state machine / inner control loop
	uint32_t DispTaskTimer;			//!< Display refresh tick
	uint32_t DistCtrlTaskTimer; 	//!< Outer (distance) control loop tick

	/* ---------- Methods ---------- */
	void (*init) (Balancer_t *b);			//!< Reset state + load defaults
    void (*updatePitch) (Balancer_t *b);	//!< Inner balance loop task
    void (*updateDist) (Balancer_t *b);		//!< Outer distance loop task
    void (*updateDisplay)(Balancer_t *b);	//!< Refresh TFT data area
    void (*paramEdit) (Balancer_t *b);		//!< Use rotary encoder
    void (*DispAlphaNumMPU)(Balancer_t *b); //!< IMU display

};

/**
 * @brief   Const "prototype" instance carrying the method pointers.
 * @details Defined in balancer_t.c. Copied into every user-supplied
 *          Balancer_t by BalancerCreate() to wire up the method table.
 */
extern const Balancer_t Balancer;


/**
 * @brief       Constructor for the Balancer_t aggregate object.
 *
 * @details     Performs the OOP-in-C "construct" sequence:
 *              1. Copies the const Balancer prototype into *b
 *                 (this wires up the method-pointer table).
 *              2. Stores the supplied HW pointers into the aggregate.
 *              3. Calls Balancer.init(b) to load defaultParam[] into
 *                 ParamValue[], initialise the three PID controllers
 *                 and the TOF low-pass filter, and reset all operating
 *                 state to a known starting condition.
 *
 * @param[out]  b      Pointer to Balancer_t instance.
 * @param[in]   imu    Pointer to the MPU6050 object.
 * @param[in]   tof    Pointer to the TOF sensor object.
 * @param[in]   stepL  Pointer to the LEFT stepper object.
 * @param[in]   stepR  Pointer to the RIGHT stepper object.
 * @param[in]   bat    Pointer to the battery ADC channel object.
 *
 */
void BalancerCreate(Balancer_t *b,
                     MPU6050_t *imu, TOFSensor_t *tof,
                     Stepper_t *stepL, Stepper_t *stepR,
                     analogCh_t *bat);


#endif /* BALO_BALANCER_T_H_ */
