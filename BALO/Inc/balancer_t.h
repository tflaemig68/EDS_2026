/**
 *      balancer_t.h
 *
 *      @file balancer_t.h provides architecture of the balancer superclass
 *      @author: Luis Brunn
 *      Created on: Mar. 18, 2026
 */

#ifndef BALO_BALANCER_T_H_
#define BALO_BALANCER_T_H_

/**
 *  includes to enable access to basic functions and hardware components
 *
 */
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx.h"
#include <i2cMPU.h>
#include <i2cAMIS.h>
#include <i2cTOF.h>
#include <adcBAT.h>
#include <regler.h>

// *DevMask copies the #defines from original main.c
#define DevStepR 0x01
#define DevStepL 0x02
#define DevMPU1  0x04
#define DevTOF1  0x08

/** declared here, defined in balancer_t.c
 * The actual values and the #defines they reference belong in balancer_t.c
 */
extern uint32_t StepTaskTime[];
extern uint32_t DistCtrlTaskTimeMs;

/**
 * typedef for Task-Modes
 *
 */
typedef enum
{
	M_InitBat = 0,
	M_DispMpuData,  	// 1
	M_DispTofData,
	M_CheckI2cSlaves,  	// prüfen welche I2C Slaves vorhanden sind
	M_StepFollowPitch,  // open loop control - stepper follows the pitch
	M_3DGinit,			//
	M_Bala,
    M_DistCtrl,   		// new: for cascade distance control
} TaskModus;

/** Parameter index enum
 *  Order must match defaultParam[] in balancer_t.c.
 *  First 13 = original main.c params from argParam (unchanged).
 *  Last 5  = new distance control params.
 */
#define PARAM_COUNT  25

typedef enum {
    /* same order as original ParamValue[] */
    a_MODE   = 0,  // task mode selector (write to switch mode)
    a_Cour,        // route number
    a_posTol,      // position tolerance [steps]
    a_phiZ,        // pitchZero offset [rad]
    a_GyAc,        // pitchFilt: gyro vs accel blend weight
    a_HwLP,        // hardware DLPF index (0–6)
    a_LP,          // software accel low-pass weight
    a_piKP,        // PID_phi proportional gain
    a_piKI,        // PID_phi integral gain
    a_piKD,        // PID_phi derivative gain
    a_raRo,        // rotation ramp increment per cycle
    a_maRo,        // max rotation increment (clamp)
    a_raTr,        // translation ramp (reserved)
    /* new: distance control */
    a_dKP,         // PID_dist proportional gain
    a_dKI,         // PID_dist integral gain
    a_dKD,         // PID_dist derivative gain
    a_dSP,         // distance setpoint [mm]
    a_dLPF,        // MeanVal weight for TOF low-pass filter
    a_distBlind,   // [mm] used when TOF out of range
    a_vKP,         // PID_velo proportional gain
    a_vKI,         // PID_velo integral gain
    a_vKD,         // PID_velo derivative gain
    a_vMax,        // velocity clamp [mm/s]
    a_a2p,         // accel-to-pitch gain
    a_pClamp,      // variable pitch clamp [rad]
} ParamIdx;

// ******************** Predeclare Balancer Type ********************
typedef struct Balancer Balancer_t;

/**
 * TODO: Beschreibung überarbeiten
 * @brief
 *
 */

struct Balancer {
	/* ---------- HW Components - given as ptr ----------
	 * Create these objects as globals in main.c
	 * Pass their addresses to BalancerCreate()
	 */
	MPU6050_t *pIMU;
	TOFSensor_t *pTOF;
	Stepper_t *pStepL;
	Stepper_t *pStepR;
	analogCh_t *pBatADC;

	/* ---------- Control Objects ---------- */
	PIDContr_t PID_phi;
	PIDContr_t PID_dist;
	PIDContr_t PID_velo;
	MeanVal_t LPF_dist;

	/* ---------- Operating variables/stated/modes ---------- */
	float pitchOffset;		// output of PID_dist [rad]
	float distSetpoint;		// target distance [mm]
	float tarVelo;          // clamped target velocity [mm/s]
	float veloMeas;         // fused velocity measurement [mm/s]
	float prevDist;         // previous filtered dist for derivative
	float veloL;            // left wheel velocity [mm/s]
	float veloR;            // right wheel velocity [mm/s]
	int16_t prevPosL;       // previous left motor position [steps]
	int16_t prevPosR;       // previous right motor position [steps]
	float accel2pitchK;     // conversion factor
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
	uint8_t  routeNum;     // which pre-defined route is active
	uint8_t  routeStep;    // current waypoint index
	uint8_t  motionVar;    // 0 = load waypoint, 1 = wait for arrival
	int16_t  posMotL;      // last commanded position, left motor
	int16_t  posMotR;      // last commanded position, right motor


	TaskModus TaskMode;
	uint8_t devMask;	// which HW was found on I2C
	bool RunInit;

	/* ---------- Params ---------- */
	float ParamValue[PARAM_COUNT];

	/* ---------- SysTick timers ---------- */
	uint32_t StepTaskTimer;		// inner control loop (PID_phi)
	uint32_t DispTaskTimer;		// loop countdown for display communication
	uint32_t DistCtrlTaskTimer; // outer control loop (PID_dist)

	/* ---------- Methods ---------- */
	void (*init) (Balancer_t *b);
    void (*updatePitch) (Balancer_t *b);
    void (*updateDist) (Balancer_t *b);
    void (*updateDisplay)(Balancer_t *b);
    void (*paramEdit) (Balancer_t *b);
    void (*DispAlphaNumMPU)(Balancer_t *b);

};

// ******************** Balancer ********************
extern const Balancer_t Balancer;

/**
 * Constructor/Helper
 * TODO: Erweitern und Anpassung an die finale Balancer Struktur
 *
 */
void BalancerCreate(Balancer_t *b,
                     MPU6050_t *imu, TOFSensor_t *tof,
                     Stepper_t *stepL, Stepper_t *stepR,
                     analogCh_t *bat);


#endif /* BALO_BALANCER_T_H_ */
