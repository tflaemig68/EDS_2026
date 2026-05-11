/**
 ******************************************************************************
 * @file         balancer_t.c
 * @author       Luis Brunn <https://github.com/LuBru70>
 * @brief        Implementation of the Balancer_t aggregate object.
 * @date         March 2026
 ******************************************************************************
 *
 * @details
 * This file implements the OOP-in-C style Balancer_t module. It contains the
 * default parameter tables, the module-level timing constants, the method
 * implementations assigned to the Balancer_t function pointers, private helper
 * functions, the const Balancer prototype instance, and the BalancerCreate()
 * constructor.
 *
 * @par Module responsibilities
 * - Defines the default values for the 25 tunable parameters:
 *   defaultParam[], ParamTitle[] and ParamScale[].
 *   The element order must stay synchronized with the ParamIdx enum declared
 *   in balancer_t.h.
 * - Implements the Balancer_t methods:
 *   BalancerInit(), BalancerUpdatePitch(), BalancerUpdateDist(),
 *   BalancerUpdateDisplay(), BalancerParamEdit() and
 *   BalancerDispAlphaNumMPU().
 * - Provides private static helper functions:
 *   StepperIHold(), dispMPUBat(), visualisationTOF() and applyParams().
 * - Defines the const Balancer_t prototype instance. This prototype carries
 *   the method-pointer table and is copied into every user-supplied Balancer_t
 *   instance inside BalancerCreate().
 * - Provides BalancerCreate(), which connects the externally owned hardware
 *   objects from main.c to the Balancer_t aggregate and initializes the object.
 *
 * @par Timing constants
 * The cycle times are defined by preprocessor macros at the top of this file:
 * - StepTaskTimeSet: 7 ms inner pitch-control period.
 * - DistCtrlTaskTimeSet: 49 ms outer/middle distance-control period.
 * - DispTaskTimeSet: 700 ms display refresh period.
 *
 * The corresponding sample times in seconds are provided by TA_INNER and
 * TA_OUTER and are used when initializing the PIDContr_t objects.
 *
 * @par Cascade integration
 * The distance-control extension is coupled into the original pitch-control
 * loop through b->pitchOffset. BalancerUpdateDist() computes this value from
 * the outer distance loop and the middle velocity loop. BalancerUpdatePitch()
 * then subtracts the clamped pitch offset from the measured pitch before the
 * inner PID_phi controller is evaluated.
 *
 * In M_Bala, BalancerUpdateDist() is not executed, so the pitch controller
 * behaves as the original balancing loop. When the selected mode is not
 * M_DistCtrl, BalancerUpdateDisplay() resets the distance-control state,
 * including pitchOffset and tarVelo, to avoid reusing stale outer-loop values.
 *
 * If a fall is detected in BalancerUpdatePitch(), the distance and velocity
 * controller states are additionally reset so that accumulated outer-loop
 * integrator values do not remain active after recovery.
 *
 * @par Visibility
 * Functions that are only used inside this translation unit are declared
 * static. The public Balancer_t interface is provided through balancer_t.h.
 * The main public construction entry point is BalancerCreate(); the method
 * pointers themselves are initialized from the const Balancer prototype.
 *
 ******************************************************************************
 * @attention This software is licensed based on CC BY-NC-SA 4.0
 ******************************************************************************
 */

/* ----------- Includes ----------- */
#include <stdio.h>
#include "balancer_t.h"
#include <math.h>
#include <mcalSysTick.h>
#include <mcalGPIO.h>
#include <ST7735.h>
#include <RotaryPushButton.h>
#include <adcBAT.h>
#include "i2cDevices.h"

/* ============================================================================
 *  Timing configuration
 * ==========================================================================*/

/**
 * @name    Task-cycle times (compile-time constants, file-private)
 * @{
 */
#define StepTaskTimeSet 7UL			//!< Inner state machine [ms] (communication to stepper takes <1ms therefore total StepTaskTime = 7ms)
#define DistCtrlTaskTimeSet 49UL	//!< Outer (distance) loop [ms] (currently 7x inner StepTaskTime)
#define DispTaskTimeSet 700UL		//!< Display refresh [ms]
/** @} */

/**
 * @name    Sample-time constants in seconds (used by PID.init)
 * @{
 */
#define TA_INNER ((float)StepTaskTimeSet    * 1e-3f)   //!< 0.007 s
#define TA_OUTER ((float)DistCtrlTaskTimeSet * 1e-3f)  //!< 0.049 s
/** @} */

uint32_t DistCtrlTaskTimeMs = DistCtrlTaskTimeSet;

uint32_t StepTaskTime[] = {
  50UL, 50UL, 100UL, 50UL,
  StepTaskTimeSet,
  StepTaskTimeSet,
  StepTaskTimeSet,
  StepTaskTimeSet
};									//!< StepTask cycle-time table (same order as TaskModus).

/* ----------- Stepper motors position conversion ----------- */
/**
 * @brief   Conversion gain to translate pitch value to stepper step counts
 * @details rad2step converts the balancing controller’s pitch correction into stepper microsteps,
 * 			so the motors can execute the correction.
 * 			This increment is added to the current left/right motor target position
 *          before calling StepperSetPos().
 *          Approximate theoretical value for 200 steps/rev and 1/16 microstepping:
 *          200 * 16 / (2*pi) = 509,3.
 *          The implementation uses 520.
 */
static const int16_t rad2step = 520;

/**
 * @name Wheel-velocity scaling constants
 * @{
 */
#define WHEEL_CIRC_MM   375.0f                                  //!< wheel circumference [mm]
#define STEPS_PER_REV   3200.0f                                 //!< 200 full steps x 16 microsteps
#define STEP_TO_MM_S    (WHEEL_CIRC_MM / STEPS_PER_REV / TA_OUTER)
/** @} */

/* ----------- Fall detection ----------- */
#define PITCH_FALL_RAD 0.35f  		//!< |pitch| > this: robot fallen
#define PITCH_ACTIVE 0.05f  		//!< |pitch| < this: balanced


/* ============================================================================
 *  Default parameters
 * ==========================================================================*/

/**
 * @brief   Default parameter values, copied into ParamValue[] at init().
 * @details Order MUST match the ParamIdx enum in balancer_t.h.
 */
static const float defaultParam[PARAM_COUNT] = {
		0,        	// a_MODE task mode selector
		0,        	// a_Cour route number
		0.0f,     	// a_posTol position tolerance [steps]
		-0.004f,  	// a_phiZ pitchZero offset [rad]
		0.98f,    	// a_GyAc complementary filter weight (98% gyro)
		5,        	// a_HwLP hardware DLPF index (→ LPBW_10)
		0.36f,    	// a_LP software accel low-pass weight
		0.75f,    	// a_piKP PID_phi proportional gain
		0.058f / TA_INNER,   	// a_piKI PID_phi integral gain
		0.27f * TA_INNER,    	// a_piKD PID_phi derivative gain
		0.002f,   	// a_raRo rotation ramp increment per cycle
		10.0f,    	// a_maRo max rotation increment (clamp)
		0.01f,    	// a_raTr translation ramp (reserved)

		/* New params for distance control */
		0.5f,    	// a_dKP PID_dist proportional gain
		0.000f,   	// a_dKI PID_dist integral gain
		0.00f,   	// a_dKD PID_dist derivative gain
		280.0f,  	// a_dSP distance setpoint [mm]
		0.2f,      	// a_dLPF MeanVal weight for TOF low-pass
		100.0f,		// a_distBlind: fallback distance [mm] used when TOF is invalid
		0.00f,      // a_vKP
		0.00f,      // a_vKI
		0.00f,      // a_vKD
		2000.0f,    // a_vMax [mm/s]
		1.0f,       // a_a2p accel2pitch gain
		0.05f,      // a_pClamp variable pitch clamp [rad]
};

static const char ParamTitle[PARAM_COUNT][5] = {
	"MODE", "Cour", "poTo", "phiZ", "GyAc", "HwLP", "LP  ",
	"piKP", "piKI", "piKD", "raRo", "maRo", "raTr",
	"dKP ", "dKI ", "dKD ", "dSP ", "dLPF", "dBli",
	"vKP ", "vKI ", "vKD ", "vMax", "a2p ", "pClp"
};					//!< labels for parameter names on the TFT.

/**
 * @brief   Parameter scale factor for rotary encoder editing.
 * @details ParamValue[i] = (float)encoderTicks / ParamScale[i].
 *          Larger scale -> finer increment per click. The values are
 *          adjusted to give a sensible "step" per encoder click for the
 *          expected operating range of each parameter.
 */
static const float ParamScale[PARAM_COUNT] = {
    1,   1,   0.2,  500,  100,  1,    500,
    100, 500,  100,  500,  1,    500,
    /* Distance loop */
    20,    // a_dKP    		inc 0.05
    1000,  // a_dKI    		inc 0.001
    100,   // a_dKD    		inc 0.01
    0.1,   // a_dSP    		inc 10mm
    100,   // a_dLPF   		inc 0.01
    0.1,   // a_distBlind 	inc 10mm
    /* Velocity loop */
    10000, 100000, 10000, 0.5,
    100000, 1000
};

/* ============================================================================
 *  PRIVATE Function IMPLEMENTATIONS
 *  ---------------------------------------------------------------------------
 *  All functions below are file-private because they are declared static.
 *  Some of them are used as Balancer_t methods by storing their addresses in
 *  the const Balancer prototype instance at the bottom of this file.
 *  Others are internal helper functions used only by those methods.
 * ==========================================================================*/

/**
 * void StepperIHold(Balancer_t *b, bool OnSwitch)
 * @param[in]	b			Balancer instance whose left and right steppers are updated
 *
 * @param[in]  	OnSwitch == true;  IHold active;
 *						 == false; IHold reduced to minimum
 * @returns ---
 * function that toggles the stepper holding current
 * Takes Balancer_t* so accessing b->pStepL and R is possible
 */
static void StepperIHold(Balancer_t *b, bool OnSwitch)
{
	static bool OldStatus = false;
	const uint8_t iOff = 0x0;  	// switch off the PWM Regulator Value 0xFF only for AMIS
								// 0xFF only for AMIS IC
								// 0x0 reduced current to 59mA

	if (OnSwitch == OldStatus) return;   	// commands only active if OnSwitch Status changed

	if (OnSwitch) {
		stepper.iHold.set(b->pStepL, 2);   	// vMin = 2 (from StepPaValue[2])
		stepper.iHold.set(b->pStepR, 2);
		setLED(YELLOW);
	} else {
		setLED(RED);
		stepper.iHold.set(b->pStepL, iOff);
		stepper.iHold.set(b->pStepR, iOff);
	}
	OldStatus = OnSwitch;
}

/**
 * @fn    BalancerInit
 *
 * @brief       Initialise the software state of a Balancer_t instance.
 *
 * @details     Called once by BalancerCreate() and again whenever a
 *              full re-init is required. Performs:
 *              1. Copy defaultParam[] into ParamValue[].
 *              2. Initialise PID_phi, PID_dist, PID_velo.
 *              3. Clear the TOF distance low-pass filter.
 *              4. Reset every operating-state field (mode, flags,
 *                 timers, motor positions, route counters).
 *
 *              After return the Balancer is in M_InitBat, with RunInit
 *              set so the first M_Bala / M_DistCtrl entry will perform
 *              its initialisation.
 *
 * @param[in]   b   Pointer to the Balancer_t to initialise.
 *
 * @returns     ---
 */
static void BalancerInit(Balancer_t *b) {
	/* Copy default params */
	for (int i = 0; i < PARAM_COUNT; i++) {
		b->ParamValue[i] = defaultParam[i];
	}

	/* Init PID controllers */
    PID.init(&b->PID_phi,
             b->ParamValue[a_piKP], b->ParamValue[a_piKI],
             b->ParamValue[a_piKD], TA_INNER);
    PID.init(&b->PID_dist,
             b->ParamValue[a_dKP], b->ParamValue[a_dKI],
             b->ParamValue[a_dKD], TA_OUTER);
    PID.init(&b->PID_velo,
             b->ParamValue[a_vKP], b->ParamValue[a_vKI],
             b->ParamValue[a_vKD], TA_OUTER);

    /* Init / clear Low Pass Filter for TOF distance measurement */
    MeanVALclr(&b->LPF_dist, b->ParamValue[a_dLPF]);

    /* Set all operating states (previously done as local variables in main.c) */
	b->TaskMode          = M_InitBat;
	b->devMask           = 0;
	b->RunInit           = true;
	b->activeMove        = false;
	b->resetStepL        = true;
	b->resetStepR        = true;
	b->stepLenable       = false;
	b->stepRenable       = false;
	b->incRot            = 0.0f;
	b->rampRot           = 0.0f;
	b->pitchOffset       = 0.0f;
	b->distSetpoint      = b->ParamValue[a_dSP];
	b->tarPosL           = 0.0f;
	b->tarPosR           = 0.0f;
	b->tarVelo    		 = 0.0f;
	b->veloMeas  		 = 0.0f;
	b->prevDist  		 = 0.0f;
	b->veloL      		 = 0.0f;
	b->veloR     		 = 0.0f;
	b->prevPosL  		 = 0;
	b->prevPosR   		 = 0;
	b->accel2pitchK = b->ParamValue[a_a2p];
	b->targetTra         = 0.0f;
	b->targetRot         = 0.0f;
	b->curMotL           = 0;
	b->curMotR           = 0;
	b->posMotL           = 0;
	b->posMotR           = 0;
	b->routeNum          = (uint8_t)b->ParamValue[a_Cour];
	b->routeStep         = 0;
	b->motionVar         = 0;
	b->StepTaskTimer     = 0UL;
	b->DispTaskTimer     = 0UL;
	b->DistCtrlTaskTimer = 0UL;
}

/* ---------- Display helpers (moved from main.c into balancer_t) ---------- */

/**
 * @fn		    dispMPUBat
 *
 * @brief       Display IMU temperature and battery voltage on one TFT line.
 *
 * @details     Reads the temperature from the mpuGetTemp and the
 *              latest battery voltage from the ADC channel (getBatVoltage).
 *              Set print color depending on the battery status
 *              (green = ok, yellow = half, red = empty/undervolt/overvolt).
 *
 * @param[in]   pMPU1   Pointer to the MPU6050 object.
 * @param[in]   pADChn  Pointer to the battery ADC channel object.
 *
 * @returns     --- (Written prints on the TFT display)
 */
static void dispMPUBat(MPU6050_t *pMPU1, analogCh_t *pADChn)
{
    char strT[32];
    float temp = mpuGetTemp(pMPU1);
    BatStat_t batStatus = getBatVolt(pADChn);

    if (batStatus == halfBat)       { tftSetColor(tft_BLACK, tft_YELLOW); }
    else                            { tftSetColor(tft_WHITE, tft_RED);    }
    if (batStatus == okBat)         { tftSetColor(tft_GREEN, tft_BLACK);  }

    sprintf(strT, "T:%+3.1f Bat:%3.1fV", temp, pADChn->BatVolt);
    tftPrint((char *)strT, 10, 110, 0);
}

/**
 * @fn		    BalancerDispAlphaNumMPU
 *
 * @brief       Display MPU accel components (X/Y/Z) as numeric values.
 *
 * @details     Used by the M_DispMpuData diagnostic/debugging mode. Triggers a
 *              fresh accel read on the IMU, then prints the three axes
 *              one above the other at fixed TFT coordinates.
 *
 * @param[in]   b   Pointer to the Balancer_t.
 *
 * @returns     --- (Written prints on the TFT display)
 */
static void BalancerDispAlphaNumMPU(Balancer_t *b)
{
    char str[16];
    mpuGetAccel(b->pIMU);
    sprintf(str, "%+6.3f", b->pIMU->accel[0]); tftPrint(str, 20, 50, 0);
    sprintf(str, "%+6.3f", b->pIMU->accel[1]); tftPrint(str, 20, 60, 0);
    sprintf(str, "%+6.3f", b->pIMU->accel[2]); tftPrint(str, 20, 70, 0);
}

/**
 * @fn		    visualisationTOF
 *
 * @brief       Display the latest TOF distance value in centimetres.
 *
 * @param[in]   TOFSENS  Pointer to the TOFSensor_t
 *
 * @returns     --- (Written prints on the TFT display)
 */
static void visualisationTOF(TOFSensor_t *TOFSENS)
{
    char buffer[8];
    static uint16_t oldDistance = TOF_VL53L0X_OUT_OF_RANGE;

    if (TOFSENS->distanceFromTOF != TOF_VL53L0X_OUT_OF_RANGE)
    {
        if (oldDistance == TOF_VL53L0X_OUT_OF_RANGE)
        	tftPrint((char *)"cm ", 24, 94, 0);
        if ((int16_t)fabs((float)TOFSENS->distanceFromTOF - (float)oldDistance) > 10)
        {
            sprintf(buffer, "%02d", TOFSENS->distanceFromTOF / 10);
            tftPrint(buffer, 0, 94, 0);
        }
    }
    else
    {
        tftPrint((char *)"out of range", 0, 94, 0);
    }
    oldDistance = TOFSENS->distanceFromTOF;
}

/**
 * @fn		    BalancerUpdatePitch
 *
 * @brief       Inner balance loop.
 *
 * @details     Translation of "case M_Bala" from the reference
 *              into a method of the Balancer_t object. Sequence:
 *
 *                1. RunInit branch (on entering M_Bala/M_DistCtrl):
 *                   - Switch TFT to landscape-flip and paint header.
 *                   - Activate stepper hold current and zero positions.
 *                   - Push parameter values into the IMU
 *                     (pitchZero, swLowPassFilt, pitchFilt, HW DLPF).
 *                   - (Re-)initialise PID_phi.
 *                   - Set the IMU axis remap (RPY[2,3,-1]) which matches
 *                     the way the MPU is mounted on this robot.
 *                   - Set IMU integration time-base.
 *
 *                2. Read pitch from the IMU and run fall detection
 *                   (|pitch| > PITCH_FALL_RAD -> robot fallen):
 *                   - Re-init PID_phi and clear PID_dist/PID_velo to
 *                     drop integrator wind-up accumulated during the fall.
 *                   - Reduce stepper current (relax the motors).
 *                   - return.
 *
 *                3. Determine "actively balancing" state:
 *                   - |pitch| < PITCH_ACTIVE -> activeMove = true.
 *                   - Otherwise hold current is kept on but no motor
 *                     command is issued.
 *
 *                4. Active PID (pitch) step:
 *                   - Clamp pitchOffset to +/- a_pClamp so the outer loop
 *                     can never destabilise the inner loop.
 *                   - error_phi = pitch - pitchOffset.
 *                   - setPitch  = rad2step * PID_phi.run(error_phi).
 *                   - Once activeMove is asserted, command ONE motor (left/right
 *                     alternating via stepRenable toggle)- This keeps
 *                     the per-call I2C load below the 7 ms timing.
 *
 * @param[in]   b   Pointer to the Balancer_t object.
 *
 * @returns     ---
 */
static void BalancerUpdatePitch(Balancer_t *b) {
	/* RunInit: when TaskMode M_Bala or M_DistCtrl is entered */
	if (b->RunInit) {
		tftSetRotation(LANDSCAPE_FLIP);
		tftFillScreen(tft_BLACK);
		tftSetColor(tft_RED, tft_WHITE);
		tftPrint("DHBW Bala-V2.0", 0, 0, 0);	// TODO: replace with SwVersion so the right value is always displayed
		tftSetColor(tft_GREEN, tft_BLACK);
		dispMPUBat(b->pIMU, b->pBatADC);
		StepperIHold(b, true);					//IHold switched on
		StepperResetPosition(b->pStepL);  		//resetPosition
		StepperResetPosition(b->pStepR);
		b->incRot = 0;
		b->tarPosR = 0;

		// Replacement of SetRegParameter()
		b->pIMU->pitchZero = b->ParamValue[a_phiZ];
		b->pIMU->swLowPassFilt = b->ParamValue[a_LP];
		b->pIMU->pitchFilt = b->ParamValue[a_GyAc];
		PID.init(&b->PID_phi,
				 b->ParamValue[a_piKP], b->ParamValue[a_piKI],
				 b->ParamValue[a_piKD], TA_INNER);

		// HwLP: config MPU low pass filter
		MPUlpbw tableLPFValue[7] = {LPBW_260, LPBW_184, LPBW_94, LPBW_44, LPBW_21, LPBW_10, LPBW_5};

		if (b->ParamValue[a_HwLP] < 0) b->ParamValue[a_HwLP] = 0;
		if (b->ParamValue[a_HwLP] > 6) b->ParamValue[a_HwLP] = 6;
		b->pIMU->LowPassFilt = tableLPFValue[(uint8_t)b->ParamValue[a_HwLP]];
		mpuSetLpFilt(b->pIMU);

		// set MPU assemble
        b->pIMU->RPY[0] = 2;            // MPU y-axis points forward
        b->pIMU->RPY[1] = 3;            // MPU z-axis points to the left
        b->pIMU->RPY[2] = -1;           // MPU x-axis points down
		b->pIMU->timebase = (float) (StepTaskTimeSet+1) * 10e-4;  		// CycleTime for calc from Gyro to angle fitting
		b->RunInit = false;
	}

	/* Read pitch from IMU + fall detection */
	setLED(RED_off);
	mpuGetPitch(b->pIMU);
	setLED(RED_on);

	float pitch = b->pIMU->pitch;

	if (fabs(pitch) > PITCH_FALL_RAD) {
		// Robot fallen —> reset
		b->activeMove = false;

		// Re-init PID to clear integral windup
		PID.init(&b->PID_phi,
				b->ParamValue[a_piKP], b->ParamValue[a_piKI],
				b->ParamValue[a_piKD], TA_INNER);

		PIDclear(&b->PID_dist);
		PIDclear(&b->PID_velo);
		b->pitchOffset = 0.0f;
		b->tarVelo = 0.0f;

		StepperIHold(b, false);         // reduce current to save power
		b->incRot     = 0;
		b->tarPosR    = 0;
		b->tarPosL    = 0;
		b->resetStepL = true;
		b->resetStepR = true;
		b->routeNum   = b->ParamValue[a_Cour];
		b->routeStep  = 0;
		b->motionVar  = 0;
		b->rampRot    = 0;
		return;                         // skip balance computation
	}

	/* Check if balance action needed */
	if (fabs(pitch) < PITCH_ACTIVE) {
		setLED(GREEN);
		b->activeMove = true;           // pitch small enough start balancing
	} else {
		setLED(YELLOW);
		StepperIHold(b, true);          // not balanced yet, but hold motors
	}

	/* Active PID balancing with stepper commands */
	if (b->activeMove) {
		// Clamp pitchOffset so outer loop cannot destabilise the inner loop
		float offset = b->pitchOffset;
		float pClamp = b->ParamValue[a_pClamp];
		if (offset >  pClamp) offset =  pClamp;
		if (offset < -pClamp) offset = -pClamp;

		// Inner loop error: positive offset leans forward (closing distance)
		float error_phi = pitch - offset;
		float setPitch  = (float)rad2step * PID.run(&b->PID_phi, error_phi);

		// Rotation ramp
		if (b->rampRot < 1.0f) {
			b->rampRot += b->ParamValue[a_raRo];
		}

		if (b->stepRenable) {
			// Right motor cycle
			setLED(RED_off);
			if (!b->resetStepR) {
				b->curMotR = (int16_t)StepperGetPos(b->pStepR);     // 0.3 ms
			} else {
				b->curMotR = 0;
				StepperResetPosition(b->pStepR);
				b->resetStepR = false;
			}
			b->tarPosR = b->curMotR + b->incRot * b->rampRot;
			b->posMotR = (int16_t)(setPitch + b->tarPosR);
			StepperSetPos(b->pStepR, b->posMotR);                   // 0.4 ms
			b->stepRenable = false;
			setLED(RED_on);
		} else {
			// Left motor cycle
			if (!b->resetStepL) {
				b->curMotL = (int16_t)StepperGetPos(b->pStepL);     // 0.3 ms
			} else {
				b->curMotL = 0;
				StepperResetPosition(b->pStepL);
				b->resetStepL = false;
			}

			b->tarPosL = b->curMotL - b->incRot * b->rampRot;
			b->posMotL = (int16_t)(setPitch + b->tarPosL);
			StepperSetPos(b->pStepL, b->posMotL);
			b->stepRenable = true;
		}
	}
	setLED(RED_off);
}

/**
 * @fn		    BalancerUpdateDist
 *
 * @brief       Outer (distance) + middle (velocity) loop procedure.
 *
 * @details     Runs at TA_OUTER (currently 49 ms). Sequence:
 *
 *              1. Sample TOF; skip if no new sample is ready.
 *              2. Switch on TOF validity:
 *                 - Valid: filter through MeanVal LPF, compute
 *                   distance-derived velocity.
 *                 - Invalid (out of range or below a_distBlind):
 *                   freeze last filtered distance, zero the
 *                   distance-derived velocity, clear PID_velo to
 *                   prevent integrator wind-up while blind.
 *              3. Outer loop:  error_dist = dist - distSetpoint,
 *                              tarVelo  = clamp(PID_dist.run, +/- a_vMax).
 *              4. Velocity measurement: combine wheel-encoder velocity
 *                 (mean of veloL/veloR) with TOF-derived velocity by
 *                 picking the larger magnitude.
 *              5. Middle loop: error_velo = tarVelo - veloMeas,
 *                              tarAccel = PID_velo.run.
 *              6. pitchOffset = tarAccel * accel2pitchK
 *                 (given to the inner loop next call to updatePitch).
 *
 * @param[in]   b   Pointer to the Balancer_t object.
 *
 * @returns     ---
 */
static void BalancerUpdateDist(Balancer_t *b)
{
    /* ----- 1. TOF read ----- */
    if (!TOF_read_distance_task(b->pTOF)) return;

    uint16_t rawDist = b->pTOF->distanceFromTOF;

    /* TOF switch */
    bool tofValid = (rawDist != TOF_VL53L0X_OUT_OF_RANGE) &&
                    ((float)rawDist > b->ParamValue[a_distBlind]);

    float dist;
    float distanceVelo;

    if (tofValid) {
        dist = MeanVALrun(&b->LPF_dist, (float)rawDist);

        /* First valid reading: seed prevDist + motor positions, skip output */
        if (b->prevDist == 0.0f) {
            b->prevDist = dist;
            b->prevPosL = (int16_t)StepperGetPos(b->pStepL);
            b->prevPosR = (int16_t)StepperGetPos(b->pStepR);
            return;
        }

        distanceVelo = (b->prevDist - dist) / TA_OUTER;
        b->prevDist  = dist;
    } else {
        /* Freeze: hold last filtered dist, zero velocity, bleed integrals */
        dist         = b->prevDist;
        distanceVelo = 0.0f;
        PIDclear(&b->PID_velo);
    }

    /* ----- 2. Outer loop: distance error to target velocity ----- */
    float error_dist = dist - b->distSetpoint;
    b->tarVelo = PID.run(&b->PID_dist, error_dist);

    /* Velocity clamp */
    float vMax = b->ParamValue[a_vMax];
    if (b->tarVelo >  vMax) b->tarVelo =  vMax;
    if (b->tarVelo < -vMax) b->tarVelo = -vMax;




    /* ----- 3. Velocity measurement ----- */

    int16_t curPosL = (int16_t)StepperGetPos(b->pStepL);
    int16_t curPosR = (int16_t)StepperGetPos(b->pStepR);

    b->veloL = (float)(curPosL - b->prevPosL) * STEP_TO_MM_S;
    b->veloR = (float)(curPosR - b->prevPosR) * STEP_TO_MM_S;

    b->prevPosL = curPosL;
    b->prevPosR = curPosR;

    float translationVeloMean = 0.5f * (b->veloL + b->veloR);

    /* switch: pick the source with larger magnitude */
    b->veloMeas = (fabsf(translationVeloMean) >= fabsf(distanceVelo))
                  ? translationVeloMean
                  : distanceVelo;

    /* ----- 4. Middle loop: velocity error to target acceleration ----- */
    float error_velo = b->tarVelo - b->veloMeas;
    float tarAccel   = PID.run(&b->PID_velo, error_velo);

    b->pitchOffset = tarAccel * b->accel2pitchK;
}

/**
 * @fn		    BalancerUpdateDisplay
 *
 * @brief       Refresh the data area of the TFT.
 *
 * @details     Two responsibilities:
 *              1. If TOF was detected (devMask DevTOF1) and a fresh
 *                 sample is ready, call @ref visualisationTOF().
 *              2. In passive / diagnostic modes (and in M_Bala/M_DistCtrl when not
 *                 actively moving) call @ref dispMPUBat(). Skipping it during active balancing
 *                 prevents the SPI display refresh (and the embedded I2C
 *                 mpuGetTemp call).
 *
 * @param[in]   b   Pointer to the Balancer_t object.
 *
 * @returns     ---
 */
static void BalancerUpdateDisplay(Balancer_t *b) {
	/* TOF distance visualisation (reference lines 748–755) */
	if ((b->devMask & DevTOF1) != 0) {
		if (TOF_read_distance_task(b->pTOF)) {
			visualisationTOF(b->pTOF);
		}
	}

	/* Battery + temperature (reference lines 834–841)
	 * Show in diagnostic modes and in M_Bala when not actively moving */
	if ((b->TaskMode == M_DispMpuData)||
		(b->TaskMode == M_DispTofData)||
		(b->TaskMode == M_StepFollowPitch)||
	    ((b->TaskMode == M_Bala)&&(b->activeMove != true))
		)
	{
		dispMPUBat(b->pIMU, b->pBatADC);
	}
}

/* ----------------------------------------------------------------------------
 *   Parameter editor
 * --------------------------------------------------------------------------*/

/**
 * @fn		    applyParams
 *
 * @brief       Push the current ParamValue[] to the underlying HW objects.
 *
 * @details     Whenever the user edits a parameter via the rotary encoder
 *              (in BalancerParamEdit) this helper:
 *              - copies the IMU-related parameters into the MPU6050_t,
 *              - re-inits PID_phi, PID_dist and PID_velo,
 *              - updates the distance setpoint and clears the LPF,
 *              - clamps the HW DLPF index to its valid range and pushes
 *                it to the MPU,
 *
 *              Effect: parameter changes take effect on the next
 *              control-loop tick, no full re-init required.
 *
 * @param[in]   b   Pointer to the Balancer_t object.
 *
 * @returns     ---
 */
static void applyParams(Balancer_t *b) {
	MPUlpbw tableLPFValue[7] = {
		LPBW_260, LPBW_184, LPBW_94, LPBW_44, LPBW_21, LPBW_10, LPBW_5
	};

	b->pIMU->pitchZero = b->ParamValue[a_phiZ];
	b->pIMU->swLowPassFilt = b->ParamValue[a_LP];
	b->pIMU->pitchFilt = b->ParamValue[a_GyAc];

	PID.init(&b->PID_phi,
	         b->ParamValue[a_piKP], b->ParamValue[a_piKI],
	         b->ParamValue[a_piKD], TA_INNER);

	PID.init(&b->PID_dist,
	         b->ParamValue[a_dKP], b->ParamValue[a_dKI],
	         b->ParamValue[a_dKD], TA_OUTER);
	b->distSetpoint = b->ParamValue[a_dSP];
	MeanVALclr(&b->LPF_dist, b->ParamValue[a_dLPF]);

	PID.init(&b->PID_velo,
	         b->ParamValue[a_vKP], b->ParamValue[a_vKI],
	         b->ParamValue[a_vKD], TA_OUTER);
	b->accel2pitchK = b->ParamValue[a_a2p];

	/* Clamp hardware low-pass index to valid range 0–6 */
	if (b->ParamValue[a_HwLP] < 0) b->ParamValue[a_HwLP] = 0;
	if (b->ParamValue[a_HwLP] > 6) b->ParamValue[a_HwLP] = 6;
	b->pIMU->LowPassFilt = tableLPFValue[(uint8_t)b->ParamValue[a_HwLP]];
	mpuSetLpFilt(b->pIMU);

	/* Clamp route number */
	if (b->ParamValue[a_Cour] < 0) b->ParamValue[a_Cour] = 0;
	/* TODO: clamp to routeNumMax-1 when route.h is integrated */
	b->routeNum = (uint8_t)b->ParamValue[a_Cour];
}

/**
 * @fn		    BalancerParamEdit
 *
 * @brief       Poll the rotary push button + encoder; edit ParamValue[].
 *
 * @details     Called from the main loop on every iteration. Behaviour:
 *              - On push-button press: cycle "modif" (currently selected
 *                parameter index) one step forward and wrap around.
 *                If a_MODE was set to a non-zero value by an earlier
 *                edit, apply it as the new TaskMode and reset the
 *                outer/middle loops.
 *                Display the new parameter name + current value.
 *              - On encoder rotation: update ParamValue[modif] using the
 *                per-parameter ParamScale, repaint the value in yellow,
 *                and call applyParams() so the change takes effect on
 *                the next control-loop tick.
 *
 * @param[in]   b   Pointer to the Balancer_t object.
 *
 * @returns     ---
 */
static void BalancerParamEdit(Balancer_t *b) {
	int ButtPos;
	static int oldButtPos = 0;
	static int modif = 0;          // index of currently selected parameter
	char strT[32];

	ButtPos = getRotaryPosition();

	// Button press: cycle to next parameter
	if (getRotaryPushButton()) {
		if (++modif >= PARAM_COUNT) {
			modif = 0;
		}

		// Mode switch: if a_MODE was set to a non-zero value, apply it as the new TaskMode
		if ((modif > 0) && (b->ParamValue[a_MODE] > 0)) {
		    b->TaskMode = (TaskModus)b->ParamValue[a_MODE];
		    b->RunInit  = true;
		    b->pitchOffset = 0.0f;
		    b->tarVelo     = 0.0f;
		    PIDclear(&b->PID_dist);
		    PIDclear(&b->PID_velo);
		    b->ParamValue[a_MODE] = 0;
		}

		// Display current parameter name and value
		sprintf(strT, "%s :", ParamTitle[modif]);
		tftPrintColor((char *)strT, 10, 20, tft_GREEN);
		sprintf(strT, "%+5.3f  ", b->ParamValue[modif]);
		tftPrintColor((char *)strT, 60, 20, tft_GREEN);

		// Sync encoder position to current param value
		ButtPos = (int)(ParamScale[modif] * b->ParamValue[modif]);
		oldButtPos = ButtPos;
		setRotaryPosition(ButtPos);
	}

	// --- Rotation: adjust current parameter ---
	if (ButtPos != oldButtPos) {
		b->ParamValue[modif] = ((float)ButtPos / ParamScale[modif]);
		sprintf(strT, "%+5.3f  ", b->ParamValue[modif]);
		tftPrintColor((char *)strT, 60, 20, tft_YELLOW);
		oldButtPos = ButtPos;

		// Push changed values to HW immediately
		applyParams(b);
	}
}


/* ============================================================================
 *  PUBLIC PROTOTYPE INSTANCE — wires the static methods into the method table
 * ==========================================================================*/

/**
 * @brief   Const "prototype" Balancer with method pointers.
 * @details NOT static -> visible outside this .c file. BalancerCreate()
 *          dereferences this object and copies it into the user-supplied
 *          Balancer_t to wire up methods (prototype-copy pattern,
 *          identical to the one used by PIDContr_t in regler.c).
 */
const Balancer_t Balancer = {
    .init          		= BalancerInit,
    .updatePitch   		= BalancerUpdatePitch,
    .updateDist    		= BalancerUpdateDist,
	.updateDisplay 		= BalancerUpdateDisplay,
	.DispAlphaNumMPU 	= BalancerDispAlphaNumMPU,
	.paramEdit     		= BalancerParamEdit,

};

/* ============================================================================
 *  PUBLIC CONSTRUCTOR
 * ==========================================================================*/

/**
 * @fn		    BalancerCreate
 *
 * @brief       Construct a Balancer_t from a set of HW pointers.
 *
 * @details     Performs the OOP-in-C construction sequence:
 *              copies the const Balancer prototype into *b,
 *              stores the hardware-object pointers, then calls
 *              Balancer.init(b) to configure ParamValue[] and reset
 *              every operating field.
 *
 * @param[in]  	b      Pointer to a Balancer_t.
 * @param[in]   imu    Pointer to MPU6050 object.
 * @param[in]   tof    Pointer to TOF sensor object.
 * @param[in]   stepL  Pointer to LEFT stepper object.
 * @param[in]   stepR  Pointer to RIGHT stepper object.
 * @param[in]   bat    Pointer to battery ADC channel object.
 *
 * @returns     ---
 */
void BalancerCreate(Balancer_t *b,
                     MPU6050_t *imu, TOFSensor_t *tof,
                     Stepper_t *stepL, Stepper_t *stepR,
                     analogCh_t *bat) {
    *b = Balancer;          // Copy vtable
    b->pIMU   = imu;		// Connect HW pointers
    b->pTOF   = tof;
    b->pStepL = stepL;
    b->pStepR = stepR;
    b->pBatADC= bat;
    Balancer.init(b);		// Call BalancerInit() -> fill params and states
}
