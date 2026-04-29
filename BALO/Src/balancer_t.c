/**
 *      balancer_t.c
 *
 *      @file balancer_t.c provides architecture of the balancer superclass
 *      @author: Luis Brunn
 *      Created on: Mar. 18, 2026
 */

#include <stdio.h>
#include "balancer_t.h"
#include <math.h>
#include <mcalSysTick.h>
#include <mcalGPIO.h>
#include <ST7735.h>
#include <RotaryPushButton.h>
#include <adcBAT.h>
#include "i2cDevices.h"   /* CheckAndInitI2cSlaves */

// *Timing
#define StepTaskTimeSet 7UL			// the communication to stepper takes <1ms therefore total StepTaskTime = 7ms
#define DistCtrlTaskTimeSet 49UL	// TODO: Adjust time for outer control loop (now 7x StepTaskTimeSet)
#define DispTaskTimeSet 700UL		// Task for Position control and Display Status

#define TA_INNER ((float)StepTaskTimeSet    * 1e-3f)   // 0.007 s
#define TA_OUTER ((float)DistCtrlTaskTimeSet * 1e-3f)  // 0.049 s

uint32_t DistCtrlTaskTimeMs = DistCtrlTaskTimeSet;

uint32_t StepTaskTime[] = {
  50UL, 50UL, 100UL, 50UL,
  StepTaskTimeSet,
  StepTaskTimeSet,
  StepTaskTimeSet,
  StepTaskTimeSet
};

//* Stepper motors position conversion
const int16_t rad2step = 520;		// Ratio step-counts (200 Full-Steps div 1/16 Steps) per rotation at rad:  509.4 =  200* 16 / (2 PI) or 1600/PI

//* Fall detection
#define PITCH_FALL_RAD 0.35f  		// |pitch| > this → robot fallen
#define PITCH_ACTIVE 0.05f  		// |pitch| < this → balanced


// ******************** Set default params ********************
static const float defaultParam[PARAM_COUNT] = {
		/* Same order as ParamIdx enum n balancer_t.h */
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
		100.0f,		// dist_blind [mm] -> dist used when TOF out of range
		0.00f,     // a_vKP
		0.00f,    // a_vKI
		0.00f,     // a_vKD
		2000.0f,      // a_vMax [mm/s]
		0.000102f,       // a_a2p accel2pitch gain
		0.05f,      // a_pClamp variable pitch clamp [rad], replaces fixed PITCH_CLAMP
};

static const char ParamTitle[PARAM_COUNT][5] = {
	"MODE", "Cour", "poTo", "phiZ", "GyAc", "HwLP", "LP  ",
	"piKP", "piKI", "piKD", "raRo", "maRo", "raTr",
	"dKP ", "dKI ", "dKD ", "dSP ", "dLPF", "dBli",
	"vKP ", "vKI ", "vKD ", "vMax", "a2p ", "pClp"
};

static const float ParamScale[PARAM_COUNT] = {
    1,   1,   0.2,  500,  100,  1,    500,
    100, 500,  100,  500,  1,    500,
    /* Distance loop with new units */
    20,    // a_dKP    inc 0.05    range 0.1–3.0
    1000,  // a_dKI    inc 0.001   range 0–0.05
    100,   // a_dKD    inc 0.01    range 0–0.5
    0.1,   // a_dSP    inc 10mm    range 100–800
    100,   // a_dLPF   inc 0.01    range 0.05–0.5
    0.1,   // a_distBlind inc 10mm range 50–200
    /* Velocity loop (still unused in 2-loop test) */
    10000, 100000, 10000, 0.5,
    100000, 1000
};

// ******************** Method implementations ********************
/**
 * All methods static -> invisible outside of this .c file
 * Vtable is public interface
 */

/**
 * void StepperIHold(Balancer_t *b, bool OnSwitch)
 * @param  OnSwitch == true;  IHold active;
 *					== false; IHold reduced to minimum
 * @returns ---
 * function that toggles the stepper holding current
 * file private (static) and takes Balancer_t* so accessing b->pStepL and R is possible
 */
static void StepperIHold(Balancer_t *b, bool OnSwitch)
{
	static bool OldStatus = false;
	const uint8_t iOff = 0x0;  	//switch off the PWM Regulator Value 0xFF only for AMIS
								// 0xFF only for AMIS IC
								// 0x0 reduced current to 59mA

	if (OnSwitch == OldStatus) return;   	// commands only active of OnSwitch Status changed

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
	// Routine für init balancer stepper (WHILE) ab Z.283 checkandinit()
	// oder als methoden der stepper (generell HW obj) implementieren
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

static void BalancerDispAlphaNumMPU(Balancer_t *b)
{
    char str[16];
    mpuGetAccel(b->pIMU);
    sprintf(str, "%+6.3f", b->pIMU->accel[0]); tftPrint(str, 20, 50, 0);
    sprintf(str, "%+6.3f", b->pIMU->accel[1]); tftPrint(str, 20, 60, 0);
    sprintf(str, "%+6.3f", b->pIMU->accel[2]); tftPrint(str, 20, 70, 0);
}

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

/* ----- core inner balance loop -----
 * translation of case M_Bala from reference
 * RunInit, IMU read + fall detection, check if balance action needed, active PID balancing with stepper commands
 */
static void BalancerUpdatePitch(Balancer_t *b) {
	/* RunInit: when TaskMode M_Bala is entered */
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
		b->pIMU->RPY[0] = 2;			// MPU y Axis goes to the front
		b->pIMU->RPY[1] = 3;			// MPU z-Axis goes to the left side
		b->pIMU->RPY[2] = -1;			// MPU x-Axis goes down
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
    #define WHEEL_CIRC_MM   375.0f
    #define STEPS_PER_REV   3200.0f
    #define STEP_TO_MM_S    (WHEEL_CIRC_MM / STEPS_PER_REV / TA_OUTER)

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

/* ---------- Param editing ---------- */
// TODO: Re write after first balancing test with new OOP architecture
/* Apply current ParamValue[] to IMU config and re-init PID_phi. */

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

/* Parameter editor — polls rotary encoder and push button. */
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

		// Mode switch: if a_MODE was set to a non-zero value,
		// apply it as the new TaskMode (same mechanism as original)
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


/* ---------- Vtable instance ---------- */
// Connecting pointers to functions - NO static -> visible outside .c File
const Balancer_t Balancer = {
    .init          = BalancerInit,
    .updatePitch   = BalancerUpdatePitch,
    .updateDist    = BalancerUpdateDist,
	.updateDisplay = BalancerUpdateDisplay,
	.DispAlphaNumMPU = BalancerDispAlphaNumMPU,
	.paramEdit     = BalancerParamEdit,

};

/* ---------- Constructor ---------- */
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
