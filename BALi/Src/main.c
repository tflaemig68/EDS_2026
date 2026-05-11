/**
 ******************************************************************************
 * @file        main.c
 * @author      Luis Brunn <https://github.com/LuBru70>
 * @author 		based on reference main from Prof. Flämig
 * @brief       Top-level application of the balancer firmware.
 * 				Entry point and cooperative scheduler for the balancer_t OOP
 * 				architecture.
 *              Owns the hardware object instances, performs peripheral init,
 *              runs the I2C slave detection, and drives the mode scheduled
 *              StepTask and the periodic DispTask on their respective SysTick
 *              ticks.
 * @date        May 2026
 ******************************************************************************

 ******************************************************************************
 * @attention This software is licensed based on CC BY-NC-SA 4.0
 ******************************************************************************
 */


/* ----------- Standard library includes ----------- */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>

/* ----------- Project module: central Balancer aggregate ----------- */
#include "balancer_t.h"

/* ----------- MCAL / drivers ----------- */
#include <mcalSysTick.h>
#include <mcalGPIO.h>
#include <mcalI2C.h>
#include <ST7735.h>
#include <RotaryPushButton.h>
#include <adcBAT.h>
#include <i2cMPU.h>
#include <i2cAMIS.h>
#include <i2cTOF.h>

#include "i2cDevices.h"

/* ----------- Debug toggle: 1 = TFT debug prints, 0 = production ----------- */
#define BALA_DEBUG  0		// Switch for frequent TFT prints


bool timerTrigger = false;
uint32_t ST7735_Timer = 0UL;

/* ============================================================================
 *  Static HW object instances (owned by main.c)
 *  pass their addresses to BalancerCreate().
 * ==========================================================================*/
static MPU6050_t   MPU1;    //!< IMU instance (MPU6050)
static TOFSensor_t TOF1;    //!< TOF instance (VL53L0X)
static Stepper_t   StepL;   //!< Left  stepper (AMIS-30543)
static Stepper_t   StepR;   //!< Right stepper (AMIS-30543)
static analogCh_t  adChn;   //!< Battery-voltage ADC channel
static Balancer_t  bala;    //!< Central Balancer aggregate

/* ============================================================================
 *  Constants for CheckAndInitI2cSlaves()
 * ==========================================================================*/

/**
 * @name    Expected I2C slave addresses
 * @{
 */
#define i2cAddr_motL  0x61	//!< Left  AMIS stepper I2C address
#define i2cAddr_motR  0x60	//!< Right AMIS stepper I2C address
#define i2cAddr_TOF1  0x29	//!< VL53L0X default I2C address
/** @} */

#define StepPaCount   5
static const uint8_t StepPaValue[StepPaCount] = {15, 7, 2, 5, 4};
static const uint8_t stepMode   = 3;	//!< 1/16 microstepping
static const bool    stepRotDir = true;	//!< right motor: forward (left motor uses !stepRotDir)
static uint16_t TOF_DISTANCE_1  = 10;	//!< initial measuredRange [mm]

/* ============================================================================
 *  Initialisation of I2C devices
 * ==========================================================================*/
static int  CheckAndInitI2cSlaves(uint8_t *DevMask,
                                  Stepper_t *pStepL, Stepper_t *pStepR,
                                  MPU6050_t *pMPU1, TOFSensor_t *pTOF1);

/**
 * @fn		    CheckAndInitI2cSlaves
 *
 * @brief       Staged I2C bus discovery + slave initialisation.
 *
 * @details     Replica of the old main.c init sequence
 *              (lines 284..414). Called once per StepTask tick (50 ms)
 *              while bala.TaskMode == M_CheckI2cSlaves. Drives an internal
 *              static state variable "CycleRun" that walks through the
 *              following stages — see also the call sequence below:
 *
 *				Call 1  CycleRun==-4 : StepL find+init, CycleRun++ -3, return
 *				Call 2  CycleRun==-3 : StepR init (falls thru) + TOF detect
 *										+ MPU detect + mpuInit#1 CycleRun=-2
 *				Call 3  CycleRun==-2 : TOF_init_device + mpuInit#2 CycleRun=-1
 *				Call 4  CycleRun==-1 : TOF config+start + mpuInit#3 CycleRun=0
 *
 * @note		this routine intentionally remains in main.c because it reproduces
 *    	  	  	the staged hardware discovery sequence of the reference
 *    	  	  	implementation. A future refactoring could move device-specific
 *    	  	  	discovery steps into the corresponding hardware abstraction modules.
 *
 *
 * @param[in,out] DevMask  Pointer to the Balancer's devMask byte; bits are
 *                         set as devices are discovered.
 * @param[in,out] pStepL   Pointer to LEFT stepper object.
 * @param[in,out] pStepR   Pointer to RIGHT stepper object.
 * @param[in,out] pMPU1    Pointer to MPU6050 object.
 * @param[in,out] pTOF1    Pointer to TOF sensor object.
 *
 * @returns     int — current value of the internal CycleRun counter.
 *              The main loop watches for "0", which means "all required
 *              init steps complete". The DevMask tells the main loop
 *              which devices were actually found.
 */
static int CheckAndInitI2cSlaves(uint8_t *DevMask,
                                 Stepper_t *pStepL, Stepper_t *pStepR,
                                 MPU6050_t *pMPU1, TOFSensor_t *pTOF1)
{
    I2C_TypeDef *i2c = I2C1, *i2c2 = I2C2;
    static I2C_TypeDef *i2cMPU  = I2C1;
    static I2C_TypeDef *i2cSTEP = I2C1;
    static I2C_TypeDef *i2cTOF  = I2C1;
    uint8_t foundAddr;
    static uint8_t i2c_Addr = 1;
    static int CycleRun = -4;		// stage counter (preserved across calls)
    int MPU6050ret;

#if BALA_DEBUG
    static int mpuInitCallCount = 0;
#endif

    /* CycleRun == -5: dummy bus warm-up - RESERVED, not entered with current static init = -4 */
    if (CycleRun == -5)
    {
        foundAddr = i2cFindSlaveAddr(i2c, i2c_Addr);
        foundAddr = i2cFindSlaveAddr(i2c2, i2c_Addr);
        CycleRun++;
        return (CycleRun);
    }

    /* CycleRun == -4: LEFT stepper — detect, init, return */
    if (((*DevMask & DevStepL) == 0) && (CycleRun == -4))
    {
        i2c_Addr = i2cAddr_motL;
        foundAddr = i2cFindSlaveAddr(i2cSTEP, i2c_Addr);
        if (foundAddr == 0)
        {
            i2cSTEP = I2C2;
            foundAddr = i2cFindSlaveAddr(i2cSTEP, i2c_Addr);
        }
        if (foundAddr == i2c_Addr)
        {
            tftPrint((char *)"<-Left  -OK-  \0", 0, 110, 0);
            StepperInit(pStepL, i2cSTEP, i2c_Addr,
                        StepPaValue[0], StepPaValue[1],
                        StepPaValue[2], StepPaValue[3],
                        stepMode, (uint8_t)!stepRotDir,
                        StepPaValue[4], 0);
            stepper.pwmFrequency.set(pStepL, 0);
            *DevMask |= DevStepL;
        }
        else
        {
            pStepL->i2cAddress.value = 0;
        }
        CycleRun++;
        return (CycleRun);
    }

    /* CycleRun == -3: RIGHT stepper */
    if ((*DevMask & DevStepR) == 0)
    {
        foundAddr = i2cFindSlaveAddr(i2cSTEP, i2cAddr_motR);
        if (foundAddr != 0)
        {
            tftPrint((char *)"Right->\0", 104, 110, 0);
            StepperInit(pStepR, i2cSTEP, i2cAddr_motR,
                        StepPaValue[0], StepPaValue[1],
                        StepPaValue[2], StepPaValue[3],
                        stepMode, (uint8_t)stepRotDir,
                        StepPaValue[4], 0);
            stepper.pwmFrequency.set(pStepR, 0);
            *DevMask |= DevStepR;
            i2cSetClkSpd(i2cSTEP, I2C_CLOCK_200);
        }
        else
        {
            pStepR->i2cAddress.value = 0;
        }
    }
    /* no return — falls through to TOF + MPU */

    /* TOF: set I2C address (CycleRun == -3) */
    if (((*DevMask & DevTOF1) == 0) && (CycleRun == -3))
    {
        pTOF1->i2c_tof          = i2cTOF;
        pTOF1->TOF_address_used = i2cAddr_TOF1;
    }

    /* TOF: detect sensor (CycleRun <= -3) */
    if (((*DevMask & DevTOF1) == 0) && (CycleRun <= -3))
    {
        if (TOF_init_address(pTOF1))
        {
            initTOFSensorData(pTOF1, i2cTOF, TOF_ADDR_VL53LOX,
                              TOF_DEFAULT_MODE_D, TOF_DISTANCE_1);
            tftPrint((char *)"TOF found \0", 0, 80, 0);
            *DevMask |= DevTOF1;
        }
        else
        {
            tftPrint((char *)"TOF not found \0", 0, 80, 0);
        }
    }

    /* TOF: device init (CycleRun == -2) */
    if (((*DevMask & DevTOF1) != 0) && (CycleRun == -2))
    {
        bool initResult = TOF_init_device(pTOF1);
        if (initResult)
        {
            tftPrint((char *)"TOF init OK\0", 0, 80, 0);
        }
    }

    /* TOF: configure + start continuous (CycleRun == -1) */
    if (((*DevMask & DevTOF1) != 0) && (CycleRun == -1))
    {
        configTOFSensor(pTOF1, TOF_DEFAULT_MODE_D, true);
        TOF_start_continuous(pTOF1);
    }

    /* MPU: detect + first init step (CycleRun == -3) */
    if (CycleRun == -3)
    {
        foundAddr = i2cFindSlaveAddr(i2cMPU, i2cAddr_MPU6050);
        if (foundAddr == 0)
        {
            i2cMPU = I2C2;
            foundAddr = i2cFindSlaveAddr(i2cMPU, i2cAddr_MPU6050);
        }
        if (foundAddr != 0)
        {
            tftPrint((char *)"MPU6050 OK \0", 0, 95, 0);
            *DevMask |= DevMPU1;
            MPU6050ret = mpuInit(pMPU1, i2cMPU, i2cAddr_MPU6050,
                                 FSCALE_250, ACCEL_2g, LPBW_184, NO_RESTART);
            CycleRun = MPU6050ret;
#if BALA_DEBUG
            mpuInitCallCount++;
            { char d[24]; sprintf(d, "mI#%d r=%d ", mpuInitCallCount, MPU6050ret);
              tftPrintColor(d, 0, 70, tft_MAGENTA); }
#endif
        }
        else
        {
            pMPU1->i2c_address = 0;
        }
        return (CycleRun);
    }

    /* MPU: continue init (mpuInit steps -2 -> -1 -> 0) */
    if (((*DevMask & DevMPU1) > 0) && (CycleRun < 0))
    {
        MPU6050ret = mpuInit(pMPU1, i2cMPU, i2cAddr_MPU6050,
                             FSCALE_250, ACCEL_2g, LPBW_184, NO_RESTART);
        CycleRun = MPU6050ret;
#if BALA_DEBUG
        mpuInitCallCount++;
        { char d[24]; sprintf(d, "mI#%d r=%d ", mpuInitCallCount, MPU6050ret);
          tftPrintColor(d, 0, 70, tft_MAGENTA); }
#endif
        return (CycleRun);
    }

    return (CycleRun);
}


/* ============================================================================
 *  Begin of the main routine
 * ==========================================================================*/

/**
 * @fn		    main
 *
 * @brief       Balancer firmware entry point.
 *
 * @details     Performs peripheral initialisation in the same order as
 *              the reference (LED, both I2C buses, ADC, SysTick,
 *              SPI to TFT, TFT init+rotation+font, rotary encoder),
 *              constructs the central Balancer aggregate object, sets
 *              up the SysTick countdown timers and enters the
 *              loop.
 *
 *              Loop:
 *              - Every 1 ms: timerTrigger -> update countdown timers.
 *              - Every iteration:  bala.paramEdit (rotary encoder).
 *              - StepTaskTimer expired: switch on bala.TaskMode and
 *                run the corresponding state.
 *              - DispTaskTimer expired (700 ms): refresh the TFT
 *                diagnostic area (see the @ref DispTask documentation on when and why
 *                this should be limited when active distance control).
 *
 *              The state machine inside the StepTask switch:
 *                M_InitBat        -> read battery
 *                M_CheckI2cSlaves -> drive CheckAndInitI2cSlaves;
 *                                   when ready, pick the best mode:
 *                                     - all of MPU+StepL+StepR -> M_DistCtrl,
 *                                     - else only TOF          -> M_DispTofData,
 *                                     - else only MPU          -> M_DispMpuData.
 *                M_DispMpuData    -> show numeric IMU data.
 *                M_DispTofData    -> show distance.
 *                M_Bala           -> inner balance loop control.
 *                M_DistCtrl       -> inner balance + outer distance control.
 *                default          -> reset to M_InitBat.
 *
 * @return      int Program return value. This function normally never
 *              returns because the firmware runs inside an infinite loop.
 */
int main(void)
{
    int  I2cCheckResult = -1;
    BatStat_t BatStatus;

    adChn.adc = ADC1;

    /* Peripheral init (same order as reference lines 480-511) */
    initLED(&LEDbala);
    activateI2C1();
    activateI2C2();
    setLED(RED_on);
    activateADC(PIN1);

    systickInit(SYSTICK_1MS);
    IOspiInit(&ST7735bala);

    tftInitR(INITR_REDTAB);
    tftSetRotation(LANDSCAPE);
    tftSetFont((uint8_t *)&SmallFont[0]);
    tftFillScreen(tft_BLACK);

    initRotaryPushButton(&PuBio_bala);

    /* Build Balancer object */
    BalancerCreate(&bala, &MPU1, &TOF1, &StepL, &StepR, &adChn);

    /* Timer list */
    uint32_t *timerList[] = {
        &bala.StepTaskTimer,
        &ST7735_Timer,
        &bala.DispTaskTimer,
        &bala.DistCtrlTaskTimer
    };
    size_t arraySize = sizeof(timerList) / sizeof(timerList[0]);

    systickSetMillis(&bala.StepTaskTimer,     StepTaskTime[M_InitBat]);
    systickSetMillis(&bala.DispTaskTimer,      700UL);
    systickSetMillis(&bala.DistCtrlTaskTimer, DistCtrlTaskTimeMs);

    setLED(RED_off);
    tftPrintColor((char *)"I2C Scanner running", 0, 0, tft_MAGENTA);

    /* ============================== MAIN LOOP ============================== */
    while (1)
    {
        if (timerTrigger)
        {
            systickUpdateTimerList((uint32_t *)timerList, arraySize);
        }

        bala.paramEdit(&bala);

        /* ---------------------- StepTask ---------------------- */
        if (isSystickExpired(bala.StepTaskTimer))
        {
            systickSetTicktime(&bala.StepTaskTimer,
                               StepTaskTime[bala.TaskMode]);

            switch (bala.TaskMode)
            {
                case M_InitBat:
                {
                    BatStatus = getBatVolt(&adChn);
                    if (BatStatus == okBat) { setLED(GREEN); }
                    else                    { setLED(RED);    }
                    bala.TaskMode = M_CheckI2cSlaves;
                }
                break;

                case M_CheckI2cSlaves:
                {
                    setLED(MAGENTA);

                    I2cCheckResult = CheckAndInitI2cSlaves(
                        &bala.devMask,
                        bala.pStepL, bala.pStepR,
                        bala.pIMU, bala.pTOF);

                    /* All three required devices present: balance */
                    if ((I2cCheckResult == 0) &&
                        (bala.devMask & DevMPU1) &&
                        (bala.devMask & DevStepL) &&
                        (bala.devMask & DevStepR))
                    {
                        bala.stepLenable = true;
                        bala.stepRenable = true;
                        bala.TaskMode    = M_DistCtrl;			// Change default task mode here
                        bala.RunInit     = true;
                        setLED(GREEN);
#if BALA_DEBUG
                        tftPrintColor((char *)"-> M_DistCtrl     ", 0, 0, tft_GREEN);
#endif
                        break;
                    }

                    /* Fallback: only TOF */
                    if ((I2cCheckResult == 0) && (bala.devMask & DevTOF1))
                    {
                        bala.TaskMode = M_DispTofData;
                        setLED(BLUE);
                        break;
                    }

                    /* Fallback: only MPU */
                    if ((I2cCheckResult == 0) && (bala.devMask & DevMPU1))
                    {
                        bala.TaskMode = M_DispMpuData;
                        setLED(GREEN);
                        break;
                    }
                }
                break;

                case M_DispMpuData:
                {
                    bala.DispAlphaNumMPU(&bala);
                }
                break;

                case M_DispTofData:
                {
                    setLED(WHITE);
                    bala.updateDisplay(&bala);
                    setLED(BLUE);
                }
                break;

                case M_Bala:
                {
                    bala.updatePitch(&bala);

#if BALA_DEBUG
                    /* Show MPU config + live pitch every ~700ms */
                    /* ATTENTION: makes balancing slightly unstable (long write times): Fall possible */
                    {
                        static int dbgCnt = 0;
                        if (++dbgCnt >= 100)
                        {
                            char d[32];

                            /* RPY must be 2,3,-1 for correct mounting */
                            sprintf(d, "RPY:%d,%d,%d   ",
                                    bala.pIMU->RPY[0],
                                    bala.pIMU->RPY[1],
                                    bala.pIMU->RPY[2]);
                            tftPrintColor(d, 0, 20, tft_CYAN);

                            /* pitch must react to tilt, pitchFilt must be 0.98 */
                            sprintf(d, "P:%+6.3f pF:%.2f",
                                    bala.pIMU->pitch,
                                    bala.pIMU->pitchFilt);
                            tftPrintColor(d, 0, 30, tft_CYAN);

                            /* timebase must be 0.0008, swLP must be 0.36 */
                            sprintf(d, "tb:%.4f LP:%.2f",
                                    bala.pIMU->timebase,
                                    bala.pIMU->swLowPassFilt);
                            tftPrintColor(d, 0, 40, tft_CYAN);

                            dbgCnt = 0;
                        }
                    }
#endif
                }
                break;

                case M_DistCtrl:
                {
                	/* Inner loop runs at every StepTask tick; outer loop
                	 * runs only when its own timer expires. This is the
                	 * cascade implementation for distance control. */
                    bala.updatePitch(&bala);

                    if (isSystickExpired(bala.DistCtrlTaskTimer))
                    {
                    	systickSetTicktime(&bala.DistCtrlTaskTimer, DistCtrlTaskTimeMs);
                        bala.updateDist(&bala);
                    }
                }
                break;

                case M_StepFollowPitch:
                case M_3DGinit:
                default:
                {
                    bala.TaskMode = M_InitBat;
                    bala.RunInit  = true;
                }
                break;
            }
        } /* end StepTask */

        /* ---------------------- DispTask (700 ms) ---------------------- */
        if (isSystickExpired(bala.DispTaskTimer))
        {
            systickSetTicktime(&bala.DispTaskTimer, 700UL);
            /* In M_DistCtrl, only update the display when balancer robot is nearly upright.
             * When updating the display during active tilting (to generate translation acceleration/velocity),
             * the SPI write delays the pitch control and therefore lead to instability/falling.
             */
            if (bala.TaskMode == M_DistCtrl) {
            	if (fabs(bala.pIMU->pitch) < 0.0175f) { 	// abs(pitch) < 1°: safe to write display
            		bala.updateDisplay(&bala);
            	}
            } else {
            	bala.updateDisplay(&bala);
            }
        }

    }
}
