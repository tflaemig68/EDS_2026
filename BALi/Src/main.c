/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Luis Brunn / based on reference main by Prof Flaemig
 * @brief          : Main for new balancer_t OOP architecture
 ******************************************************************************
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>

#include "balancer_t.h"

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
#define BALA_DEBUG  0


bool timerTrigger = false;
uint32_t ST7735_Timer = 0UL;

/* ----------- Hardware object instances ----------- */
static MPU6050_t   MPU1;
static TOFSensor_t TOF1;
static Stepper_t   StepL;
static Stepper_t   StepR;
static analogCh_t  adChn;
static Balancer_t  bala;

/* ----------- Constants for CheckAndInitI2cSlaves ----------- */
#define i2cAddr_motL  0x61
#define i2cAddr_motR  0x60
#define i2cAddr_TOF1  0x29

#define StepPaCount   5
static const uint8_t StepPaValue[StepPaCount] = {15, 7, 2, 5, 4};
static const uint8_t stepMode   = 3;
static const bool    stepRotDir = true;
static uint16_t TOF_DISTANCE_1  = 10;

/* ----------- Private prototype ----------- */
static int  CheckAndInitI2cSlaves(uint8_t *DevMask,
                                  Stepper_t *pStepL, Stepper_t *pStepR,
                                  MPU6050_t *pMPU1, TOFSensor_t *pTOF1);

/* ----------- CheckAndInitI2cSlaves ----------- */
/*                                                                        */
/* Exact replica of reference_old_main.c lines 284–414.                   */
/* Call sequence (one call per 50ms StepTask tick in M_CheckI2cSlaves):   */
/*   Call 1  CycleRun==-4 : StepL find+init, CycleRun++ -3, return        */
/*   Call 2  CycleRun==-3 : StepR init (falls thru) + TOF detect          */
/*                          + MPU detect + mpuInit#1 CycleRun=-2          */
/*   Call 3  CycleRun==-2 : TOF_init_device + mpuInit#2 CycleRun=-1       */
/*   Call 4  CycleRun==-1 : TOF config+start + mpuInit#3 CycleRun=0       */
/*   Main loop sees return==0, all devMask bits set enters M_Bala         */
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
    static int CycleRun = -4;
    int MPU6050ret;

#if BALA_DEBUG
    static int mpuInitCallCount = 0;
#endif

    /* CycleRun == -5: dummy bus warm-up */
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

    /* CycleRun == -3: RIGHT stepper — no guard, no return */
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

    /* MPU: continue init (mpuInit steps -2→-1→0) */
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

/* -------------------------------------- */
/* main                                   */
/* -------------------------------------- */
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

    /* ---------------------- MAIN LOOP ---------------------- */
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
                        bala.TaskMode    = M_Bala;
                        bala.RunInit     = true;
                        setLED(GREEN);
#if BALA_DEBUG
                        tftPrintColor((char *)"-> M_Bala     ", 0, 0, tft_GREEN);
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
                    /* ATTENTION: makes balancing slightly unstable (long write times): Falls possible */
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
            bala.updateDisplay(&bala);
        }

    }
}
