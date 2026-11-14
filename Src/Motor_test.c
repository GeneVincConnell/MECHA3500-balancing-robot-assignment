/**
 * @file    Motor_test.c
 * @brief   Motor and LED test for the balancing robot.
 *
 * PROVENANCE: AI-generated. Written by Claude (Anthropic), Nov 2026. It drives
 * the TMC2240s through Analog Devices' TMC-API (TMC2240.c/.h, downloaded).
 * The callbacks TMC-API needs to reach the hardware are in TMC2240_Callback.c.
 *
 * Put this file in Core/Src. Call Motor_test() once from main.c in
 * USER CODE BEGIN 2, after the MX_xxx_Init() calls. The timers then run in
 * hardware, so nothing needs to go in the while(1) loop.
 *
 * The LED blink (0.5 s period, 50 %) and the STEP pulse width (5 us) come
 * straight from the CubeMX timer settings; this file only changes the STEP
 * rate and, if a driver fails, the LED blink rate.
 *
 * Note: the TMC2240 steps on STEP rising edges, so the duty cycle does not
 * change motor speed. Speed comes from the STEP frequency:
 *     step rate = 1 MHz / (ARR + 1)      (ARR 999 -> 1 kHz)
 *     speed     = step rate / 3200 rev/s (200 steps x 16 microsteps per rev)
 */
#include "Robot.h"

/* ------------------------------------------------------------------------- */
/* Variables declared in Robot.h                                             */
/* ------------------------------------------------------------------------- */
uint8_t  tmc_ok[TMC_COUNT]      = {0, 0};
uint8_t  tmc_version[TMC_COUNT] = {0, 0};

/* ========================================================================= */
/* Motor test                                                                */
/* ========================================================================= */
void Motor_test(void)
{
    /* Stop the LED and STEP timers while the debugger is paused, so the
       wheels stop at a breakpoint. Encoder timers (TIM3/TIM5) keep counting. */
    __HAL_DBGMCU_FREEZE_TIM2();
    __HAL_DBGMCU_FREEZE_TIM4();
    __HAL_DBGMCU_FREEZE_TIM8();

    /* Configure both TMC2240s over SPI while TMC_EN is still high (outputs off).
       Needs the battery on: the TMC2240 does not answer SPI without VS. */
    Motor_Disable();
    uint8_t ok_L = Motor_DriverInit(TMC_LEFT);
    uint8_t ok_R = Motor_DriverInit(TMC_RIGHT);

    if (ok_L && ok_R) {
        /* Step rate = 1 MHz / (ARR + 1). ARR 60 -> 16.4 kHz = 5.1 rev/s,
           about 307 rpm (32 rad/s). The new ARR loads at the end of the
           first period (ARR preload is on in CubeMX). */
        __HAL_TIM_SET_AUTORELOAD(&htim8, 60);   /* STEP_L, PC6 */
        __HAL_TIM_SET_AUTORELOAD(&htim4, 60);   /* STEP_R, PB6 */

        /* Turn the driver power stages on (TMC_EN is active low) */
        Motor_Enable();

        /* Start the STEP outputs. HAL_TIM_PWM_Start also sets MOE for TIM8. */
        HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    } else {
        /* A driver did not answer or did not take its settings: leave the
           motors off and blink the LED fast. Check tmc_ok[] / tmc_version[].
           TIM2 ticks at 10 kHz, so ARR 999 / CCR 500 = 0.1 s period, 50 %.
           The update event restarts the counter and loads the new CCR. */
        __HAL_TIM_SET_AUTORELOAD(&htim2, 999U);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 500U);
        HAL_TIM_GenerateEvent(&htim2, TIM_EVENTSOURCE_UPDATE);
    }
}

/* ========================================================================= */
/* TMC2240 setup, using TMC-API register fields                              */
/* ========================================================================= */
uint8_t Motor_DriverInit(uint16_t icID)
{
    /* Enable SPI2 first so SCK is already idling high (mode 3) when CS falls.
       HAL would otherwise enable it inside the first transfer, after CS. */
    __HAL_SPI_ENABLE(&hspi2);

    tmc_ok[icID] = 0U;

    /* 1. Is the chip answering? A TMC2240 reads VERSION 0x40; no reply
          reads as 0x00 or 0xFF. */
    tmc_version[icID] = (uint8_t)tmc2240_fieldRead(icID, TMC2240_VERSION_FIELD);
    if (tmc_version[icID] == 0x00U || tmc_version[icID] == 0xFFU) {
        return 0U;
    }

    /* 2. Clear the reset / error flags left over from power-up (write 1s) */
    tmc2240_writeRegister(icID, TMC2240_GSTAT, 0x1F);

    /* 3. GCONF: spreadCycle, step filter, direction */
    tmc2240_fieldWrite(icID, TMC2240_EN_PWM_MODE_FIELD,    0U);
    tmc2240_fieldWrite(icID, TMC2240_MULTISTEP_FILT_FIELD, 1U);
    tmc2240_fieldWrite(icID, TMC2240_SHAFT_FIELD,
                       (icID == TMC_RIGHT) ? TMC_SHAFT_R : TMC_SHAFT_L);

    /* 4. Current (values and the RREF calculation are in Robot.h) */
    tmc2240_fieldWrite(icID, TMC2240_CURRENT_RANGE_FIELD, TMC_CURRENT_RANGE);
    tmc2240_fieldWrite(icID, TMC2240_SLOPE_CONTROL_FIELD, TMC_SLOPE_CONTROL);
    tmc2240_fieldWrite(icID, TMC2240_GLOBALSCALER_FIELD,  TMC_GLOBAL_SCALER);
    tmc2240_fieldWrite(icID, TMC2240_IRUN_FIELD,          TMC_IRUN);
    tmc2240_fieldWrite(icID, TMC2240_IHOLD_FIELD,         TMC_IHOLD);
    tmc2240_fieldWrite(icID, TMC2240_IHOLDDELAY_FIELD,    TMC_IHOLDDELAY);
    tmc2240_fieldWrite(icID, TMC2240_TPOWERDOWN_FIELD,    TMC_TPOWERDOWN);

    /* 5. Chopper and microsteps. TOFF last: it switches the output stage on,
          so everything else is in place first. */
    tmc2240_fieldWrite(icID, TMC2240_TBL_FIELD,          TMC_TBL);
    tmc2240_fieldWrite(icID, TMC2240_HSTRT_TFD210_FIELD, TMC_HSTRT);
    tmc2240_fieldWrite(icID, TMC2240_HEND_OFFSET_FIELD,  TMC_HEND);
    tmc2240_fieldWrite(icID, TMC2240_TPFD_FIELD,         TMC_TPFD);
    tmc2240_fieldWrite(icID, TMC2240_MRES_FIELD,         TMC_MRES);
    tmc2240_fieldWrite(icID, TMC2240_INTPOL_FIELD,       TMC_INTPOL);
    tmc2240_fieldWrite(icID, TMC2240_TOFF_FIELD,         TMC_TOFF);

    /* 6. Read a few settings back to prove the writes landed */
    if (tmc2240_fieldRead(icID, TMC2240_TOFF_FIELD) == TMC_TOFF &&
        tmc2240_fieldRead(icID, TMC2240_MRES_FIELD) == TMC_MRES &&
        tmc2240_fieldRead(icID, TMC2240_IRUN_FIELD) == TMC_IRUN) {
        tmc_ok[icID] = 1U;
    }

    return tmc_ok[icID];
}

/* ========================================================================= */
/* Driver enable                                                             */
/* ========================================================================= */
void Motor_Enable(void)
{
    HAL_GPIO_WritePin(TMC_EN_GPIO_Port, TMC_EN_Pin, GPIO_PIN_RESET);
}

void Motor_Disable(void)
{
    HAL_GPIO_WritePin(TMC_EN_GPIO_Port, TMC_EN_Pin, GPIO_PIN_SET);
}
