/**
 * @file    Motor_test.c
 * @brief   Motor and LED test for the balancing robot.
 *
 * PROVENANCE: AI-generated. Written by Claude (Anthropic), Nov 2026. It drives
 * the TMC2240s through Analog Devices' TMC-API (TMC2240.c/.h, downloaded) and
 * supplies the callbacks that library needs.
 *
 * Put this file in Core/Src. Call Motor_test() once from main.c in
 * USER CODE BEGIN 2, after the MX_xxx_Init() calls. The timers then run in
 * hardware, so nothing needs to go in the while(1) loop.
 *
 * The TMC2240s are driven through ADI's TMC-API (TMC2240.c/.h). This file
 * supplies the board-specific SPI callback TMC-API needs, and the settings.
 *
 * Note: the TMC2240 steps on STEP rising edges, so the duty cycle does not
 * change motor speed. Speed comes from the STEP frequency:
 *     step rate = 1 MHz / (ARR + 1)      (ARR 999 -> 1 kHz)
 */
#include "Robot.h"
#include <string.h>

/* ------------------------------------------------------------------------- */
/* Variables declared in Robot.h                                             */
/* ------------------------------------------------------------------------- */
float    motor_duty_L = 0.0f;
float    motor_duty_R = 0.0f;
float    led_period_s = 0.0f;

uint8_t  tmc_ok[TMC_COUNT]         = {0, 0};
uint8_t  tmc_version[TMC_COUNT]    = {0, 0};
uint32_t tmc_gstat[TMC_COUNT]      = {0, 0};
uint32_t tmc_drv_status[TMC_COUNT] = {0, 0};

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
        /* LED: blink with period T = 0.5 s (on 0.25 s, off 0.25 s) */
        LED_SetPeriod(LED_PERIOD_S, LED_DUTY);


        /* Step rate = 1 MHz / (ARR + 1) */
        __HAL_TIM_SET_AUTORELOAD(&htim8, 60);   // STEP_L: 2 kHz, ~38 rpm
        __HAL_TIM_SET_AUTORELOAD(&htim4, 60);   // STEP_R: 2 kHz, ~38 rpm


        /* Motors: 75 % duty on both STEP outputs, frequency as set in CubeMX.
           1 kHz STEP at 16 microsteps = 0.31 rev/s (about 19 rpm). */
        Motor_SetDuty(&htim8, TIM_CHANNEL_1, MOTOR_TEST_DUTY);   /* STEP_L, PC6 */
        Motor_SetDuty(&htim4, TIM_CHANNEL_1, MOTOR_TEST_DUTY);   /* STEP_R, PB6 */

        /* Load the new CCR values into the shadow registers now, rather than
           after the first period (CCR preload is on in CubeMX's PWM setup). */
        HAL_TIM_GenerateEvent(&htim8, TIM_EVENTSOURCE_UPDATE);
        HAL_TIM_GenerateEvent(&htim4, TIM_EVENTSOURCE_UPDATE);

        /* Turn the driver power stages on (TMC_EN is active low) */
        Motor_Enable();

        /* Start the STEP outputs. HAL_TIM_PWM_Start also sets MOE for TIM8. */
        HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    } else {
        /* A driver did not answer or did not take its settings: leave the
           motors off and blink fast. Check tmc_ok[] / tmc_version[]. */
        LED_SetPeriod(LED_FAULT_PERIOD_S, LED_DUTY);
    }

    HAL_TIM_GenerateEvent(&htim2, TIM_EVENTSOURCE_UPDATE);


    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
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

    Motor_UpdateStatus(icID);
    return tmc_ok[icID];
}

void Motor_UpdateStatus(uint16_t icID)
{
    tmc_gstat[icID]      = (uint32_t)tmc2240_readRegister(icID, TMC2240_GSTAT);
    tmc_drv_status[icID] = (uint32_t)tmc2240_readRegister(icID, TMC2240_DRVSTATUS);
}

/* ========================================================================= */
/* TMC-API callbacks: how the library reaches the two chips on this board    */
/* ========================================================================= */
TMC2240BusType tmc2240_getBusType(uint16_t icID)
{
    (void)icID;
    return IC_BUS_SPI;
}

/* One SPI datagram. TMC-API passes the bytes to send in data[] and expects
   the received bytes back in the same buffer. */
void tmc2240_readWriteSPI(uint16_t icID, uint8_t *data, size_t dataLength)
{
    GPIO_TypeDef *cs_port = (icID == TMC_LEFT) ? TMC_CS_L_GPIO_Port : TMC_CS_R_GPIO_Port;
    uint16_t      cs_pin  = (icID == TMC_LEFT) ? TMC_CS_L_Pin       : TMC_CS_R_Pin;

    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    if (HAL_SPI_TransmitReceive(&hspi2, data, data, (uint16_t)dataLength, 10U) != HAL_OK) {
        memset(data, 0, dataLength);   /* reads back as 0 -> treated as no reply */
    }
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

    /* Keep CS high for ~1 us before the next datagram */
    for (volatile uint32_t i = 0; i < 20U; i++) { }
}

/* UART is not used on this board, but TMC-API links against these. */
bool tmc2240_readWriteUART(uint16_t icID, uint8_t *data, size_t writeLength, size_t readLength)
{
    (void)icID; (void)data; (void)writeLength; (void)readLength;
    return false;
}

uint8_t tmc2240_getNodeAddress(uint16_t icID)
{
    (void)icID;
    return 0U;
}

/* ========================================================================= */
/* PWM, LED and enable helpers                                               */
/* ========================================================================= */
void Motor_SetDuty(TIM_HandleTypeDef *htim, uint32_t channel, float duty)
{
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;

    /* PWM mode 1: output high while CNT < CCR, and CNT runs 0..ARR,
       so duty = CCR / (ARR + 1). ARR 999, duty 0.75 -> CCR 750. */
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
    uint32_t ccr = (uint32_t)(duty * (float)(arr + 1U) + 0.5f);
    __HAL_TIM_SET_COMPARE(htim, channel, ccr);

    if (htim == &htim8) motor_duty_L = duty;
    if (htim == &htim4) motor_duty_R = duty;
}

void LED_SetPeriod(float period_s, float duty)
{
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;

    /* Counter tick = timer clock / (PSC + 1) = 84 MHz / 8400 = 10 kHz */
    uint32_t tick_hz = TIM_APB1ClockHz() / (htim2.Instance->PSC + 1U);

    /* Period = (ARR + 1) / tick -> ARR = 0.5 s * 10 kHz - 1 = 4999.
       TIM2 is 32-bit, so long periods will not overflow ARR. */
    uint32_t counts = (uint32_t)(period_s * (float)tick_hz + 0.5f);
    if (counts < 2U) counts = 2U;

    __HAL_TIM_SET_AUTORELOAD(&htim2, counts - 1U);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,
                          (uint32_t)(duty * (float)counts + 0.5f));   /* 2500 */

    led_period_s = period_s;
}

void Motor_Enable(void)
{
    HAL_GPIO_WritePin(TMC_EN_GPIO_Port, TMC_EN_Pin, GPIO_PIN_RESET);
}

void Motor_Disable(void)
{
    HAL_GPIO_WritePin(TMC_EN_GPIO_Port, TMC_EN_Pin, GPIO_PIN_SET);
}

uint32_t TIM_APB1ClockHz(void)
{
    /* APB1 timers run at 2 x PCLK1 whenever the APB1 prescaler is not 1
       (TIMPRE = 0). Here PCLK1 = 42 MHz -> 84 MHz. */
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) {
        return 2U * pclk1;
    }
    return pclk1;
}
