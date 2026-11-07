/**
 * @file    Robot.h
 * @brief   Shared declarations for the balancing robot (NUCLEO-F446RE).
 *
 * Put this file in Core/Inc. Every variable and function used by the robot
 * code (Motor_test.c, ...) is declared here and defined in its .c file.
 *
 * The TMC2240 drivers are run with Analog Devices' TMC-API library
 * (TMC2240.c in Core/Src, TMC2240.h + TMC2240_HW_Abstraction.h in Core/Inc,
 * unmodified). Register and field names such as TMC2240_IRUN_FIELD come from
 * that library.
 *
 * Timer map (from MECHA3500-Balancingrobot.ioc):
 *   TIM2 CH1  PA5  LD2 user LED    APB1 timer clock 84 MHz, PSC 8399 -> 10 kHz tick
 *   TIM8 CH1  PC6  STEP_L (U3)     APB2 timer clock 84 MHz, PSC 83   -> 1 MHz tick
 *   TIM4 CH1  PB6  STEP_R (U4)     APB1 timer clock 84 MHz, PSC 83   -> 1 MHz tick
 *   PB5       TMC_EN, active low, shared by U3 and U4
 *
 * SPI2 (PB13 SCK, PB14 MISO, PB15 MOSI), mode 3, shared by both TMC2240s:
 *   PB12      TMC_CS_L (U3, left,  J1)
 *   PB1       TMC_CS_R (U4, right, J2 near the fuse)
 */
#ifndef ROBOT_H
#define ROBOT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"      /* HAL, pin labels from CubeMX */
#include "TMC2240.h"   /* ADI TMC-API: tmc2240_readRegister, tmc2240_fieldWrite, ... */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ========================================================================= */
/* Test settings                                                             */
/* ========================================================================= */
#define MOTOR_TEST_DUTY     0.75f  /* STEP PWM duty for both motors, 0.0 to 1.0  */
#define LED_PERIOD_S        0.50f  /* LED blink period T, seconds (all OK)       */
#define LED_FAULT_PERIOD_S  0.10f  /* fast blink = a TMC2240 failed to configure */
#define LED_DUTY            0.50f  /* fraction of T the LED is on                */

/* ========================================================================= */
/* Pins. Use the CubeMX labels if they exist, otherwise the schematic pins.  */
/* ========================================================================= */
#ifndef TMC_EN_Pin
#define TMC_EN_Pin          GPIO_PIN_5
#define TMC_EN_GPIO_Port    GPIOB
#endif
#ifndef TMC_CS_L_Pin
#define TMC_CS_L_Pin        GPIO_PIN_12
#define TMC_CS_L_GPIO_Port  GPIOB
#endif
#ifndef TMC_CS_R_Pin
#define TMC_CS_R_Pin        GPIO_PIN_1
#define TMC_CS_R_GPIO_Port  GPIOB
#endif

/* ========================================================================= */
/* TMC2240 chip IDs (the icID TMC-API passes to the callbacks)               */
/* ========================================================================= */
#define TMC_LEFT            0U     /* U3, J1, STEP_L PC6, CS PB12 */
#define TMC_RIGHT           1U     /* U4, J2, STEP_R PB6, CS PB1  */
#define TMC_COUNT           2U

/* ========================================================================= */
/* TMC2240 settings, written by Motor_DriverInit()                           */
/* ========================================================================= */
/* Current. Full-scale peak = K_IFS / RREF, K_IFS = 11.75 / 24 / 36 A*kOhm for
   CURRENT_RANGE 0 / 1 / 2. RREF is the resistor on the IREF pin (12-60 kOhm).
   With RREF = 12 kOhm (the smallest allowed, so the most current):
     range 1 -> 2.0 A peak full scale
     IRUN 23 -> (23+1)/32 * 2.0 = 1.5 A peak = 1.06 A RMS  (motor rated 2.0 A)
     IHOLD 8 -> ( 8+1)/32 * 2.0 = 0.56 A peak
   A bigger RREF scales all of these down in proportion.                     */
#define TMC_CURRENT_RANGE   1U     /* TMC2240_CURRENT_RANGE_FIELD             */
#define TMC_SLOPE_CONTROL   2U     /* TMC2240_SLOPE_CONTROL_FIELD, ADI preset */
#define TMC_GLOBAL_SCALER   0U     /* TMC2240_GLOBALSCALER_FIELD, 0 = full    */
#define TMC_IRUN            23U    /* TMC2240_IRUN_FIELD, 0-31                */
#define TMC_IHOLD           8U     /* TMC2240_IHOLD_FIELD, 0-31               */
#define TMC_IHOLDDELAY      7U     /* TMC2240_IHOLDDELAY_FIELD, 0-15          */
#define TMC_TPOWERDOWN      10U    /* TMC2240_TPOWERDOWN_FIELD                */

/* Chopper: spreadCycle with the timing from ADI's CHOPCONF preset
   (R6C = 0x10410153 in TMC2240.h), but 16 microsteps instead of 256.        */
#define TMC_TOFF            3U     /* TMC2240_TOFF_FIELD: > 0 turns the output stage on */
#define TMC_HSTRT           5U     /* TMC2240_HSTRT_TFD210_FIELD */
#define TMC_HEND            2U     /* TMC2240_HEND_OFFSET_FIELD  */
#define TMC_TBL             2U     /* TMC2240_TBL_FIELD          */
#define TMC_TPFD            4U     /* TMC2240_TPFD_FIELD         */
#define TMC_MRES            4U     /* TMC2240_MRES_FIELD: 0 = 256 ... 4 = 16 ... 8 = full step */
#define TMC_INTPOL          1U     /* TMC2240_INTPOL_FIELD: interpolate to 256 */

/* Direction. The wheels face opposite ways, so one motor is reversed
   (TMC2240_SHAFT_FIELD). If the robot drives backwards, swap these two.     */
#define TMC_SHAFT_L         0U
#define TMC_SHAFT_R         1U

/* ========================================================================= */
/* Peripheral handles (defined by CubeMX in main.c, or tim.c/spi.c if split) */
/* ========================================================================= */
extern TIM_HandleTypeDef htim2;   /* LED          */
extern TIM_HandleTypeDef htim4;   /* STEP_R motor */
extern TIM_HandleTypeDef htim8;   /* STEP_L motor */
extern SPI_HandleTypeDef hspi2;   /* TMC2240 x2   */

/* ========================================================================= */
/* Variables (defined in Motor_test.c)                                       */
/* ========================================================================= */
extern float    motor_duty_L;               /* last duty written to STEP_L, 0.0 to 1.0 */
extern float    motor_duty_R;               /* last duty written to STEP_R, 0.0 to 1.0 */
extern float    led_period_s;               /* last LED period written, seconds        */

/* Indexed by TMC_LEFT / TMC_RIGHT. Add these to Live Expressions. */
extern uint8_t  tmc_ok[TMC_COUNT];          /* 1 = configured and read back correctly  */
extern uint8_t  tmc_version[TMC_COUNT];     /* 0x40 = TMC2240, 0x00/0xFF = no reply    */
extern uint32_t tmc_gstat[TMC_COUNT];       /* GSTAT after setup                       */
extern uint32_t tmc_drv_status[TMC_COUNT];  /* DRV_STATUS: test with TMC2240_OLA_MASK,
                                               TMC2240_S2GA_MASK, TMC2240_OT_MASK, ... */

/* ========================================================================= */
/* Functions (defined in Motor_test.c)                                       */
/* ========================================================================= */
void     Motor_test(void);                       /* configure drivers, run motors, blink LED */
uint8_t  Motor_DriverInit(uint16_t icID);        /* returns 1 if the driver is configured    */
void     Motor_UpdateStatus(uint16_t icID);      /* refresh tmc_gstat / tmc_drv_status       */
void     Motor_SetDuty(TIM_HandleTypeDef *htim, uint32_t channel, float duty);
void     LED_SetPeriod(float period_s, float duty);
void     Motor_Enable(void);                     /* TMC_EN low  -> power stages on  */
void     Motor_Disable(void);                    /* TMC_EN high -> outputs floating */
uint32_t TIM_APB1ClockHz(void);                  /* APB1 timer kernel clock, Hz     */

/* Callbacks TMC-API calls to reach the hardware (also declared in TMC2240.h) */
TMC2240BusType tmc2240_getBusType(uint16_t icID);
void           tmc2240_readWriteSPI(uint16_t icID, uint8_t *data, size_t dataLength);
bool           tmc2240_readWriteUART(uint16_t icID, uint8_t *data, size_t writeLength, size_t readLength);
uint8_t        tmc2240_getNodeAddress(uint16_t icID);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_H */
