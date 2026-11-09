/**
 * @file    Robot.h
 * @brief   Shared declarations for the balancing robot (NUCLEO-F446RE).
 *
 * PROVENANCE: AI-generated. Written by Claude (Anthropic), Nov 2026.
 *
 * Put this file in Core/Inc. The robot code's variables and functions are
 * declared here and defined in their .c files:
 *
 *   File                  Written by                  Contents
 *   Motor_test.c          Claude (AI)                 TMC2240 set-up, motor and LED test
 *   data.c                Claude (AI)                 printing data to the console
 *   Claude_diagnostics.c  Claude (AI)                 I2C / IMU hardware tests
 *   MPU6050.c / .h        creative-apple (GitHub),    MPU-6050 library; the "AI generated
 *                         + Claude glue section       glue code" section at the end of
 *                                                     MPU6050.c is Claude's (IMU_...)
 *   Encoder.c / .h        Claude (AI), algorithm      wheel encoders, PLL velocity
 *                         from ODrive (MIT)           (declared in Encoder.h)
 *   TMC2240.c / .h,       Analog Devices TMC-API      TMC2240 register access
 *   TMC2240_HW_Abstraction.h  (GitHub)
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
#include "Encoder.h"   /* wheel encoders: enc[], Encoder_Init(), Encoder_Update() */
/* MPU6050.h (creative-apple library) is NOT included here: it has no include
   guard, so include it directly, once, in a .c file that calls the library. */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>     /* printf -> SWV ITM console */

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
extern TIM_HandleTypeDef htim3;   /* right encoder (16-bit) */
extern TIM_HandleTypeDef htim5;   /* left encoder (32-bit)  */
extern I2C_HandleTypeDef hi2c3;   /* MPU-6050               */

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

/* ========================================================================= */
/* IMU: MPU-6050 tilt estimate                                               */
/* Defined in MPU6050.c, in the "AI generated glue code" section (Claude).   */
/* ========================================================================= */
/* IMU (GY-521 / MPU-6050) mounting
     +x  towards motor_R
     +y  forwards, away from the fuse
     +z  up (right-handed: x cross y)
   theta > 0 means the robot is leaning forwards (top towards +y). Check this
   sign against the theta convention in your MATLAB model.

   How the tilt values are found (IMU_Update()):
     theta_dot   gyro rate about x, bias removed. Leaning forwards is a
                 negative rotation about +x (right-hand rule), so
                 GYRO_SIGN_X = -1.
     theta_ddot  derivative of the gyro rate, low-pass filtered.
     theta       gyro rate integrated. A gyro integral drifts, so the
                 accelerometer angle atan2(-ay, az) pulls it back slowly with
                 time constant IMU_TILT_TAU_S (complementary filter). Set
                 IMU_TILT_TAU_S to 0 for gyro only and expect slow drift.
                 The starting angle always comes from the accelerometer.
   The gyro bias is measured in IMU_Init(): keep the robot still at start-up.

   Position: about 0.12 m above the ground, roughly in the centre of the
   0.14 m x 0.155 m body.

   NOTE: the wheels are not attached yet, so these figures are provisional.
   A more precise calculation of the accelerometer's position is still needed,
   measured from the wheel axle (the pivot): height above the axle =
   height above the ground - wheel radius, plus any fore/aft offset.
   The gyro reads the same anywhere on the robot, so the position only
   affects the accelerometer correction: when the robot rotates, the
   accelerometer also feels r*theta_ddot along y and -r*theta_dot^2 along z
   (r = distance from the axle). With r known its angle becomes
     theta_acc = atan2(-(ay - r*theta_ddot), az + r*theta_dot^2)   (m/s^2)
   That correction is NOT applied yet. The accelerometer angle is also wrong
   while the robot accelerates forwards or backwards, which is why it only
   gets a slow correction weight.                                            */
#define IMU_HEIGHT_ABOVE_GROUND_M  0.12f   /* provisional, wheels not attached */
#define IMU_OFFSET_X_M             0.0f    /* provisional, "roughly centred"   */
#define IMU_OFFSET_Y_M             0.0f    /* provisional, "roughly centred"   */
#define BODY_SIZE_A_M              0.14f   /* body footprint 0.14 m x 0.155 m  */
#define BODY_SIZE_B_M              0.155f

/* MPU-6050 settings: gyro +/-250 deg/s (131 LSB per deg/s), accelerometer
   +/-2 g (16384 LSB/g), I2C address 0x68, from the library's
   MPU6050_Initialization().
   CHANGE (Claude, 9 Nov 2026): IMU_Init() then turns the chip's low-pass
   filter on (gyro 42 Hz, accelerometer 44 Hz, ~5 ms delay) and sets the
   output rate to 1 kHz. The library left the filter off at 200 Hz.
   CHANGE (Claude, 9 Nov 2026): main.c now calibrates the gyro before
   Motor_test() starts the motors, so the bias is measured without vibration. */
#define GYRO_SIGN_X                (-1.0f) /* theta_dot = -gyro x, see above   */

/* Sampling and filtering */
#define IMU_SAMPLE_MS              5U      /* IMU_Update() period (main loop)          */
#define IMU_DDOT_CUTOFF_HZ         10.0f   /* theta_ddot low-pass                      */
#define IMU_TILT_TAU_S             1.0f    /* accelerometer correction, s (0 = gyro only) */
#define IMU_GYRO_CAL_SAMPLES       200U    /* x IMU_SAMPLE_MS: ~1 s gyro bias measurement */
#define IMU_MAX_DT_S               0.1f    /* longer gap = treat as a halt             */
#define RAD_TO_DEG                 57.2957795f
#define DEG_TO_RAD                 0.0174532925f

typedef struct {
    int16_t  acc_raw[3];     /* accelerometer x, y, z, raw (16384 = 1 g)      */
    int16_t  gyro_raw[3];    /* gyro x, y, z, raw (131 = 1 deg/s)             */
    float    gyro_bias_x;    /* gyro x bias from IMU_Init(), deg/s            */
    float    theta;          /* tilt, rad, + = leaning forwards (gyro + slow accel correction) */
    float    theta_dot;      /* rad/s, from the gyro                          */
    float    theta_ddot;     /* rad/s^2, derivative of the gyro rate          */
    float    theta_acc;      /* accelerometer-only angle, rad (drift reference) */
    float    dt;             /* last sample period, s                         */
    uint8_t  imu_ok;         /* 1 = MPU-6050 answered and was configured      */
    uint8_t  imu_id;         /* WHO_AM_I value (0x68 on a genuine MPU-6050)   */
    /* Internal state used by IMU_Update() */
    uint8_t  timing_valid;
    uint32_t last_cycles;
} IMU_Data;

extern IMU_Data imu_data;    /* defined in MPU6050.c; add it to Live Expressions */

uint8_t  IMU_Init(void);                 /* WHO_AM_I check, library set-up, gyro bias; 1 = OK */
uint8_t  IMU_CalibrateGyro(void);        /* gyro bias + starting angle, robot still        */
void     IMU_Update(void);               /* read the sensor, update the tilt estimate;
                                            call every IMU_SAMPLE_MS                       */
float    IMU_LowPassAlpha(float dt, float cutoff_hz);
void     IMU_GetAccel(int16_t *ax, int16_t *ay, int16_t *az);            /* raw counts   */
void     IMU_GetGyro(int16_t *gx, int16_t *gy, int16_t *gz);             /* raw counts   */
void     IMU_GetTilt(float *theta, float *theta_dot, float *theta_ddot); /* rad, rad/s, rad/s^2 */

/* ========================================================================= */
/* Wheel encoders: everything is in Encoder.h / Encoder.c (included above)   */
/* ========================================================================= */
/* Set to 1 by main.c after Encoder_Init(0.001f). SysTick_Handler() in
   stm32f4xx_it.c then calls Encoder_Update() every 1 ms. */
extern volatile uint8_t encoder_tick_enabled;

/* ========================================================================= */
/* Console printing (data.c)                                                 */
/* ========================================================================= */
#define DATA_PRINT_SAMPLES         20U     /* IMU_Update() calls per printed line (~0.1 s) */

int      __io_putchar(int ch);           /* printf -> SWV ITM port 0 (syscalls.c's _write) */
void     Data_Print(void);               /* one line: wheel velocities, tilt, accel        */
void     Data_PrintIMUStatus(void);      /* start-up report after IMU_Init() succeeds      */

/* ========================================================================= */
/* Diagnostics (Claude_diagnostics.c)                                        */
/* ========================================================================= */
void     Diag_IMUNotResponding(void);    /* report + all tests below; main.c calls it
                                            when IMU_Init() fails                          */
void     Diag_ScanI2C(void);             /* print every address that answers on I2C3       */
void     Diag_I2CPinTest(void);          /* read SDA/SCL with no pull, pull-up, pull-down  */
void     Diag_I2CWeakPullupTest(void);   /* try the MPU-6050 at 50 kHz on internal pull-ups */

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_H */
