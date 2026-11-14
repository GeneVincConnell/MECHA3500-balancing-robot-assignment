/**
 * @file    SensorLog.h
 * @brief   Encoder + IMU recording at 500 Hz, timed (logAll) or triggered by
 *          a fall (logFall), sent as CSV over the ST-LINK COM port for
 *          MATLAB (logRobotSensors.m saves it as a .csv file).
 *
 * PROVENANCE: AI-generated. Written by Claude (Anthropic), 14 Nov 2026, for
 * the CubeIDE balancing-robot project. Nothing downloaded or copied. Reads
 * enc[] (Encoder.c) and imu_data (MPU6050.c's "AI generated glue code"),
 * and uses the TMC-API field functions (TMC2240.h, Analog Devices) to set
 * the hold current.
 * Updated by Claude, 14 Nov 2026: logFall command (pre/post-trigger capture
 * with both motors holding), 500 Hz sampling, gyro range +/-1000 deg/s,
 * #INFO line with the scale factors, read-error and timing-gap counts.
 *
 * Put this file in Core/Inc and SensorLog.c in Core/Src.
 *
 * Serial: USART2 (PA2/PA3), the ST-LINK virtual COM port, 115200 8N1 as set
 * by CubeMX's MX_USART2_UART_Init(). SensorLog.c adds the interrupt driver
 * and defines USART2_IRQHandler() itself (USART2 global interrupt is off in
 * the .ioc). If you ever turn that interrupt on in CubeMX, or add Lab5.c
 * (which has its own USART2_IRQHandler), the build fails with "multiple
 * definition of USART2_IRQHandler": keep only one of them.
 * printf still goes to SWV as before; only SensorLog uses USART2.
 *
 * ---------------------------------------------------------------------------
 * logAll [seconds]                      (default 3 s, at most SENSORLOG_MAX_S)
 *   1. Stops the STEP PWM on both motors (TIM8 CH1 PC6, TIM4 CH1 PB6) and
 *      keeps it off while recording (anything that restarts it is stopped
 *      again and counted). It is left off afterwards.
 *   2. Recalibrates the gyro (IMU_CalibrateGyro(), ~1 s): KEEP THE ROBOT
 *      STILL. If the IMU had not answered at start-up, IMU_Init() is tried
 *      again instead; if it still does not answer, nothing is logged.
 *   3. Records one sample every main-loop pass (500 Hz, right after
 *      IMU_Update()) into RAM. The time column is measured with the DWT
 *      cycle counter, so it is the real sample time, jitter included.
 *   4. Then sends the samples as CSV (see "Serial output" below).
 *
 * logFall [pre_s] [post_s] [trigger_deg]   (defaults 1.0 s, 2.0 s, 3 deg)
 *   For the open-loop fall test: the wheel motors hold still (omega = 0) and
 *   the robot falls about the wheel contact point.
 *   1. Stops the STEP PWM, configures the TMC2240s if Motor_test() has not
 *      (needs the battery on), raises the hold current to the run current
 *      (IHOLD = TMC_IRUN, 1.5 A peak) and enables both drivers. The rotors
 *      snap to their nearest step position here: hold the robot.
 *   2. Recalibrates the gyro (~1 s): hold the robot upright and STILL.
 *   3. Armed: records continuously, keeping the newest samples (ring
 *      buffer), until the tilt has moved trigger_deg from where it was at
 *      arming. Let go of the robot whenever you are ready (within
 *      SENSORLOG_FALL_TIMEOUT_S). The trigger sample is time 0.
 *   4. Keeps recording for post_s, then puts the hold current and the
 *      driver enable back as they were, and sends pre_s before the trigger
 *      to post_s after it as CSV (times before the trigger are negative).
 *   pre_s + post_s must be at most SENSORLOG_MAX_S.
 *
 * Serial output of one run (only the lines between the markers are CSV):
 *   SensorLog: ... status lines ...
 *   #INFO fall=1 pre_s=1.000 post_s=2.000 trigger_deg=3.00 sample_period_us=2000 gyro_lsb_per_dps=32.8 acc_lsb_per_g=16384
 *   #CSV_BEGIN
 *   time_s,enc_L_counts,enc_R_counts,acc_x_raw,acc_y_raw,acc_z_raw,gyro_x_raw,gyro_y_raw,gyro_z_raw,theta_rad,theta_dot_rad_s
 *   -1.000412,0,0,-152,4021,15990,-35,6,-3,0.05001,0.00012
 *   ...
 *   #CSV_END samples=1500 duration_s=2.998 rate_Hz=500.0 max_gap_ms=2.01 buffer_full=0 pwm_restarted=0 i2c_errors=0 gyro_bias_x_dps=-0.532 theta_arm_rad=0.05001
 * 115200 baud is the limit for sending: ~70 characters a line, so about
 * 160 lines/s; 1500 samples take ~9 s to send. The main loop keeps running
 * while it sends (only as much is queued each pass as fits in the TX buffer).
 *
 * Columns
 *   time_s            logAll: s since the first sample. logFall: s since
 *                     the trigger sample (negative before it). DWT, 84 MHz.
 *   enc_L/R_counts    wheel (motor) counts since the first sample sent,
 *                     4000 per rev, + = forwards on both wheels (ENC_SIGN_L/R
 *                     in Encoder.h). This is phi, the motor angle relative to
 *                     the chassis. Encoder_Update() runs at 1 kHz in SysTick,
 *                     so these are at most 1 ms older than the IMU values.
 *   acc_*_raw         MPU-6050 accelerometer, acc_lsb_per_g (16384) per g
 *   gyro_*_raw        MPU-6050 gyro, gyro_lsb_per_dps (32.8) per deg/s,
 *                     bias NOT removed (bias x is in the #CSV_END line)
 *   theta_rad         firmware tilt estimate (complementary filter), + = forwards.
 *                     During a fall the accelerometer angle is wrong (the
 *                     body is accelerating), and the filter is pulled towards
 *                     it a little, so for curve fitting integrate the gyro
 *                     instead (logRobotSensors.m adds theta_gyro_rad).
 *   theta_dot_rad_s   firmware tilt rate (gyro x, bias removed, GYRO_SIGN_X)
 *
 * #CSV_END fields: max_gap_ms is the longest time between two samples sent
 * (2.0 when nothing interrupted the 500 Hz loop); i2c_errors counts failed
 * MPU-6050 reads during the run (the previous values were repeated);
 * theta_arm_rad is the tilt when the run started.
 *
 * main.c
 *   USER CODE BEGIN Includes:   #include "SensorLog.h"
 *   USER CODE BEGIN 2:          SensorLog_Init();     (after the IMU set-up)
 *   while(1) loop, straight after IMU_Update():   SensorLog_Update();
 *   and print with Data_Print() only when SensorLog_Busy() is 0.
 */
#ifndef SENSORLOG_H
#define SENSORLOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ========================================================================= */
/* Settings                                                                  */
/* ========================================================================= */
#define SENSORLOG_DEFAULT_S        3.0f    /* logAll with no argument            */
#define SENSORLOG_MAX_S            4.5f    /* the buffer holds 4.8 s at 500 Hz   */

/* RAM buffer: 32 bytes per sample, 2400 samples = 76.8 kB of the 128 kB.
   If the build fails with "region `RAM' overflowed", lower this (and
   SENSORLOG_MAX_S with it). A run stops early, and says so in the #CSV_END
   line (buffer_full=1), if the buffer fills before the time is up. */
#define SENSORLOG_MAX_SAMPLES      2400U

/* logAll only. 1 = also switch the TMC2240 power stages off (TMC_EN high,
   Motor_Disable()) so the wheels turn freely by hand. 0 = only the STEP PWM
   is stopped; the drivers stay as they were (enabled = motors held at IHOLD
   current). logFall always holds the motors. */
#define SENSORLOG_DISABLE_DRIVERS  0

/* logFall defaults and limits */
#define SENSORLOG_FALL_PRE_S       1.0f    /* kept before the trigger            */
#define SENSORLOG_FALL_POST_S      2.0f    /* recorded after the trigger         */
#define SENSORLOG_FALL_TRIGGER_DEG 3.0f    /* tilt change from arming            */
#define SENSORLOG_FALL_TIMEOUT_S   30.0f   /* give up if no fall by then         */
/* Hold current while logFall runs, TMC2240_IHOLD_FIELD (0-31). TMC_IRUN
   (23, 1.5 A peak, Robot.h) instead of the normal TMC_IHOLD (8, 0.56 A):
   the motors must hold the wheels against the falling body (estimated
   0.05-0.13 N*m per motor) with as little shaft deflection as possible.
   Both motors at 1.5 A peak for a few seconds is well within rating. */
#define SENSORLOG_FALL_IHOLD       TMC_IRUN
#define SENSORLOG_HOLD_SETTLE_MS   300U    /* after enabling, before calibrating */

/* ========================================================================= */
/* Functions                                                                 */
/* ========================================================================= */
/* Once, in USER CODE BEGIN 2 (after MX_USART2_UART_Init()): turns on the
   USART2 receive interrupt and prints a ready line on the COM port. */
void    SensorLog_Init(void);

/* Every main-loop pass, straight after IMU_Update(). Reads serial commands
   when idle, takes a sample while recording, queues CSV lines while
   sending. logAll and logFall run from here and block for ~1-1.5 s (motor
   hold settling and gyro calibration). */
void    SensorLog_Update(void);

/* 1 from logAll / logFall until the #CSV_END (or #ERROR) line has been queued */
uint8_t SensorLog_Busy(void);

#ifdef __cplusplus
}
#endif

#endif /* SENSORLOG_H */
