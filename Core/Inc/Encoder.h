/**
 * @file    Encoder.h
 * @brief   Wheel encoder reading for the balancing robot (NUCLEO-F446RE).
 *
 * PROVENANCE: AI-generated. Written by Claude (Anthropic) in the chat "Motor
 * datasheet and firmware", 8 Nov 2026. This is not downloaded code: Encoder.c
 * re-implements in C the PLL velocity estimator from ODrive's encoder.cpp
 * (https://github.com/odriverobotics/ODrive, MIT licence,
 * Copyright (c) 2016-2018 ODrive Robotics). Acknowledge both.
 *
 * In this project Encoder_Update() runs at 1 kHz from SysTick_Handler()
 * (stm32f4xx_it.c), started by Encoder_Init(0.001f) in main.c.
 *
 * Put this file in Core/Inc and Encoder.c in Core/Src.
 *
 * Motors: StepperOnline 17HS24-2004D-E1K, 1000 PPR optical encoder.
 * CubeMX already decodes both encoders in hardware (encoder mode, TI1 and TI2,
 * so x4 = 4000 counts per revolution):
 *   TIM5  PA0/PA1  left wheel   (32-bit counter)
 *   TIM3  PA6/PA7  right wheel  (16-bit counter, motor on J2 near the fuse)
 *
 * The hardware counts every edge without the CPU, so the counts themselves are
 * exact. What makes readings deterministic is HOW they are sampled:
 *   - Call Encoder_Update() from a fixed-rate timer interrupt (the control
 *     loop), never from while(1) with HAL_Delay(). Both counters are read
 *     back to back at the top, so left and right are sampled at the same time.
 *   - Counter wrap-around is handled with a signed difference each sample, so
 *     the 16-bit TIM3 never needs an overflow interrupt.
 *
 * Velocity comes two ways, so you can compare them in Live Expressions:
 *   velocity_raw_rad_s  counts moved this period / period. Only half a period
 *                       of lag, but steps in units of one count
 *                       (0.31 rad/s at 200 Hz, 1.57 rad/s at 1 kHz).
 *   velocity_rad_s      phase-locked loop (PLL) estimate, the same algorithm as
 *                       ODrive's Encoder::update() (MIT licence, (c) 2016-2018
 *                       ODrive Robotics). Smooth and reads exactly 0 at
 *                       standstill, but lags by about 2 / bandwidth seconds
 *                       while the wheel accelerates.
 *
 * Which to use (simulated, 8 Nov 2026: ramp to 20 rad/s, then a 2 Hz
 * +/-1.5 rad/s wobble like balancing; RMS error at 200 Hz control instants):
 *                               ramp   constant   wobble   (rad/s)
 *   200 Hz, raw                 0.13     0.15      0.15
 *   200 Hz, PLL bandwidth 50    0.38     0.01      0.47   <- worse: lag
 *   1 kHz,  PLL bandwidth 200   0.10     0.03      0.13   <- best
 * So: at 200 Hz use velocity_raw_rad_s. To use the PLL, call
 * Encoder_Update() at 1 kHz and run the control loop every 5th call.
 */
#ifndef ENCODER_H
#define ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ========================================================================= */
/* Settings                                                                  */
/* ========================================================================= */
#define ENC_COUNTS_PER_REV  4000.0f   /* 1000 PPR x 4 (CubeMX: TI1 and TI2)   */

/* PLL bandwidth, rad/s. Higher = less lag (about 2 / bandwidth seconds),
   more quantisation noise. Stability limit (same check as ODrive):
   sample period x 2 x bandwidth < 1, so the maximum is 1 / (2 x period):
       200 Hz Encoder_Update ->  99 rad/s max
         1 kHz Encoder_Update -> 499 rad/s max
   Encoder_Init() clamps a value that is too high (to 90 % of the limit) and
   returns 0. 200 rad/s is for a 1 kHz Encoder_Update (10 ms lag).          */
#define ENC_PLL_BANDWIDTH   200.0f

/* The wheels face opposite ways, so one encoder counts down when the robot
   drives forwards. Check: push the robot forwards by hand; enc[].position
   should increase on BOTH wheels. If one decreases, flip its sign here.     */
#define ENC_SIGN_L          (+1)
#define ENC_SIGN_R          (-1)

/* Wheel IDs, same numbering as TMC_LEFT / TMC_RIGHT in Robot.h */
#define ENC_LEFT            0U
#define ENC_RIGHT           1U
#define ENC_COUNT           2U

/* ========================================================================= */
/* Data, refreshed by every Encoder_Update(). Add enc to Live Expressions.   */
/* ========================================================================= */
typedef struct {
    int32_t position;            /* counts since Encoder_Init(), no wrap-around */
    int32_t delta;               /* counts moved during the last sample period  */
    float   angle_rad;           /* wheel angle since Encoder_Init(), rad       */
    float   velocity_raw_rad_s;  /* delta / sample period, rad/s                */
    float   velocity_rad_s;      /* PLL velocity estimate, rad/s                */

    /* PLL internals. Position estimate = pll_pos_int + pll_pos_frac counts.
       Splitting it keeps full resolution however far the robot drives
       (ODrive keeps it in one float, which loses fractions after ~4000 rev). */
    int32_t pll_pos_int;
    float   pll_pos_frac;        /* always in [0, 1) */
    float   pll_vel_counts_s;
} Encoder_t;

extern Encoder_t enc[ENC_COUNT];

/* ========================================================================= */
/* Functions                                                                 */
/* ========================================================================= */
/* Call once after MX_TIM3_Init() / MX_TIM5_Init(). sample_period_s is the
   period of the timer interrupt that will call Encoder_Update(), e.g. 0.005f
   for 200 Hz. Zeroes enc[]. Returns 1 if ENC_PLL_BANDWIDTH is stable at this
   period, 0 if it had to be clamped.                                        */
uint8_t Encoder_Init(float sample_period_s);

/* Call exactly once per sample period, from the fixed-rate timer callback. */
void Encoder_Update(void);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_H */
