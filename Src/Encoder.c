/**
 * @file    Encoder.c
 * @brief   Wheel encoder reading for the balancing robot. See Encoder.h.
 *
 *
 *
 *
 *
 * PROVENANCE: AI-generated. Written by Claude (Anthropic) in the chat "Motor
 * datasheet and firmware", 8 Nov 2026, following ODrive's algorithm (below).
 *
 *
 *
 *
 * The PLL velocity estimator follows the algorithm in ODrive's
 * Firmware/MotorControl/encoder.cpp (Encoder::update_pll_gains() and
 * Encoder::update(), incremental-encoder path):
 *   https://github.com/odriverobotics/ODrive
 * ODrive firmware is MIT licensed, Copyright (c) 2016-2018 ODrive Robotics.
 * This is a re-implementation in C, not a copy; if you paste ODrive code in
 * directly, include their MIT licence text with it.
 *
 *
 *
 *
 *
 * Hook-up (example: TIM6 at 1 kHz, control loop every 5th tick = 200 Hz):
 *   CubeMX: TIM6 activated, PSC 83, ARR 999 -> 84 MHz / 84 / 1000 = 1 kHz,
 *           TIM6 global interrupt (TIM6_DAC_IRQn) enabled.
 *           Use TIM7 instead if TIM6 is the HAL timebase (SYS > Timebase Source).
 *           (For 200 Hz only, ARR 4999 and Encoder_Init(0.005f); then use
 *           velocity_raw_rad_s, see the table in Encoder.h.)
 *   main.c, USER CODE BEGIN 2:
 *       Encoder_Init(0.001f);
 *       HAL_TIM_Base_Start_IT(&htim6);
 *   Timer callback (if main.c already has HAL_TIM_PeriodElapsedCallback, add
 *   the TIM6 branch to that one instead of defining a second one):
 *       void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
 *       {
 *           static uint32_t tick = 0;
 *           if (htim->Instance == TIM6) {
 *               Encoder_Update();                 // every 1 ms
 *               if (++tick >= 5U) {               // every 5 ms
 *                   tick = 0;
 *                   // ... control loop: use enc[i].velocity_rad_s ...
 *               }
 *           }
 *       }
 */
#include "Encoder.h"
#include <math.h>
#include <string.h>

/* Encoder timers, set up by CubeMX (encoder mode, TI1 and TI2, filter 8) */
extern TIM_HandleTypeDef htim5;   /* left,  PA0/PA1, 32-bit, ARR 0xFFFFFFFF */
extern TIM_HandleTypeDef htim3;   /* right, PA6/PA7, 16-bit, ARR 65535      */

#define ENC_TWO_PI  6.28318530718f

Encoder_t enc[ENC_COUNT];

static uint32_t last_cnt_L;
static uint16_t last_cnt_R;
static float    dt_s;
static float    rad_per_count;
static float    rad_s_per_count;   /* rad_per_count / sample period */

/* PLL gains, from ODrive's update_pll_gains():
     kp = 2 x bandwidth            (basic conversion to discrete time)
     ki = kp^2 / 4                 (critically damped)                      */
static float    pll_kp;
static float    pll_ki;
static float    pll_snap_thresh;   /* |velocity| below this reads as 0, counts/s */

/* Move whole counts out of pll_pos_frac into pll_pos_int, keeping frac in [0, 1) */
static void pll_normalise(Encoder_t *e)
{
    float whole = floorf(e->pll_pos_frac);
    e->pll_pos_int  += (int32_t)whole;
    e->pll_pos_frac -= whole;
}

/* One PLL step for one wheel. 'measured' is the encoder count (enc[].position). */
static void pll_update(Encoder_t *e, int32_t measured)
{
    /* 1. Predict: move the estimate on by one period at the estimated speed */
    e->pll_pos_frac += dt_s * e->pll_vel_counts_s;
    pll_normalise(e);

    /* 2. Phase detector. An incremental encoder reports floor(true position),
          so compare the count with the floor of the estimate (pll_pos_int). */
    float err = (float)(measured - e->pll_pos_int);

    /* 3. Correct position (proportional) and velocity (integral) */
    e->pll_pos_frac     += dt_s * pll_kp * err;
    pll_normalise(e);
    e->pll_vel_counts_s += dt_s * pll_ki * err;

    /* 4. Snap to zero, as ODrive does: at standstill the estimate would
          otherwise dither by a fraction of a count per second. */
    if (fabsf(e->pll_vel_counts_s) < pll_snap_thresh) {
        e->pll_vel_counts_s = 0.0f;
    }
}

uint8_t Encoder_Init(float sample_period_s)
{
    uint8_t stable = 1U;

    /* Harmless if main.c has already started them: HAL just returns an
       error for the second start and the timer keeps counting. */
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    dt_s            = sample_period_s;
    rad_per_count   = ENC_TWO_PI / ENC_COUNTS_PER_REV;
    rad_s_per_count = rad_per_count / sample_period_s;

    /* PLL gains. ODrive flags ERROR_UNSTABLE_GAIN when period x kp >= 1;
       here the bandwidth is clamped to 90 % of that limit instead. */
    float bandwidth = ENC_PLL_BANDWIDTH;
    float bw_max    = 1.0f / (2.0f * sample_period_s);
    if (bandwidth >= bw_max) {
        bandwidth = 0.9f * bw_max;
        stable    = 0U;
    }
    pll_kp          = 2.0f * bandwidth;
    pll_ki          = 0.25f * pll_kp * pll_kp;
    pll_snap_thresh = 0.5f * dt_s * pll_ki;

    memset(enc, 0, sizeof(enc));

    /* Start from wherever the counters are now; no need to reset them */
    last_cnt_L = htim5.Instance->CNT;
    last_cnt_R = (uint16_t)htim3.Instance->CNT;

    return stable;
}

void Encoder_Update(void)
{
    /* 1. Sample both counters first, back to back (a few bus cycles apart),
          so left and right are taken at the same instant every period. */
    uint32_t cnt_L = htim5.Instance->CNT;
    uint16_t cnt_R = (uint16_t)htim3.Instance->CNT;

    /* 2. Counts moved since the last sample. Unsigned subtraction then a
          signed cast gives the right answer across wrap-around in either
          direction, as long as a wheel moves less than half the counter
          range per period (TIM3: 32767 counts = 8 rev in one period).
          ODrive's encoder.cpp uses the same signed 16-bit difference. */
    int32_t d_L = (int32_t)(cnt_L - last_cnt_L);
    int32_t d_R = (int32_t)(int16_t)(uint16_t)(cnt_R - last_cnt_R);
    last_cnt_L = cnt_L;
    last_cnt_R = cnt_R;

    /* 3. Apply wheel direction and accumulate into wrap-free 32-bit positions
          (4000 counts/rev -> over 500 000 revolutions before int32 overflow). */
    enc[ENC_LEFT].delta  = ENC_SIGN_L * d_L;
    enc[ENC_RIGHT].delta = ENC_SIGN_R * d_R;

    for (uint32_t i = 0; i < ENC_COUNT; i++) {
        Encoder_t *e = &enc[i];

        e->position          += e->delta;
        e->angle_rad          = (float)e->position * rad_per_count;
        e->velocity_raw_rad_s = (float)e->delta    * rad_s_per_count;

        pll_update(e, e->position);
        e->velocity_rad_s     = e->pll_vel_counts_s * rad_per_count;
    }
}
