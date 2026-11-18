/**
 * @file    Balance_AIgen.c
 * @brief   Runs the MATLAB-generated balance controller (Subsystem1) on the
 *          robot: sensors in, stepper speeds out, start / stop and safety.
 *
 * PROVENANCE: AI-generated. Written by Claude (Anthropic), 17 Nov 2026, for
 * Gene Connell's MCHA3500 balancing robot. Not copied from any outside
 * source. How to use it, the sign conventions and the settings are in
 * Balance_AIgen.h.
 *
 * Every 2 ms (Balance_Update(), straight after IMU_Update()):
 *   y = [phi; theta; theta_dot]   encoders and IMU, units the model uses
 *   Subsystem1_step()             Kalman filter + LQR + stepper limits,
 *                                 exactly the blocks simulated in Simulink
 *   omega = Subsystem1_Y.omega    -> STEP frequency and DIR pin of each motor
 *
 * Stepper speed. Each motor's STEP pin is a PWM channel (TIM8 CH1 left,
 * TIM4 CH1 right, 1 MHz counter from CubeMX). A rising edge is one
 * microstep, so the speed is set by the period:
 *     steps/s = |omega| * 3200 / (2 pi),   ARR = timer tick / (steps/s) - 1
 * 27 rad/s (omegaMax) is ARR ~71; the 16-bit ARR runs out below 0.03 rad/s,
 * which is sent as "no steps". The timers' preload bits are turned off so a
 * new period starts counting at once rather than after the old (possibly
 * 65 ms) one ends; if the counter is already past the new period, the
 * overdue step is sent straight away (update event).
 *
 * Why the Kalman filter state is set at engage: Subsystem1_initialize()
 * starts it at the simulation's xhat0 (theta = 6.5 deg, baked in when the
 * code was generated). On the robot that would start the estimate several
 * degrees away from the real tilt and the first corrections would be wrong.
 * So at engage the filter starts at [0; measured theta; 0] (the robot is
 * held still, so ptheta is ~0), and every other controller state (wheel
 * speed, integrator, rate limiter) starts at 0.
 *
 * Updated by Claude, 18 Nov 2026: moves. The position reference phi_ref is
 * no longer fixed at 0: it comes from Robot_Foward_AIgen.c (rf.phi_ref),
 * which holds it still or moves it along the path of a Robot_Foward() move.
 * Added the feedforward below (BAL_FEEDFORWARD), the phi_ref column at the
 * end of "bal stream", and the run-away check is now against phi_ref.
 * Updated by Claude, 18 Nov 2026 (later): Balance_Start(seconds) balances in
 * place for that long before main.c carries on ('N' = forever). Arming
 * without waiting, for B1 and "bal on", is now bal_arm().
 *
 * Feedforward for moves. The generated controller only has a position
 * reference:
 *     accel = -K [phi - phi_ref; theta; ptheta; omega; z]
 * Driving at a steady speed looks like an error to it, because omega and
 * ptheta are not zero, so it holds the robot back: it trails phi_ref by
 * about 0.6 s. A controller that is told the whole reference would use
 *     accel = alpha_ref - K [phi - phi_ref; theta - theta_ref;
 *                            ptheta - ptheta_ref; omega - omega_ref; z]
 *     theta_ref  = (M21 / mgl) alpha_ref   the lean that accelerates the
 *                                          robot steadily at alpha_ref
 *     ptheta_ref = M21 omega_ref           tilt momentum when rolling upright
 * The extra terms are one number added to accel each sample. Moving phi_ref
 * ahead by that number / K_phi (the "lead") adds exactly the same, without
 * touching the generated code:
 *     lead = (K_theta theta_ref + K_p ptheta_ref + K_omega omega_ref
 *             + alpha_ref) / K_phi
 * The controller is given phi_ref + lead. Its integrator z = integral of
 * (phi - phi_ref) must keep integrating the real position error, so the
 * lead it was given is added back to z after the step. The gains are read
 * from the generated code (Subsystem1_ConstP.LQRgain_Gain = -K), so they
 * follow a regenerated controller. M21 and mgl are in Balance_AIgen.h.
 */
#include "Robot.h"          /* HAL, htim2/4/8, imu_data, enc[], Motor_*(), tmc_ok[],
                               TMC2240 fields, Lab5_printf(), LAB5_PRINTF_MAX */
#include "SensorLog.h"      /* SensorLog_Busy(): SensorLog takes the motors over */
#include "Balance_AIgen.h"
#include "Subsystem1.h"     /* generated: MATLAB_firmware_subsystem/ */
#include "Robot_Foward_AIgen.h"   /* rf: position reference, still or a move */
#include <math.h>
#include <stdlib.h>
#include <string.h>

#if (IMU_SAMPLE_US != 2000U)
#error "Subsystem1 was generated for Ts = 2 ms (robotLQG_setup_AIgen.m). Regenerate it for the new IMU_SAMPLE_US."
#endif

/* ========================================================================= */
/* Constants                                                                 */
/* ========================================================================= */
#define BAL_TS_S             0.002f
#define BAL_TWO_PI           6.28318530718f
#define BAL_RAD_PER_COUNT    (BAL_TWO_PI / ENC_COUNTS_PER_REV)
#define BAL_STEPS_PER_RAD    (BAL_STEPS_PER_REV / BAL_TWO_PI)
#define BAL_ARR_MIN          (2U * BAL_STEP_PULSE_TICKS)  /* STEP high <= half the period */
#define BAL_ARR_MAX          65535U                       /* TIM4 and TIM8 are 16-bit   */
#define BAL_SAMPLES_PER_S    (1000000U / IMU_SAMPLE_US)
#define BAL_BUTTON_SAMPLES   10U                          /* B1 debounce: 20 ms          */

/* LD2 patterns, TIM2 at 10 kHz (ARR, CCR). CCR > ARR = on solid. */
#define BAL_LED_OFF_ARR      4999U   /* CubeMX heartbeat: 0.5 s, 50 %           */
#define BAL_LED_OFF_CCR      2500U
#define BAL_LED_ARMED_ARR    1999U   /* 5 Hz blink                               */
#define BAL_LED_ARMED_CCR    1000U
#define BAL_LED_RUN_ARR      4999U   /* solid                                    */
#define BAL_LED_RUN_CCR      5000U
#define BAL_LED_FALLEN_ARR   9999U   /* short flash once a second                */
#define BAL_LED_FALLEN_CCR   1000U
#define BAL_LED_FAULT_ARR    499U    /* 20 Hz flicker                            */
#define BAL_LED_FAULT_CCR    250U

/* ========================================================================= */
/* Variables                                                                 */
/* ========================================================================= */
Bal_Status bal = { .state = BAL_OFF, .reason = "not started" };

static uint8_t  hw_ready;           /* STEP timers set up for live speed changes */
static float    tick_hz_L;          /* STEP timer counter clock, Hz              */
static float    tick_hz_R;
static int32_t  pos0_L, pos0_R;     /* encoder counts at engage / sign test start */
static uint32_t upright_samples;    /* armed: samples held upright and still     */
static uint32_t imu_err_last;       /* imu_data.i2c_errors at the last sample    */
static uint32_t imu_err_run;        /* failed IMU reads in a row                 */
static uint32_t missed_last;        /* imu_data.missed_periods at the last sample */
static uint32_t stream_every;       /* samples per stream line, 0 = off          */
static uint32_t stream_count;
static uint32_t sign_samples;
static uint8_t  btn_pressed;        /* B1 debounced level, 1 = pressed           */
static uint8_t  btn_count;

static uint8_t bal_arm(void);

static const char *const state_names[] = {
    "OFF", "ARMED", "RUNNING", "FALLEN", "FAULT", "SIGN_TEST"
};

/* ========================================================================= */
/* Small helpers                                                             */
/* ========================================================================= */
/* Lab5_printf() waits when its buffer is full. From inside the 2 ms loop,
   print only if the line is sure to fit, so the controller never waits. */
#define BAL_PRINT_NOWAIT(...)                                   \
    do {                                                        \
        if (Lab5_TxFree() >= LAB5_PRINTF_MAX) {                 \
            Lab5_printf(__VA_ARGS__);                           \
        }                                                       \
    } while (0)

/* Both wheel counts, read together (SysTick updates enc[] at 1 kHz) */
static void bal_read_counts(int32_t *left, int32_t *right)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *left  = enc[ENC_LEFT].position;
    *right = enc[ENC_RIGHT].position;
    __set_PRIMASK(primask);
}

static void bal_led(uint32_t arr, uint32_t ccr)
{
    if ((htim2.Instance->ARR == arr) && (htim2.Instance->CCR1 == ccr)) {
        return;
    }
    __HAL_TIM_SET_AUTORELOAD(&htim2, arr);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ccr);
    /* TIM2 is 32-bit with no ARR preload: restart the count, or a counter
       already past the new ARR would run on for minutes before wrapping */
    HAL_TIM_GenerateEvent(&htim2, TIM_EVENTSOURCE_UPDATE);
}

static void bal_led_for_state(void)
{
    switch (bal.state) {
    case BAL_ARMED:     bal_led(BAL_LED_ARMED_ARR,  BAL_LED_ARMED_CCR);  break;
    case BAL_RUNNING:
    case BAL_SIGN_TEST: bal_led(BAL_LED_RUN_ARR,    BAL_LED_RUN_CCR);    break;
    case BAL_FALLEN:    bal_led(BAL_LED_FALLEN_ARR, BAL_LED_FALLEN_CCR); break;
    case BAL_FAULT:     bal_led(BAL_LED_FAULT_ARR,  BAL_LED_FAULT_CCR);  break;
    case BAL_OFF:
    default:            bal_led(BAL_LED_OFF_ARR,    BAL_LED_OFF_CCR);    break;
    }
}

/* ========================================================================= */
/* Stepper drive                                                             */
/* ========================================================================= */
/* Counter clock of a STEP timer: the APB clock, doubled when that APB is
   divided (RM0390, clock tree), divided by the timer's PSC. TIM8 is on APB2,
   TIM4 on APB1; with this project's clocks both come to 1 MHz. */
static float bal_timer_tick_hz(TIM_HandleTypeDef *htim)
{
    uint32_t pclk;
    uint32_t divided;
    if ((htim->Instance == TIM1) || (htim->Instance == TIM8)) {
        pclk    = HAL_RCC_GetPCLK2Freq();
        divided = RCC->CFGR & RCC_CFGR_PPRE2_2;    /* top bit set = APB2 divided */
    } else {
        pclk    = HAL_RCC_GetPCLK1Freq();
        divided = RCC->CFGR & RCC_CFGR_PPRE1_2;
    }
    uint32_t timclk = divided ? (2U * pclk) : pclk;
    return (float)timclk / (float)(htim->Instance->PSC + 1U);
}

/* Once: stop the STEP timers at a breakpoint (as Motor_test() does) and turn
   off ARR / CCR1 preload, so a speed change applies at once. */
static void bal_hw_setup(void)
{
    __HAL_DBGMCU_FREEZE_TIM4();
    __HAL_DBGMCU_FREEZE_TIM8();

    htim8.Instance->CR1   &= ~TIM_CR1_ARPE;
    htim4.Instance->CR1   &= ~TIM_CR1_ARPE;
    htim8.Instance->CCMR1 &= ~TIM_CCMR1_OC1PE;
    htim4.Instance->CCMR1 &= ~TIM_CCMR1_OC1PE;

    tick_hz_L = bal_timer_tick_hz(&htim8);
    tick_hz_R = bal_timer_tick_hz(&htim4);
    hw_ready  = 1U;
}

/* One wheel: DIR pin, then the STEP period for |omega| */
static void bal_wheel_set(TIM_TypeDef *tim, float tick_hz,
                          GPIO_TypeDef *dir_port, uint16_t dir_pin,
                          float sign, float omega)
{
    float w = sign * omega;
    HAL_GPIO_WritePin(dir_port, dir_pin, (w >= 0.0f) ? GPIO_PIN_RESET : GPIO_PIN_SET);

    float steps_per_s = fabsf(w) * BAL_STEPS_PER_RAD;
    if (!(steps_per_s * (float)(BAL_ARR_MAX + 1U) >= tick_hz)) {
        tim->CCR1 = 0U;                    /* slower than the timer can count (or NaN): no steps */
        return;
    }
    float ticks = tick_hz / steps_per_s;   /* period in counter ticks, <= 65536 here */
    uint32_t arr = (ticks < (float)(BAL_ARR_MIN + 1U)) ? BAL_ARR_MIN
                                                       : (uint32_t)(ticks + 0.5f) - 1U;
    if (arr > BAL_ARR_MAX) { arr = BAL_ARR_MAX; }

    tim->ARR  = arr;                       /* no preload: counts to the new ARR from now */
    tim->CCR1 = BAL_STEP_PULSE_TICKS;
    if (tim->CNT > arr) {
        tim->EGR = TIM_EGR_UG;             /* already past it: this step is due now */
    }
}

static void bal_wheels_set(float omega)
{
    bal_wheel_set(htim8.Instance, tick_hz_L, DIR_L_GPIO_Port, DIR_L_Pin,
                  (float)BAL_MOTOR_SIGN_L, omega);
    bal_wheel_set(htim4.Instance, tick_hz_R, DIR_R_GPIO_Port, DIR_R_Pin,
                  (float)BAL_MOTOR_SIGN_R, omega);
}

/* Configure both TMC2240s again (they lose their settings when the battery
   is switched off) and raise the hold current. Returns 0 if a driver did
   not answer: the TMC2240 needs the battery to talk over SPI. */
static uint8_t bal_drivers_prepare(void)
{
    for (uint16_t id = 0U; id < TMC_COUNT; id++) {
        if (!Motor_DriverInit(id)) {
            return 0U;
        }
        tmc2240_fieldWrite(id, TMC2240_IHOLD_FIELD, BAL_IHOLD_BALANCING);
        if (tmc2240_fieldRead(id, TMC2240_IHOLD_FIELD) != BAL_IHOLD_BALANCING) {
            return 0U;
        }
    }
    return 1U;
}

/* Put the normal hold current back */
static void bal_drivers_release(void)
{
    for (uint16_t id = 0U; id < TMC_COUNT; id++) {
        if (tmc_ok[id]) {
            tmc2240_fieldWrite(id, TMC2240_IHOLD_FIELD, TMC_IHOLD);
        }
    }
}

/* Power stages on, STEP outputs running with no steps yet */
static void bal_motors_on(void)
{
    htim8.Instance->CCR1 = 0U;
    htim4.Instance->CCR1 = 0U;
    if (TIM_CHANNEL_STATE_GET(&htim8, TIM_CHANNEL_1) == HAL_TIM_CHANNEL_STATE_READY) {
        HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);   /* also sets TIM8's MOE */
    }
    if (TIM_CHANNEL_STATE_GET(&htim4, TIM_CHANNEL_1) == HAL_TIM_CHANNEL_STATE_READY) {
        HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    }
    Motor_Enable();                                 /* TMC_EN low */
}

/* No more steps, power stages off (wheels free), STEP pins pulled low the
   same way SensorLog.c's sl_motor_pwm_off() does, so a floating STEP input
   cannot pick up noise */
static void bal_motors_off(void)
{
    htim8.Instance->CCR1 = 0U;
    htim4.Instance->CCR1 = 0U;
    Motor_Disable();                                /* TMC_EN high */
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
    GPIOC->PUPDR = (GPIOC->PUPDR & ~(3U << (6U * 2U))) | (2U << (6U * 2U));   /* PC6 STEP_L */
    GPIOB->PUPDR = (GPIOB->PUPDR & ~(3U << (6U * 2U))) | (2U << (6U * 2U));   /* PB6 STEP_R */
}

/* ========================================================================= */
/* State changes                                                             */
/* ========================================================================= */
/* Stop with a reason. hw = 0 when SensorLog has already taken the motors
   over: then nothing is touched, or its hold would be undone. */
static void bal_stop(Bal_State new_state, const char *reason, uint8_t hw)
{
    Bal_State old = bal.state;
    if (old == BAL_RUNNING) {
        Robot_Foward_BalanceStopped();              /* drops a move under way */
    }
    if (hw) {
        bal_motors_off();
        bal_drivers_release();
    }
    bal.state = new_state;
    bal.reason = reason;
    bal.omega = 0.0f;
    bal_led_for_state();

    if (old == BAL_RUNNING) {
        BAL_PRINT_NOWAIT("bal: stopped after %.2f s: %s (theta %.1f deg, phi %.2f rad, "
                         "slip %.3f rad, loop %.0f us max)\n",
                         (double)((float)bal.run_samples * BAL_TS_S), reason,
                         (double)(bal.theta * RAD_TO_DEG), (double)bal.phi,
                         (double)bal.slip, (double)bal.exec_us_max);
    } else {
        BAL_PRINT_NOWAIT("bal: %s: %s\n", state_names[new_state], reason);
    }
}

/* Robot held upright and still: start balancing */
static void bal_engage(void)
{
    bal_read_counts(&pos0_L, &pos0_R);

    /* Generated controller: every state to 0, then the Kalman filter at the
       measured tilt (see the top of this file) */
    memset(&Subsystem1_DW, 0, sizeof(Subsystem1_DW));
    memset(&Subsystem1_U,  0, sizeof(Subsystem1_U));
    memset(&Subsystem1_Y,  0, sizeof(Subsystem1_Y));
    Subsystem1_initialize();
    Subsystem1_DW.Kalmanfilter_DSTATE[0] = 0.0;                    /* phi    */
    Subsystem1_DW.Kalmanfilter_DSTATE[1] = (double)imu_data.theta; /* theta  */
    Subsystem1_DW.Kalmanfilter_DSTATE[2] = 0.0;                    /* ptheta */

    Robot_Foward_Engage();                          /* phi_ref = 0 = this spot */
    bal.phi = 0.0f;
    bal.phi_ref = 0.0f;
    bal.omega = 0.0f;
    bal.z = 0.0f;
    bal.phi_cmd = 0.0f;
    bal.slip = 0.0f;
    bal.run_samples = 0U;
    bal.late_samples = 0U;
    bal.exec_us_max = 0.0f;
    bal.engage_count++;
    missed_last = imu_data.missed_periods;

    bal_motors_on();
    bal.state = BAL_RUNNING;
    bal.reason = "running";
    bal_led_for_state();
    BAL_PRINT_NOWAIT("bal: engaged at theta %.2f deg. Let go gently.\n",
                     (double)(imu_data.theta * RAD_TO_DEG));
}

/* How far to move phi_ref ahead so the controller also follows the speed
   and acceleration of a move (see "Feedforward for moves" at the top).
   0 when no move is running. */
static float bal_feedforward_lead(void)
{
#if BAL_FEEDFORWARD
    const double *g = Subsystem1_ConstP.LQRgain_Gain;      /* -K: phi, theta, ptheta, omega, z */
    if (g[0] == 0.0) {
        return 0.0f;
    }
    float theta_ref  = (BAL_FF_M21 / BAL_FF_MGL) * rf.alpha_ref;
    float ptheta_ref = BAL_FF_M21 * rf.omega_ref;
    return (float)((g[1] * (double)theta_ref + g[2] * (double)ptheta_ref
                    + g[3] * (double)rf.omega_ref - (double)rf.alpha_ref) / g[0]);
#else
    return 0.0f;
#endif
}

/* One controller step while balancing */
static void bal_run_step(void)
{
    int32_t cl, cr;
    bal_read_counts(&cl, &cr);
    bal.phi = 0.5f * (float)((cl - pos0_L) + (cr - pos0_R)) * BAL_RAD_PER_COUNT;

    /* Where the wheels should be this sample: a fixed spot, or the path of a
       Robot_Foward() move (Robot_Foward_AIgen.c) */
    Robot_Foward_Update(bal.run_samples);
    bal.phi_ref = rf.phi_ref;

    if (fabsf(bal.theta) > BAL_FALL_DEG * DEG_TO_RAD) {
        bal_stop(BAL_FALLEN, "fell over (|theta| > BAL_FALL_DEG)", 1U);
        return;
    }
    if (fabsf(bal.phi - bal.phi_ref) > BAL_PHI_LIMIT_RAD) {
        bal_stop(BAL_FALLEN, "ran away (|phi - phi_ref| > BAL_PHI_LIMIT_RAD)", 1U);
        return;
    }
    if (imu_err_run >= BAL_IMU_FAIL_SAMPLES) {
        bal_stop(BAL_FAULT, "IMU reads failing (I2C3 wiring?)", 1U);
        return;
    }
    if (imu_data.missed_periods != missed_last) {
        bal.late_samples += imu_data.missed_periods - missed_last;
        missed_last = imu_data.missed_periods;
    }

    /* The generated controller: y in the model's order [phi; theta; dtheta] */
    float lead = bal_feedforward_lead();
    Subsystem1_U.phi_ref = (double)bal.phi_ref + (double)lead;
    Subsystem1_U.y[0] = (double)bal.phi;
    Subsystem1_U.y[1] = (double)bal.theta;
    Subsystem1_U.y[2] = (double)bal.theta_dot;
    Subsystem1_step();
    /* The integrator has just added (phi - phi_ref - lead) x Ts: give the
       lead back, so z stays the integral of the real position error */
    Subsystem1_DW.Integralofphierror_DSTATE += (double)lead * (double)BAL_TS_S;

    bal.omega = (float)Subsystem1_Y.omega;
    if (!isfinite(bal.omega)) {
        bal_stop(BAL_FAULT, "controller output not a number", 1U);
        return;
    }
    bal_wheels_set(bal.omega);

    bal.xhat[0] = (float)Subsystem1_Y.xhat[0];
    bal.xhat[1] = (float)Subsystem1_Y.xhat[1];
    bal.xhat[2] = (float)Subsystem1_Y.xhat[2];
    bal.z       = (float)Subsystem1_Y.z;
    bal.phi_cmd += bal.omega * BAL_TS_S;
    bal.slip    = bal.phi_cmd - bal.phi;
    bal.run_samples++;
}

/* "bal sign": both wheels forwards at BAL_SIGN_OMEGA_RAD_S, then report */
static void bal_sign_start(void)
{
    if ((bal.state == BAL_RUNNING) || (bal.state == BAL_ARMED) || (bal.state == BAL_SIGN_TEST)) {
        Lab5_printf("bal sign: stop first (bal off)\n");
        return;
    }
    if (SensorLog_Busy()) {
        Lab5_printf("bal sign: a SensorLog run is in progress\n");
        return;
    }
    if (!hw_ready) {
        bal_hw_setup();
    }
    if (!bal_drivers_prepare()) {
        bal_drivers_release();
        bal.state = BAL_FAULT;
        bal.reason = "TMC2240 not answering (battery on?)";
        bal_led_for_state();
        Lab5_printf("bal sign: %s. tmc_ok L %u R %u, version L 0x%02X R 0x%02X\n", bal.reason,
                    (unsigned)tmc_ok[TMC_LEFT], (unsigned)tmc_ok[TMC_RIGHT],
                    (unsigned)tmc_version[TMC_LEFT], (unsigned)tmc_version[TMC_RIGHT]);
        return;
    }
    Lab5_printf("bal sign: both wheels forwards at %.1f rad/s for %.1f s. "
                "Wheels must be OFF the ground.\n",
                (double)BAL_SIGN_OMEGA_RAD_S, (double)BAL_SIGN_TIME_MS / 1000.0);
    bal_read_counts(&pos0_L, &pos0_R);
    sign_samples = 0U;
    bal_motors_on();
    bal_wheels_set(BAL_SIGN_OMEGA_RAD_S);
    bal.state = BAL_SIGN_TEST;
    bal.reason = "sign test";
    bal_led_for_state();
}

static void bal_sign_report_wheel(const char *name, float turned, float expected,
                                  const char *sign_macro)
{
    if (turned > 0.5f * expected) {
        Lab5_printf("  %s: %+.2f rad (expected %+.2f): OK%s\n", name, (double)turned,
                    (double)expected,
                    (fabsf(turned - expected) > 0.2f * expected) ? ", but not the full "
                    "amount: missed steps?" : "");
    } else if (turned < -0.5f * expected) {
        Lab5_printf("  %s: %+.2f rad (expected %+.2f): REVERSED. Flip %s in Balance_AIgen.h\n",
                    name, (double)turned, (double)expected, sign_macro);
    } else {
        Lab5_printf("  %s: %+.2f rad (expected %+.2f): did not turn. Driver off, "
                    "battery low or wheel jammed?\n", name, (double)turned, (double)expected);
    }
}

static void bal_sign_step(void)
{
    if (++sign_samples < (BAL_SIGN_TIME_MS * BAL_SAMPLES_PER_S) / 1000U) {
        return;
    }
    bal_motors_off();
    bal_drivers_release();
    bal.state = BAL_OFF;
    bal.reason = "sign test done";
    bal_led_for_state();

    int32_t cl, cr;
    bal_read_counts(&cl, &cr);
    float expected = BAL_SIGN_OMEGA_RAD_S * (float)BAL_SIGN_TIME_MS / 1000.0f;
    Lab5_printf("bal sign: encoder turns (+ = forwards, ENC_SIGN_x in Encoder.h):\n");
    bal_sign_report_wheel("left ", (float)(cl - pos0_L) * BAL_RAD_PER_COUNT, expected,
                          "BAL_MOTOR_SIGN_L");
    bal_sign_report_wheel("right", (float)(cr - pos0_R) * BAL_RAD_PER_COUNT, expected,
                          "BAL_MOTOR_SIGN_R");
    Lab5_printf("  Also check by eye: both wheels turned the way that rolls the robot "
                "forwards (+y, away from the fuse).\n");
}

/* B1 (blue user button, PC13, low when pressed): start or stop */
static void bal_button_poll(void)
{
    uint8_t pressed = (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
    if (pressed == btn_pressed) {
        btn_count = 0U;
        return;
    }
    if (++btn_count < BAL_BUTTON_SAMPLES) {
        return;
    }
    btn_count = 0U;
    btn_pressed = pressed;
    if (!pressed) {
        return;                                     /* act on the press only */
    }
    if ((bal.state == BAL_ARMED) || (bal.state == BAL_RUNNING) || (bal.state == BAL_SIGN_TEST)) {
        bal_stop(BAL_OFF, "stopped by B1", 1U);
    } else {
        (void)bal_arm();
    }
}

/* ",-12.3456" into p with integer maths: newlib's float printf can take
   hundreds of us per number, too slow for 11 numbers inside the 2 ms loop */
static char *bal_put_fixed(char *p, float v, uint32_t decimals)
{
    static const uint32_t scale[] = {1U, 10U, 100U, 1000U, 10000U, 100000U};
    if (!isfinite(v)) {
        v = 0.0f;
    }
    *p++ = ',';
    if (v < 0.0f) {
        *p++ = '-';
        v = -v;
    }
    float vmax = 4.0e9f / (float)scale[decimals];   /* keep v * scale inside uint32 */
    if (v > vmax) {
        v = vmax;
    }
    uint32_t n = (uint32_t)(v * (float)scale[decimals] + 0.5f);
    uint32_t whole = n / scale[decimals];
    uint32_t frac  = n % scale[decimals];
    char tmp[12];
    int  len = 0;
    do { tmp[len++] = (char)('0' + (whole % 10U)); whole /= 10U; } while (whole != 0U);
    while (len > 0) { *p++ = tmp[--len]; }
    if (decimals > 0U) {
        *p++ = '.';
        for (uint32_t d = decimals; d > 0U; d--) {
            *p++ = (char)('0' + (frac / scale[d - 1U]) % 10U);
        }
    }
    return p;
}

static void bal_stream_line(void)
{
    if ((stream_every == 0U) || (++stream_count < stream_every)) {
        return;
    }
    stream_count = 0U;
    char line[160];
    char *p = line + snprintf(line, 24, "%lu,%d", (unsigned long)HAL_GetTick(), (int)bal.state);
    p = bal_put_fixed(p, bal.theta,     5U);
    p = bal_put_fixed(p, bal.theta_dot, 4U);
    p = bal_put_fixed(p, bal.phi,       4U);
    p = bal_put_fixed(p, bal.omega,     3U);
    p = bal_put_fixed(p, bal.xhat[0],   4U);
    p = bal_put_fixed(p, bal.xhat[1],   5U);
    p = bal_put_fixed(p, bal.xhat[2],   5U);
    p = bal_put_fixed(p, bal.z,         4U);
    p = bal_put_fixed(p, bal.slip,      4U);
    p = bal_put_fixed(p, bal.phi_ref,   4U);
    *p++ = '\n';
    *p   = '\0';
    BAL_PRINT_NOWAIT("%s", line);
}

/* ========================================================================= */
/* Public                                                                    */
/* ========================================================================= */
/* Arm the controller and return at once (B1, "bal on" and Balance_Start()).
   Returns 1 if it is armed or already balancing, 0 if it could not arm. */
static uint8_t bal_arm(void)
{
    if ((bal.state == BAL_ARMED) || (bal.state == BAL_RUNNING)) {
        Lab5_printf("bal: already %s\n", state_names[bal.state]);
        return 1U;
    }
    if (bal.state == BAL_SIGN_TEST) {
        Lab5_printf("bal: sign test running\n");
        return 0U;
    }
    if (SensorLog_Busy()) {
        Lab5_printf("bal: not armed, a SensorLog run is in progress\n");
        return 0U;
    }
    if (!hw_ready) {
        bal_hw_setup();
    }
    if (!imu_data.imu_ok) {
        bal.state = BAL_FAULT;
        bal.reason = "MPU-6050 not answering (imu_ok = 0)";
        bal_led_for_state();
        Lab5_printf("bal: not armed: %s\n", bal.reason);
        return 0U;
    }
    if (!bal_drivers_prepare()) {
        bal_drivers_release();
        bal.state = BAL_FAULT;
        bal.reason = "TMC2240 not answering (battery on?)";
        bal_led_for_state();
        Lab5_printf("bal: not armed: %s. tmc_ok L %u R %u, version L 0x%02X R 0x%02X\n",
                    bal.reason, (unsigned)tmc_ok[TMC_LEFT], (unsigned)tmc_ok[TMC_RIGHT],
                    (unsigned)tmc_version[TMC_LEFT], (unsigned)tmc_version[TMC_RIGHT]);
        return 0U;
    }

    upright_samples = 0U;
    imu_err_run = 0U;
    imu_err_last = imu_data.i2c_errors;
    bal.state = BAL_ARMED;
    bal.reason = "armed";
    bal_led_for_state();
    Lab5_printf("bal: armed. Hold the robot upright (|theta| < %.0f deg) and still for "
                "%.1f s to start. B1 or \"bal off\" stops.\n",
                (double)BAL_ENGAGE_DEG, (double)BAL_ENGAGE_HOLD_MS / 1000.0);
    return 1U;
}

/* One 2 ms sample: the same steps, in the same order, as the while(1) loop
   in main.c (keep the two the same). Used while Balance_Start() waits. */
static void bal_one_sample(uint32_t *print_count)
{
    IMU_WaitForNextSample();
    IMU_Update();
    Balance_Update();
    SensorLog_Update();
    if (!SensorLog_Busy()) {
        Lab5_Process();
    }
    if (++(*print_count) >= DATA_PRINT_SAMPLES) {
        *print_count = 0U;
        if (!SensorLog_Busy() && !Balance_Active()) {
            Data_Print();
        }
    }
}

/*
 * Balance_Start( seconds )
 *
 *   seconds   how long to balance in place before carrying on to the next
 *             line of main.c, e.g. Balance_Start(3.0f).
 *             'N' = balance forever: Balance_Start('N') never returns.
 *
 *   Arms the controller (the TMC2240s are set up: battery on), then runs
 *   the normal 2 ms loop itself until the robot has been balancing in place
 *   for that long. The time counts only while it is balancing with no
 *   Robot_Foward() move running:
 *     - first hold it upright and still for 0.5 s so it engages;
 *     - a move still running is finished first, so in main.c
 *           Balance_Start(3); Robot_Foward(...); Balance_Start(2); ...
 *       runs one step after the other;
 *     - if it falls or is stopped, the count starts again once B1 (or
 *       "bal on") re-arms it and it is balancing again.
 *   After it returns the robot keeps balancing (main.c's while(1) loop
 *   carries on with it). Serial commands work while it waits.
 *   78 s exactly cannot be asked for: 78 is the code of the character 'N'
 *   (use 78.5f). 0 = return as soon as it has engaged.
 */
void Balance_Start(float seconds)
{
    static uint8_t inside;
    if (inside) {
        Lab5_printf("bal: Balance_Start() is already waiting; it cannot be called again from inside\n");
        return;
    }
    inside = 1U;

    uint8_t forever = ((seconds == BAL_FOREVER) || (seconds == (float)'n')) ? 1U : 0U;
    if (!(seconds > 0.0f)) {
        seconds = 0.0f;                                  /* negative or NaN */
    }
    uint32_t need = forever ? 0xFFFFFFFFU : (uint32_t)(seconds * (float)BAL_SAMPLES_PER_S + 0.5f);

    if (bal.state != BAL_RUNNING) {
        if (!bal_arm()) {
            Lab5_printf("bal: Balance_Start() is waiting. Fix that, then press B1 (or type bal on).\n");
        }
    }
    if (forever) {
        Lab5_printf("bal: Balance_Start('N'): balancing until reset\n");
    } else {
        Lab5_printf("bal: Balance_Start(%lu ms): balances in place that long, then goes on to the "
                    "next line of main.c\n", (unsigned long)(need * (IMU_SAMPLE_US / 1000U)));
    }

    uint32_t count = 0U;
    uint32_t print_count = 0U;
    uint8_t  was_running = (bal.state == BAL_RUNNING) ? 1U : 0U;
    for (;;) {
        if ((bal.state == BAL_RUNNING) && !Robot_Foward_Busy() && (count >= need)) {
            break;
        }
        bal_one_sample(&print_count);
        if (bal.state == BAL_RUNNING) {
            was_running = 1U;
            if (!Robot_Foward_Busy()) {
                count++;                                 /* balancing in place */
            }
        } else {
            if (was_running) {
                BAL_PRINT_NOWAIT("bal: Balance_Start() is still waiting: press B1 (or type bal on) "
                                 "to try again. Its count starts again.\n");
            }
            was_running = 0U;
            count = 0U;
        }
    }
    inside = 0U;
}

void Balance_Stop(void)
{
    if ((bal.state == BAL_ARMED) || (bal.state == BAL_RUNNING) || (bal.state == BAL_SIGN_TEST)) {
        bal_stop(BAL_OFF, "stopped (bal off)", 1U);
    } else {
        Lab5_printf("bal: already stopped (%s)\n", state_names[bal.state]);
    }
}

uint8_t Balance_Active(void)
{
    return ((bal.state == BAL_RUNNING) || (bal.state == BAL_SIGN_TEST)) ? 1U : 0U;
}

void Balance_Update(void)
{
    uint32_t t0 = DWT->CYCCNT;

    /* Measurements from this sample's IMU_Update() */
    bal.theta     = imu_data.theta;
    bal.theta_dot = imu_data.theta_dot;
    if (imu_data.i2c_errors != imu_err_last) {
        imu_err_run++;
        imu_err_last = imu_data.i2c_errors;
    } else {
        imu_err_run = 0U;
    }

    bal_button_poll();

    /* SensorLog's logAll / logFall switch the motors over to their own use */
    if (SensorLog_Busy() &&
        ((bal.state == BAL_ARMED) || (bal.state == BAL_RUNNING) || (bal.state == BAL_SIGN_TEST))) {
        bal_stop(BAL_OFF, "a SensorLog run took over the motors", 0U);
    }

    switch (bal.state) {
    case BAL_ARMED:
        if ((fabsf(bal.theta) < BAL_ENGAGE_DEG * DEG_TO_RAD) &&
            (fabsf(bal.theta_dot) < BAL_ENGAGE_RATE_DEG_S * DEG_TO_RAD) &&
            (imu_err_run == 0U)) {
            if (++upright_samples >= (BAL_ENGAGE_HOLD_MS * BAL_SAMPLES_PER_S) / 1000U) {
                bal_engage();
            }
        } else {
            upright_samples = 0U;
        }
        break;
    case BAL_RUNNING:
        bal_run_step();
        break;
    case BAL_SIGN_TEST:
        bal_sign_step();
        break;
    default:
        break;
    }

    bal_stream_line();

    float us = (float)(DWT->CYCCNT - t0) / ((float)SystemCoreClock * 1.0e-6f);
    bal.exec_us = us;
    if ((bal.state == BAL_RUNNING) && (us > bal.exec_us_max)) {
        bal.exec_us_max = us;
    }
}

/* ========================================================================= */
/* Serial command "bal" (Lab5.c's command table)                             */
/* ========================================================================= */
static void bal_print_status(void)
{
    Lab5_printf("bal: %s (%s), engaged %lu times\n", state_names[bal.state],
                bal.reason, (unsigned long)bal.engage_count);
    Lab5_printf("  theta %.2f deg, theta_dot %.1f deg/s, phi %.3f rad (wheels L %.3f R %.3f rad "
                "since reset)\n",
                (double)(imu_data.theta * RAD_TO_DEG), (double)(imu_data.theta_dot * RAD_TO_DEG),
                (double)bal.phi, (double)enc[ENC_LEFT].angle_rad, (double)enc[ENC_RIGHT].angle_rad);
    Lab5_printf("  omega %.2f rad/s, xhat [%.3f rad, %.2f deg, %.4f], z %.3f, slip %.3f rad, "
                "phi_ref %.3f rad\n",
                (double)bal.omega, (double)bal.xhat[0], (double)(bal.xhat[1] * RAD_TO_DEG),
                (double)bal.xhat[2], (double)bal.z, (double)bal.slip, (double)bal.phi_ref);
    Lab5_printf("  ran %.2f s, late samples %lu, loop %.0f us (max %.0f), IMU errors %lu, "
                "tmc_ok L %u R %u\n",
                (double)((float)bal.run_samples * BAL_TS_S), (unsigned long)bal.late_samples,
                (double)bal.exec_us, (double)bal.exec_us_max,
                (unsigned long)imu_data.i2c_errors,
                (unsigned)tmc_ok[TMC_LEFT], (unsigned)tmc_ok[TMC_RIGHT]);
}

void Balance_Cmd(int argc, char *argv[])
{
    if (argc < 2) {
        bal_print_status();
    } else if (strcmp(argv[1], "on") == 0) {
        (void)bal_arm();
    } else if (strcmp(argv[1], "off") == 0) {
        Balance_Stop();
    } else if (strcmp(argv[1], "sign") == 0) {
        bal_sign_start();
    } else if (strcmp(argv[1], "stream") == 0) {
        int hz = (argc >= 3) ? atoi(argv[2]) : 0;
        if (hz <= 0) {
            stream_every = 0U;
            Lab5_printf("bal: stream off\n");
            return;
        }
        if (hz > (int)BAL_STREAM_MAX_HZ) {
            hz = (int)BAL_STREAM_MAX_HZ;
        }
        stream_count = 0U;
        stream_every = BAL_SAMPLES_PER_S / (uint32_t)hz;
        Lab5_printf("#BAL t_ms,state,theta_rad,dtheta_rad_s,phi_rad,omega_rad_s,"
                    "xhat_phi,xhat_theta,xhat_ptheta,z,slip_rad,phi_ref_rad (%u Hz)\n",
                    (unsigned)(BAL_SAMPLES_PER_S / stream_every));
    } else {
        Lab5_printf("usage: bal [on|off|sign|stream Hz]   (bal alone = status)\n");
    }
}
