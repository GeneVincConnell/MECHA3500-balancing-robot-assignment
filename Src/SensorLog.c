/**
 * @file    SensorLog.c
 * @brief   Encoder + IMU recording (timed or fall-triggered), sent as CSV for
 *          MATLAB. See SensorLog.h.
 *
 * PROVENANCE: AI-generated. Written by Claude (Anthropic), 14 Nov 2026.
 * Nothing downloaded or copied. The USART2 interrupt driver follows the same
 * design as Claude's Lab5.c for this course (ring buffers, register level),
 * rewritten here so the robot firmware does not need Lab5.c. The hold
 * current is set with the TMC-API field functions (TMC2240.h, Analog
 * Devices), the same way Motor_test.c does.
 * Updated by Claude, 14 Nov 2026: logFall (pre/post-trigger capture with the
 * motors holding), #INFO line, read-error and timing-gap counts, negative
 * times before the trigger.
 *
 * States
 *   IDLE        reads serial commands
 *   ARMED       logAll / logFall has calibrated the gyro; the next
 *               SensorLog_Update() takes the first sample
 *   RECORDING   logAll: one sample per SensorLog_Update() until [seconds] is
 *               up or the buffer is full
 *   FALL_WAIT   logFall: records into the buffer as a ring (the oldest sample
 *               is overwritten) until the tilt has moved trigger_deg
 *   FALL_POST   logFall: records post_s more after the trigger
 *   SEND_*      queue #INFO, #CSV_BEGIN, the header, the rows and #CSV_END,
 *               only as much each pass as the TX buffer has room for
 */
#include "Robot.h"       /* HAL, htim4/htim8, enc[] (Encoder.h), imu_data, IMU_*(),
                            Motor_*(), tmc_ok[], TMC2240.h (TMC-API) */
#include "SensorLog.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ========================================================================= */
/* Private settings                                                          */
/* ========================================================================= */
#define SL_TX_BUF_SIZE      2048U    /* bytes, power of 2                     */
#define SL_RX_BUF_SIZE      64U      /* bytes, power of 2                     */
#define SL_LINE_MAX         200U     /* longest line sl_printf() sends        */
#define SL_CMD_MAX          32U      /* longest command line, incl. '\0'      */
#define SL_CMD_MAX_ARGS     4U
#define SL_ACC_LSB_PER_G    16384U   /* +/-2 g, the library's setting         */

/* NVIC_PRIORITYGROUP_0 (stm32f4xx_hal_msp.c): no pre-emption, so USART2
   never interrupts SysTick's Encoder_Update(); this only sets the order
   when both are pending. SysTick is 0. */
#define SL_USART2_SUBPRIORITY  2U

#define SL_CSV_HEADER   "time_s,enc_L_counts,enc_R_counts," \
                        "acc_x_raw,acc_y_raw,acc_z_raw,"    \
                        "gyro_x_raw,gyro_y_raw,gyro_z_raw," \
                        "theta_rad,theta_dot_rad_s"

/* ========================================================================= */
/* Data                                                                      */
/* ========================================================================= */
typedef struct {
    uint32_t t_us;        /* us since the run was armed           */
    int32_t  enc_L;       /* enc[].position, counts               */
    int32_t  enc_R;
    int16_t  acc[3];      /* raw, 16384 per g                     */
    int16_t  gyro[3];     /* raw, gyro_lsb_per_dps per deg/s, bias not removed */
    float    theta;       /* rad                                  */
    float    theta_dot;   /* rad/s                                */
} SensorLogSample_t;      /* 32 bytes                             */

typedef enum {
    SL_IDLE = 0,
    SL_ARMED,
    SL_RECORDING,
    SL_FALL_WAIT,
    SL_FALL_POST,
    SL_SEND_INFO,         /* every state from here on is sending */
    SL_SEND_BEGIN,
    SL_SEND_HEADER,
    SL_SEND_ROWS,
    SL_SEND_END
} SensorLogState_t;

/* Sample k of a run (k = 0, 1, 2, ...) is kept in sl_buf[k % SENSORLOG_MAX_SAMPLES] */
static SensorLogSample_t sl_buf[SENSORLOG_MAX_SAMPLES];
static SensorLogState_t  sl_state = SL_IDLE;
static uint8_t           sl_fall;             /* 1 = logFall run, 0 = logAll */
static uint32_t          sl_count;            /* samples taken since arming  */

/* Run settings */
static uint32_t          sl_duration_us;      /* logAll run length           */
static uint32_t          sl_pre_us;           /* logFall                     */
static uint32_t          sl_post_us;
static float             sl_trigger_rad;
static uint32_t          sl_timeout_us;

/* Run state */
static uint32_t          sl_cycles_per_us;    /* 84 at 84 MHz                */
static uint32_t          sl_last_cycles;      /* DWT at the last sample      */
static uint64_t          sl_elapsed_cycles;   /* since the first sample      */
static uint32_t          sl_trig_t_us;        /* logFall: trigger sample time */
static float             sl_theta_arm;        /* tilt at the first sample    */
static uint8_t           sl_buffer_full;
static uint32_t          sl_pwm_restarted;    /* times something restarted the PWM */
static float             sl_gyro_bias_dps;
static uint32_t          sl_i2c_errors0;      /* imu_data.i2c_errors at the first sample */
static uint32_t          sl_i2c_errors_run;   /* failed IMU reads during the run */

/* Sending: samples sl_first .. sl_first + sl_nsend - 1 */
static uint32_t          sl_first;
static uint32_t          sl_nsend;
static uint32_t          sl_sent;             /* rows queued so far          */
static uint32_t          sl_t_ref_us;         /* time 0 of the CSV           */
static int32_t           sl_enc0_L;           /* positions of the first row  */
static int32_t           sl_enc0_R;
static uint32_t          sl_prev_t_us;
static uint32_t          sl_max_gap_us;

/* Motor hold (logFall) */
static uint8_t           sl_hold_active;          /* IHOLD raised, drivers on   */
static uint8_t           sl_hold_restore_disable; /* drivers were off before    */

/* ========================================================================= */
/* Serial: USART2 interrupt driver                                           */
/* ========================================================================= */
/* Ring buffers. Each index has one writer, so no locking is needed: main
   writes sl_tx_head and sl_rx_tail, the interrupt writes sl_tx_tail and
   sl_rx_head. */
static volatile uint8_t  sl_tx_buf[SL_TX_BUF_SIZE];
static volatile uint16_t sl_tx_head = 0U;
static volatile uint16_t sl_tx_tail = 0U;
static volatile uint8_t  sl_rx_buf[SL_RX_BUF_SIZE];
static volatile uint16_t sl_rx_head = 0U;
static volatile uint16_t sl_rx_tail = 0U;

/* Free bytes in the TX buffer (one slot is always kept empty) */
static uint16_t sl_tx_free(void)
{
    return (uint16_t)((sl_tx_tail - sl_tx_head - 1U) & (SL_TX_BUF_SIZE - 1U));
}

/* Queue one character. If the buffer is full, wait for the interrupt to
   send some (main context only). */
static void sl_putc(char c)
{
    uint16_t next = (uint16_t)((sl_tx_head + 1U) & (SL_TX_BUF_SIZE - 1U));
    while (next == sl_tx_tail) {
        ATOMIC_SET_BIT(USART2->CR1, USART_CR1_TXEIE);
    }
    sl_tx_buf[sl_tx_head] = (uint8_t)c;
    sl_tx_head = next;
}

/* printf to the ST-LINK COM port. Main context only. Lines longer than
   SL_LINE_MAX - 1 characters are cut short. */
static int sl_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static int sl_printf(const char *fmt, ...)
{
    char    buf[SL_LINE_MAX];
    va_list ap;

    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) {
        return n;
    }

    size_t len = ((size_t)n < sizeof(buf)) ? (size_t)n : (sizeof(buf) - 1U);
    for (size_t i = 0; i < len; i++) {
        sl_putc(buf[i]);
    }
    /* TXE interrupt on. ATOMIC_SET_BIT (CMSIS) because the interrupt clears
       TXEIE in the same register. */
    ATOMIC_SET_BIT(USART2->CR1, USART_CR1_TXEIE);
    return (int)len;
}

/* Next received character, or -1 if none is waiting */
static int sl_getchar(void)
{
    if (sl_rx_tail == sl_rx_head) {
        return -1;
    }
    uint8_t c = sl_rx_buf[sl_rx_tail];
    sl_rx_tail = (uint16_t)((sl_rx_tail + 1U) & (SL_RX_BUF_SIZE - 1U));
    return (int)c;
}

/* USART2 interrupt. Not generated by CubeMX (USART2 global interrupt is off
   in the .ioc), so it is defined here. */
void USART2_IRQHandler(void)
{
    uint32_t sr = USART2->SR;

    /* Receive. Reading SR then DR also clears the error flags (ORE, NE, FE,
       PE), so an overrun or framing error cannot leave the interrupt stuck. */
    if (sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) {
        uint8_t c = (uint8_t)USART2->DR;
        if ((sr & USART_SR_RXNE) && !(sr & (USART_SR_NE | USART_SR_FE | USART_SR_PE))) {
            uint16_t next = (uint16_t)((sl_rx_head + 1U) & (SL_RX_BUF_SIZE - 1U));
            if (next != sl_rx_tail) {             /* full: drop the character */
                sl_rx_buf[sl_rx_head] = c;
                sl_rx_head = next;
            }
        }
    }

    /* Transmit the next queued character, or stop the TXE interrupt when the
       buffer is empty */
    if ((USART2->CR1 & USART_CR1_TXEIE) && (sr & USART_SR_TXE)) {
        if (sl_tx_tail != sl_tx_head) {
            USART2->DR = sl_tx_buf[sl_tx_tail];
            sl_tx_tail = (uint16_t)((sl_tx_tail + 1U) & (SL_TX_BUF_SIZE - 1U));
        } else {
            USART2->CR1 &= ~USART_CR1_TXEIE;
        }
    }
}

/* ========================================================================= */
/* Motors                                                                    */
/* ========================================================================= */
/* Stop the STEP PWM on both motors. HAL_TIM_PWM_Stop() turns the channel
   output off (CC1E = 0; on TIM8 it also clears MOE once no channel is left
   on). With the output off, TIM8's pin is no longer driven (RM0390, advanced
   timer output control with MOE = 0, OSSI = 0), so a pull-down is added on
   both STEP pins: a floating STEP input could pick up driver noise and step
   the motor. The pull-down does nothing while the PWM is running. */
static void sl_motor_pwm_off(void)
{
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);          /* STEP_L, PC6 */
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);          /* STEP_R, PB6 */

    GPIOC->PUPDR = (GPIOC->PUPDR & ~(3U << (6U * 2U))) | (2U << (6U * 2U));   /* PC6 pull-down */
    GPIOB->PUPDR = (GPIOB->PUPDR & ~(3U << (6U * 2U))) | (2U << (6U * 2U));   /* PB6 pull-down */
}

static uint8_t sl_motor_pwm_is_on(void)
{
    return ((htim8.Instance->CCER & TIM_CCER_CC1E) ||
            (htim4.Instance->CCER & TIM_CCER_CC1E)) ? 1U : 0U;
}

/* logFall: both motors hold the wheels still (omega = 0) at the run current.
   With the STEP PWM off, a TMC2240 holds its present microstep position at
   IHOLD; IHOLD is raised to SENSORLOG_FALL_IHOLD for the run. If Motor_test()
   has not configured the drivers, Motor_DriverInit() does it here (needs the
   battery: the TMC2240 does not answer SPI without VS). Enabling the power
   stages makes each rotor snap to its nearest microstep position, so this
   waits SENSORLOG_HOLD_SETTLE_MS before the gyro is calibrated.
   Returns 0 if a driver did not answer or did not take the hold current;
   call sl_motor_hold_off() then. */
static uint8_t sl_motor_hold_on(void)
{
    /* TMC_EN is active low: remember whether the drivers were off before */
    sl_hold_restore_disable =
        (HAL_GPIO_ReadPin(TMC_EN_GPIO_Port, TMC_EN_Pin) == GPIO_PIN_SET) ? 1U : 0U;
    sl_hold_active = 1U;

    for (uint16_t id = 0U; id < TMC_COUNT; id++) {
        if (!tmc_ok[id] && !Motor_DriverInit(id)) {
            return 0U;
        }
        tmc2240_fieldWrite(id, TMC2240_IHOLD_FIELD, SENSORLOG_FALL_IHOLD);
        if (tmc2240_fieldRead(id, TMC2240_IHOLD_FIELD) != SENSORLOG_FALL_IHOLD) {
            return 0U;
        }
    }

    Motor_Enable();                                   /* TMC_EN low */
    HAL_Delay(SENSORLOG_HOLD_SETTLE_MS);
    return 1U;
}

/* Put the hold current and the driver enable back as they were */
static void sl_motor_hold_off(void)
{
    if (!sl_hold_active) {
        return;
    }
    for (uint16_t id = 0U; id < TMC_COUNT; id++) {
        if (tmc_ok[id]) {
            tmc2240_fieldWrite(id, TMC2240_IHOLD_FIELD, TMC_IHOLD);
        }
    }
    if (sl_hold_restore_disable) {
        Motor_Disable();                              /* TMC_EN high */
    }
    sl_hold_active = 0U;
}

/* ========================================================================= */
/* IMU                                                                       */
/* ========================================================================= */
/* Fresh gyro bias and starting angle (~1 s, robot still). If the MPU-6050
   did not answer at start-up, try the whole IMU_Init() again: it checks
   WHO_AM_I first, so it cannot hang. Returns imu_data.imu_ok. */
static uint8_t sl_calibrate_imu(void)
{
    if (imu_data.imu_ok) {
        imu_data.imu_ok = IMU_CalibrateGyro();
    } else {
        (void)IMU_Init();                             /* sets imu_data.imu_ok */
    }
    return imu_data.imu_ok;
}

/* ========================================================================= */
/* Recording                                                                 */
/* ========================================================================= */
/* Arm a run: the next SensorLog_Update() takes sample 0. The DWT cycle
   counter is already running (IMU_Init() starts it); this only makes sure. */
static void sl_arm(uint8_t fall)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;
    sl_cycles_per_us  = SystemCoreClock / 1000000U;
    sl_fall           = fall;
    sl_count          = 0U;
    sl_first          = 0U;
    sl_nsend          = 0U;
    sl_sent           = 0U;
    sl_buffer_full    = 0U;
    sl_pwm_restarted  = 0U;
    sl_state          = SL_ARMED;
}

/* Recording done: send samples first .. first + n - 1, times relative to t_ref_us */
static void sl_start_send(uint32_t first, uint32_t n, uint32_t t_ref_us)
{
    sl_first          = first;
    sl_nsend          = n;
    sl_sent           = 0U;
    sl_t_ref_us       = t_ref_us;
    sl_max_gap_us     = 0U;
    sl_i2c_errors_run = imu_data.i2c_errors - sl_i2c_errors0;
    if (n > 0U) {
        const SensorLogSample_t *s0 = &sl_buf[first % SENSORLOG_MAX_SAMPLES];
        sl_enc0_L = s0->enc_L;
        sl_enc0_R = s0->enc_R;
    }
    sl_state = SL_SEND_INFO;
}

static void sl_take_sample(void)
{
    uint32_t now = DWT->CYCCNT;

    /* Keep the STEP PWM off for the whole run, whatever else in the loop does */
    if (sl_motor_pwm_is_on()) {
        sl_motor_pwm_off();
        sl_pwm_restarted++;
    }

    /* Both encoder positions from the same 1 kHz Encoder_Update() */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    int32_t pos_L = enc[ENC_LEFT].position;
    int32_t pos_R = enc[ENC_RIGHT].position;
    __set_PRIMASK(primask);

    if (sl_state == SL_ARMED) {
        /* First sample: time starts at zero */
        sl_last_cycles    = now;
        sl_elapsed_cycles = 0U;
        sl_theta_arm      = imu_data.theta;
        sl_i2c_errors0    = imu_data.i2c_errors;
        sl_state          = sl_fall ? SL_FALL_WAIT : SL_RECORDING;
    } else {
        /* Add up the time between samples, so the 51 s wrap of the 32-bit
           cycle counter never matters */
        sl_elapsed_cycles += (uint32_t)(now - sl_last_cycles);
        sl_last_cycles     = now;
    }
    uint32_t t_us = (uint32_t)(sl_elapsed_cycles / sl_cycles_per_us);

    /* Is the run over? (checked before this sample is stored) */
    switch (sl_state) {
    case SL_RECORDING:
        if (t_us >= sl_duration_us) {
            sl_start_send(0U, sl_count, 0U);          /* time is up: send */
            return;
        }
        if (sl_count >= SENSORLOG_MAX_SAMPLES) {
            sl_buffer_full = 1U;                      /* buffer full: send early */
            sl_start_send(0U, sl_count, 0U);
            return;
        }
        break;

    case SL_FALL_WAIT:
        if (t_us >= sl_timeout_us) {
            sl_motor_hold_off();
            sl_printf("#ERROR logFall: no fall within %.0f s (the tilt never moved %.1f deg "
                      "from where it was at arming); motors are back as they were\n",
                      (double)SENSORLOG_FALL_TIMEOUT_S, (double)(sl_trigger_rad * RAD_TO_DEG));
            sl_state = SL_IDLE;
            return;
        }
        break;

    case SL_FALL_POST: {
        /* Stop before the ring would overwrite sample sl_first */
        uint8_t full = ((sl_count - sl_first) >= SENSORLOG_MAX_SAMPLES) ? 1U : 0U;
        if (((t_us - sl_trig_t_us) >= sl_post_us) || full) {
            sl_buffer_full = full;
            sl_motor_hold_off();
            sl_start_send(sl_first, sl_count - sl_first, sl_trig_t_us);
            return;
        }
        break;
    }

    default:
        break;
    }

    /* imu_data was refreshed by the IMU_Update() just before this call, in
       the same (main) context, so it cannot change while it is copied */
    SensorLogSample_t *s = &sl_buf[sl_count % SENSORLOG_MAX_SAMPLES];
    s->t_us      = t_us;
    s->enc_L     = pos_L;
    s->enc_R     = pos_R;
    for (uint32_t i = 0; i < 3U; i++) {
        s->acc[i]  = imu_data.acc_raw[i];
        s->gyro[i] = imu_data.gyro_raw[i];
    }
    s->theta     = imu_data.theta;
    s->theta_dot = imu_data.theta_dot;
    sl_count++;

    /* logFall trigger: the tilt has moved trigger_deg from where it was held.
       Then go back through the ring for the first sample to send: pre_s
       before the trigger, or the oldest sample still kept. */
    if ((sl_state == SL_FALL_WAIT) && (fabsf(imu_data.theta - sl_theta_arm) >= sl_trigger_rad)) {
        uint32_t trig   = sl_count - 1U;
        uint32_t oldest = (sl_count > SENSORLOG_MAX_SAMPLES) ? (sl_count - SENSORLOG_MAX_SAMPLES) : 0U;
        uint32_t first  = trig;
        while ((first > oldest) &&
               ((t_us - sl_buf[(first - 1U) % SENSORLOG_MAX_SAMPLES].t_us) <= sl_pre_us)) {
            first--;
        }
        sl_first     = first;
        sl_trig_t_us = t_us;
        sl_state     = SL_FALL_POST;
        sl_printf("SensorLog: fall detected, recording %.1f s more\n",
                  (double)sl_post_us * 1e-6);
    }
}

/* ========================================================================= */
/* Sending                                                                   */
/* ========================================================================= */
static void sl_send(void)
{
    while ((sl_state >= SL_SEND_INFO) && (sl_tx_free() >= SL_LINE_MAX)) {
        switch (sl_state) {
        case SL_SEND_INFO:
            /* Run settings and scale factors, for MATLAB */
            sl_printf("#INFO fall=%u pre_s=%.3f post_s=%.3f trigger_deg=%.2f sample_period_us=%lu "
                      "gyro_lsb_per_dps=%.1f acc_lsb_per_g=%lu\n",
                      (unsigned)sl_fall,
                      sl_fall ? (double)sl_pre_us * 1e-6 : 0.0,
                      sl_fall ? (double)sl_post_us * 1e-6 : (double)sl_duration_us * 1e-6,
                      sl_fall ? (double)(sl_trigger_rad * RAD_TO_DEG) : 0.0,
                      (unsigned long)IMU_SAMPLE_US,
                      (double)imu_data.gyro_lsb_per_dps,
                      (unsigned long)SL_ACC_LSB_PER_G);
            sl_state = SL_SEND_BEGIN;
            break;

        case SL_SEND_BEGIN:
            sl_printf("#CSV_BEGIN\n");
            sl_state = SL_SEND_HEADER;
            break;

        case SL_SEND_HEADER:
            sl_printf(SL_CSV_HEADER "\n");
            sl_state = (sl_nsend > 0U) ? SL_SEND_ROWS : SL_SEND_END;
            break;

        case SL_SEND_ROWS: {
            const SensorLogSample_t *s = &sl_buf[(sl_first + sl_sent) % SENSORLOG_MAX_SAMPLES];
            if ((sl_sent > 0U) && ((s->t_us - sl_prev_t_us) > sl_max_gap_us)) {
                sl_max_gap_us = s->t_us - sl_prev_t_us;
            }
            sl_prev_t_us = s->t_us;

            /* Time relative to the first sample (logAll) or the trigger (logFall) */
            int32_t  t_rel = (int32_t)(s->t_us - sl_t_ref_us);
            uint32_t t_mag = (t_rel < 0) ? (uint32_t)(-t_rel) : (uint32_t)t_rel;
            sl_printf("%s%lu.%06lu,%ld,%ld,%d,%d,%d,%d,%d,%d,%.5f,%.5f\n",
                      (t_rel < 0) ? "-" : "",
                      (unsigned long)(t_mag / 1000000U),
                      (unsigned long)(t_mag % 1000000U),
                      (long)(s->enc_L - sl_enc0_L), (long)(s->enc_R - sl_enc0_R),
                      s->acc[0], s->acc[1], s->acc[2],
                      s->gyro[0], s->gyro[1], s->gyro[2],
                      (double)s->theta, (double)s->theta_dot);
            if (++sl_sent >= sl_nsend) {
                sl_state = SL_SEND_END;
            }
            break;
        }

        case SL_SEND_END: {
            double span_s = 0.0;
            if (sl_nsend > 1U) {
                uint32_t t0 = sl_buf[sl_first % SENSORLOG_MAX_SAMPLES].t_us;
                uint32_t t1 = sl_buf[(sl_first + sl_nsend - 1U) % SENSORLOG_MAX_SAMPLES].t_us;
                span_s = (double)(t1 - t0) * 1e-6;
            }
            double rate = (span_s > 0.0) ? (double)(sl_nsend - 1U) / span_s : 0.0;
            sl_printf("#CSV_END samples=%lu duration_s=%.3f rate_Hz=%.1f max_gap_ms=%.2f "
                      "buffer_full=%u pwm_restarted=%lu i2c_errors=%lu gyro_bias_x_dps=%.3f "
                      "theta_arm_rad=%.5f\n",
                      (unsigned long)sl_nsend, span_s, rate, (double)sl_max_gap_us * 1e-3,
                      (unsigned)sl_buffer_full, (unsigned long)sl_pwm_restarted,
                      (unsigned long)sl_i2c_errors_run, (double)sl_gyro_bias_dps,
                      (double)sl_theta_arm);
            sl_state = SL_IDLE;
            break;
        }

        default:
            sl_state = SL_IDLE;
            break;
        }
    }
}

/* ========================================================================= */
/* Commands                                                                  */
/* ========================================================================= */
/* logAll [seconds] */
static void sl_cmd_logAll(int argc, char *argv[])
{
    float seconds = SENSORLOG_DEFAULT_S;
    if (argc >= 2) {
        seconds = (float)atof(argv[1]);
        if (!((seconds >= 0.1f) && (seconds <= SENSORLOG_MAX_S))) {
            sl_printf("#ERROR logAll: seconds must be 0.1 to %.1f\n", (double)SENSORLOG_MAX_S);
            return;
        }
    }

    /* 1. Motors off */
    sl_motor_pwm_off();
#if SENSORLOG_DISABLE_DRIVERS
    Motor_Disable();                                  /* TMC_EN high: wheels free */
#endif

    /* 2. Gyro bias, robot still. Blocks the main loop for ~1 s. */
    sl_printf("SensorLog: motor PWM off. Keep the robot still: calibrating the gyro\n");
    if (!sl_calibrate_imu()) {
        sl_printf("#ERROR SensorLog: the MPU-6050 is not answering (imu_ok = 0), nothing logged\n");
        return;
    }
    sl_gyro_bias_dps = imu_data.gyro_bias_x;

    /* 3. Arm: the next SensorLog_Update() is time 0 */
    sl_duration_us = (uint32_t)((seconds * 1.0e6f) + 0.5f);
    sl_arm(0U);

    sl_printf("SensorLog: gyro bias x = %.3f deg/s. Recording %.1f s\n",
              (double)sl_gyro_bias_dps, (double)seconds);
}

/* logFall [pre_s] [post_s] [trigger_deg] */
static void sl_cmd_logFall(int argc, char *argv[])
{
    float pre  = SENSORLOG_FALL_PRE_S;
    float post = SENSORLOG_FALL_POST_S;
    float trig = SENSORLOG_FALL_TRIGGER_DEG;
    if (argc >= 2) { pre  = (float)atof(argv[1]); }
    if (argc >= 3) { post = (float)atof(argv[2]); }
    if (argc >= 4) { trig = (float)atof(argv[3]); }
    if (!((pre >= 0.0f) && (post >= 0.1f) && ((pre + post) <= SENSORLOG_MAX_S))) {
        sl_printf("#ERROR logFall: need pre_s >= 0, post_s >= 0.1 and pre_s + post_s <= %.1f\n",
                  (double)SENSORLOG_MAX_S);
        return;
    }
    if (!((trig >= 0.5f) && (trig <= 20.0f))) {
        sl_printf("#ERROR logFall: trigger_deg must be 0.5 to 20\n");
        return;
    }

    /* 1. STEP PWM off, both motors holding at the run current */
    sl_motor_pwm_off();
    sl_printf("SensorLog: motor PWM off. Hold the robot upright: enabling both motors to hold\n");
    if (!sl_motor_hold_on()) {
        sl_motor_hold_off();
        sl_printf("#ERROR logFall: a TMC2240 did not answer or did not take the hold current "
                  "(tmc_ok L %u R %u, version L 0x%02X R 0x%02X). Is the battery on?\n",
                  (unsigned)tmc_ok[TMC_LEFT], (unsigned)tmc_ok[TMC_RIGHT],
                  (unsigned)tmc_version[TMC_LEFT], (unsigned)tmc_version[TMC_RIGHT]);
        return;
    }

    /* 2. Gyro bias, robot held still. Blocks the main loop for ~1 s. */
    sl_printf("SensorLog: motors holding. Keep the robot upright and STILL: calibrating the gyro\n");
    if (!sl_calibrate_imu()) {
        sl_motor_hold_off();
        sl_printf("#ERROR SensorLog: the MPU-6050 is not answering (imu_ok = 0), nothing logged\n");
        return;
    }
    sl_gyro_bias_dps = imu_data.gyro_bias_x;

    /* 3. Arm: record into the ring until the trigger */
    sl_pre_us      = (uint32_t)((pre  * 1.0e6f) + 0.5f);
    sl_post_us     = (uint32_t)((post * 1.0e6f) + 0.5f);
    sl_trigger_rad = trig * DEG_TO_RAD;
    sl_timeout_us  = (uint32_t)(SENSORLOG_FALL_TIMEOUT_S * 1.0e6f);
    sl_arm(1U);

    sl_printf("SensorLog: gyro bias x = %.3f deg/s. Armed: let go of the robot "
              "(trigger %.1f deg, %.0f s to fall)\n",
              (double)sl_gyro_bias_dps, (double)trig, (double)SENSORLOG_FALL_TIMEOUT_S);
}

/* Split a line into words and run the command. Nothing is echoed back. */
static void sl_run_line(char *line)
{
    char *argv[SL_CMD_MAX_ARGS + 1U];
    int   argc = 0;

    for (char *p = line; (*p != '\0') && (argc < (int)SL_CMD_MAX_ARGS); ) {
        while ((*p == ' ') || (*p == '\t')) {
            p++;
        }
        if (*p == '\0') {
            break;
        }
        argv[argc++] = p;
        while ((*p != '\0') && (*p != ' ') && (*p != '\t')) {
            p++;
        }
        if (*p != '\0') {
            *p++ = '\0';
        }
    }
    argv[argc] = NULL;

    if (argc == 0) {
        return;
    }
    if (strcmp(argv[0], "logAll") == 0) {
        sl_cmd_logAll(argc, argv);
    } else if (strcmp(argv[0], "logFall") == 0) {
        sl_cmd_logFall(argc, argv);
    } else if (strcmp(argv[0], "help") == 0) {
        sl_printf("SensorLog commands:\n");
        sl_printf("  logAll [seconds]  motor PWM off, calibrate gyro, record, send CSV\n");
        sl_printf("  logFall [pre_s] [post_s] [trigger_deg]  motors hold, calibrate gyro,\n");
        sl_printf("      wait for the tilt to move trigger_deg, send pre_s..post_s as CSV\n");
    } else {
        sl_printf("#ERROR unknown command \"%s\" (try help)\n", argv[0]);
    }
}

/* Collect received characters into a line; a line ends at '\r' or '\n', so
   MATLAB's writeline (LF), CoolTerm (CR+LF) and other terminals all work. */
static void sl_rx_process(void)
{
    static char     line[SL_CMD_MAX];
    static uint16_t len      = 0U;
    static uint8_t  overflow = 0U;
    int c;

    while ((sl_state == SL_IDLE) && ((c = sl_getchar()) >= 0)) {
        if ((c == '\r') || (c == '\n')) {
            if (overflow) {
                sl_printf("#ERROR command too long\n");
            } else if (len > 0U) {
                line[len] = '\0';
                sl_run_line(line);
            }
            len      = 0U;
            overflow = 0U;
        } else if ((c == '\b') || (c == 0x7F)) {
            if (len > 0U) {
                len--;
            }
        } else if (len < (SL_CMD_MAX - 1U)) {
            line[len++] = (char)c;
        } else {
            overflow = 1U;
        }
    }
}

/* ========================================================================= */
/* Public                                                                    */
/* ========================================================================= */
void SensorLog_Init(void)
{
    /* MX_USART2_UART_Init() has set 115200 8N1 with TX and RX on. Throw away
       anything received before now, then turn on the receive interrupt. */
    (void)USART2->SR;
    (void)USART2->DR;
    ATOMIC_SET_BIT(USART2->CR1, USART_CR1_RXNEIE);
    HAL_NVIC_SetPriority(USART2_IRQn, 0U, SL_USART2_SUBPRIORITY);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    sl_printf("SensorLog ready: send logAll [seconds] or logFall [pre_s] [post_s] [trigger_deg]\n");
}

void SensorLog_Update(void)
{
    switch (sl_state) {
    case SL_IDLE:
        sl_rx_process();
        break;
    case SL_ARMED:
    case SL_RECORDING:
    case SL_FALL_WAIT:
    case SL_FALL_POST:
        sl_take_sample();
        break;
    default:
        sl_send();
        break;
    }
}

uint8_t SensorLog_Busy(void)
{
    return (sl_state != SL_IDLE) ? 1U : 0U;
}
