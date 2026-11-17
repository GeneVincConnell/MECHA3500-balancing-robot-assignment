/**
 * @file    Lab5.c
 * @brief   MCHA3500 Lab 5 (Serial Communication and Data Logging) in the
 *          CubeIDE balancing-robot project: no MCHA toolchain, no FreeRTOS.
 *
 * PROVENANCE: AI-generated. Written by Claude (Anthropic), 11 Nov 2026, from
 * the Lab 5 handout (MCHA3500 S2 2026). Nothing was downloaded or copied: the
 * course template's uart.c, cmd_line_buffer.c, cmd_parser.c and cmd_task.c
 * were not used, this is a new implementation of the same behaviour. Names
 * taken from the handout are kept so the code lines up with the lab steps:
 * CMD_T and the command table layout, the help text, heartbeat, getPot,
 * _cmd_getPotentiometerVoltage, logCount, log_pendulum, logging_init,
 * pend_logging_start, pend_logging_stop, logPot and the [time],[voltage]
 * output format. Uses Robot.h, htim2 (LD2) and enc[] (Encoder.c) from this
 * project.
 * Updated by Claude, 17 Nov 2026: this file is now the project's only serial
 * driver. SensorLog.c no longer has its own USART2 code: its logAll and
 * logFall commands are in the command table below and it prints with
 * Lab5_printf(). Added Lab5_TxFree() (SensorLog queues CSV rows only when
 * they fit, so the 500 Hz loop never waits), TX buffer 1024 -> 2048 bytes and
 * the longest Lab5_printf() line 160 -> 200 characters (LAB5_PRINTF_MAX, now
 * in Robot.h) for SensorLog's #INFO / #CSV_END lines.
 *
 * What replaces what
 *   Course template (MCHA toolchain)         This file (CubeIDE)
 *   uart.c: printf -> osMessageQueue ->      Lab5_printf() -> TX ring buffer
 *           _uart_txe_isr()                    -> USART2 TXE interrupt
 *   uart.c: _uart_rxne_isr() -> queue ->     USART2 RXNE interrupt -> RX ring
 *           getchar()                          buffer -> uart_getchar()
 *   cmd_task.c (RTOS task, infinite loop)    Lab5_Process(), called from the
 *                                              while(1) loop in main.c
 *   cmd_line_buffer.c / cmd_parser.c         sections 4 and 5 below
 *   Lab 3 potentiometer read (HAL ADC)       pot_read_voltage(): ADC1 IN8, PB0
 *   osTimer, periodic, 200 Hz                TIM6 update interrupt, 200 Hz
 *
 * Why the lab output does not use printf
 *   printf already goes to SWV (data.c's __io_putchar) for the robot's live
 *   debug lines. Lab 5's output has to go to the ST-LINK virtual COM port
 *   (USART2, PA2/PA3) for CoolTerm and MATLAB, without those 10 Hz debug
 *   lines mixed into it. So the lab code prints with Lab5_printf(), which
 *   takes the same arguments as printf, and SWV keeps working as before.
 *
 * Why samples are taken in the interrupt but printed in main
 *   The TIM6 interrupt reads the ADC at exact 5 ms instants (a few us of
 *   jitter), which matters for the 100 Hz and 202 Hz aliasing test in 3.3.
 *   It puts the sample in log_fifo and Lab5_Process() prints it. Float
 *   printf must not run inside an interrupt: newlib's float formatting is not
 *   re-entrant, and main could be half-way through a printf when it fires.
 *   (In the template the osTimer callback runs in the RTOS timer task, which
 *   is why the handout can call printf there.)
 *
 * Hardware (none of it is used anywhere else in the project)
 *   USART2  PA2/PA3   ST-LINK virtual COM port, 115200 8N1. CubeMX already
 *                     sets it up (MX_USART2_UART_Init); this file adds the
 *                     interrupts.
 *   ADC1    PB0 = A3  ADC1_IN8, pin 4 of the Nucleo's CN8 Arduino header.
 *                     Unconnected on Balancing_robot_PCB, so it is free.
 *                     Potentiometer: ends to 3V3 and GND, wiper to A3.
 *                     Wave generator (3.3): 0-3.3 V only (CMOS 3.3 preset,
 *                     High-Z load), Gen Out to A3, shield to GND.
 *   TIM6              logging tick, 100 kHz counter, 200 Hz by default
 *   TIM2 CH1  PA5     LD2, already blinking: heartbeat starts / stops it
 *
 * The ADC, TIM6 and both interrupts are set up here with registers, not in
 * CubeMX, so the .ioc and main.c's generated code are untouched.
 * NOTE: this file defines USART2_IRQHandler() and TIM6_DAC_IRQHandler(). If
 * you ever enable "USART2 global interrupt" or TIM6 in CubeMX, CubeMX will
 * generate its own copy in stm32f4xx_it.c and the build fails with "multiple
 * definition": delete one of the two.
 *
 * main.c (inside the USER CODE blocks)
 *   USER CODE BEGIN 2, first:            Lab5_Init();  logging_init();
 *                                        (then SensorLog_Init())
 *   while(1) loop, after IMU_Update():   Lab5_Process();
 *                                        (while SensorLog_Busy() is 0, so
 *                                        nothing lands in the middle of a CSV)
 *
 * Build setting: Project Properties > C/C++ Build > Settings > MCU/MPU
 * Settings > tick "Use float with printf from newlib-nano" (the course
 * makefile's -u _printf_float). Lab5_Init() prints a warning if it is off.
 *
 * Serial commands (CoolTerm: 115200 baud, Line Mode)
 *   help                         list the commands (Task 1.1.4)
 *   reset                        software reset (Task 1.1.5)
 *   heartbeat [start|stop]       LD2 blink status / start / stop (Task 1.1.5)
 *   getPot                       potentiometer voltage (Task 1.2)
 *   logPot [samples] [rate_Hz]   log the pot, default 400 at 200 Hz (Problem 2, R.1)
 *   getEnc                       wheel encoder counts (R.2)
 *   logEnc [samples] [rate_Hz]   log the encoder counts (R.2)
 *   logAll [seconds]             SensorLog: encoder + IMU log as CSV
 *   logFall [pre_s] [post_s] [trigger_deg]
 *                                SensorLog: motors hold, log a fall as CSV
 *                                (both for logRobotSensors.m; see SensorLog.h)
 *   bal [on|off|sign|stream Hz]  balance controller: status, arm, stop, motor
 *                                direction test, CSV stream (Balance_AIgen.h)
 */
#include "Robot.h"
#include "SensorLog.h"   /* logAll / logFall are in the command table */
#include "Balance_AIgen.h"  /* "bal" is in the command table. AI-GENERATED (Claude, 2026-11-17) */
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ========================================================================= */
/* 1. Settings                                                               */
/* ========================================================================= */
#define POT_VREF_V             3.3f      /* VDDA = VREF+ = 3.3 V on the Nucleo  */
#define POT_FULL_SCALE         4095.0f   /* 12-bit ADC                          */
#define POT_ADC_CHANNEL        8U        /* PB0 = ADC1_IN8                      */
#define POT_SAMPLE_TIME        6U        /* SMP code 6 = 144 ADC cycles: 7.4 us
                                            per conversion at 21 MHz, enough
                                            for a 10 kOhm potentiometer        */

#define LOG_DEFAULT_SAMPLES    400U      /* 2 s at 200 Hz (Problem 2.3)         */
#define LOG_DEFAULT_RATE_HZ    200.0f    /* Problem 2.4                         */
#define LOG_TICK_HZ            100000U   /* TIM6 counter clock                  */
#define LOG_MIN_RATE_HZ        2.0f      /* TIM6 ARR is 16 bits: 100 kHz / 65536 */
#define LOG_MAX_RATE_HZ        500.0f    /* a line is ~20 characters; 115200 baud
                                            sends 11520 characters/s, so ~575
                                            lines/s at most                     */

#define UART_TX_BUF_SIZE       2048U     /* bytes, power of 2 (SensorLog keeps
                                            ~10 CSV rows queued)               */
#define UART_RX_BUF_SIZE       128U      /* bytes, power of 2                   */
/* LAB5_PRINTF_MAX (longest single Lab5_printf() line) is in Robot.h */
#define LOG_FIFO_SIZE          128U      /* samples, power of 2: 0.64 s at 200 Hz
                                            before Lab5_Process() falls behind  */
#define CLB_LINE_MAX           64U       /* longest command line, incl. '\0'    */
#define CMD_MAX_ARGS           8U

/* NVIC_PRIORITYGROUP_0 (stm32f4xx_hal_msp.c): no pre-emption, these only set
   which interrupt goes first when several are pending. SysTick is 0. */
#define TIM6_IRQ_SUBPRIORITY   1U
#define USART2_IRQ_SUBPRIORITY 2U

#define ARRAY_LEN(a)           (sizeof(a) / sizeof((a)[0]))

/* ========================================================================= */
/* Function declarations (public ones are also in Robot.h)                   */
/* ========================================================================= */
void     Lab5_Init(void);
void     Lab5_Process(void);
int      Lab5_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
uint16_t Lab5_TxFree(void);
void     logging_init(void);
void     pend_logging_start(void);
void     pend_logging_stop(void);
void     logging_stop(void);

static void     uart_init(void);
static void     uart_putc(char c);
static void     uart_write(const char *s);
static void     uart_start_tx(void);
static int      uart_getchar(void);
static void     uart_flush(uint32_t timeout_ms);

static void     pot_init(void);
static uint16_t pot_read_raw(void);
static float    pot_read_voltage(void);

static void     clb_process(void);
static void     cmd_parse(char *line);

static void     _help(int, char *[]);
static void     _reset(int, char *[]);
static void     heartbeat_cmd(int, char *[]);
static void     _cmd_getPotentiometerVoltage(int, char *[]);
static void     _cmd_logPot(int, char *[]);
static void     _cmd_getEncoderCounts(int, char *[]);
static void     _cmd_logEnc(int, char *[]);

static void     log_pendulum(void *argument);
static void     log_encoder(void *argument);
static void     logging_start(void (*func)(void *argument), uint16_t samples, float rate_hz);
static uint8_t  logging_is_running(void);
static uint32_t tim6_clock_hz(void);

/* ========================================================================= */
/* 2. UART (replaces the template's uart.c)                                  */
/* ========================================================================= */
/* Ring buffers. Each index has exactly one writer, so no locking is needed:
   main writes uart_tx_head and uart_rx_tail, the interrupt writes
   uart_tx_tail and uart_rx_head. */
static volatile uint8_t  uart_tx_buf[UART_TX_BUF_SIZE];
static volatile uint16_t uart_tx_head = 0U;
static volatile uint16_t uart_tx_tail = 0U;
static volatile uint8_t  uart_rx_buf[UART_RX_BUF_SIZE];
static volatile uint16_t uart_rx_head = 0U;
static volatile uint16_t uart_rx_tail = 0U;
static volatile uint32_t uart_rx_dropped = 0U;   /* RX buffer was full */

static void uart_init(void)
{
    /* MX_USART2_UART_Init() has already set 115200 8N1 with TX and RX on.
       Throw away anything received before now, then turn on the receive
       interrupt. The transmit interrupt is turned on only while there is
       something to send. */
    (void)USART2->SR;
    (void)USART2->DR;
    ATOMIC_SET_BIT(USART2->CR1, USART_CR1_RXNEIE);

    HAL_NVIC_SetPriority(USART2_IRQn, 0U, USART2_IRQ_SUBPRIORITY);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

/* TXE interrupt on. ATOMIC_SET_BIT (CMSIS) because the interrupt clears
   TXEIE in the same register. */
static void uart_start_tx(void)
{
    ATOMIC_SET_BIT(USART2->CR1, USART_CR1_TXEIE);
}

/* Queue one character. If the buffer is full, wait for the interrupt to
   send some (main context only, never from an interrupt). */
static void uart_putc(char c)
{
    uint16_t next = (uint16_t)((uart_tx_head + 1U) & (UART_TX_BUF_SIZE - 1U));
    while (next == uart_tx_tail) {
        uart_start_tx();
    }
    uart_tx_buf[uart_tx_head] = (uint8_t)c;
    uart_tx_head = next;
}

/* Queue a string of any length (used for long fixed text) */
static void uart_write(const char *s)
{
    while (*s != '\0') {
        uart_putc(*s++);
    }
    uart_start_tx();
}

/* printf to the ST-LINK COM port. Same arguments as printf. Main context
   only. Lines longer than LAB5_PRINTF_MAX - 1 characters are cut short. */
int Lab5_printf(const char *fmt, ...)
{
    char    buf[LAB5_PRINTF_MAX];
    va_list ap;

    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) {
        return n;
    }

    size_t len = ((size_t)n < sizeof(buf)) ? (size_t)n : (sizeof(buf) - 1U);
    for (size_t i = 0; i < len; i++) {
        uart_putc(buf[i]);
    }
    uart_start_tx();
    return (int)len;
}

/* Bytes that can be queued right now without waiting (one slot is always
   kept empty). Main context only. SensorLog uses it to queue a CSV row only
   when the whole row fits, so the 500 Hz loop never waits for the UART. */
uint16_t Lab5_TxFree(void)
{
    return (uint16_t)((uart_tx_tail - uart_tx_head - 1U) & (UART_TX_BUF_SIZE - 1U));
}

/* Next received character, or EOF if none is waiting (like getchar() in
   the template) */
static int uart_getchar(void)
{
    if (uart_rx_tail == uart_rx_head) {
        return EOF;
    }
    uint8_t c = uart_rx_buf[uart_rx_tail];
    uart_rx_tail = (uint16_t)((uart_rx_tail + 1U) & (UART_RX_BUF_SIZE - 1U));
    return (int)c;
}

/* Wait until everything queued has left the pin (used before a reset) */
static void uart_flush(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    while (((uart_tx_tail != uart_tx_head) || !(USART2->SR & USART_SR_TC)) &&
           ((HAL_GetTick() - start) < timeout_ms)) {
    }
}

/* USART2 interrupt: _uart_rxne_isr() and _uart_txe_isr() in one handler.
   Not generated by CubeMX (USART2 global interrupt is off in the .ioc). */
void USART2_IRQHandler(void)
{
    uint32_t sr = USART2->SR;

    /* Receive. Reading SR then DR also clears the error flags (ORE, NE, FE,
       PE), so a framing error or overrun cannot leave the interrupt stuck. */
    if (sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) {
        uint8_t c = (uint8_t)USART2->DR;
        if ((sr & USART_SR_RXNE) && !(sr & (USART_SR_NE | USART_SR_FE | USART_SR_PE))) {
            uint16_t next = (uint16_t)((uart_rx_head + 1U) & (UART_RX_BUF_SIZE - 1U));
            if (next != uart_rx_tail) {
                uart_rx_buf[uart_rx_head] = c;
                uart_rx_head = next;
            } else {
                uart_rx_dropped++;
            }
        }
    }

    /* Transmit: send the next queued character, or stop the TXE interrupt
       when the buffer is empty */
    if ((USART2->CR1 & USART_CR1_TXEIE) && (sr & USART_SR_TXE)) {
        if (uart_tx_tail != uart_tx_head) {
            USART2->DR = uart_tx_buf[uart_tx_tail];
            uart_tx_tail = (uint16_t)((uart_tx_tail + 1U) & (UART_TX_BUF_SIZE - 1U));
        } else {
            USART2->CR1 &= ~USART_CR1_TXEIE;
        }
    }
}

/* ========================================================================= */
/* 3. Potentiometer: ADC1 channel 8 on PB0 (A3)                              */
/* ========================================================================= */
/* Register-level, because the HAL ADC driver is not in this project
   (ADC is not enabled in the .ioc). Reference: RM0390 chapter 13. */
static void pot_init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();

    /* PB0 to analog mode (MODER = 11), no pull-up or pull-down */
    GPIOB->MODER |= (3U << (0U * 2U));
    GPIOB->PUPDR &= ~(3U << (0U * 2U));

    /* ADC clock = PCLK2 / 4 = 84 / 4 = 21 MHz (36 MHz is the maximum) */
    ADC->CCR = (ADC->CCR & ~ADC_CCR_ADCPRE) | ADC_CCR_ADCPRE_0;

    ADC1->CR1   = 0U;                      /* 12-bit, no scan, no interrupts   */
    ADC1->CR2   = 0U;                      /* single conversion, right aligned */
    ADC1->SQR1  = 0U;                      /* one conversion in the sequence   */
    ADC1->SQR3  = POT_ADC_CHANNEL;         /* ... and it is channel 8          */
    ADC1->SMPR2 = (ADC1->SMPR2 & ~ADC_SMPR2_SMP8) | (POT_SAMPLE_TIME << ADC_SMPR2_SMP8_Pos);
    ADC1->CR2  |= ADC_CR2_ADON;            /* power the ADC up                 */

    /* Wait for the ADC to stabilise (tSTAB, 3 us max), then do one
       conversion and throw it away */
    for (volatile uint32_t i = 0; i < 1000U; i++) {
    }
    (void)pot_read_raw();
}

/* One 12-bit conversion, 0-4095. Interrupts are held off for the ~8 us it
   takes, because main (getPot) and the TIM6 interrupt (logPot) both use
   ADC1 and must not start a conversion on top of each other. */
static uint16_t pot_read_raw(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    ADC1->SR   = 0U;                       /* clear EOC / OVR / STRT */
    ADC1->CR2 |= ADC_CR2_SWSTART;
    for (uint32_t guard = 0U; !(ADC1->SR & ADC_SR_EOC) && (guard < 10000U); guard++) {
    }
    uint16_t raw = (uint16_t)ADC1->DR;     /* reading DR clears EOC */

    __set_PRIMASK(primask);
    return raw;
}

static float pot_read_voltage(void)
{
    return (float)pot_read_raw() * (POT_VREF_V / POT_FULL_SCALE);
}

/* ========================================================================= */
/* 4. Command line buffer (replaces the template's cmd_line_buffer.c)        */
/* ========================================================================= */
/* Collects received characters into a line. A line ends at '\r' or '\n', so
   CoolTerm (CR+LF), MATLAB's writeline (LF) and other terminals (CR) all
   work; the empty line between CR and LF is ignored. Nothing is echoed back:
   collectPendData.m reads the first line after writeline() as data. */
static char     clb_line[CLB_LINE_MAX];
static uint16_t clb_len      = 0U;
static uint8_t  clb_overflow = 0U;

static void clb_process(void)
{
    int c;
    while ((c = uart_getchar()) != EOF) {
        if ((c == '\r') || (c == '\n')) {
            if (clb_overflow) {
                Lab5_printf("Command too long (%u characters at most)\n",
                            (unsigned)(CLB_LINE_MAX - 1U));
            } else if (clb_len > 0U) {
                clb_line[clb_len] = '\0';
                cmd_parse(clb_line);
            }
            clb_len      = 0U;
            clb_overflow = 0U;
        } else if ((c == '\b') || (c == 0x7F)) {       /* backspace / delete */
            if (clb_len > 0U) {
                clb_len--;
            }
        } else if (clb_len < (CLB_LINE_MAX - 1U)) {
            clb_line[clb_len++] = (char)c;
        } else {
            clb_overflow = 1U;
        }
    }
}

/* ========================================================================= */
/* 5. Command parser (replaces the template's cmd_parser.c)                  */
/* ========================================================================= */
typedef struct
{
    void       (*func)(int argc, char *argv[]);   /* function to call        */
    const char  *cmd;                             /* serial command          */
    const char  *args;                            /* valid arguments (help)  */
    const char  *help;                            /* description (help)      */
} CMD_T;

// Command table
static CMD_T cmd_table[] =
{
    {_help                        , "help"     , ""                   , "Print this help message"                      },
    {_reset                       , "reset"    , ""                   , "Restarts the system"                          },
    {heartbeat_cmd                , "heartbeat", "[start|stop]"       , "Get status or start/stop heartbeat task"      },
    {_cmd_getPotentiometerVoltage , "getPot"   , ""                   , "Displays the potentiometer voltage level."    },
    {_cmd_logPot                  , "logPot"   , "[samples] [rate_Hz]", "Logs the pot voltage (default 400 samples at 200 Hz)"},
    {_cmd_getEncoderCounts        , "getEnc"   , ""                   , "Displays the wheel encoder counts"            },
    {_cmd_logEnc                  , "logEnc"   , "[samples] [rate_Hz]", "Logs the encoder counts (default 400 samples at 200 Hz)"},
    {SensorLog_CmdLogAll          , "logAll"   , "[seconds]"          , "Encoder + IMU log at 500 Hz, sent as CSV (SensorLog)"},
    {SensorLog_CmdLogFall         , "logFall"  , "[pre_s] [post_s] [trigger_deg]", "Motors hold; logs a fall, sent as CSV (SensorLog)"},
    {Balance_Cmd                  , "bal"      , "[on|off|sign|stream Hz]", "Balance controller; no argument = status (Balance_AIgen.c)"},  /* AI-GENERATED (Claude, 2026-11-17) */
};

/* Split the line into words, then call the matching command with
   argc / argv in the same form main() gets them: argv[0] is the command,
   argv[1] .. argv[argc-1] are its arguments, argv[argc] is NULL. */
static void cmd_parse(char *line)
{
    char *argv[CMD_MAX_ARGS + 1U];
    int   argc = 0;
    char *p    = line;

    while ((*p != '\0') && (argc < (int)CMD_MAX_ARGS)) {
        while ((*p == ' ') || (*p == '\t')) {
            p++;                                   /* skip spaces */
        }
        if (*p == '\0') {
            break;
        }
        argv[argc++] = p;                          /* start of a word */
        while ((*p != '\0') && (*p != ' ') && (*p != '\t')) {
            p++;
        }
        if (*p != '\0') {
            *p++ = '\0';                           /* end of the word */
        }
    }
    argv[argc] = NULL;

    if (argc == 0) {
        return;
    }
    for (size_t i = 0; i < ARRAY_LEN(cmd_table); i++) {
        if (strcmp(argv[0], cmd_table[i].cmd) == 0) {
            cmd_table[i].func(argc, argv);
            return;
        }
    }
    Lab5_printf("Unknown command \"%s\". Type help for the list of commands.\n", argv[0]);
}

/* Contemporary modern art (the template prints the chip pinout here) */
static void _print_robot_art(void)
{
    uart_write(
        "\n"
        "          _________________\n"
        "         |  _____________  |    MCHA3500 balancing robot\n"
        "         | |  NUCLEO-64  | |    STM32F446RE, 84 MHz\n"
        "         | |   F446RE    | |\n"
        "         | |_____________| |    Lab 5 wiring:\n"
        "         |     MPU-6050    |      pot wiper / Gen Out -> A3 (PB0)\n"
        "         | TMC2240 TMC2240 |      pot ends -> 3V3 and GND\n"
        "         |_________________|      serial: ST-LINK COM port, 115200\n"
        "        (@)       |       (@)\n"
        "     ~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
}

static void _help(int argc, char *argv[])
{
    UNUSED(argc);
    UNUSED(argv);

    _print_robot_art();
    Lab5_printf("\nAvailable commands:\n");
    for (size_t i = 0; i < ARRAY_LEN(cmd_table); i++) {
        char usage[40];
        snprintf(usage, sizeof(usage), "%s %s", cmd_table[i].cmd, cmd_table[i].args);
        Lab5_printf("    %-28s %s\n", usage, cmd_table[i].help);
    }
}

static void _reset(int argc, char *argv[])
{
    UNUSED(argc);
    UNUSED(argv);

    Lab5_printf("Restarting...\n");
    uart_flush(100U);                     /* let the message out first */
    NVIC_SystemReset();
}

/* The template's heartbeat task blinks an LED. Here LD2 is already blinking
   in hardware on TIM2 CH1 (started in main.c, period set by Motor_test.c),
   so the heartbeat command starts and stops that PWM output. */
static void heartbeat_cmd(int argc, char *argv[])
{
    uint8_t running =
        (TIM_CHANNEL_STATE_GET(&htim2, TIM_CHANNEL_1) == HAL_TIM_CHANNEL_STATE_BUSY) ? 1U : 0U;

    if (argc < 2) {
        Lab5_printf("Heartbeat is %s\n", running ? "running" : "stopped");
    } else if (strcmp(argv[1], "start") == 0) {
        if (!running) {
            HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
        }
        Lab5_printf("Heartbeat started\n");
    } else if (strcmp(argv[1], "stop") == 0) {
        if (running) {
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
        }
        Lab5_printf("Heartbeat stopped\n");
    } else {
        Lab5_printf("heartbeat: unknown argument \"%s\", use heartbeat [start|stop]\n", argv[1]);
    }
}

/* Task 1.2.1 */
static void _cmd_getPotentiometerVoltage(int argc, char *argv[])
{
    /* Supress compiler warnings for unused arguments */
    UNUSED(argc);
    UNUSED(argv);

    /* Read the potentiometer voltage */
    float voltage = pot_read_voltage();

    /* Print the voltage to the serial terminal */
    Lab5_printf("Potentiometer voltage: %f V\n", (double)voltage);
}

/* Optional [samples] [rate_Hz] arguments for logPot and logEnc (R.1).
   Returns 0 and prints why if they are out of range. */
static uint8_t log_parse_args(int argc, char *argv[], uint16_t *samples, float *rate_hz)
{
    *samples = LOG_DEFAULT_SAMPLES;
    *rate_hz = LOG_DEFAULT_RATE_HZ;

    if (argc >= 2) {
        int n = atoi(argv[1]);
        if ((n < 1) || (n > 65535)) {
            Lab5_printf("%s: samples must be 1 to 65535\n", argv[0]);
            return 0U;
        }
        *samples = (uint16_t)n;
    }
    if (argc >= 3) {
        float f = (float)atof(argv[2]);
        if ((f < LOG_MIN_RATE_HZ) || (f > LOG_MAX_RATE_HZ)) {
            Lab5_printf("%s: rate must be %.0f to %.0f Hz\n", argv[0],
                        (double)LOG_MIN_RATE_HZ, (double)LOG_MAX_RATE_HZ);
            return 0U;
        }
        *rate_hz = f;
    }
    return 1U;
}

/* Task 2.5: serial command that starts the data logging. With no arguments
   it is exactly the lab: pend_logging_start(), 400 samples at 200 Hz. */
static void _cmd_logPot(int argc, char *argv[])
{
    if (argc < 2) {
        pend_logging_start();
        return;
    }

    uint16_t samples;
    float    rate_hz;
    if (log_parse_args(argc, argv, &samples, &rate_hz)) {
        logging_start(log_pendulum, samples, rate_hz);
    }
}

/* R.2: absolute encoder counts since start-up (Encoder.c, 4000 per wheel
   revolution, + = forwards on both wheels) */
static void _cmd_getEncoderCounts(int argc, char *argv[])
{
    UNUSED(argc);
    UNUSED(argv);

    __disable_irq();                      /* both from the same SysTick update */
    int32_t left  = enc[ENC_LEFT].position;
    int32_t right = enc[ENC_RIGHT].position;
    __enable_irq();

    Lab5_printf("Encoder counts: left %ld, right %ld\n", (long)left, (long)right);
}

static void _cmd_logEnc(int argc, char *argv[])
{
    uint16_t samples;
    float    rate_hz;
    if (log_parse_args(argc, argv, &samples, &rate_hz)) {
        logging_start(log_encoder, samples, rate_hz);
    }
}

/* ========================================================================= */
/* 6. Data logging (Problem 2; replaces data_logging.c)                      */
/* ========================================================================= */
/* Variable declarations */
static volatile uint16_t logCount = 0U;   /* samples taken this run (written in
                                             the TIM6 interrupt, so volatile) */

typedef struct {
    uint16_t n;          /* logCount when the sample was taken: time = n x period */
    uint8_t  kind;       /* LOG_KIND_POT or LOG_KIND_ENC                         */
    float    voltage;    /* pot voltage, V                                       */
    int32_t  count_L;    /* encoder counts                                       */
    int32_t  count_R;
} LogSample_t;

#define LOG_KIND_POT   1U
#define LOG_KIND_ENC   2U

/* Samples waiting to be printed: the TIM6 interrupt writes log_fifo_head,
   Lab5_Process() writes log_fifo_tail. */
static volatile LogSample_t log_fifo[LOG_FIFO_SIZE];
static volatile uint16_t    log_fifo_head = 0U;
static volatile uint16_t    log_fifo_tail = 0U;
static volatile uint32_t    log_dropped   = 0U;

/* What the timer calls each tick (Lab 7 Problem 1 turns the lab's timer into
   this same function-pointer form), how many samples, and the real period */
static void (*volatile log_function)(void *argument) = NULL;
static volatile uint16_t log_samples  = LOG_DEFAULT_SAMPLES;
static float             log_period_s = 1.0f / LOG_DEFAULT_RATE_HZ;

/* Store one sample for Lab5_Process() to print (TIM6 interrupt) */
static void log_push(uint8_t kind, float voltage, int32_t count_L, int32_t count_R)
{
    uint16_t next = (uint16_t)((log_fifo_head + 1U) & (LOG_FIFO_SIZE - 1U));
    if (next == log_fifo_tail) {
        log_dropped++;                    /* main has fallen 0.64 s behind */
        return;
    }
    log_fifo[log_fifo_head].n       = logCount;
    log_fifo[log_fifo_head].kind    = kind;
    log_fifo[log_fifo_head].voltage = voltage;
    log_fifo[log_fifo_head].count_L = count_L;
    log_fifo[log_fifo_head].count_R = count_R;
    log_fifo_head = next;
}

/* Function definitions */

/* Task 2.3. Called by the TIM6 interrupt at the logging rate. */
static void log_pendulum(void *argument)
{
    /* Supress compiler warnings for unused arguments */
    UNUSED(argument);

    /* Read the potentiometer voltage */
    float voltage = pot_read_voltage();

    /* Print the sample time and potentiometer voltage to the serial terminal
       in the format [time],[voltage]. This runs in an interrupt, so the
       sample is queued here and Lab5_Process() prints it. */
    log_push(LOG_KIND_POT, voltage, 0, 0);

    /* Increment log count */
    logCount++;

    /* Stop logging once 2 seconds (log_samples samples) is reached */
    if (logCount >= log_samples) {
        pend_logging_stop();
    }
}

/* R.2, same pattern: [time],[left counts],[right counts] */
static void log_encoder(void *argument)
{
    UNUSED(argument);

    /* SysTick cannot interrupt this (no pre-emption), so left and right come
       from the same Encoder_Update() */
    log_push(LOG_KIND_ENC, 0.0f, enc[ENC_LEFT].position, enc[ENC_RIGHT].position);

    logCount++;
    if (logCount >= log_samples) {
        logging_stop();
    }
}

/* Task 2.4: initialise the logging timer. TIM6 is a basic timer on APB1,
   not used by anything else. It counts at 100 kHz and interrupts on each
   overflow; ARR sets the rate (200 Hz: ARR = 499). */
void logging_init(void)
{
    /* Initialise timer for use with pendulum data logging */
    __HAL_RCC_TIM6_CLK_ENABLE();
    __HAL_DBGMCU_FREEZE_TIM6();          /* pause with the debugger, like TIM2/4/8 */

    TIM6->CR1  = TIM_CR1_ARPE | TIM_CR1_URS;   /* stopped; only overflows interrupt */
    TIM6->PSC  = (uint16_t)((tim6_clock_hz() / LOG_TICK_HZ) - 1U);   /* 84 MHz / 840 */
    TIM6->ARR  = (uint16_t)((LOG_TICK_HZ / 200U) - 1U);
    TIM6->EGR  = TIM_EGR_UG;                   /* load PSC and ARR now */
    TIM6->SR   = 0U;
    TIM6->DIER = TIM_DIER_UIE;

    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0U, TIM6_IRQ_SUBPRIORITY);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

/* TIM6's clock. APB1 timers run at 2 x PCLK1 whenever the APB1 prescaler is
   not 1: PCLK1 = 42 MHz here, so 84 MHz. */
static uint32_t tim6_clock_hz(void)
{
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) {
        return 2U * pclk1;
    }
    return pclk1;
}

/* Start a run: 'func' is called every tick until 'samples' have been taken.
   Anything from an earlier run that has not been printed yet is dropped. */
static void logging_start(void (*func)(void *argument), uint16_t samples, float rate_hz)
{
    logging_stop();

    /* Rate -> timer reload. 200 Hz -> 500 ticks -> ARR 499. log_period_s is
       the period the timer really runs at, used for the printed time. */
    uint32_t ticks = (uint32_t)(((float)LOG_TICK_HZ / rate_hz) + 0.5f);
    if (ticks < 2U) {
        ticks = 2U;
    } else if (ticks > 65536U) {
        ticks = 65536U;
    }
    log_period_s = (float)ticks / (float)LOG_TICK_HZ;

    log_fifo_tail = log_fifo_head;        /* timer is stopped: safe to empty  */
    log_dropped   = 0U;
    log_samples   = samples;
    log_function  = func;

    /* Reset the log counter */
    logCount = 0U;

    /* Start data logging timer. First sample one period from now. */
    TIM6->ARR = (uint16_t)(ticks - 1U);
    TIM6->CNT = 0U;
    TIM6->EGR = TIM_EGR_UG;
    TIM6->SR  = 0U;
    TIM6->CR1 |= TIM_CR1_CEN;
}

void pend_logging_start(void)
{
    /* Reset the log counter, start data logging timer at 200 Hz:
       400 samples = 2 s */
    logging_start(log_pendulum, LOG_DEFAULT_SAMPLES, LOG_DEFAULT_RATE_HZ);
}

void pend_logging_stop(void)
{
    /* Stop data logging timer */
    logging_stop();
}

/* Stops whichever log is running. Safe from main or from the interrupt. */
void logging_stop(void)
{
    TIM6->CR1 &= ~TIM_CR1_CEN;
    TIM6->SR   = 0U;
    NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);
}

static uint8_t logging_is_running(void)
{
    return (TIM6->CR1 & TIM_CR1_CEN) ? 1U : 0U;
}

/* TIM6 overflow: the logging tick (the osTimer callback in the template).
   Not generated by CubeMX (TIM6 is not in the .ioc). */
void TIM6_DAC_IRQHandler(void)
{
    if (TIM6->SR & TIM_SR_UIF) {
        TIM6->SR = ~TIM_SR_UIF;           /* clear the update flag */
        void (*func)(void *argument) = log_function;
        if ((func != NULL) && logging_is_running()) {
            func(NULL);
        }
    }
}

/* ========================================================================= */
/* 7. Called from main.c                                                     */
/* ========================================================================= */
/* Once, at the top of USER CODE BEGIN 2 */
void Lab5_Init(void)
{
    uart_init();

    /* Task 1.1.3 */
    Lab5_printf("Hello World!\n");

    /* Without "Use float with printf from newlib-nano", %f prints nothing */
    char test[8];
    snprintf(test, sizeof(test), "%.1f", 1.5);
    if (strcmp(test, "1.5") != 0) {
        uart_write("WARNING: printf cannot print floats. Tick Project Properties > "
                   "C/C++ Build > Settings > MCU/MPU Settings > \"Use float with "
                   "printf from newlib-nano\", then rebuild.\n");
    }

    pot_init();
}

/* Often, from the while(1) loop (every 5 ms there). Does the job of the
   template's command task: prints the logged samples, then runs any
   complete command lines that have arrived. */
void Lab5_Process(void)
{
    /* Logged samples -> [time],[voltage]  or  [time],[left],[right] */
    while (log_fifo_tail != log_fifo_head) {
        uint16_t    t = log_fifo_tail;
        LogSample_t s;
        s.n       = log_fifo[t].n;
        s.kind    = log_fifo[t].kind;
        s.voltage = log_fifo[t].voltage;
        s.count_L = log_fifo[t].count_L;
        s.count_R = log_fifo[t].count_R;
        log_fifo_tail = (uint16_t)((t + 1U) & (LOG_FIFO_SIZE - 1U));

        double time_s = (double)s.n * (double)log_period_s;
        if (s.kind == LOG_KIND_POT) {
            Lab5_printf("%f,%f\n", time_s, (double)s.voltage);
        } else {
            Lab5_printf("%f,%ld,%ld\n", time_s, (long)s.count_L, (long)s.count_R);
        }
    }

    /* Only after a run has finished, so it cannot land in the middle of the
       data. If this ever appears, call Lab5_Process() more often. */
    if ((log_dropped != 0U) && !logging_is_running()) {
        Lab5_printf("WARNING: %lu logged samples were dropped\n", (unsigned long)log_dropped);
        log_dropped = 0U;
    }

    /* Serial commands */
    clb_process();
}
