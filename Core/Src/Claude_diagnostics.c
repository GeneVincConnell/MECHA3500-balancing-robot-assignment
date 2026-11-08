/**
 * @file    Claude_diagnostics.c
 * @brief   Hardware diagnostic tests used while bringing up the robot.
 *
 * PROVENANCE: AI-generated. Written by Claude (Anthropic), 8 Nov 2026, to find
 * out why the MPU-6050 did not answer on I2C3 (the original GY-521 turned out
 * to be dead). None of this is downloaded code.
 *
 * main.c calls Diag_IMUNotResponding() automatically when IMU_Init() fails.
 * Output goes to CubeIDE's SWV ITM Data Console. Declarations are in Robot.h.
 */
#include "Robot.h"
#include "MPU6050.h"   /* MPU6050_WHO_AM_I. No include guard: include it once per file. */

/* Report a missing MPU-6050 and run every I2C test below */
void Diag_IMUNotResponding(void)
{
    /* ErrorCode: 0x04 = no acknowledge (wiring, power or address),
       0x20 = timeout (bus stuck or missing pull-ups) */
    printf("MPU-6050 not responding on I2C3: WHO_AM_I read 0x%02X, HAL error code 0x%02lX\n",
           imu_data.imu_id, (unsigned long)hi2c3.ErrorCode);
    Diag_ScanI2C();

    /* Pin levels read straight from the pins (works in I2C mode too).
       An idle bus sits at 1 on both, held up by the GY-521 pull-ups. */
    printf("I2C line levels: SDA (PC9) = %d, SCL (PA8) = %d  (idle bus = 1 and 1)\n",
           (int)HAL_GPIO_ReadPin(IMU_SDA_GPIO_Port, IMU_SDA_Pin),
           (int)HAL_GPIO_ReadPin(IMU_SCL_GPIO_Port, IMU_SCL_Pin));

    /* Same pins as plain inputs with the STM32's weak pull-up and pull-down,
       to tell a floating line from one held low */
    Diag_I2CPinTest();

    /* Try the sensor on the STM32's internal pull-ups: answers = wiring OK but
       the module's pull-ups are missing; no answer = open connection */
    Diag_I2CWeakPullupTest();
}

void Diag_ScanI2C(void)
{
    /* Print every 7-bit address on I2C3 that acknowledges. The MPU-6050
       should answer at 0x68 (0x69 if its AD0 pin is high). */
    uint32_t found = 0;
    printf("I2C3 scan:");
    for (uint16_t addr = 0x08U; addr <= 0x77U; addr++) {
        if (HAL_I2C_IsDeviceReady(&hi2c3, (uint16_t)(addr << 1), 2U, 5U) == HAL_OK) {
            printf(" 0x%02X", (unsigned)addr);
            found++;
        }
    }
    printf(found ? "\n" : " nothing answered\n");
}

void Diag_I2CPinTest(void)
{
    /* Take PA8 (SCL) and PC9 (SDA) away from I2C3 and read them as plain
       inputs three ways: no pull, the STM32's internal pull-up (~40 kOhm)
       and its internal pull-down (~40 kOhm). The weak internal resistors
       only win if nothing else is driving the line:
         pull-down reads 1          -> something holds the line high (healthy:
                                       the GY-521's pull-up is there)
         pull-up 1, pull-down 0     -> floating: nothing connected to the pin
         pull-up reads 0            -> something holds the line low (short to
                                       GND, or an unpowered/faulty device)    */
    static const uint32_t     pulls[3] = { GPIO_NOPULL, GPIO_PULLUP, GPIO_PULLDOWN };
    static const char * const names[3] = { "no pull  ", "pull-up  ", "pull-down" };
    GPIO_PinState sda[3], scl[3];
    GPIO_InitTypeDef g = {0};

    HAL_I2C_DeInit(&hi2c3);                  /* releases PA8 and PC9 */

    g.Mode  = GPIO_MODE_INPUT;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    for (uint32_t i = 0; i < 3U; i++) {
        g.Pull = pulls[i];
        g.Pin  = IMU_SDA_Pin;
        HAL_GPIO_Init(IMU_SDA_GPIO_Port, &g);
        g.Pin  = IMU_SCL_Pin;
        HAL_GPIO_Init(IMU_SCL_GPIO_Port, &g);
        HAL_Delay(2);                        /* let the line settle */
        sda[i] = HAL_GPIO_ReadPin(IMU_SDA_GPIO_Port, IMU_SDA_Pin);
        scl[i] = HAL_GPIO_ReadPin(IMU_SCL_GPIO_Port, IMU_SCL_Pin);
        printf("I2C pin test, %s: SDA (PC9) = %d, SCL (PA8) = %d\n",
               names[i], (int)sda[i], (int)scl[i]);
    }

    const GPIO_PinState *lines[2] = { sda, scl };
    static const char * const line_names[2] = { "SDA (PC9)", "SCL (PA8)" };
    for (uint32_t k = 0; k < 2U; k++) {
        const GPIO_PinState *v = lines[k];
        const char *verdict;
        if (v[2] == GPIO_PIN_SET) {
            verdict = "held HIGH by a pull-up: the line and the GY-521 pull-up are fine";
        } else if (v[1] == GPIO_PIN_SET) {
            verdict = "FLOATING: nothing is connected to this pin (open joint or missing pull-up)";
        } else {
            verdict = "held LOW: shorted to GND, or pulled down by an unpowered or faulty device";
        }
        printf("  %s: %s\n", line_names[k], verdict);
    }

    HAL_I2C_Init(&hi2c3);                    /* back to I2C (calls HAL_I2C_MspInit) */
}

void Diag_I2CWeakPullupTest(void)
{
    /* Try the MPU-6050 with the STM32's own weak pull-ups (~40 kOhm) on SDA and
       SCL and I2C slowed to 50 kHz (weak pull-ups give slow rising edges).
         answers   -> the wiring is connected; the GY-521's pull-ups are missing
         no answer -> the connection between the STM32 and the MPU-6050 is open
                      (or the chip is dead)
       Afterwards I2C3 goes back to its CubeMX settings (400 kHz, no pull-ups). */
    I2C_InitTypeDef saved = hi2c3.Init;
    GPIO_InitTypeDef g = {0};
    uint8_t who = 0U;

    HAL_I2C_DeInit(&hi2c3);
    hi2c3.Init.ClockSpeed = 50000U;
    HAL_I2C_Init(&hi2c3);                    /* pins back to I2C, no pull */

    g.Mode      = GPIO_MODE_AF_OD;
    g.Pull      = GPIO_PULLUP;               /* the STM32's internal pull-ups */
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF4_I2C3;
    g.Pin       = IMU_SDA_Pin;
    HAL_GPIO_Init(IMU_SDA_GPIO_Port, &g);
    g.Pin       = IMU_SCL_Pin;
    HAL_GPIO_Init(IMU_SCL_GPIO_Port, &g);
    HAL_Delay(2);

    /* Reset the peripheral now the lines are high, so it starts with the bus
       idle instead of "busy" from when the lines were floating low */
    hi2c3.Instance->CR1 |=  I2C_CR1_SWRST;
    hi2c3.Instance->CR1 &= ~I2C_CR1_SWRST;
    HAL_I2C_Init(&hi2c3);                    /* already initialised: keeps the pull-ups */

    printf("Weak pull-up test (50 kHz): line levels SDA = %d, SCL = %d\n",
           (int)HAL_GPIO_ReadPin(IMU_SDA_GPIO_Port, IMU_SDA_Pin),
           (int)HAL_GPIO_ReadPin(IMU_SCL_GPIO_Port, IMU_SCL_Pin));

    /* Try both MPU-6050 addresses: 0x68 with AD0 low, 0x69 with AD0 high.
       AD0 is not connected on the PCB (U2 pad 7), so on a GY-521 without its
       usual AD0 pull-down resistor the address is not fixed. */
    static const uint8_t addrs[2] = { 0x68U, 0x69U };
    HAL_StatusTypeDef st = HAL_ERROR;
    uint8_t addr = 0U;
    for (uint32_t i = 0; i < 2U && st != HAL_OK; i++) {
        who = 0U;
        st = HAL_I2C_Mem_Read(&hi2c3, (uint16_t)(addrs[i] << 1), MPU6050_WHO_AM_I,
                              I2C_MEMADD_SIZE_8BIT, &who, 1U, 10U);
        if (st == HAL_OK) addr = addrs[i];
    }
    if (st == HAL_OK && addr == 0x68U) {
        printf("  MPU-6050 ANSWERED at 0x68 (WHO_AM_I 0x%02X): the wiring is connected but the\n"
               "  GY-521 has no working pull-ups. Fit 4.7 kOhm from SDA and from SCL to 3.3 V.\n",
               (unsigned)who);
    } else if (st == HAL_OK) {
        printf("  MPU-6050 ANSWERED at 0x69 (WHO_AM_I 0x%02X): the wiring is connected, but AD0\n"
               "  is high or floating and the module's resistors are missing. Fit 4.7 kOhm from\n"
               "  SDA and SCL to 3.3 V, and tie AD0 (U2 pad 7) to GND (or use address 0x69).\n",
               (unsigned)who);
    } else {
        printf("  No answer at 0x68 or 0x69 even with pull-ups (HAL error code 0x%02lX): the\n"
               "  connection between the STM32 and the MPU-6050 is open (joint, header pin),\n"
               "  or the chip is dead.\n", (unsigned long)hi2c3.ErrorCode);
    }
    printf("  With pull-ups, ");
    Diag_ScanI2C();                          /* anything else on the bus? */

    HAL_I2C_DeInit(&hi2c3);                  /* back to the CubeMX settings */
    hi2c3.Init = saved;
    HAL_I2C_Init(&hi2c3);
}
