/**
 * @file    data.c
 * @brief   Prints the robot's sensor data to the console.
 *
 * PROVENANCE: AI-generated. Written by Claude (Anthropic), Nov 2026.
 *
 * Only printing lives here. The values come from:
 *   enc[]      Encoder.c: wheel encoders, updated at 1 kHz from SysTick
 *   imu_data   MPU6050.c, "AI generated glue code" section: IMU_Update()
 *
 * printf output goes through SWV: ITM stimulus port 0 on the SWO pin (PB3) to
 * CubeIDE's "SWV ITM Data Console". Needs the ST-LINK GDB server with Serial
 * Wire Viewer enabled (core clock 84 MHz), port 0 ticked in the console's
 * settings, and Start Trace pressed before resuming. With no debugger or no
 * trace running, printf simply returns, so the robot also runs untethered.
 */
#include "Robot.h"

/* printf -> SWV. syscalls.c's _write() sends each character here.
   ITM_SendChar() (CMSIS) returns straight away when no debugger has enabled
   ITM stimulus port 0, so this is safe without the debugger attached. */
int __io_putchar(int ch)
{
    ITM_SendChar((uint32_t)ch);
    return ch;
}

/* One line of live data: wheel velocities, tilt and raw accelerometer */
void Data_Print(void)
{
    printf("vel L %6.2f R %6.2f rad/s | theta %6.2f deg | ax %6d ay %6d | "
           "theta_dot %7.1f deg/s | theta_ddot %8.1f deg/s^2\n",
           enc[ENC_LEFT].velocity_rad_s,
           enc[ENC_RIGHT].velocity_rad_s,
           imu_data.theta * RAD_TO_DEG,
           imu_data.acc_raw[0], imu_data.acc_raw[1],
           imu_data.theta_dot  * RAD_TO_DEG,
           imu_data.theta_ddot * RAD_TO_DEG);
}

/* Start-up report after IMU_Init() succeeds */
void Data_PrintIMUStatus(void)
{
    /* The library prints "LSB_Sensitivity_GYRO: 131" during its set-up;
       IMU_Init() changes the range afterwards, so this line is the real one */
    printf("MPU-6050 OK, WHO_AM_I = 0x%02X, gyro %.1f LSB per deg/s, x bias %.2f deg/s, "
           "start angle %.1f deg\n",
           imu_data.imu_id, imu_data.gyro_lsb_per_dps, imu_data.gyro_bias_x,
           imu_data.theta * RAD_TO_DEG);
}

/* Start-up report if IMU_Init() fails: the sensor did not answer with 0x68.
   HAL error code 0x04 = no acknowledge (wiring, power or address),
   0x20 = timeout (bus stuck or missing pull-ups). */
void Data_PrintIMUError(void)
{
    printf("MPU-6050 not responding on I2C3: WHO_AM_I read 0x%02X, HAL error code 0x%02lX\n",
           imu_data.imu_id, (unsigned long)hi2c3.ErrorCode);
}
