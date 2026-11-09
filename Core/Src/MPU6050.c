/* Source: https://github.com/creative-apple/STM32-MPU6050 (downloaded 8 Nov 2026).
 * Changed for the MECHA3500 balancing robot (only these lines):
 *   - the IMU is on I2C3, so hi2c1 -> hi2c3
 *   - this project has no i2c.h (CubeMX keeps the handles in main.c), so the
 *     handle is declared here instead of #include "i2c.h"
 *   - added #include <stdio.h> for the printf calls
 * Do not call MPU6050_DataReady(): it reads PB5, which is TMC_EN on this board.
 *
 * Everything from the "AI generated glue code" banner to the end of this file
 * was written by Claude (Anthropic) for this project and is NOT part of the
 * creative-apple library. It only calls the library's functions. */
#include "MPU6050.h"
#include "main.h"
#include <stdio.h>            /* printf, used below */
extern I2C_HandleTypeDef hi2c3;   /* was: #include "i2c.h" */

Struct_MPU6050 MPU6050;

static float LSB_Sensitivity_ACC;
static float LSB_Sensitivity_GYRO;

void MPU6050_Writebyte(uint8_t reg_addr, uint8_t val)
{
	HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, &val, 1, 1);
}

void MPU6050_Writebytes(uint8_t reg_addr, uint8_t len, uint8_t* data)
{
	HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, 1);
}

void MPU6050_Readbyte(uint8_t reg_addr, uint8_t* data)
{
	HAL_I2C_Mem_Read(&hi2c3, MPU6050_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, data, 1, 1);
}

void MPU6050_Readbytes(uint8_t reg_addr, uint8_t len, uint8_t* data)
{
	HAL_I2C_Mem_Read(&hi2c3, MPU6050_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, 1);
}

void MPU6050_Initialization(void)
{
	HAL_Delay(50);
	uint8_t who_am_i = 0;
	printf("Checking MPU6050...\n");

	MPU6050_Readbyte(MPU6050_WHO_AM_I, &who_am_i);
	if(who_am_i == 0x68)
	{
		printf("MPU6050 who_am_i = 0x%02x...OK\n", who_am_i);
	}
	else
	{
		printf("ERROR!\n");
		printf("MPU6050 who_am_i : 0x%02x should be 0x68\n", who_am_i);
		while(1)
		{
			printf("who am i error. Can not recognize mpu6050\n");
			HAL_Delay(100);
		}
	}

	//Reset the whole module before initialization
	MPU6050_Writebyte(MPU6050_PWR_MGMT_1, 0x1<<7);
	HAL_Delay(100);

	//Power Management setting
	/* Default is sleep mode
	 * necessary to wake up MPU6050*/
	MPU6050_Writebyte(MPU6050_PWR_MGMT_1, 0x00);
	HAL_Delay(50);

	//Sample rate divider
	/*Sample Rate = Gyroscope Output Rate / (1 + SMPRT_DIV) */
	//	MPU6050_Writebyte(MPU6050_SMPRT_DIV, 0x00); // ACC output rate is 1kHz, GYRO output rate is 8kHz
	MPU6050_Writebyte(MPU6050_SMPRT_DIV, 39); // Sample Rate = 200Hz
	HAL_Delay(50);

	//FSYNC and DLPF setting
	/*DLPF is set to 0*/
	MPU6050_Writebyte(MPU6050_CONFIG, 0x00);
	HAL_Delay(50);

	//GYRO FULL SCALE setting
	/*FS_SEL  Full Scale Range
	  0    	+-250 degree/s
	  1		+-500 degree/s
	  2		+-1000 degree/s
	  3		+-2000 degree/s	*/
	uint8_t FS_SCALE_GYRO = 0x0;
	MPU6050_Writebyte(MPU6050_GYRO_CONFIG, FS_SCALE_GYRO<<3);
	HAL_Delay(50);

	//ACCEL FULL SCALE setting
	/*FS_SEL  Full Scale Range
	  0    	+-2g
	  1		+-4g
	  2		+-8g
	  3		+-16g	*/
	uint8_t FS_SCALE_ACC = 0x0;
	MPU6050_Writebyte(MPU6050_ACCEL_CONFIG, FS_SCALE_ACC<<3);
	HAL_Delay(50);

	MPU6050_Get_LSB_Sensitivity(FS_SCALE_GYRO, FS_SCALE_ACC);
	printf("LSB_Sensitivity_GYRO: %f, LSB_Sensitivity_ACC: %f\n",LSB_Sensitivity_GYRO, LSB_Sensitivity_ACC);

	//Interrupt PIN setting
	uint8_t INT_LEVEL = 0x0; //0 - active high, 1 - active low
	uint8_t LATCH_INT_EN = 0x0; //0 - INT 50us pulse, 1 - interrupt clear required
	uint8_t INT_RD_CLEAR = 0x1; //0 - INT flag cleared by reading INT_STATUS, 1 - INT flag cleared by any read operation
	MPU6050_Writebyte(MPU6050_INT_PIN_CFG, (INT_LEVEL<<7)|(LATCH_INT_EN<<5)|(INT_RD_CLEAR<<4)); //
	HAL_Delay(50);

	//Interrupt enable setting
	uint8_t DATA_RDY_EN = 0x1; // 1 - enable, 0 - disable
	MPU6050_Writebyte(MPU6050_INT_ENABLE, DATA_RDY_EN);
	HAL_Delay(50);

	printf("MPU6050 setting is finished\n");
}
/*Get Raw Data from sensor*/
void MPU6050_Get6AxisRawData(Struct_MPU6050* mpu6050)
{
	uint8_t data[14];
	MPU6050_Readbytes(MPU6050_ACCEL_XOUT_H, 14, data);

	mpu6050->acc_x_raw = (data[0] << 8) | data[1];
	mpu6050->acc_y_raw = (data[2] << 8) | data[3];
	mpu6050->acc_z_raw = (data[4] << 8) | data[5];

	mpu6050->temperature_raw = (data[6] << 8) | data[7];

	mpu6050->gyro_x_raw = ((data[8] << 8) | data[9]);
	mpu6050->gyro_y_raw = ((data[10] << 8) | data[11]);
	mpu6050->gyro_z_raw = ((data[12] << 8) | data[13]);
}

void MPU6050_Get_LSB_Sensitivity(uint8_t FS_SCALE_GYRO, uint8_t FS_SCALE_ACC)
{
	switch(FS_SCALE_GYRO)
	{
	case 0:
		LSB_Sensitivity_GYRO = 131.f;
		break;
	case 1:
		LSB_Sensitivity_GYRO = 65.5f;
		break;
	case 2:
		LSB_Sensitivity_GYRO = 32.8f;
		break;
	case 3:
		LSB_Sensitivity_GYRO = 16.4f;
		break;
	}
	switch(FS_SCALE_ACC)
	{
	case 0:
		LSB_Sensitivity_ACC = 16384.f;
		break;
	case 1:
		LSB_Sensitivity_ACC = 8192.f;
		break;
	case 2:
		LSB_Sensitivity_ACC = 4096.f;
		break;
	case 3:
		LSB_Sensitivity_ACC = 2048.f;
		break;
	}
}

/*Convert Unit. acc_raw -> g, gyro_raw -> degree per second*/
void MPU6050_DataConvert(Struct_MPU6050* mpu6050)
{
	//printf("LSB_Sensitivity_GYRO: %f, LSB_Sensitivity_ACC: %f\n",LSB_Sensitivity_GYRO,LSB_Sensitivity_ACC);
	mpu6050->acc_x = mpu6050->acc_x_raw / LSB_Sensitivity_ACC;
	mpu6050->acc_y = mpu6050->acc_y_raw / LSB_Sensitivity_ACC;
	mpu6050->acc_z = mpu6050->acc_z_raw / LSB_Sensitivity_ACC;

	mpu6050->temperature = (float)(mpu6050->temperature_raw)/340+36.53;

	mpu6050->gyro_x = mpu6050->gyro_x_raw / LSB_Sensitivity_GYRO;
	mpu6050->gyro_y = mpu6050->gyro_y_raw / LSB_Sensitivity_GYRO;
	mpu6050->gyro_z = mpu6050->gyro_z_raw / LSB_Sensitivity_GYRO;
}


int MPU6050_DataReady(void)
{
	//old school way
	/*
	static uint8_t INT_STATE_FLAG = 0;
	static uint8_t DATA_RDY_INT_FLAG = 0;
	static uint8_t INT_PIN = 0;
	INT_PIN = LL_GPIO_IsInputPinSet(MPU6050_INT_PORT, MPU6050_INT_PIN);
	if(INT_PIN == 1)
	{
		MPU6050_Readbyte(MPU6050_INT_STATUS, &INT_STATE_FLAG); //flag cleared automatically within the sensor
		DATA_RDY_INT_FLAG = INT_STATE_FLAG & 0x01;
		if(DATA_RDY_INT_FLAG == 1)
		{
			INT_STATE_FLAG = 0; //flag clearing
			DATA_RDY_INT_FLAG = 0;
			INT_PIN = 0;
			return 1;
		}
	}
	return 0;
	 */
	return HAL_GPIO_ReadPin(MPU6050_INT_PORT, MPU6050_INT_PIN);
}

void MPU6050_ProcessData(Struct_MPU6050* mpu6050)
{
	MPU6050_Get6AxisRawData(mpu6050);
	MPU6050_DataConvert(mpu6050);
}



/* ========================================================================= */
/* ========================================================================= */
/*                         AI generated glue code                            */
/* ========================================================================= */
/* ========================================================================= */
/* Written by Claude (Anthropic), 8-9 Nov 2026, for the MECHA3500 balancing
 * robot. Not part of the creative-apple library above: it only calls the
 * library's functions. Declarations, settings and the IMU mounting notes are
 * in Robot.h.
 *
 *   IMU_Init()          Checks WHO_AM_I first, because the library's
 *                       MPU6050_Initialization() loops forever if the sensor
 *                       does not answer. Then runs the library set-up
 *                       (gyro +/-250 deg/s, accelerometer +/-2 g), turns on
 *                       the chip's low-pass filter (see CHANGE below) and
 *                       runs IMU_CalibrateGyro().
 *   IMU_CalibrateGyro() Gyro bias and starting angle. Keep the robot still.
 *   IMU_Update()        Call every IMU_SAMPLE_MS. Reads the sensor and updates
 *                       the tilt estimate in imu_data (+ = leaning forwards):
 *     theta_dot   gyro rate about x, bias removed
 *     theta_ddot  derivative of the gyro rate, low-pass filtered
 *     theta       gyro rate integrated, pulled slowly towards the
 *                 accelerometer angle atan2(-ay, az) so it cannot drift
 *                 (complementary filter, time constant IMU_TILT_TAU_S;
 *                 0 = gyro only)
 *
 * Timing uses the Cortex-M4 DWT cycle counter (84 MHz). A gap longer than
 * IMU_MAX_DT_S (e.g. a breakpoint) is not integrated across: the next
 * IMU_Update() takes a fresh baseline and theta keeps its last value; the
 * accelerometer correction removes the missed rotation over a few
 * IMU_TILT_TAU_S.
 */
#include "Robot.h"   /* IMU_Data and settings. Robot.h does not include
                        MPU6050.h, which has no include guard. */
#include <math.h>
#include <string.h>

IMU_Data imu_data;   /* declared in Robot.h; add it to Live Expressions */

uint8_t IMU_Init(void)
{
    memset(&imu_data, 0, sizeof(imu_data));

    /* Start the DWT cycle counter, used to measure dt between samples */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    /* Ask for WHO_AM_I with the library's own read function first, and only
       run the library set-up once the sensor has replied with 0x68 */
    MPU6050_Readbyte(MPU6050_WHO_AM_I, &imu_data.imu_id);
    if (imu_data.imu_id != 0x68U) {
        imu_data.imu_ok = 0U;
        return 0U;
    }

    MPU6050_Initialization();             /* library set-up */

    /* CHANGE (Claude, 9 Nov 2026): turn on the MPU-6050's digital low-pass
       filter (DLPF). The library leaves it off, so every reading was a single
       unfiltered sample with 256 Hz of bandwidth, and motor vibration at
       hundreds of Hz aliased into the 200 Hz readings as fake tilt motion
       (seen as ay swinging by 0.7 g and theta_dot by +/-50 deg/s with the
       robot sitting still). The filter acts before sampling, so it removes
       that vibration before it can alias; no filter after sampling can.
         CONFIG     DLPF_CFG = 3: gyro 42 Hz, accelerometer 44 Hz,
                    about 5 ms delay
         SMPRT_DIV  0: data registers update at 1 kHz (with the DLPF on the
                    gyro runs at 1 kHz, not 8 kHz), so every IMU_Update()
                    gets a fresh filtered sample
       These overwrite the library's CONFIG = 0x00 and SMPRT_DIV = 39. */
    MPU6050_Writebyte(MPU6050_CONFIG,    0x03);
    MPU6050_Writebyte(MPU6050_SMPRT_DIV, 0x00);
    HAL_Delay(50);                        /* let the filter settle before calibrating */

    imu_data.imu_ok = IMU_CalibrateGyro();
    return imu_data.imu_ok;
}

uint8_t IMU_CalibrateGyro(void)
{
    /* Average the gyro x rate with the robot still: that average is the bias.
       Average the accelerometer too, for the starting angle. */
    float sum_gx = 0.0f, sum_ay = 0.0f, sum_az = 0.0f;

    for (uint32_t i = 0; i < IMU_GYRO_CAL_SAMPLES; i++) {
        MPU6050_ProcessData(&MPU6050);    /* library: read and convert */
        sum_gx += MPU6050.gyro_x;         /* deg/s */
        sum_ay += MPU6050.acc_y;          /* g     */
        sum_az += MPU6050.acc_z;          /* g     */
        HAL_Delay(IMU_SAMPLE_MS);
    }

    imu_data.gyro_bias_x = sum_gx / (float)IMU_GYRO_CAL_SAMPLES;
    imu_data.theta_acc   = atan2f(-sum_ay, sum_az);
    imu_data.theta       = imu_data.theta_acc;
    return 1U;
}

void IMU_Update(void)
{
    IMU_Data *d = &imu_data;
    uint32_t now = DWT->CYCCNT;

    /* Gyro rate (theta_dot) and accelerometer angle (drift reference) */
    float rate = d->theta_dot;
    if (d->imu_ok) {
        MPU6050_ProcessData(&MPU6050);    /* library: raw values, then g and deg/s */
        d->acc_raw[0]  = MPU6050.acc_x_raw;
        d->acc_raw[1]  = MPU6050.acc_y_raw;
        d->acc_raw[2]  = MPU6050.acc_z_raw;
        d->gyro_raw[0] = MPU6050.gyro_x_raw;
        d->gyro_raw[1] = MPU6050.gyro_y_raw;
        d->gyro_raw[2] = MPU6050.gyro_z_raw;
        rate = GYRO_SIGN_X * (MPU6050.gyro_x - d->gyro_bias_x) * DEG_TO_RAD;
        d->theta_acc = atan2f(-MPU6050.acc_y, MPU6050.acc_z);
    }

    float dt = (float)(now - d->last_cycles) / (float)SystemCoreClock;
    d->last_cycles = now;

    if (!d->timing_valid || dt <= 0.0f || dt > IMU_MAX_DT_S) {
        /* First sample, or a long halt: no integration or differentiation
           across the gap. theta keeps its last value. */
        d->theta_dot    = rate;
        d->timing_valid = 1U;
        return;
    }
    d->dt = dt;

    /* theta_ddot: derivative of the gyro rate, low-pass filtered */
    float a_dd      = IMU_LowPassAlpha(dt, IMU_DDOT_CUTOFF_HZ);
    float rate_prev = d->theta_dot;
    d->theta_ddot  += a_dd * ((rate - rate_prev) / dt - d->theta_ddot);
    d->theta_dot    = rate;

    /* theta: integrate the gyro (trapezoidal), then pull slowly towards the
       accelerometer angle so gyro bias drift cannot build up */
    float theta_gyro = d->theta + 0.5f * (rate + rate_prev) * dt;
    float k = (IMU_TILT_TAU_S > 0.0f) ? dt / (IMU_TILT_TAU_S + dt) : 0.0f;
    d->theta = theta_gyro + k * (d->theta_acc - theta_gyro);
}

float IMU_LowPassAlpha(float dt, float cutoff_hz)
{
    /* First-order low-pass: y += alpha * (x - y), alpha = dt / (tau + dt) */
    float tau = 1.0f / (2.0f * 3.14159265f * cutoff_hz);
    return dt / (tau + dt);
}

/* Values from the latest IMU_Update(). Any pointer may be NULL. */
void IMU_GetAccel(int16_t *ax, int16_t *ay, int16_t *az)
{
    if (ax) *ax = imu_data.acc_raw[0];
    if (ay) *ay = imu_data.acc_raw[1];
    if (az) *az = imu_data.acc_raw[2];
}

void IMU_GetGyro(int16_t *gx, int16_t *gy, int16_t *gz)
{
    if (gx) *gx = imu_data.gyro_raw[0];
    if (gy) *gy = imu_data.gyro_raw[1];
    if (gz) *gz = imu_data.gyro_raw[2];
}

void IMU_GetTilt(float *theta, float *theta_dot, float *theta_ddot)
{
    if (theta)      *theta      = imu_data.theta;
    if (theta_dot)  *theta_dot  = imu_data.theta_dot;
    if (theta_ddot) *theta_ddot = imu_data.theta_ddot;
}
