/**
 * @file mpu_9250_data_processing_functions.c
 *
 * This file contains the variables and definitions for the functions
 * that involve gathering and processing the MPU-9250 sensor data.
 *
 *  Version: 2.3
 *  Created on: June 25th, 2020
 *      Author: Danijel Camdzic
 */

#include "mpu_9250_data_processing_functions.h"
#include "peripheral_functions.h"

/* Variables */
KalmanFilter KalmanX; /**< Kalman filter for X-axis */
KalmanFilter KalmanY; /**< Kalman filter for Y-axis */
KalmanFilter KalmanZ; /**< Kalman filter for Z-axis */

uint8_t buf[100] = {0}; /**< Array used for sendind messages to UART */
uint8_t data[8] = {0}; 	/**< Array used for sending data bytes */

int16_t X_AXIS = 0;	  /**< Variable used to receive 2 bytes from the X-axis measurements */
int16_t Y_AXIS = 0;	  /**< Variable used to receive 2 bytes from the Y-axis measurements  */
int16_t Z_AXIS = 0;	  /**< Variable used to receive 2 bytes from the Z-axis measurements  */
float X_OUTPUT = 0.0; /**< Final X-axis value */
float Y_OUTPUT = 0.0; /**< Final Y-axis value */
float Z_OUTPUT = 0.0; /**< Final Z-axis value */

uint8_t ASAX = 0; /**< X-axis adjustment value for the magnetometer */
uint8_t ASAY = 0; /**< Y-axis adjustment value for the magnetometer */
uint8_t ASAZ = 0; /**< Z-axis adjustment value for the magnetometer */

float pitch = 0.0; /**< Pitch value */
float roll = 0.0;  /**< Roll value */
float yaw = 0.0;   /**< Yaw value */

float xaccel = 0.0;
float yaccel = 0.0;
float zaccel = 0.0;

float MagneticOffset[3] = {-0.5, 3.0, -61.5}; /**< Magnetic offset for the magnetometer sensor. Calculated during one
																						  of the test runs from the function inside this project. */

/* Sensor and data functions */
/**
 * @brief KalmanFilterXYZ
 *
 * Function takes the structure of the Kalman filter of the appropriate axis, the position calculated by the
 * accelerometer data, velocity calculated from the gyroscope data and the interval of time that had passed
 * between the measurements. It is a void function because all the calculations are stored back into the Kalman
 * filter structure to be used again in the next calculation.
 *
 */
void KalmanFilterXYZ(KalmanFilter *Kalman, float GyroRate, float AccelAngle, float dt)
{
	/* State prediction. This step uses the previous value to predict the next value */
	Kalman->Xk[0] = Kalman->Xk[0] + dt * GyroRate - dt * Kalman->Xk[1];

	/* Eliminating the drift from the gyroscope velocity readings */
	Kalman->NoDriftGyroRate = GyroRate - Kalman->Xk[1];

	/* Error covariance prediction. This step uses the prevoious error covariance to determine the current */
	Kalman->P[0][0] = Kalman->P[0][0] - dt * Kalman->P[1][0] - dt * (Kalman->P[0][1] - dt * Kalman->P[1][1]) + Kalman->Q[0];
	Kalman->P[0][1] = Kalman->P[0][1] - dt * Kalman->P[1][1];
	Kalman->P[1][0] = Kalman->P[1][0] - dt * Kalman->P[1][1];
	Kalman->P[1][1] = Kalman->P[1][1] + Kalman->Q[1];

	/* Kalman gain computation. It is later used as weight to determine the estimation of the position */
	float K[2];
	K[0] = Kalman->P[0][0] / ((float)(Kalman->P[0][0] + Kalman->R));
	K[1] = Kalman->P[1][0] / ((float)(Kalman->P[0][0] + Kalman->R));

	/* Estimate computation. In this step, the algorithm determines the difference between the actual reading and
	the estimated readings */
	float difference = AccelAngle - Kalman->Xk[0];
	Kalman->Xk[0] += K[0] * difference;
	Kalman->Xk[1] += K[1] * difference;

	/* Error covariance computation. Larger Pk signifies a bigger error in estimation */
	float P0 = Kalman->P[0][0];
	float P1 = Kalman->P[0][1];

	Kalman->P[0][0] -= K[0] * P0;
	Kalman->P[0][1] -= K[0] * P1;
	Kalman->P[1][0] -= K[1] * P0;
	Kalman->P[1][1] -= K[1] * P1;
}

/**
 * @brief InitKalmanXYZ
 *
 * Function initializes the Kalman filter structures with appropriate values. Some values were chosen
 * based on experiments and judgement. They were given subjectively.
 *
 */
void InitKalmanXYZ(void)
{
	/* Kalman filter structure for the X-axis */
	KalmanX.Q[0] = 0.001;
	KalmanX.Q[1] = 0.003;
	KalmanX.R = 0.003;
	KalmanX.Xk[0] = 0.0;
	KalmanX.Xk[1] = 0.0;
	KalmanX.NoDriftGyroRate = 0.0;
	KalmanX.P[0][0] = 0.0;
	KalmanX.P[0][1] = 0.0;
	KalmanX.P[1][0] = 0.0;
	KalmanX.P[1][1] = 0.0;

	/* Kalman filter structure for the Y-axis */
	KalmanY.Q[0] = 0.001;
	KalmanY.Q[1] = 0.003;
	KalmanY.R = 0.003;
	KalmanY.Xk[0] = 0.0;
	KalmanY.Xk[1] = 0.0;
	KalmanY.NoDriftGyroRate = 0.0;
	KalmanY.P[0][0] = 0.0;
	KalmanY.P[0][1] = 0.0;
	KalmanY.P[1][0] = 0.0;
	KalmanY.P[1][1] = 0.0;

	/* Kalman filter structure for the Z-axis */
	KalmanZ.Q[0] = 0.001;
	KalmanZ.Q[1] = 0.003;
	KalmanZ.R = 0.003;
	KalmanZ.Xk[0] = 0.0;
	KalmanZ.Xk[1] = 0.0;
	KalmanZ.NoDriftGyroRate = 0.0;
	KalmanZ.P[0][0] = 0.0;
	KalmanZ.P[0][1] = 0.0;
	KalmanZ.P[1][0] = 0.0;
	KalmanZ.P[1][1] = 0.0;
}

/**
 * @brief WakeUpSensor
 *
 * Function checks the connection to the MPU-9250 sensor by reading the WHO_AM_I register and then writes to
 * power management register to turn off sleep mode.
 *
 */
void WakeUpSensor(void)
{
	/* Reading the WHO_AM_I register of the Accel-Gyro Slave */
	I2C_Read(AccelGyroAddress, WHO_AM_I, data);
	if (data[0] != 0x68)
	{
		strcpy((char *)buf, "Invalid WHO_AM_I Register value!\r\n");
		SendToUART(buf);
	}

	data[0] = 0x00;
	/* Writing to the PWR_MGMT_1 register to turn off the sleep mode */
	I2C_Write(AccelGyroAddress, PWR_MGMT_1, data);
}

/**
 * @brief GetMagnetometerAdjustment
 *
 * Function checks the connection to the magnetometer sensor, reads the magnetometer adjustment values
 * and sets the appropriate mode of operation for the sensor to be read later. It uses the timer 3 peripheral to
 * time the waiting necessary to switch between modes of operation.
 *
 */
void GetMagnetometerAdjustment(void)
{
	/* Reading the WIA register (ID of Magnetometer) */
	I2C_Read(MagnetometerAddress, WIA, data);
	if (data[0] != 0x48)
	{
		strcpy((char *)buf, "Incorrect WIA ID!\r\n");
		SendToUART(buf);
	}

	data[0] = 0x00;
	/* Writing to the CNTL1_AD register (POWER-DOWN MODE)*/
	I2C_Write(MagnetometerAddress, CNTL1_AD, data);

	/* Starting the timer 3 peripheral and counting for 500 ms to change the mode of operation */
	TIM3->CNT = 0x00;
	TIM3->CR1 |= TIM_CR1_CEN; /**< Enabling the timer 3 peripheral */
	while (1)
	{
		if (TIM3->CNT > 1000)
			break;
	}
	TIM3->CR1 &= ~TIM_CR1_CEN; /**< Disabling the timer 3 peripheral */
	TIM3->CNT = 0x00;

	data[0] = 0x0F;
	/* Writing to the CNTL1_AD register (FUSE-ROM-MODE)*/
	I2C_Write(MagnetometerAddress, CNTL1_AD, data);

	TIM3->CNT = 0x00;
	TIM3->CR1 |= TIM_CR1_CEN;
	while (1)
	{
		if (TIM3->CNT > 1000)
			break;
	}
	TIM3->CR1 &= ~TIM_CR1_CEN;
	TIM3->CNT = 0x00;

	/* Reading the adjustment values of the magnetometer */
	for (uint8_t i = 0; i < 3; i++)
	{
		I2C_Read(MagnetometerAddress, (ASAX_REG + i), (data + i));
	}

	ASAX = data[0];
	ASAY = data[1];
	ASAZ = data[2];

	data[0] = 0x00;
	/* Writing to the CNTL1_AD register (POWER-DOWN MODE)*/
	I2C_Write(MagnetometerAddress, CNTL1_AD, data);

	TIM3->CNT = 0x00;
	TIM3->CR1 |= TIM_CR1_CEN;
	while (1)
	{
		if (TIM3->CNT > 1000)
			break;
	}
	TIM3->CR1 &= ~TIM_CR1_CEN;
	TIM3->CNT = 0x00;

	data[0] = 0x02;
	/* Writing to the CNTL1_AD register (Continuous Mode 2)*/
	I2C_Write(MagnetometerAddress, CNTL1_AD, data);

	TIM3->CNT = 0x00;
	TIM3->CR1 |= TIM_CR1_CEN;
	while (1)
	{
		if (TIM3->CNT > 1000)
			break;
	}
	TIM3->CR1 &= ~TIM_CR1_CEN;
	TIM3->CNT = 0x00;

	// data[0] = 0xFF;
	/* Checking value of CNTL1_AD */
	// I2C_Read(MagnetometerAddress, CNTL1_AD, data);
}

/**
 * @brief SwitchSlaveDevice
 *
 * Function switches between the AccelGyro sensor and Magnetometer sensor based on the command it receives.
 *
 */
void SwitchSlaveDevice(uint8_t command)
{
	data[0] = command;
	/* Writing to the INT_BYPASS_CONFIG_AD register */
	I2C_Write(AccelGyroAddress, INT_BYPASS_CONFIG_AD, data);
}

/**
 * @brief CalculateAccelGyroError
 *
 * Function that calculates the static noise error of the accelerometer and gyroscope sensor by doing 100 reading of each
 * while the sensor sits still in order to find the average difference of the readings and then subtract it from the actual
 * readings.
 *
 */
void CalculateAccelGyroError(double *AccelXError, double *AccelYError, double *AccelZError, double *GyroXError, double *GyroYError, double *GyroZError)
{
	/* ------------------------------------------ Calculating the ERROR  ---------------------------------------- */
	uint8_t counter = 0;
	while (counter < 100) /**< Read 100 times */
	{
		/* --------------------------------------- ACCELEROMETER ERROR --------------------------------------------- */
		for (uint8_t i = 0; i < 6; i++)
		{
			I2C_Read(AccelGyroAddress, (ACCEL_XOUT_H + i), (data + i));
		}
		X_AXIS = (data[0] << 8) | data[1];
		Y_AXIS = (data[2] << 8) | data[3];
		Z_AXIS = (data[4] << 8) | data[5];

		*AccelXError += X_AXIS / 16384.0;
		*AccelYError += Y_AXIS / 16384.0;
		*AccelZError += Z_AXIS / 16384.0;

		counter++;
	}
	*AccelXError = *AccelXError / 100.0;
	*AccelYError = *AccelYError / 100.0;
	*AccelZError = *AccelZError / 100.0 - 1; /**< Don't want to count gravity as noise! */
	counter = 0;

	while (counter < 100) /**< Read 100 times */
	{
		/* ----------------------------------------- GYROSCOPE ERROR --------------------------------------------- */
		for (uint8_t i = 0; i < 6; i++)
		{
			I2C_Read(AccelGyroAddress, (GYRO_XOUT_H + i), (data + i));
		}
		X_AXIS = (data[0] << 8) | data[1];
		Y_AXIS = (data[2] << 8) | data[3];
		Z_AXIS = (data[4] << 8) | data[5];

		*GyroXError = *GyroXError + X_AXIS / 131.0;
		*GyroYError = *GyroYError + Y_AXIS / 131.0;
		*GyroZError = *GyroZError + Z_AXIS / 131.0;
		counter++;
	}
	*GyroXError = *GyroXError / 100.0;
	*GyroYError = *GyroYError / 100.0;
	*GyroZError = *GyroZError / 100.0;
}

/**
 * @brief ReadAccelerometer
 *
 * Function that reads the accelerometer data by starting from the X-axis high byte and itterating through 5 more registers
 * which represent the rest of the data registers in order X-Y-Z.
 *
 */
void ReadAccelerometer(void)
{
	for (uint8_t i = 0; i < 6; i++)
	{
		I2C_Read(AccelGyroAddress, (ACCEL_XOUT_H + i), (data + i));
	}
}

/**
 * @brief ReadGyroscope
 *
 * Function that reads the gyroscope data by starting from the X-axis high byte and itterating through 5 more registers
 * which represent the rest of the data registers in order X-Y-Z.
 *
 */
void ReadGyroscope(void)
{
	for (uint8_t i = 0; i < 6; i++)
	{
		I2C_Read(AccelGyroAddress, (GYRO_XOUT_H + i), (data + i));
	}
}

/**
 * @brief ReadMagnetometer
 *
 * Function that reads the magnetometer data by starting from the X-axis low byte and itterating through 6 more registers
 * which represent the rest of the data registers in order X-Y-Z. The 7th register is the STATUS2 register which need to be read
 * to complete the read cycle as per datasheet of the MPU-9250 sensor.
 *
 */
void ReadMagnetometer(void)
{
	/* Don't want to check the DataReady register because the entire system is slowed then */
	/*data[0] = 0x00;
	while((data[0] & 0x01) == 0) {
			I2C_Read(MagnetometerAddress, STATUS1, data);
	}*/

	for (uint8_t i = 0; i < 7; i++)
	{
		I2C_Read(MagnetometerAddress, (MAGN_XOUT_L + i), (data + i));
	}
}

/**
 * @brief CalibrateMagnetometer
 *
 * Function that calibrated the magnetometer by doing 60000 readings from the sensor that is moved in
 * all direction in order to find the maximum and minimums for all axises and adjust for hard-iron distortion.
 * The values have already been calculated and stored inside an array because the process takes too long and is
 * unnecessary to do at every start of  the program. It was done in one of the test runs and the values have been found.
 *
 */
void CalibrateMagnetometer(void)
{
	uint32_t i = 0;
	uint32_t calibration_count = 60000;
	float MAGX_MAX = 0.0;
	float MAGY_MAX = 0.0;
	float MAGZ_MAX = 0.0;
	float MAGX_MIN = 0.0;
	float MAGY_MIN = 0.0;
	float MAGZ_MIN = 0.0;

	/* Turn on the LED to singify the start of the calibration */
	GPIOA->ODR ^= GPIO_ODR_5;

	for (i = 0; i < calibration_count; i++)
	{ /**< Do 60000 measurements */
		ReadMagnetometer();

		/* Find the appropriate values */
		X_AXIS = (data[1] << 8) | data[0];
		Y_AXIS = (data[3] << 8) | data[2];
		Z_AXIS = (data[5] << 8) | data[4];

		X_AXIS = (X_AXIS << 2);
		Y_AXIS = (Y_AXIS << 2);
		Z_AXIS = (Z_AXIS << 2);

		X_OUTPUT = ((float)(X_AXIS)) / 4.0;
		Y_OUTPUT = ((float)(Y_AXIS)) / 4.0;
		Z_OUTPUT = ((float)(Z_AXIS)) / 4.0;

		/* Finding the maximum and minimum of each axis */
		if (X_OUTPUT > MAGX_MAX)
			MAGX_MAX = X_OUTPUT;
		if (X_OUTPUT < MAGX_MIN)
			MAGX_MIN = X_OUTPUT;
		if (Y_OUTPUT > MAGY_MAX)
			MAGY_MAX = Y_OUTPUT;
		if (Y_OUTPUT < MAGY_MIN)
			MAGY_MIN = Y_OUTPUT;
		if (Z_OUTPUT > MAGZ_MAX)
			MAGZ_MAX = Z_OUTPUT;
		if (Z_OUTPUT < MAGZ_MIN)
			MAGZ_MIN = Z_OUTPUT;
	}

	/* Store the magnetic offsets from the hard iron distortion in order to subtract from the actual readings */
	MagneticOffset[0] = ((float)MAGX_MAX + (float)MAGX_MIN) / 2;
	MagneticOffset[1] = ((float)MAGY_MAX + (float)MAGY_MIN) / 2;
	MagneticOffset[2] = ((float)MAGZ_MAX + (float)MAGZ_MIN) / 2;

	/* Turn off the LED to singify the end of the calibration */
	GPIOA->ODR ^= GPIO_ODR_5;
}

