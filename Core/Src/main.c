/**
 * @file main.c
 *
 * Project that uses STM32F030R8 microcontroller and IMU-9250 sensor
 * in order to determine the orientation of the sensor and display it's
 * orientation on the PC. It shows it's orientation in all 3 axises and has
 * no or neglegible drift over time. Processing Toolbox software is used to
 * receive the data from the microcontroller and display the box representing
 * the sensor orientation.
 *
 *  Version: 2.3
 *  Created on: June 25th, 2020
 *      Author: Danijel Camdzic
 */

/* Include files which are necessary for the project */
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "main.h"
#include "peripheral_functions.h"
#include "mpu_9250_data_processing_functions.h"

/**
 * @brief main
 *
 * Function that does all the initializations of the peripherals, the microcontroller and the sensor. It features a
 * forever loop inside of which the data from the accelerometer, gyroscope and magnetometer sensor is acquired and refined
 * in order to represent the actual readings suitable for real-life orientation determination.
 *
 */
int main(void)
{
	/* Initialize clock to max frequency of 48 MHz */
	Clock_Init();

	/* Initialize the general input output pins */
	GPIO_Init();

	/* Initialize the timer 3 */
	TIMER3_Init();

	/* Initialize UART */
	USART2_Init();

	/* Initialize I2C1 */
	I2C1_Init();

	/* Initiliaze Kalman filters with starting values */
	InitKalmanXYZ();

	/* Initialize sensor by writing in the power management register */
	WakeUpSensor();

	/* Switch to magnetometer slave in order to get the adjustment values */
	SwitchSlaveDevice(MAGNETOMETER_SLAVE);

	GetMagnetometerAdjustment();

	/* Calibrate the adjustment values to show true readings */
	float calibrationX = (((ASAX - 128) * 0.5) / 128.0 + 1.0);
	float calibrationY = (((ASAY - 128) * 0.5) / 128.0 + 1.0);
	float calibrationZ = (((ASAZ - 128) * 0.5) / 128.0 + 1.0);

	/* Function which calibrated the magnetometer by reading 1000 times from it while it's being moved
	in all directions to find the appropriate offset. Not necessary to include as the values have already been
  found in one of the test runs	*/
	// CalibrateMagnetometer();

	MagneticOffset[0] = MagneticOffset[0] * MagneticResolution * calibrationX;
	MagneticOffset[1] = MagneticOffset[1] * MagneticResolution * calibrationY;
	MagneticOffset[2] = MagneticOffset[2] * MagneticResolution * calibrationZ;

	/* Continue with the accel+gyro sensor */
	SwitchSlaveDevice(ACCEL_GYRO_SLAVE);

	double AccelXError = 0.0;
	double AccelYError = 0.0;
	double AccelZError = 0.0;
	double GyroXError = 0.0;
	double GyroYError = 0.0;
	double GyroZError = 0.0;

	/* Calculate the sensor noise by reading from the sensor while it's sitting still and find the deviations */
	CalculateAccelGyroError(&AccelXError, &AccelYError, &AccelZError, &GyroXError, &GyroYError, &GyroZError);

	/* Variables used to hold the gyroscope angles */
	float GyroAngleX = 0.0;
	float GyroAngleY = 0.0;
	float GyroAngleZ = 0.0;

	float yaw_tempf = 0.0; /**< Used to reduce the noise of the magnetometer yaw calculation */
	float yaw_tempf2 = 0.0;
	float vrijeme = 0.0;

	float repeat = 0.0;
	int flag = 0;
	float yaw_starting = 0.0;

	float xdistance = 0.0;
	float ydistance = 0.0;
	float zdistance = 0.0;

	/* Enabling the timer 3 to count */
	TIM3->CNT = 0x00;
	TIM3->CR1 |= TIM_CR1_CEN; /**< Writing TIM_CR1_CEN to CR1 register of the TIM3 enables the counting */

	/*                                  --FOREVER LOOP--                                                     */
	while (1)
	{
		/*                           --Reading the ACCELEROMETER--                                             */
		ReadAccelerometer();

		/* Storing the data inside the variables */
		X_AXIS = (data[0] << 8) | data[1];
		Y_AXIS = (data[2] << 8) | data[3];
		Z_AXIS = (data[4] << 8) | data[5];

		/* Adjusting for the noise and transforming the data to represent true values */
		X_OUTPUT = X_AXIS / 16384.0 - AccelXError;
		Y_OUTPUT = Y_AXIS / 16384.0 - AccelYError;
		Z_OUTPUT = Z_AXIS / 16384.0 - AccelZError;

		xaccel = X_OUTPUT;
		yaccel = Y_OUTPUT;
		zaccel = Z_OUTPUT;

		/* Calculating the roll and pitch values from the accelerometer data */
		// float AccelAngleY = atan2(X_OUTPUT, Z_OUTPUT)*180/PI;																				/**< Pitch */
		// float AccelAngleX = atan2(Y_OUTPUT, Z_OUTPUT)*180/PI;																				/**< Roll */
		float AccelAngleX = atan(Y_OUTPUT / sqrt(pow(X_OUTPUT, 2) + pow(Z_OUTPUT, 2))) * 180 / PI;		/**< Roll y, z*/
		float AccelAngleY = atan(-1 * X_OUTPUT / sqrt(pow(Y_OUTPUT, 2) + pow(Z_OUTPUT, 2))) * 180 / PI; /**< Pitch x, z */

		/*                             --Reading the GYROSCOPE--                                            */
		/* Getting the time it took for the data aquisition from the counter in order to integrate the gyroscope data */
		float current_time = (float)(TIM3->CNT) * TurnToMilisecond;
		float dt = (current_time) / 1000.0;
		vrijeme += dt;
		TIM3->CNT = 0; /**< Reseting the counter value */

		ReadGyroscope();

		X_AXIS = (data[0] << 8) | data[1];
		Y_AXIS = (data[2] << 8) | data[3];
		Z_AXIS = (data[4] << 8) | data[5];

		/* Adjusting for the noise and transforming the data to represent true values */
		float GyroRateX = X_AXIS / 131.0 - GyroXError;
		float GyroRateY = Y_AXIS / 131.0 - GyroYError;
		float GyroRateZ = Z_AXIS / 131.0 - GyroZError;

		/* Integrate the data from the gyroscope in order to turn the angular velocity to angular position */
		GyroAngleX += GyroRateX * dt;
		GyroAngleY += GyroRateY * dt;
		GyroAngleZ += GyroRateZ * dt;

		/*                           --Reading the Magnetometer--                                            */
		float MAGX = 0.0;
		float MAGY = 0.0;
		float MAGZ = 0.0;

		SwitchSlaveDevice(MAGNETOMETER_SLAVE);

		ReadMagnetometer();

		/* Reading the 14-bit data from the magnetometer slave */
		X_AXIS = (data[1] << 8) | data[0];
		Y_AXIS = (data[3] << 8) | data[2];
		Z_AXIS = (data[5] << 8) | data[4];

		X_AXIS = (X_AXIS << 2);
		Y_AXIS = (Y_AXIS << 2);
		Z_AXIS = (Z_AXIS << 2);

		X_OUTPUT = ((float)(X_AXIS)) / 4.0;
		Y_OUTPUT = ((float)(Y_AXIS)) / 4.0;
		Z_OUTPUT = ((float)(Z_AXIS)) / 4.0;

		/* Transform the data to represent the true readings by calibrating and subtracting offset */
		MAGX = X_OUTPUT * MagneticResolution * calibrationX - MagneticOffset[0];
		MAGY = Y_OUTPUT * MagneticResolution * calibrationY - MagneticOffset[1];
		MAGZ = -1 * (Z_OUTPUT * MagneticResolution * calibrationZ - MagneticOffset[2]); /**< Z-axis is in the reversed
																																									direction from the accel-gyro sensor */

		SwitchSlaveDevice(ACCEL_GYRO_SLAVE);

		/*                           --Calculating PITCH, ROLL and YAW--                                 */

		/* Using Kalman filtering to obtain the non-noise-plagued data. It receives the position from the accelerometer calculated
		pitch and yaw values and the velocity it calculated from the gyroscope readings. The accelerometer data is plagued by noise
		while the gyroscope data is plagued by drift. The Kalman filter predicts the future readings based on the current and the
		past ones and should eliminate any noise or drift present in the data */
		KalmanFilterXYZ(&KalmanX, GyroRateX, AccelAngleX, dt);
		KalmanFilterXYZ(&KalmanY, GyroRateY, AccelAngleY, dt);

		/* Determining roll and pitch from the gyroscope data with removed drift (Kalman filtering) */
		roll += KalmanX.NoDriftGyroRate * dt;
		pitch += KalmanY.NoDriftGyroRate * dt;

		/* Tilt compensation */
		/* Obtaining the yaw value from the magnetometer data based on the pitch and roll values and the read magnetometer data */
		float Hy = MAGY * cos(pitch / (180 / PI)) + MAGZ * sin(pitch / (180 / PI));
		float Hx = MAGY * sin(roll / (180 / PI)) * sin(pitch / (180 / PI)) + MAGX * cos(roll / (180 / PI)) - MAGZ * sin(roll / (180 / PI)) * cos(pitch / (180 / PI));

		/* Temporary yaw value which is really noise, obtained from the equation for the tilt compensation */
		float yaw_temp = atan2(Hy, Hx) * 180 / PI;

		/* Using complementary filter to reduce the noise from the yaw value */
		yaw_tempf = 0.5 * yaw_tempf + 0.5 * yaw_temp;

		/* Using Kalman filter to reduce more noise and eliminate drift from the Z-axis gyroscope reading */
		KalmanFilterXYZ(&KalmanZ, GyroRateZ, yaw_tempf, dt);

		/* Possibilities of calculating the yaw angle. The uncommented one was the one that gave the best results */
		// yaw = 0;																				/**< 0 to see only roll and pitch */
		// yaw = GyroAngleZ;																/**< Only calculated by the gyroscope reading */
		// yaw = yaw_tempf;																/**< Using only the complementary filter */
		yaw += KalmanZ.NoDriftGyroRate * dt; /**< Using complementary filter and Kalman filter */

		if (flag == 0)
		{
			if (repeat < 200)
			{
				repeat += 1;
			}
			else
			{
				flag = 1;
				yaw_starting = yaw;
			}
		}

		// yaw = 0.81*GyroAngleZ + 0.19*yaw_tempf;					/**< Using the complementary filter with gyroscope data */
		// yaw_tempf2 = 0.7*yaw_tempf2 + 0.3*yaw;

		if ((xaccel * xaccel + yaccel * yaccel + zaccel * zaccel) > 1.25)
		{
			GPIOA->ODR |= GPIO_ODR_5;
			xdistance += xaccel * dt * dt;
			ydistance += yaccel * dt * dt;
			zdistance += zaccel * dt * dt;
		}
		else
		{
			GPIOA->ODR &= ~GPIO_ODR_5;
			xdistance = 0.0;
			ydistance = 0.0;
			zdistance = 0.0;
		}

		/*                                    --VISUALIZATION--                                          */
		/* Sending the data to be read inside the Processing Toolbox */
		if (flag == 1)
		{
			sprintf((char *)buf, "%f\r\n", GyroAngleY);
			SendToUART(buf);
		}

		/*if(flag == 1) {
			sprintf((char*)buf, "%f", roll);
			SendToUART(buf);
			strcpy((char*)buf, "/");
			SendToUART(buf);
			sprintf((char*)buf, "%f", pitch);
			SendToUART(buf);
			strcpy((char*)buf, "/");
			SendToUART(buf);
			sprintf((char*)buf, "%f\n", (yaw - yaw_starting));
			SendToUART(buf);
		}*/
	}
}

