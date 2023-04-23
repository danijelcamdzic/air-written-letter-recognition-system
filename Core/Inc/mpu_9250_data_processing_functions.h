/**
 ******************************************************************************
 * @file           : mpu_9250_data_processing_functions.h
 * @brief          : Header for mpu_9250_3D_orientation_detection_functions.c file.
 *                   This file contains the defines and prototypes for the functions
 *                   that involve gathering and processing the MPU-9250 sensor data.
 ******************************************************************************
 */

#ifndef __MPU_9250_DATA_PROCESSING_H
#define __MPU_9250_DATA_PROCESSING_H

#include <string.h>
#include <stdio.h>
#include <math.h>
#include "main.h"

/* Definitions of the sensor register addresses */
#define AccelGyroAddress (0x69 << 1) /**< Sensor slave address of the accel+gyro */
#define WHO_AM_I (0x75)              /**< WHO_AM_I register of the accel and gyro sensor */
#define PWR_MGMT_1 (0x6B)            /**< Register for power management */
#define ACCEL_XOUT_H (0x3B)          /**< Accelerometer X-axis high byte output */
#define GYRO_XOUT_H (0x43)           /**< Gyroscope X-axis high byte output */
#define INT_BYPASS_CONFIG_AD (0x37)  /**< Register for switching accel+gyro to magnetometer slave */

#define MagnetometerAddress (0x0C << 1) /**< Sensor slave address of the magnetometer */
#define WIA (0x00)                      /**< Device ID address of the magnetometer */
#define STATUS1 (0x02)                  /**< Status1 register of the magnetometer slave */
#define MAGN_XOUT_L (0x03)              /**< Magnetometer X-axis low byte output  */
#define STATUS2 (0x09)                  /**< Status2 register of the magnetometer slave */
#define CNTL1_AD (0x0A)                 /**< Control 1 register */
#define ASAX_REG (0x10)                 /**< X-axis sensitivity adjustment value register */

#define ACCEL_GYRO_SLAVE (0x00)   /**< Address to switch from magnetometer slave to accel+gyro slave */
#define MAGNETOMETER_SLAVE (0x02) /**< Address to switch from accel+gyro slave to magnetometer slave */

#define MagneticResolution (4912 / 8190.0) /**< Value necessary to find out the right magnetometer readings */

#define PI (3.141592) /**< Value of the mathematical constant PI */

#define TurnToMilisecond (0.5) /**< Value used to turn counter readings to miliseconds */

/* Type Definitions */
typedef struct
{
  /* Covariance matrices */
  float Q[2];
  float R;

  float Xk[2]; /**< State variables - position and velocity */
  float P[2][2];

  float NoDriftGyroRate; /**< Velocity from the gyroscopes adjusted for drift */

} KalmanFilter;

/* Variables */
extern KalmanFilter KalmanX; /**< Kalman filter for X-axis */
extern KalmanFilter KalmanY; /**< Kalman filter for Y-axis */
extern KalmanFilter KalmanZ; /**< Kalman filter for Z-axis */

extern uint8_t buf[100]; /**< Array used for sendind messages to UART */
extern uint8_t data[8];  /**< Array used for sending data bytes */

extern int16_t X_AXIS; /**< Variable used to receive 2 bytes from the X-axis measurements */
extern int16_t Y_AXIS; /**< Variable used to receive 2 bytes from the Y-axis measurements  */
extern int16_t Z_AXIS; /**< Variable used to receive 2 bytes from the Z-axis measurements  */
extern float X_OUTPUT; /**< Final X-axis value */
extern float Y_OUTPUT; /**< Final Y-axis value */
extern float Z_OUTPUT; /**< Final Z-axis value */

extern uint8_t ASAX; /**< X-axis adjustment value for the magnetometer */
extern uint8_t ASAY; /**< Y-axis adjustment value for the magnetometer */
extern uint8_t ASAZ; /**< Z-axis adjustment value for the magnetometer */

extern float pitch; /**< Pitch value */
extern float roll;  /**< Roll value */
extern float yaw;   /**< Yaw value */

extern float xaccel;
extern float yaccel;
extern float zaccel;

extern float MagneticOffset[3]; /**< Magnetic offset for the magnetometer sensor. Calculated during one
                                              of the test runs from the function inside this project. */

/* Function Prototypes */
void KalmanFilterXYZ(KalmanFilter *Kalman, float GyroRate, float AccelAngle, float dt);
void InitKalmanXYZ(void);
void WakeUpSensor(void);
void GetMagnetometerAdjustment(void);
void SwitchSlaveDevice(uint8_t command);
void CalculateAccelGyroError(double *AccelXError, double *AccelYError, double *AccelZError, double *GyroXError, double *GyroYError, double *GyroZError);
void ReadAccelerometer(void);
void ReadGyroscope(void);
void ReadMagnetometer(void);
void CalibrateMagnetometer(void);

#endif


