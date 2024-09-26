/*
 * helper.h
 *
 *  Created on: Sep 10, 2024
 *      Author: AD
 *
 *  Contain helper functions
 */

#ifndef INC_HELPER_H_
#define INC_HELPER_H_

#include "main.h"
#include "helper.h"


/* STRUCT BEGINS */
typedef struct kalman_typedef {
	float last_est;
	float est;
	float mea;
} kalman_t;

typedef struct pid_typedef {
	float error_old;
	float error_accumulate;
	float Kp;	// Proportional
	float Ki;	// Integral
	float Kd;	// Derivative
} pid_t;


/* STRUCT ENDS*/


/* DEFINITION IN ICM20948 */
/*
Reference: https://github.com/drcpattison/ICM-20948/blob/master/src/ICM20948.h
ICM20948 Manual: https://invensense.tdk.com/wp-content/uploads/2016/06/DS-000189-ICM-20948-v1.3.pdf
*/




/*---------- OLED INTERACTION ----------*/
bool is_USER_button_pressed();
void print_value(int x, int y, uint8_t* msg, int32_t val);


/*---------- ICM20948 ----------*/
void ICM20948_init(
	I2C_HandleTypeDef* hi2c,
	uint8_t I2C_address,
	uint8_t gyro_sensitivity,
	uint8_t  accel_sensitivity);

HAL_StatusTypeDef _ICM20948_BrustRead(I2C_HandleTypeDef* hi2c, uint8_t I2cAddress, uint8_t startAddress, uint16_t amountOfRegistersToRead, uint8_t* readData);
void ICM20948_readGyroscope_Z(I2C_HandleTypeDef* hi2c, uint8_t selectI2cAddress, uint8_t selectGyroSensitivity, float* gyroZ);
void ICM20948_readAccelerometer_all(I2C_HandleTypeDef* hi2c, uint8_t selectI2cAddress, uint8_t selectAccelSensitivity, float readings[3]);


/*---------- KALMAN FILTER ----------*/
void kalman_init(kalman_t* kalman, float init_est, float est, float mea);
void kalman_update(kalman_t* kalman, float est, float mea);


/*---------- PID ----------*/
void pid_reset(pid_t* pid);
void pid_init(pid_t* pid, float Kp, float Ki, float Kd);
float pid_adjust(pid_t* pid, float error, float scale);


#endif /* INC_HELPER_H_ */
