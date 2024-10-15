/*
 * sensor.h
 *
 *  Created on: Sep 10, 2024
 *      Author: AD
 *
 *  Control sensors (ICM20948)
 */

#ifndef INC_SENSOR_H_
#define INC_SENSOR_H_

#include "main.h"
#include "helper.h"

#define ICM_I2C_ADDR 	0
#define US_IC_CHANNEL	TIM_CHANNEL_1
#define GRAVITY 		9.80665e-4f		//in cm/ms^2


// Global variables
extern TIM_HandleTypeDef htim1, htim6;
extern uint32_t tc1, tc2, echo;
extern float dist;
extern bool is_first_captured;


// Define struct
typedef struct sensor_typedef {
	float error_old;
	float error_accumulate;
	float Kp;	// Proportional
	float Ki;	// Integral
	float Kd;	// Derivative

	float gyroZ;			//gyroscope Z reading.
	float accel[3];			//accelerometer [X, Y, Z] readings.
	float heading;			//heading from -180 to 180 degrees.

	float gyroZ_bias;
	float accel_bias[3];
	float heading_bias;

} sensor_t;


void sensors_init(I2C_HandleTypeDef* i2c_ptr, TIM_HandleTypeDef* ic_ptr, sensor_t* ss_ptr);

/*-------- Gyroscope (z-axis) Sensors -------*/
void sensors_read_gyroZ();

/*---------- Accelerometer Sensors ----------*/
void sensors_read_accel();

#endif /* INC_SENSOR_H_ */
