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


typedef struct sensor_typedef {
	float error_old;
	float error_accumulate;
	float Kp;	// Proportional
	float Ki;	// Integral
	float Kd;	// Derivative
} sensor_t;


void sensors_init(I2C_HandleTypeDef* i2c_ptr, TIM_HandleTypeDef* ic_ptr, sensor_t* ss_ptr);
void sensors_read_gyroZ();
void sensors_read_accel();

#endif /* INC_SENSOR_H_ */
