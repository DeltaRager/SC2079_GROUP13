/*
 * Sensor.c
 *
 *  Created on: Sep 10, 2024
 *      Author: AD
 *
 *  Control sensors (ICM20948)
 */

#include "sensor.h"
#include "helper.h"


I2C_HandleTypeDef* hi2c_ptr;
TIM_HandleTypeDef* hic_ptr;
sensor_t* sensor_ptr;

// void sensors_init(I2C_HandleTypeDef* i2c_ptr, TIM_HandleTypeDef* ic_ptr, sensor_t* ss_ptr) {
// 	hi2c_ptr = i2c_ptr;
// 	hic_ptr = ic_ptr;
// 	sensor_ptr = ss_ptr;

// 	ICM20948_init(hi2c_ptr, ICM_I2C_ADDR, GYRO_SENS, ACCEL_SENS);

// 	HAL_TIM_IC_Start_IT(ic_ptr, US_IC_CHANNEL);

// 	ss_ptr->gyroZ_bias = 0;
// 	ss_ptr->accel_bias[0] = ss_ptr->accel_bias[1] = ss_ptr->accel_bias[2] = 0;
// }

// void sensors_read_gyroZ() {
// 	float val;
// 	ICM20948_readGyroscope_Z(hi2c_ptr, ICM_I2C_ADDR, GYRO_SENS, &val);
// 	sensors_ptr->gyroZ = (val - sensors_ptr->gyroZ_bias) / 1000; //convert to ms
// }

// void sensors_read_accel() {
// 	float accel_new[3];

// 	ICM20948_readAccelerometer_all(hi2c_ptr, ICM_I2C_ADDR, ACCEL_SENS, accel_new);
// 	for (int i = 0; i < 3; i++)
// 		sensors_ptr->accel[i] = (accel_new[i] - sensors_ptr->accel_bias[i]) * GRAVITY;
// }
