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


/*--------- ICM20948 DEFINITION ---------*/
/*
Reference: https://github.com/drcpattison/ICM-20948/blob/master/src/ICM20948.h
ICM20948 Manual: https://invensense.tdk.com/wp-content/uploads/2016/06/DS-000189-ICM-20948-v1.3.pdf
*/
#define ICM20948_DISABLE_SENSORS					0x00
#define ICM20948_AUTO_SELECT_CLOCK					0x01
#define ICM20948_RESET								0x80

#define ICM20948_I2C_SLAVE_ADDR_1					0x68
#define ICM20948_I2C_SLAVE_ADDR_2					0x69
#define ICM20948_USER_BANK_ALL_REG_BANK_SEL_REG		0x7F
#define ICM20948_USER_BANK_0_PWR_MGMT_1_REG			0x06
#define ICM20948_USER_BANK_0_PWR_MGMT_2_REG			0x07
#define ICM20948_USER_BANK_0_INT_PIN_CFG_REG		0x0F
#define ICM20948_USER_BANK_0_ACCEL_XOUT_H_REG		0x2D
#define ICM20948_USER_BANK_0_GYRO_ZOUT_H_REG		0x37
#define ICM20948_USER_BANK_2_ACCEL_CONFIG_REG		0x14
#define ICM20948_USER_BANK_2_ACCEL_SMPLRT_DIV_2_REG 0x11
#define ICM20948_USER_BANK_2_GYRO_CONFIG_1_REG		0x01
#define ICM20948_USER_BANK_2_GYRO_SMPLRT_DIV_REG	0x00
#define AK09916_CNTL2_REG							0x31


// User banks
#define USER_BANK_0 0x0
#define USER_BANK_1 0x1
#define USER_BANK_2	0x2
#define USER_BANK_3 0x3

// Accelerometer and Gyroscope
#define ACCEL_FCHOICE_BIT	0
#define ACCEL_FS_SEL_BIT	1
#define ACCEL_DLPFCFG_BIT	3
#define GYRO_FCHOICE_BIT	0
#define GYRO_FS_SEL_BIT		1
#define GYRO_DLPFCFG_BIT	3
#define EN_GYRO_DLPF		1

// Gyroscope Sensitivity
#define GYRO_SEN_SCALE_FACTOR_250DPS	131
#define GYRO_SEN_SCALE_FACTOR_500DPS	65.5
#define GYRO_SEN_SCALE_FACTOR_1000DPS	32.8
#define GYRO_SEN_SCALE_FACTOR_2000DPS	16.4
#define GYRO_FULL_SCALE_250DPS			0
#define GYRO_FULL_SCALE_500DPS			1
#define GYRO_FULL_SCALE_1000DPS			2
#define GYRO_FULL_SCALE_2000DPS			3

// Acceleromenter Sensitivity
#define ACCEL_SEN_SCALE_FACTOR_2G		16384
#define ACCEL_SEN_SCALE_FACTOR_4G		8192
#define ACCEL_SEN_SCALE_FACTOR_8G		4096
#define ACCEL_SEN_SCALE_FACTOR_16G		2048
#define ACCEL_FULL_SCALE_2G				0
#define ACCEL_FULL_SCALE_4G				1
#define ACCEL_FULL_SCALE_8G				2
#define ACCEL_FULL_SCALE_16G			3

// Bit and Byte
#define X 0
#define Y 1
#define Z 2
#define BIT_1 1
#define BIT_2 2
#define BIT_3 3
#define BIT_4 4
#define BIT_5 5
#define BIT_6 5
#define BIT_7 7
#define BIT_8 8
#define X_HIGH_BYTE 0
#define X_LOW_BYTE 1
#define Y_HIGH_BYTE 2
#define Y_LOW_BYTE 3
#define Z_HIGH_BYTE 4
#define Z_LOW_BYTE 5
#define T_HIGH_BYTE 0
#define T_LOW_BYTE 1
#define ONE_BYTE 8


/*--------- ICM20948 DEFINITION ---------*/
#define FORWARD 		'w'
#define BACKWARD 		's'
#define FORWARD_LEFT 	'a'
#define FORWARD_RIGHT 	'd'
#define BACKWARD_LEFT 	'z'
#define BACKWARD_RIGHT 	'c'
#define STOP 			'x'
//#define LINH 	'l'


/*--------------- STRUCT ---------------*/
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

typedef struct command_typedef {
	uint8_t dir;
	struct command_typedef* next;
} cmd_t;


/*---------- OLED INTERACTION ----------*/
bool is_USER_button_pressed();
void print_value(int x, int y, uint8_t* msg, int32_t val);


/*-------------- COMMANDS --------------*/
void send_ack();


/*-------------- ICM20948 --------------*/
HAL_StatusTypeDef ICM20948_BrustRead(
	I2C_HandleTypeDef* hi2c, 
	uint8_t sel_I2C_addr, 
	uint8_t start_addr, 
	uint16_t number_of_reg_to_read, 
	uint8_t* read_data);

HAL_StatusTypeDef ICM20948_SelectUserBank(
	I2C_HandleTypeDef* hi2c_ptr, 
	uint8_t const selectI2cAddress, 
	int user_bank_num);


void ICM20948_init(
	I2C_HandleTypeDef* hi2c,
	uint8_t I2C_address,
	uint8_t gyro_sensitivity,
	uint8_t  accel_sensitivity);

void ICM20948_readGyroscope_Z(
	I2C_HandleTypeDef* hi2c, 
	uint8_t selectI2cAddress, 
	uint8_t selectGyroSensitivity, 
	float* gyroZ);

void ICM20948_readAccelerometer_all(
	I2C_HandleTypeDef* hi2c, 
	uint8_t selectI2cAddress, 
	uint8_t selectAccelSensitivity, 
	float readings[3]);


/*------------ KALMAN FILTER ------------*/
void kalman_init(kalman_t* kalman, float init_est, float est, float mea);
void kalman_update(kalman_t* kalman, float est, float mea);


/*----------------- PID -----------------*/
void pid_reset(pid_t* pid);
void pid_init(pid_t* pid, float Kp, float Ki, float Kd);
float pid_adjust(pid_t* pid, float error, float scale);


#endif /* INC_HELPER_H_ */
