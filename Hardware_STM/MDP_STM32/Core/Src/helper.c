/*
 * helper.c
 *
 *  Created on: Sep 10, 2024
 *      Author: AD
 *
 *  Contain helper functions
 */

#include "helper.h"


/*---------- OLED INTERACTION ----------*/
bool is_USER_button_pressed() {
	return HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin) == GPIO_PIN_RESET;
}

void print_OLED(int x, int y, uint8_t* msg, bool var_exist, int32_t val) {
	uint8_t* buf[100];

	if (var_exist) {
		sprintf(buf, msg, val);
	} else {
		sprintf(buf, msg);
	}

	OLED_ShowString(x, y, buf);
	OLED_Refresh_Gram();
}


/*-------------- COMMANDS --------------*/
void send_ack(UART_HandleTypeDef* uart_ptr) {
	uint8_t ack[] = "l";
	HAL_UART_Transmit(uart_ptr, ack, sizeof(ack), 2000);
}

void move(uint8_t cmd) {
	switch ((uint8_t) cmd) {
	case 'w':
		print_OLED(0, 30, "Forward", false, 0);
		forward(40);
		break;
	case 's':
		print_OLED(0, 30, "Backward", false, 0);
		backward_move();
		break;
	case 'a':
		print_OLED(0, 30, "Forward left", false, 0);
		forward_left();
		break;
	case 'd':
		print_OLED(0, 30, "Forward right", false, 0);
		forward_right();
		break;
	case 'z':
		print_OLED(0, 30, "Backward left", false, 0);
		backward_left();
		break;
	case 'c':
		print_OLED(0, 30, "Backward right", false, 0);
		backward_right();
		break;
	case 'x':
		print_OLED(0, 30, "Stop", false, 0);
		stop();
		break;
	default:
		print_OLED(0, 30, "cmd wrong type", false, 0);
	}
}


/*-------------- ICM20948 --------------*/
HAL_StatusTypeDef ICM20948_BrustRead(
	I2C_HandleTypeDef* hi2c_ptr, 
	uint8_t sel_I2C_addr, 
	uint8_t start_addr, 
	uint16_t number_of_reg_to_read, 
	uint8_t* read_data) {	
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t device_I2C_addr = 
		(sel_I2C_addr == 0) ? 
		ICM20948_I2C_SLAVE_ADDR_1 : 
		ICM20948_I2C_SLAVE_ADDR_1;

	status = HAL_I2C_Mem_Read(
		hi2c_ptr,
		device_I2C_addr << 1,
		start_addr,
		I2C_MEMADD_SIZE_8BIT,
		read_data,
		number_of_reg_to_read * I2C_MEMADD_SIZE_8BIT,
		10
	);

	return status;
}

HAL_StatusTypeDef ICM20948_SelectUserBank(
	I2C_HandleTypeDef* hi2c_ptr, 
	uint8_t const sel_I2C_addr, 
	int user_bank_num) {
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t write_data = user_bank_num << BIT_4;
	uint8_t device_I2C_addr = 
		(sel_I2C_addr == 0) ? 
		ICM20948_I2C_SLAVE_ADDR_1 : 
		ICM20948_I2C_SLAVE_ADDR_2;

	status = HAL_I2C_Mem_Write(
		hi2c_ptr,
		device_I2C_addr << 1,
		ICM20948_USER_BANK_ALL_REG_BANK_SEL_REG,
		I2C_MEMADD_SIZE_8BIT,
		&write_data,
		I2C_MEMADD_SIZE_8BIT,
		10
	);

	return status;
}

HAL_StatusTypeDef ICM20948_WriteByte(
	I2C_HandleTypeDef* hi2c_ptr, 
	uint8_t const sel_I2c_addr, 
	uint8_t const reg_addr, 
	uint8_t write_data) {
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t device_I2C_addr = 
		(sel_I2c_addr == 0) ? 
		ICM20948_I2C_SLAVE_ADDR_1 : 
		ICM20948_I2C_SLAVE_ADDR_1;

	status = HAL_I2C_Mem_Write(
		hi2c_ptr,
		device_I2C_addr << 1,
		reg_addr,
		I2C_MEMADD_SIZE_8BIT,
		&write_data,
		I2C_MEMADD_SIZE_8BIT,
		10
	);

	return status;
}

void ICM20948_init(
	I2C_HandleTypeDef* hi2c_ptr,
	uint8_t sel_I2C_addr,
	uint8_t sel_gyro_sen,
	uint8_t sel_accel_sen) {
	HAL_StatusTypeDef status = HAL_OK;
	status = ICM20948_SelectUserBank(hi2c_ptr, sel_I2C_addr, USER_BANK_0);
	status = ICM20948_WriteByte(
		hi2c_ptr,
		sel_I2C_addr,
		ICM20948_USER_BANK_0_PWR_MGMT_1_REG,
		ICM20948_RESET
	);

	HAL_Delay(200);

	status = ICM20948_WriteByte(
		hi2c_ptr,
		sel_I2C_addr,
		ICM20948_USER_BANK_0_PWR_MGMT_1_REG,
		ICM20948_AUTO_SELECT_CLOCK
	);

	// Enable both the gyroscope and accelerometer
	status = ICM20948_WriteByte(
		hi2c_ptr,
		sel_I2C_addr,
		ICM20948_USER_BANK_0_PWR_MGMT_2_REG,
		ICM20948_DISABLE_SENSORS
	); // For some reason this needs to be tested

	status = ICM20948_SelectUserBank(hi2c_ptr, sel_I2C_addr, USER_BANK_2);

	// Gyroscope sampling rate settings
	status = ICM20948_WriteByte(
		hi2c_ptr,
		sel_I2C_addr,
		ICM20948_USER_BANK_2_GYRO_CONFIG_1_REG,
		(0 << GYRO_DLPFCFG_BIT) | 
		(sel_gyro_sen << GYRO_FS_SEL_BIT) | 
		(EN_GYRO_DLPF << GYRO_FCHOICE_BIT)
	);

	status = ICM20948_WriteByte(
		hi2c_ptr,
		sel_I2C_addr,
		ICM20948_USER_BANK_2_GYRO_SMPLRT_DIV_REG,
		4
	);

	// Accelerometer sampling rate settings
	status = ICM20948_WriteByte(
		hi2c_ptr,
		sel_I2C_addr,
		ICM20948_USER_BANK_2_ACCEL_CONFIG_REG,
		(1 << ACCEL_DLPFCFG_BIT) |
		(sel_accel_sen << ACCEL_FS_SEL_BIT) |
		(0x01 << ACCEL_FCHOICE_BIT)
	);

	status = ICM20948_WriteByte(
		hi2c_ptr,
		sel_I2C_addr,
		ICM20948_USER_BANK_2_ACCEL_SMPLRT_DIV_2_REG,
		4
	);

	status = ICM20948_SelectUserBank(hi2c_ptr, sel_I2C_addr, USER_BANK_0);

	status = ICM20948_WriteByte(
		hi2c_ptr,
		sel_I2C_addr,
		ICM20948_USER_BANK_0_INT_PIN_CFG_REG,
		0x02
	); // Don't understand how this works

	status = _AK09916_WriteByte(
		hi2c_ptr,
		AK09916_CNTL2_REG,
		0x08
	);
}

void ICM20948_readGyroscope_Z(
	I2C_HandleTypeDef* hi2c, 
	uint8_t sel_I2C_addr, 
	uint8_t sel_gyro_sen, 
	float* gyroZ) {
	uint8_t read_data[2];

	// ICM20948_SelectUserBank(hi2c, sel_I2C_addr, USER_BANK_0);
	ICM20948_BrustRead(hi2c, sel_I2C_addr, ICM20948_USER_BANK_0_GYRO_ZOUT_H_REG, 2, read_data);

	int16_t reading = (read_data[0] << 8) | read_data[1];
	*gyroZ = (float) -reading;
	switch (sel_gyro_sen) {
	case GYRO_FULL_SCALE_250DPS:
		*gyroZ /= GYRO_SEN_SCALE_FACTOR_250DPS;
		break;
	case GYRO_FULL_SCALE_500DPS:
		*gyroZ /= GYRO_SEN_SCALE_FACTOR_500DPS;
		break;
	case GYRO_FULL_SCALE_1000DPS:
		*gyroZ /= GYRO_SEN_SCALE_FACTOR_1000DPS;
		break;
	case GYRO_FULL_SCALE_2000DPS:
		*gyroZ /= GYRO_SEN_SCALE_FACTOR_2000DPS;
		break;
	}
}

void ICM20948_readAccelerometer_all(
	I2C_HandleTypeDef* hi2c, 
	uint8_t sel_I2C_addr, 
	uint8_t sel_accel_sen, 
	float readings[3]) {
	uint8_t read_data[6];

	// ICM20948_SelectUserBank(hi2c, sel_I2C_addr, USER_BANK_0);
	ICM20948_BrustRead(hi2c, sel_I2C_addr, ICM20948_USER_BANK_0_ACCEL_XOUT_H_REG, 6, read_data);

	int16_t rD_int[3];
	rD_int[X] = (read_data[X_HIGH_BYTE] << 8) | read_data[X_LOW_BYTE];
	rD_int[Y] = (read_data[Y_HIGH_BYTE] << 8) | read_data[Y_LOW_BYTE];
	rD_int[Z] = (read_data[Z_HIGH_BYTE] << 8) | read_data[Z_LOW_BYTE];

	float rD[3];
	rD[X] = (float) rD_int[X];
	rD[Y] = (float) rD_int[Y];
	rD[Z] = (float) rD_int[Z];

	switch (sel_accel_sen) {
	case ACCEL_FULL_SCALE_2G:
		readings[X] = rD[X] / ACCEL_SEN_SCALE_FACTOR_2G;
		readings[Y] = rD[Y] / ACCEL_SEN_SCALE_FACTOR_2G;
		readings[Z] = rD[Z] / ACCEL_SEN_SCALE_FACTOR_2G;
		break;
	case ACCEL_FULL_SCALE_4G:
		readings[X] = rD[X] / ACCEL_SEN_SCALE_FACTOR_4G;
		readings[Y] = rD[Y] / ACCEL_SEN_SCALE_FACTOR_4G;
		readings[Z] = rD[Z] / ACCEL_SEN_SCALE_FACTOR_4G;
		break;
	case ACCEL_FULL_SCALE_8G:
		readings[X] = rD[X] / ACCEL_SEN_SCALE_FACTOR_8G;
		readings[Y] = rD[Y] / ACCEL_SEN_SCALE_FACTOR_8G;
		readings[Z] = rD[Z] / ACCEL_SEN_SCALE_FACTOR_8G;
		break;
	case ACCEL_FULL_SCALE_16G:
		readings[X] = rD[X] / ACCEL_SEN_SCALE_FACTOR_16G;
		readings[Y] = rD[Y] / ACCEL_SEN_SCALE_FACTOR_16G;
		readings[Z] = rD[Z] / ACCEL_SEN_SCALE_FACTOR_16G;
		break;
	}
}


/*------------ KALMAN FILTER ------------*/
void kalman_init(kalman_t* kalman, float init_est, float est, float mea) {
	kalman->last_est = init_est;
	kalman->est = est;
	kalman->mea = mea;
}

void kalman_update(kalman_t* kalman, float est, float mea) {
	// Kalman gain
	float gain = (kalman->est) / (kalman->est + kalman->mea);

	// Update estimate
	kalman->last_est += gain * (mea - est);
}


/*----------------- PID -----------------*/
//void pid_reset(pid_t* pid)  {
//	pid->error_accumulate = 0;
//	pid->error_old = 0;
//}
//
//void pid_init(pid_t* pid, float Kp, float Ki, float Kd) {
//	pid_reset(pid);
//
//	pid->Kp = Kp;
//	pid->Ki = Ki;
//	pid->Kd = Kd;
//}
//
//float pid_adjust(pid_t* pid, float error, float scale) {
//	pid->error_accumulate += error;
//	float error_rate = (error - pid->error_old);
//	pid->error_old = error;
//
//	return scale * (pid->Kp*error + pid->Ki*pid->error_accumulate + pid->Kd*error_rate);
//}
