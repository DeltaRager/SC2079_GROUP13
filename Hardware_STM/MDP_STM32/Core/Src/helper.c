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

void print_value(int x, int y, uint8_t* msg, int32_t val) {
	OLED_Clear();
	uint8_t* buf[100];
	sprintf(buf, msg, val);
	OLED_ShowString(x, y, buf);
	OLED_Refresh_Gram();
}


/*---------- ICM20948 ----------*/
// void ICM20948_init(
// 	I2C_HandleTypeDef* hi2c,
// 	uint8_t I2C_address,
// 	uint8_t gyro_sensitivity,
// 	uint8_t accel_sensitivity) {
// 	HAL_StatusTypeDef status = HAL_OK;

// 	status = _ICM20948_SelectUserBank(hi2c, selectI2cAddress, USER_BANK_0);

// 	status = _ICM20948_WriteByte(
// 			hi2c,
// 			selectI2cAddress,
// 			ICM20948__USER_BANK_0__PWR_MGMT_1__REGISTER,
// 			ICM20948_RESET);

// 	HAL_Delay(200);

// 	status = _ICM20948_WriteByte(
// 			hi2c,
// 			selectI2cAddress,
// 			ICM20948__USER_BANK_0__PWR_MGMT_1__REGISTER,
// 			ICM20948_AUTO_SELECT_CLOCK);

// 	//enable both gyroscope and accelerometer
// 	status = _ICM20948_WriteByte(
// 			hi2c,
// 			selectI2cAddress,
// 			ICM20948__USER_BANK_0__PWR_MGMT_2__REGISTER,
// 			ICM20948_DISABLE_SENSORS); // For some reason this needs to be tested

// 	status = _ICM20948_SelectUserBank(hi2c, selectI2cAddress, USER_BANK_2);

// 	////temperature configuration.
// 	//	status = _ICM20948_WriteByte(
// 	//			hi2c,
// 	//			selectI2cAddress,
// 	//			ICM20948__USER_BANK_2__TEMP_CONFIG__REGISTER,
// 	//			0x03);

// 	//gyroscope sampling rate settings.
// 	status = _ICM20948_WriteByte(
// 			hi2c,
// 			selectI2cAddress,
// 			ICM20948__USER_BANK_2__GYRO_CONFIG_1__REGISTER,
// 			0 << GYRO_DLPFCFG_BIT|selectGyroSensitivity << GYRO_FS_SEL_BIT|EN_GRYO_DLPF << GYRO_FCHOICE_BIT);
// 	status = _ICM20948_WriteByte(
// 			hi2c,
// 			selectI2cAddress,
// 			ICM20948__USER_BANK_2__GYRO_SMPLRT_DIV__REGISTER,
// 			4);

// 	//accelerometer sampling rate settings.
// 	status = _ICM20948_WriteByte(
// 			hi2c,
// 			selectI2cAddress,
// 			ICM20948__USER_BANK_2__ACCEL_CONFIG__REGISTER,
// 			1 << ACCEL_DLPFCFG_BIT|selectAccelSensitivity << ACCEL_FS_SEL_BIT|0x01 << ACCEL_FCHOICE_BIT);
// 	status = _ICM20948_WriteByte(
// 			hi2c,
// 			selectI2cAddress,
// 			ICM20948__USER_BANK_2__ACCEL_SMPLRT_DIV_2__REGISTER,
// 			4);


// 	status = _ICM20948_SelectUserBank(hi2c, selectI2cAddress, USER_BANK_0);

// 	status = _ICM20948_WriteByte(
// 			hi2c,
// 			selectI2cAddress,
// 			ICM20948__USER_BANK_0__INT_PIN_CFG__REGISTER,
// 			0x02); // Don't understand how this works

// 	status = _AK09916_WriteByte(
// 			hi2c,
// 			AK09916__CNTL2__REGISTER,
// 			0x08);
// }

// HAL_StatusTypeDef _ICM20948_BrustRead(I2C_HandleTypeDef * hi2c, uint8_t selectI2cAddress, uint8_t startAddress, uint16_t amountOfRegistersToRead, uint8_t* readData) {
// 	HAL_StatusTypeDef status = HAL_OK;
// 	uint8_t deviceI2CAddress = (selectI2cAddress == 0)? ICM20948__I2C_SLAVE_ADDRESS_1: ICM20948__I2C_SLAVE_ADDRESS_2;

// 	status = HAL_I2C_Mem_Read(
// 				hi2c,
// 				deviceI2CAddress << 1,
// 				startAddress,
// 				I2C_MEMADD_SIZE_8BIT,
// 				readData,
// 				amountOfRegistersToRead * I2C_MEMADD_SIZE_8BIT,
// 				10
// 			);

// 	return status;
// }

// void ICM20948_readGyroscope_Z(I2C_HandleTypeDef* hi2c, uint8_t selectI2cAddress, uint8_t selectGyroSensitivity, float* gyroZ) {
// 	uint8_t readData[2];

// //	_ICM20948_SelectUserBank(hi2c, selectI2cAddress, USER_BANK_0);
// 	_ICM20948_BrustRead(hi2c, selectI2cAddress, ICM20948__USER_BANK_0__GYRO_ZOUT_H__REGISTER, 2, readData);

// 	int16_t reading = readData[0]<<8 | readData[1];
// 	*gyroZ = (float) -reading;
// 	switch (selectGyroSensitivity) {
// 		case GYRO_FULL_SCALE_250DPS:
// 			*gyroZ /= GRYO_SENSITIVITY_SCALE_FACTOR_250DPS;
// 			break;
// 		case GYRO_FULL_SCALE_500DPS:
// 			*gyroZ /= GRYO_SENSITIVITY_SCALE_FACTOR_500DPS;
// 			break;
// 		case GYRO_FULL_SCALE_1000DPS:
// 			*gyroZ /= GRYO_SENSITIVITY_SCALE_FACTOR_1000DPS;
// 			break;
// 		case GYRO_FULL_SCALE_2000DPS:
// 			*gyroZ /= GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS;
// 			break;
// 	}
// }

// void ICM20948_readAccelerometer_all(I2C_HandleTypeDef * hi2c, uint8_t selectI2cAddress, uint8_t selectAccelSensitivity, float readings[3]) {
// 	uint8_t readData[6];

// //	_ICM20948_SelectUserBank(hi2c, selectI2cAddress, USER_BANK_0);
// 	_ICM20948_BrustRead(hi2c, selectI2cAddress, ICM20948__USER_BANK_0__ACCEL_XOUT_H__REGISTER, 6, readData);


// 	int16_t rD_int[3];
// 	rD_int[X] = readData[X_HIGH_BYTE]<<8|readData[X_LOW_BYTE];
// 	rD_int[Y] = readData[Y_HIGH_BYTE]<<8|readData[Y_LOW_BYTE];
// 	rD_int[Z] = readData[Z_HIGH_BYTE]<<8|readData[Z_LOW_BYTE];

// 	float rD[3];
// 	rD[X] = (float) rD_int[X];
// 	rD[Y] = (float) rD_int[Y];
// 	rD[Z] = (float) rD_int[Z];

// 	switch (selectAccelSensitivity) {
// 		case ACCEL_FULL_SCALE_2G:
// 			readings[X] = rD[X] / ACCEL_SENSITIVITY_SCALE_FACTOR_2G;
// 			readings[Y] = rD[Y] / ACCEL_SENSITIVITY_SCALE_FACTOR_2G;
// 			readings[Z] = rD[Z] / ACCEL_SENSITIVITY_SCALE_FACTOR_2G;
// 			break;
// 		case ACCEL_FULL_SCALE_4G:
// 			readings[X] = rD[X] / ACCEL_SENSITIVITY_SCALE_FACTOR_4G;
// 			readings[Y] = rD[Y] / ACCEL_SENSITIVITY_SCALE_FACTOR_4G;
// 			readings[Z] = rD[Z] / ACCEL_SENSITIVITY_SCALE_FACTOR_4G;
// 			break;
// 		case ACCEL_FULL_SCALE_8G:
// 			readings[X] = rD[X] / ACCEL_SENSITIVITY_SCALE_FACTOR_8G;
// 			readings[Y] = rD[Y] / ACCEL_SENSITIVITY_SCALE_FACTOR_8G;
// 			readings[Z] = rD[Z] / ACCEL_SENSITIVITY_SCALE_FACTOR_8G;
// 			break;
// 		case ACCEL_FULL_SCALE_16G:
// 			readings[X] = rD[X] / ACCEL_SENSITIVITY_SCALE_FACTOR_16G;
// 			readings[Y] = rD[Y] / ACCEL_SENSITIVITY_SCALE_FACTOR_16G;
// 			readings[Z] = rD[Z] / ACCEL_SENSITIVITY_SCALE_FACTOR_16G;
// 			break;
// 	}
// }


/*---------- KALMAN FILTER ----------*/
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


/*---------- PID ----------*/
void pid_reset(pid_t* pid)  {
	pid->error_accumulate = 0;
	pid->error_old = 0;
}

void pid_init(pid_t* pid, float Kp, float Ki, float Kd) {
	pid_reset(pid);

	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
}

float pid_adjust(pid_t* pid, float error, float scale) {
	pid->error_accumulate += error;
	float error_rate = (error - pid->error_old);
	pid->error_old = error;

	return scale * (pid->Kp*error + pid->Ki*pid->error_accumulate + pid->Kd*error_rate);
}
