/*
 * motor.c
 *
 *  Created on: Sep 10, 2024
 *      Author: AD
 *
 *  Control the motor (back wheels)
 */

#include "motor.h"


// Timers for PWM, L and R Encoders
TIM_HandleTypeDef *motor_pwm_tim, *l_enc_tim, *r_enc_tim;

// For matching motor speeds.
int16_t pwmValAccel = 0, pwmValTarget = 0, lPwmVal = 0, rPwmVal = 0;
int16_t lLastCount = 0, rLastCount = 0;
// For bi-directional correction.
int8_t curDir = 0;


void motor_init(TIM_HandleTypeDef *pwm, TIM_HandleTypeDef *l_enc, TIM_HandleTypeDef *r_enc) {
	// Assign timer pointers
	motor_pwm_tim = pwm;
	l_enc_tim = l_enc;
	r_enc_tim = r_enc;

	// Start Encoders and PWM for L, R motors
	HAL_TIM_Encoder_Start_IT(l_enc, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start_IT(r_enc, TIM_CHANNEL_ALL);
	HAL_TIM_PWM_Start(pwm, L_CHANNEL);
	HAL_TIM_PWM_Start(pwm, R_CHANNEL);
}

void setDriveDir(int8_t dir) {
	if (dir == 1) {
		// Forward
		HAL_GPIO_WritePin(MOTOR_A_IN1_GPIO_Port, MOTOR_A_IN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MOTOR_A_IN2_GPIO_Port, MOTOR_A_IN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR_B_IN1_GPIO_Port, MOTOR_B_IN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MOTOR_B_IN2_GPIO_Port, MOTOR_B_IN2_Pin, GPIO_PIN_RESET);
	} else if (dir == -1) {
		// Backward
		HAL_GPIO_WritePin(MOTOR_A_IN1_GPIO_Port, MOTOR_A_IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR_A_IN2_GPIO_Port, MOTOR_A_IN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MOTOR_B_IN1_GPIO_Port, MOTOR_B_IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR_B_IN2_GPIO_Port, MOTOR_B_IN2_Pin, GPIO_PIN_SET);
	} else if (dir == 0) {
		// Stop
		HAL_GPIO_WritePin(MOTOR_A_IN1_GPIO_Port, MOTOR_A_IN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MOTOR_A_IN2_GPIO_Port, MOTOR_A_IN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MOTOR_B_IN1_GPIO_Port, MOTOR_B_IN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MOTOR_B_IN2_GPIO_Port, MOTOR_B_IN2_Pin, GPIO_PIN_SET);
	} else if (dir == 2)  {
		// Turn right (Motor A ON, Motor B OFF)
		HAL_GPIO_WritePin(MOTOR_A_IN1_GPIO_Port, MOTOR_A_IN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MOTOR_A_IN2_GPIO_Port, MOTOR_A_IN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR_B_IN1_GPIO_Port, MOTOR_B_IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR_B_IN2_GPIO_Port, MOTOR_B_IN2_Pin, GPIO_PIN_SET);
	} else if (dir == -2) {
		// Turn left (Motor A OFF, Motor B ON)
		HAL_GPIO_WritePin(MOTOR_A_IN1_GPIO_Port, MOTOR_A_IN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MOTOR_A_IN2_GPIO_Port, MOTOR_A_IN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MOTOR_B_IN1_GPIO_Port, MOTOR_B_IN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MOTOR_B_IN2_GPIO_Port, MOTOR_B_IN2_Pin, GPIO_PIN_RESET);
	}
}

void timer_reset(TIM_HandleTypeDef* htim) {
	__HAL_TIM_SET_COUNTER(htim, 0);
}

int16_t getSpeedPwm(uint8_t speed) {
	int16_t val = (int16_t)(MOTOR_PWM_MAX / 100 * speed);
	return val;
}

void resetEncoders() {
	timer_reset(l_enc_tim);
	timer_reset(r_enc_tim);

	lLastCount = rLastCount = 0;
}

void setPwmLR() {
	// Set L, R channels
	__HAL_TIM_SetCompare(motor_pwm_tim, L_CHANNEL,
		lPwmVal > MOTOR_PWM_MAX
		? MOTOR_PWM_MAX
		: lPwmVal < MOTOR_PWM_MIN
		? MOTOR_PWM_MIN
		: lPwmVal);
	__HAL_TIM_SetCompare(motor_pwm_tim, R_CHANNEL,
		rPwmVal > MOTOR_PWM_MAX
		? MOTOR_PWM_MAX
		: rPwmVal < MOTOR_PWM_MIN
		? MOTOR_PWM_MIN
		: rPwmVal);
}

// Speed: 0 - 100
void motor_setDrive(int8_t dir, uint8_t speed) {
	if (dir == 0) {
		setDriveDir(0);
		pwmValAccel = 0;
		return;
	}

	// Derive PWM value.
	pwmValTarget = getSpeedPwm(speed);

	if (dir == 2) {
		lPwmVal = pwmValTarget;
		rPwmVal = pwmValTarget;
	} else if (dir == -2) {
		lPwmVal = 0;
		rPwmVal = pwmValTarget;
	} else {
		lPwmVal = pwmValTarget;
		rPwmVal = pwmValTarget;
	}

	// Reset
	resetEncoders();

	curDir = dir;
	setDriveDir(dir);
	setPwmLR();

	uint8_t buffer1[100];
	uint8_t buffer2[100];

	//sprintf(buffer1, "lPwmVal: %" PRId16, lPwmVal);
	//sprintf(buffer2, "rPwmVal: %" PRId16, rPwmVal);
	//OLED_ShowString(0, 0, buffer1);
	//OLED_ShowString(0, 15, buffer2);
	//OLED_Refresh_Gram();
}
