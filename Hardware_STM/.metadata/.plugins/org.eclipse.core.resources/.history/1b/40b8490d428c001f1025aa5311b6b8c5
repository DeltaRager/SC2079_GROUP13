/*
 * Servo.c
 *
 *  Created on: Sep 10, 2024
 *      Author: AD
 *
 *  Control the servo (front wheels)
 */

#include "servo.h"


TIM_HandleTypeDef* pwm_tim;

void servo_init(TIM_HandleTypeDef* pwm) {
	pwm_tim = pwm;
	HAL_TIM_PWM_Start(pwm, SERVO_PWM_CHANNEL);
}

void servo_set_dir(uint32_t val) {
	HAL_Delay(500);
	pwm_tim->Instance->CCR1 = val;
}
