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

void servo_set_val(uint32_t val) {
	pwm_tim->Instance->CCR1 = val;
}

void servo_set_direction(uint8_t dir) {
	uint16_t ccr_value = STRAIGHT;

	if (dir == -1) {
		ccr_value = LEFT;
	} else if (dir == 1) {
		ccr_value = RIGHT;
	}

	servo_setVal(ccr_value);
	print_value(0, 0, "ccr_value: %u", ccr_value);
}

