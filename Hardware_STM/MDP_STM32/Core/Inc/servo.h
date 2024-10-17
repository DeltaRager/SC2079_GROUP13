/*
 * Servo.h
 *
 *  Created on: Sep 14, 2024
 *      Author: AD
 *
 *  Control the servo (front wheels)
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "main.h"
#include "oled.h"
#include "helper.h"

#define SERVO_PWM_CHANNEL TIM_CHANNEL_1
#define STRAIGHT	4775
//#define LEFT		3000	// 38 degree

#define LEFT		3000
#define RIGHT		7050	// 25 degree

void servo_init(TIM_HandleTypeDef* pwm);
void servo_set_dir(uint32_t val);

#endif /* INC_SERVO_H_ */
