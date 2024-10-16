/*
 * Motor.h
 *
 *  Created on: Sep 10, 2024
 *      Author: AD
 *
 *  Control the motor (back wheels)
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include <math.h>
#include "main.h"
#include "oled.h"
#include "servo.h"
#include "helper.h"


// PWM Parameters
#define MOTOR_PWM_PERIOD		7200
#define MOTOR_PWM_MAX 			6000 // Safe value
//#define MOTOR_PWM_MIN 			375  // Min speed
#define MOTOR_PWM_MIN 			0  // Min speed
#define MOTOR_PWM_ACCEL 		20 	 // Max change in PWM value allowed (smooth transitioning)
//#define MOTOR_PWM_DECCEL 100 //maximum negative change in PWM value allowed (smooth transitioning)
#define MOTOR_PWM_OFFSET_MAX_PERCENT	0.05f //maximum offset allowed (percentage of target PWM)
#define MOTOR_BRAKING_DIST_CM_TARGET	30.0f //30.0cm at max speed
#define MOTOR_BRAKING_DIST_CM_AWAY		40.0f //40.0cm at max speed
#define MOTOR_CORRECTION_PERIOD			10.0f //correct every 20ms (accumulate encoder)

// Left and Right PWM Channels
#define L_CHANNEL TIM_CHANNEL_1
#define R_CHANNEL TIM_CHANNEL_2


int16_t PID_Control_left();
int16_t PID_Control_right();
void motor_init(TIM_HandleTypeDef* pwm, TIM_HandleTypeDef* l_enc, TIM_HandleTypeDef* r_enc);
void motor_forward();
void motor_backward();
void motor_stop();
int16_t get_speed_pwm(uint8_t speed);
void motor_set_speed(uint8_t speed);
void reset_encoders();
void set_pwm_LR();
void motor_get_drive(int8_t dir, uint8_t speed);

void stop();
void forward(uint32_t distance);
void backward(uint32_t distance);
void forward_right();
void forward_left();
void backward_right();
void backward_left();

#endif /* INC_MOTOR_H_ */
