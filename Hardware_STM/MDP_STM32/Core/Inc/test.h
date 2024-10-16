/*
 * test.h
 *
 *  Created on: Oct 7, 2024
 *      Author: AD
 */

#ifndef INC_TEST_H_
#define INC_TEST_H_

#include "main.h"
#include "oled.h"
#include "helper.h"
#include "motor.h"
#include "servo.h"
#include "sensor.h"


// Global variables
extern uint8_t buffer[100];
extern uint32_t tc1, tc2, echo;
extern float dist;
extern UART_HandleTypeDef huart3;
extern sensor_t sensor;
extern uint8_t cmd_cnt;
extern uint8_t cmd_buffer[1000];
extern int head, curr;
extern uint8_t receive[1];


// Checklist
void task_A1();
void task_A3();
void task_A4_left();
void task_A4_right();
void abhinav_task();
void moving_task();

// Functionality
void forward_task();
void backward_task();
void forward_right_task();
void forward_left_task();
void backward_right_task();
void backward_left_task();
void ultrasonic_task();
void accelerometer_task();
void gyroscope_task();
void UART3_task();
void send_ack_task();

#endif /* INC_TEST_H_ */
