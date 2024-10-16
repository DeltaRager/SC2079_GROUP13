/*
 * test.c
 *
 *  Created on: Oct 10, 2024
 *      Author: AD
 */

#include "test.h"


// Checklist
void task_A1(uint8_t* buffer) {
	// Task A1
	if (buffer[0] == 'a') {
		servo_set_val(LEFT);
		HAL_Delay(2000);
		OLED_Clear();
		OLED_ShowString(0, 0, "Left");
		OLED_Refresh_Gram();
		motor_forward_left(90);
	}
}

void task_A3() {
	// Task A3
	uint32_t dist = 100;
	servo_set_val(STRAIGHT);
	motor_set_speed(20);
	HAL_Delay(1000);
	motor_forward(dist);
}

void task_A4_left() {
	  // Task A4 LEFT
	  uint16_t angle = 360;
	  servo_set_val(LEFT);
	  motor_set_speed(20);

	  for (int i = 0; i < angle / 90; angle -= 90)
		  motor_forward_left(90);
}

void task_A4_right() {
	  // Task A4 RIGHT
	   uint16_t angle = 360;
	   servo_set_val(RIGHT);
	   motor_set_speed(20);

	   for (int i = 0; i < angle / 90; angle -= 90)
	 	  motor_forward_right(90);
}

void abhinav_task() {
	// Abhinav's Task: left - right - right - backward - repeat
	for (int i = 0; i < 3; i++) {
		motor_forward_right(90);
		HAL_Delay(1000);
		servo_set_val(STRAIGHT);
		HAL_Delay(1000);

		motor_forward_left(90);
		motor_forward_left(90);
		HAL_Delay(2000);
		motor_backward_inf();
		HAL_Delay(3000);
		motor_stop();
		HAL_Delay(2000);
	}
}

void moving_task() {
	// Retrieving commands from the tablet
	// HAL_UART_Receive(&huart3, buffer, 1, 3000);
	memset(buffer, 0, sizeof(buffer));
	// OLED_ShowString(0, 0, buffer);
	// OLED_Refresh_Gram();

	HAL_UART_Receive(&huart3, buffer, 1, 3000);

	if (buffer[0] == 'w') {
		servo_set_val(STRAIGHT);
		HAL_Delay(500);
		OLED_Clear();
		OLED_ShowString(0, 0, "Forward");
		OLED_Refresh_Gram();
		motor_forward(80);
	} else if (buffer[0] == 's') {
		servo_set_val(STRAIGHT);
		HAL_Delay(500);
		OLED_Clear();
		OLED_ShowString(0, 0, "Backward");
		OLED_Refresh_Gram();
		motor_backward_inf();
	} else if (buffer[0] == 'a') {
		servo_set_val(LEFT);
		HAL_Delay(500);
		OLED_Clear();
		OLED_ShowString(0, 0, "Left");
		OLED_Refresh_Gram();
		motor_forward_left(90);
	} else if (buffer[0] == 'd') {
		servo_set_val(RIGHT);
		HAL_Delay(500);
		OLED_Clear();
		OLED_ShowString(0, 0, "Right");
		OLED_Refresh_Gram();
		motor_forward_right(90);
	}
}

// Functionality
void forward_task() {
	forward(80);
}

void backward_task() {
	backward(80);
}

void forward_right_task() {
	forward_right();
}

void forward_left_task() {
	forward_left();
}

void backward_right_task() {
	backward_right();
}

void backward_left_task() {
	backward_left();
}

void ultrasonic_task() {
    HCSR04_Init();
    HAL_Delay(200);

    OLED_ShowString(0, 0, "tc1:");
    OLED_ShowString(0, 15, "tc2:");
    OLED_ShowString(0, 30, "echo:");
    OLED_ShowString(0, 45, "dist:");

    sprintf(buffer, "%u", tc1);
    OLED_ShowString(40, 0, buffer);
    sprintf(buffer, "%u", tc2);
    OLED_ShowString(40, 15, buffer);
    sprintf(buffer, "%u", echo);
    OLED_ShowString(40, 30, buffer);
    sprintf(buffer, "%f", dist);
    OLED_ShowString(40, 45, buffer);
    OLED_Refresh_Gram();
    HAL_Delay(200);
}

void accelerometer_task() {

}

void gyroscope_task() {
	print_value(0, 0, "gyro: %f", sensor.gyroZ);
}

void UART3_task() {
	print_OLED(0, 0, "cmd_cnt: %u", true, cmd_cnt);
	HAL_Delay(1000);

	if (cmd_cnt == 1) {
		move(receive[0]);
		head = NULL;
		free(curr);
		curr = NULL;
		cmd_cnt--;
		send_ack(&huart3);
	} else if (cmd_cnt > 1) {
		move(receive[0]);
		cmd_t* temp = head;
		head = head->next;
		free(temp);
		cmd_cnt--;
		send_ack(&huart3);
	}
}
	
void send_ack_task() {
	uint8_t ack[] = "l";
	HAL_UART_Transmit(&huart3, ack, sizeof(ack), 2000);
}
