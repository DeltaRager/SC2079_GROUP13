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


/*--------- MOVEMENT DEFINITION ---------*/
#define FORWARD 		'w'
#define BACKWARD 		's'
#define FORWARD_LEFT 	'a'
#define FORWARD_RIGHT 	'd'
#define BACKWARD_LEFT 	'z'
#define BACKWARD_RIGHT 	'c'
#define STOP 			'x'
#define ACK 			'l'


/*---------- OLED INTERACTION ----------*/
bool is_USER_button_pressed();
void print_OLED(int x, int y, uint8_t* msg, bool var_exist, int32_t val);


/*-------------- COMMANDS --------------*/
void send_ack();
void move(uint8_t cmd);

#endif /* INC_HELPER_H_ */
