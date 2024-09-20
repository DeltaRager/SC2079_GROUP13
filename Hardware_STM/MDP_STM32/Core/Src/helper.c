/*
 * helper.c
 *
 *  Created on: Sep 10, 2024
 *      Author: AD
 *
 *  Contain helper functions
 */

#include "helper.h"


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





