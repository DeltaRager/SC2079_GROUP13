/*
 * OLED.h
 *
 *  Created on: Sep 10, 2024
 *      Author: AD
 *
 *  Control the OLED
 */

#ifndef INC_OLED_H_
#define INC_OLED_H_

#include "stm32f4xx_hal.h"
#include "main.h"


//----------------- OLED Definition ----------------
#define OLED_SCLK_Clr() HAL_GPIO_WritePin(OLED_SCLK_GPIO_Port, OLED_SCLK_Pin, GPIO_PIN_RESET)	// SCL
#define OLED_SCLK_Set() HAL_GPIO_WritePin(OLED_SCLK_GPIO_Port, OLED_SCLK_Pin, GPIO_PIN_SET)	// SCL

#define OLED_SDIN_Clr()	HAL_GPIO_WritePin(OLED_SDIN_GPIO_Port, OLED_SDIN_Pin, GPIO_PIN_RESET)	// SDA
#define OLED_SDIN_Set()	HAL_GPIO_WritePin(OLED_SDIN_GPIO_Port, OLED_SDIN_Pin, GPIO_PIN_SET)	// SDA

#define OLED_RST_Clr()	HAL_GPIO_WritePin(OLED_RESET_GPIO_Port, OLED_RESET_Pin, GPIO_PIN_RESET)	// RST = 0
#define OLED_RST_Set()	HAL_GPIO_WritePin(OLED_RESET_GPIO_Port, OLED_RESET_Pin, GPIO_PIN_SET)	// RST = 1

#define OLED_RS_Clr()	HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_RESET)	// DC = 0
#define OLED_RS_Set()	HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_SET)		// DC = 1

#define OLED_CMD  0		// Write Command
#define OLED_DATA 1		// Write Data


// OLED Control Functions
void OLED_WR_Byte(uint8_t dat, uint8_t cmd);
void OLED_Display_On();
void OLED_Display_Off();
void OLED_Refresh_Gram();
void OLED_Init();
void OLED_Clear();
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t t);
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size, uint8_t mode);
void OLED_ShowNumber(uint8_t x, uint8_t y,uint32_t num, uint8_t len, uint8_t size);
void OLED_ShowString(uint8_t x, uint8_t y, const uint8_t *p);
void OLED_Set_Pos(unsigned char x, unsigned char y);


#endif /* INC_OLED_H_ */
