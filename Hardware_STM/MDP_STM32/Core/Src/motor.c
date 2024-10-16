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
int16_t pwmValAccel = 0, pwm_val_target = 0, l_pwm_val = 0, r_pwm_val = 0;


void motor_init(TIM_HandleTypeDef* pwm, TIM_HandleTypeDef* l_enc, TIM_HandleTypeDef* r_enc) {
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

void motor_forward() {
	HAL_GPIO_WritePin(MOTOR_A_IN1_GPIO_Port, MOTOR_A_IN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_A_IN2_GPIO_Port, MOTOR_A_IN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_B_IN1_GPIO_Port, MOTOR_B_IN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_B_IN2_GPIO_Port, MOTOR_B_IN2_Pin, GPIO_PIN_RESET);
}

void motor_backward() {
	HAL_GPIO_WritePin(MOTOR_A_IN1_GPIO_Port, MOTOR_A_IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_A_IN2_GPIO_Port, MOTOR_A_IN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_B_IN1_GPIO_Port, MOTOR_B_IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_B_IN2_GPIO_Port, MOTOR_B_IN2_Pin, GPIO_PIN_SET);
}

void motor_stop() {
	HAL_GPIO_WritePin(MOTOR_A_IN1_GPIO_Port, MOTOR_A_IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_A_IN2_GPIO_Port, MOTOR_A_IN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_B_IN1_GPIO_Port, MOTOR_B_IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_B_IN2_GPIO_Port, MOTOR_B_IN2_Pin, GPIO_PIN_RESET);
}

int16_t get_speed_pwm(uint8_t speed) {
	int16_t val = (int16_t)(MOTOR_PWM_MAX / 100 * speed);
	return val;
}

// Speed: 0 - 100
void motor_set_speed(uint8_t speed) {
	pwm_val_target = get_speed_pwm(speed);
	l_pwm_val = pwm_val_target;
	r_pwm_val = pwm_val_target;
	set_pwm_LR();
}

void reset_encoders() {
	// Reset timers
	__HAL_TIM_SET_COUNTER(l_enc_tim, 0);
	__HAL_TIM_SET_COUNTER(r_enc_tim, 0);
}

void set_pwm_LR() {
	// Set L, R channels
	int16_t l_temp = l_pwm_val, r_temp = r_pwm_val;

	if (l_temp > MOTOR_PWM_MAX) {
		l_temp = MOTOR_PWM_MAX;
	} else if (l_temp < MOTOR_PWM_MIN) {
		l_temp = MOTOR_PWM_MIN;
	} 

	if (r_temp > MOTOR_PWM_MAX) {
		r_temp = MOTOR_PWM_MAX;
	} else if (r_temp < MOTOR_PWM_MIN) {
		r_temp = MOTOR_PWM_MIN;
	}

	__HAL_TIM_SET_COMPARE(motor_pwm_tim, L_CHANNEL, l_pwm_val);
	__HAL_TIM_SET_COMPARE(motor_pwm_tim, R_CHANNEL, r_pwm_val);
}

// Speed: 0 - 100
void motor_get_drive(int8_t dir, uint8_t speed) {
	// Stop
	if (dir == 0) {
		motor_stop();
		return;
	}

	// Derive PWM value
	pwm_val_target = get_speed_pwm(speed);
	l_pwm_val = pwm_val_target;
	r_pwm_val = pwm_val_target;

	// Reset encoders
	reset_encoders();

	// Move
	move_straight(dir);
	set_pwm_LR();
}


/*---------- Movement ----------*/
void stop() {
	motor_stop();
}

void forward(uint32_t distance) {
	static bool has_run = false;  // Flag to check if the function has already run
	if (has_run) return;  // Exit if the function has already been executed once
	has_run = true;  // Set the flag to true to prevent future runs

	float wheel_radius = 3.4;							// Wheel radius (cm)
    float circumference = 2 * 3.14159 * wheel_radius;	// Calculate circumference
    uint32_t pulses_per_rev = 1550;						// Encoder's specification: 11 ppr * 30 (30x reducer) = 330
    float pulses_per_cm = pulses_per_rev / circumference;
    uint32_t target_pulses = (uint32_t)(distance * pulses_per_cm);
    
    servo_set_val(STRAIGHT);
    HAL_Delay(500);
	uint8_t buf[100];
    sprintf(buf, "target_pulse: %u", target_pulses);
	OLED_Clear();
	OLED_ShowString(0, 15, buf);
	OLED_Refresh_Gram();
	HAL_Delay(500);
    
    // Reset encoder count
    reset_encoders();
    uint32_t encoder_cnt = 65535;	// Initialize to 65535 after resetting the encoder

    // Move forward
    motor_forward();
	OLED_Clear();
	OLED_ShowString(0, 0, "encoder_cnt: ");
	OLED_Refresh_Gram();
    while (65535 - encoder_cnt < target_pulses) {
        encoder_cnt = __HAL_TIM_GET_COUNTER(l_enc_tim);
        sprintf(buf, "%u", encoder_cnt);
        OLED_ShowString(0, 30, buf);
		OLED_Refresh_Gram();
    }
    
    // Stop the motors when the target distance is reached
    motor_stop();
}

void backward(uint32_t distance) {
	static bool has_run = false;  // Flag to check if the function has already run
    if (has_run) return;  // Exit if the function has already been executed once
	has_run = true;  // Set the flag to true to prevent future runs

    float wheel_radius = 3.4;  // Wheel radius (cm)
    float circumference = 2 * 3.14159 * wheel_radius;  // Calculate circumference
    uint32_t pulses_per_rev = 1550;  // Encoder's specification
    float pulses_per_cm = pulses_per_rev / circumference;
    uint32_t target_pulses = (uint32_t)(distance * pulses_per_cm);
    
    servo_set_val(STRAIGHT);
    HAL_Delay(500);
    
    // Display target pulses
    uint8_t buf[100];
    sprintf(buf, "target_pulse: %u", target_pulses);
    OLED_Clear();
    OLED_ShowString(0, 0, buf);
    OLED_Refresh_Gram();
    HAL_Delay(1000);  // Delay to allow user to see target pulse

    // Reset encoder count
    reset_encoders();
    uint32_t encoder_cnt = 0;	// Initialize to 0 after resetting the encoder

    // Move backward
    motor_backward();
    OLED_Clear();
    OLED_ShowString(0, 0, "encoder_cnt: ");
    OLED_Refresh_Gram();

    // Monitor encoder count until the target distance is reached
    while (encoder_cnt < target_pulses) {
        encoder_cnt = __HAL_TIM_GET_COUNTER(l_enc_tim);
        sprintf(buf, "%u", encoder_cnt);
        OLED_ShowString(0, 30, buf);
        OLED_Refresh_Gram();
    }

    // Stop the motors when the target distance is reached
    motor_stop();
}

void forward_right() {
    static bool has_run = false;  // Flag to check if the function has already run
    if (has_run) return;  // Exit if the function has already been executed once
    has_run = true;  // Set the flag to true to prevent future runs

    HAL_Delay(200);  // Reduced delay before moving

    // Set servo position for right turn
    servo_set_val(RIGHT);
    HAL_Delay(200);  // Reduced delay after setting servo

    uint8_t buf[100];
    uint32_t right_circumference = 17500;  // Circumference of the right wheel (in mm)

    // Reset encoder count
    reset_encoders();
    uint32_t encoder_cnt = 65535;  // Initialize encoder count to 0 after resetting

    // Length of the arc to travel based on the angle
    float arc_length = 90.0 / 360 * right_circumference;

    // Move forward
    motor_forward();

    // Display initialization for encoder count monitoring
    OLED_Clear();
    OLED_ShowString(0, 0, "encoder_cnt: ");
    OLED_Refresh_Gram();

    // Loop until the desired arc length is reached
    while (65535 - encoder_cnt < arc_length) {
        encoder_cnt = __HAL_TIM_GET_COUNTER(l_enc_tim);  // Update encoder count
        sprintf(buf, "%u", encoder_cnt);  // Format encoder count for display
        OLED_ShowString(0, 30, buf);  // Display current encoder count
        OLED_Refresh_Gram();  // Refresh OLED display
    }

    // Stop the motors when the target distance is reached
    motor_stop();  // Stop moving
}

void forward_left() {
	static bool has_run = false;  // Flag to check if the function has already run
	if (has_run) return;  // Exit if the function has already been executed once
	has_run = true;  // Set the flag to true to prevent future runs
	HAL_Delay(200);  // Reduced delay before moving

	    // Set servo position for right turn
	servo_set_val(LEFT);
	HAL_Delay(200);  // Reduced delay after setting servo

	uint8_t buf[100];
//	uint32_t left_circumference = 8575;
	uint32_t left_circumference = 15000;
    // Reset encoder count
	reset_encoders();
	uint32_t encoder_cnt = 65535;

    // Length
    float arc_length = 90.0 / 360 * left_circumference;

    // Move forward
	motor_forward();

	OLED_Clear();
	OLED_ShowString(0, 0, "encoder_cnt: ");
	OLED_Refresh_Gram();

    while (65535-encoder_cnt < arc_length) {
    	encoder_cnt = __HAL_TIM_GET_COUNTER(l_enc_tim);  // Update encoder count
    	sprintf(buf, "%u", encoder_cnt);  // Format encoder count for display
    	OLED_ShowString(0, 30, buf);  // Display current encoder count
    	OLED_Refresh_Gram();  // Refresh OLED display
    }

    // Stop the motors when the target distance is reached
    motor_stop();
    //HAL_Delay(500);
}

void backward_right() {
	// Servo direction: RIGHT
	servo_set_val(RIGHT);
	motor_backward();
}

void backward_left() {
	// Servo direction: LEFT
	servo_set_val(LEFT);
	motor_backward();
}
