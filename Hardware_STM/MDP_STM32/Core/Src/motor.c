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


/*---------- Movement ----------*/
void stop() {
	motor_stop();
	servo_set_dir(STRAIGHT);
	HAL_Delay(200);
}

void forward(uint32_t distance) {
	float wheel_radius = 3.4;							// Wheel radius (cm)
    float circumference = 2 * 3.14159 * wheel_radius;	// Calculate circumference
    uint32_t pulses_per_rev = 1550;						// Encoder's specification: 11 ppr * 30 = 330 (30x reducer)
    float pulses_per_cm = pulses_per_rev / circumference;
    uint32_t target_pulses = (uint32_t)(distance * pulses_per_cm);
    
    servo_set_dir(STRAIGHT);
    HAL_Delay(500);
    OLED_Clear();
    print_OLED(0, 0, "target_pulse: %u", true, target_pulses);
    
    // Reset the encoder and initialize encoder count to 65535 after resetting
    reset_encoders();
    uint32_t encoder_cnt = 65535; 

    // Move forward
    motor_forward();
    print_OLED(0, 15, "encoder_cnt: ", false, 0);
    while (65535 - encoder_cnt < target_pulses) {
        encoder_cnt = __HAL_TIM_GET_COUNTER(l_enc_tim);
    	print_OLED(40, 15, "%u", true, encoder_cnt);
    }
    
    // Stop the motors when the target distance is reached
    motor_stop();
    HAL_Delay(300);
}

void backward(uint32_t distance) {
    float wheel_radius = 3.4;  							// Wheel radius (cm)
    float circumference = 2 * 3.14159 * wheel_radius;  	// Calculate circumference
    uint32_t pulses_per_rev = 1550;  					// Encoder's specification
    float pulses_per_cm = pulses_per_rev / circumference;
    uint32_t target_pulses = (uint32_t)(distance * pulses_per_cm);
    
    servo_set_dir(STRAIGHT);
    HAL_Delay(500);
    OLED_Clear();
    print_OLED(0, 0, "target_pulse: %u", true, target_pulses);
    
    // Reset the encoder and initialize encoder count to 0 after resetting
    reset_encoders();
    uint32_t encoder_cnt = 0;

    // Move backward
    motor_backward();
    OLED_Clear();
    OLED_ShowString(0, 0, "encoder_cnt: ");
    OLED_Refresh_Gram();

    // Monitor encoder count until the target distance is reached
    while (encoder_cnt < target_pulses) {
        encoder_cnt = __HAL_TIM_GET_COUNTER(l_enc_tim);
//        sprintf(buf, "%u", encoder_cnt);
//        OLED_ShowString(0, 30, buf);
//        OLED_Refresh_Gram();
    }

    // Stop the motors when the target distance is reached
    HAL_Delay(300);
}

void forward_right() {
    HAL_Delay(200);  // Reduced delay before moving

    // Set servo position for right turn
    servo_set_dir(RIGHT);
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
//    OLED_Clear();
//    OLED_ShowString(0, 0, "encoder_cnt: ");
//    OLED_Refresh_Gram();

    // Loop until the desired arc length is reached
    while (65535 - encoder_cnt < arc_length) {
        encoder_cnt = __HAL_TIM_GET_COUNTER(l_enc_tim);  // Update encoder count
//        sprintf(buf, "%u", encoder_cnt);  // Format encoder count for display
//        OLED_ShowString(0, 30, buf);  // Display current encoder count
//        OLED_Refresh_Gram();  // Refresh OLED display
    }

    // Stop the motors when the target distance is reached
    HAL_Delay(300);
}

void forward_left() {
	HAL_Delay(200);  // Reduced delay before moving

	// Set servo position for right turn
	servo_set_dir(LEFT);
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

    while (65535 - encoder_cnt < arc_length) {
    	encoder_cnt = __HAL_TIM_GET_COUNTER(l_enc_tim);  // Update encoder count
//    	sprintf(buf, "%u", encoder_cnt);  // Format encoder count for display
//    	OLED_ShowString(0, 30, buf);  // Display current encoder count
//    	OLED_Refresh_Gram();  // Refresh OLED display
    }

    // Stop the motors when the target distance is reached
    HAL_Delay(300);
}

void backward_right() {
	// Servo direction: RIGHT
	servo_set_dir(RIGHT);
	motor_backward();
}

void backward_left() {
	// Servo direction: LEFT
	servo_set_dir(LEFT);
	motor_backward();
}
