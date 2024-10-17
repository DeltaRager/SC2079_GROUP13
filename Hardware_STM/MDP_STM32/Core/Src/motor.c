/*
 * motor.c
 *
 *  Created on: Sep 10, 2024
 *      Author: AD
 *
 *  Control the motor (back wheels)
 */

#include "motor.h"
#include <math.h>
#include "ICM20948.h"

// Timers for PWM, L and R Encoders
TIM_HandleTypeDef *motor_pwm_tim, *l_enc_tim, *r_enc_tim;
I2C_HandleTypeDef *gyro_i2c;
// For matching motor speeds.
int16_t pwmValAccel = 0, pwm_val_target = 0, l_pwm_val = 0, r_pwm_val = 0;

uint32_t l_counter = 0, r_counter = 0;
int16_t l_count = 0, r_count = 0;

int16_t l_speed =0, r_speed = 0;
int16_t no_of_tick = 50;

int16_t l_rpm = 0, r_rpm = 0;
int16_t pwmMax = 1500;
int16_t pwmMax_back = 2000;

int l_err, r_err;
int start=0;

int16_t l_position = 0, r_position = 0;
int16_t l_angle = 0, r_angle = 0;
int16_t target_angle = 0;
extern int16_t l_oldpos, r_oldpos;
int16_t position_target; // target position
int16_t l_error, r_error;           // error between target and actual
int32_t l_error_area = 0, r_error_area = 0;  // area under error - to calculate I for PI implementation
int32_t l_error_old, l_error_change, r_error_old, r_error_change;
float_t l_error_rate, r_error_rate; // to calculate D for PID control
int32_t l_millisOld, l_millisNow, l_dt;
int32_t r_millisOld, r_millisNow, r_dt;// to calculate I and D for PID control
int16_t Kp = 0;
float_t Kd = 0;
float_t Ki = 0;

// Car Telemetry
uint8_t readGyroZData[2];
int16_t gyroZ;
float angleNow = 0;

// Gyro PID
int angle_err;
int16_t target_turn_angle;
int16_t angle;
int16_t angle_error;
int32_t angle_error_area = 0;
int32_t angle_error_old, angle_error_change;
float_t angle_error_rate;
int32_t angle_millisOld, angle_millisNow, angle_dt;

typedef struct _pidConfig {
	float Kp;
	float Ki;
	float Kd;
	float ek1;
	float ekSum;
} PIDConfig;

PIDConfig pidSlow;

int16_t PID_Control_turn(){
	if (abs(angle_error)>2){ //more than 2 degree difference

			angle = (int16_t) (angleNow);
			angle_error = target_turn_angle - angle;
			angle_error = angle_error;

			angle_millisNow = HAL_GetTick();
			angle_dt = (angle_millisNow - angle_millisOld); // time elapsed in millisecond
			angle_millisOld = angle_millisNow; // store the current time for next round

			angle_error_area = angle_error_area + abs(angle_error)*angle_dt; // area under error for Ki

			angle_error_change = abs(angle_error) - angle_error_old; // change in error
			angle_error_old = abs(angle_error); //store the error for next round
			angle_error_rate = angle_error_change/angle_dt; // for Kd

			l_pwm_val = (int)(abs(angle_error)*Kp + angle_error_area*Ki + angle_error_rate*Kd);  // PID
			l_pwm_val = l_pwm_val; // Boost for PWM to convert from angle
	  	  //pwmVal = 2000;   // overwrite PID above, minimum pwmVal = 1000

			if (l_pwm_val > pwmMax)  // Clamp the PWM to its maximum value
			   l_pwm_val = pwmMax;

			return(l_pwm_val);

		}
}

int16_t PID_Control_left(uint8_t isback){
	  //Control Loop Left
	  if (abs(l_error)>2){ //more than 2 degree difference

    	  l_angle = (int)(l_position*360/360);  // supposed to be 260 tick per revolution?
  	      l_error = target_angle - l_angle;


  	    if (isback) {
			Kp = 10;
			Ki = 0.001;
			Kd = 800;
		} else {
			Kp = 8;
			Ki = 0.0006;
			Kd = 800;
		}

  	    if (l_error > 0) {
			motor_forward();
		} else {
			motor_backward();
		}

        l_millisNow = HAL_GetTick();
        l_dt = (l_millisNow - l_millisOld); // time elapsed in millisecond
        l_millisOld = l_millisNow; // store the current time for next round

        l_error_area = l_error_area + abs(l_error)*l_dt; // area under error for Ki

        l_error_change = abs(l_error) - l_error_old; // change in error
  	    l_error_old = abs(l_error); //store the error for next round
        l_error_rate = l_error_change/l_dt; // for Kd

		l_pwm_val = (int)(abs(l_error)*Kp + l_error_area*Ki + l_error_rate*Kd);  // PID

  	  //pwmVal = 2000;   // overwrite PID above, minimum pwmVal = 1000

		if (isback) {
			if (l_pwm_val > pwmMax_back)  // Clamp the PWM to its maximum value
				   l_pwm_val = pwmMax_back;
		} else {
			if (l_pwm_val > pwmMax)  // Clamp the PWM to its maximum value
			   l_pwm_val = pwmMax;
		}

		return(l_pwm_val);

	}
}

int16_t PID_Control_right(uint8_t isback){
	  //Control Loop Left
	  if (abs(r_error)>2){ //more than 2 degree difference

		  r_angle = (int)(r_position*360/360);  // supposed to be 260 tick per revolution?
		  r_error = target_angle - r_angle;

		  if (isback) {
				Kp = 10;
				Ki = 0.001;
				Kd = 800;
			} else {
				Kp = 8;
				Ki = 0.0006;
				Kd = 800;
			}

        if (r_error > 0) {
        	motor_forward();
        } else {
        	motor_backward();
        }


        r_millisNow = HAL_GetTick();
        r_dt = (r_millisNow - r_millisOld); // time elapsed in millisecond
        r_millisOld = r_millisNow; // store the current time for next round

        r_error_area = r_error_area + abs(r_error)*r_dt; // area under error for Ki

        r_error_change = abs(r_error) - r_error_old; // change in error
  	    r_error_old = abs(r_error); //store the error for next round
        r_error_rate = r_error_change/r_dt; // for Kd

		r_pwm_val = (int)(abs(r_error)*Kp + r_error_area*Ki + r_error_rate*Kd);  // PID

  	  //pwmVal = 2000;   // overwrite PID above, minimum pwmVal = 1000

		if (isback) {
			if (l_pwm_val > pwmMax_back)  // Clamp the PWM to its maximum value
				   l_pwm_val = pwmMax_back;
		} else {
			if (l_pwm_val > pwmMax)  // Clamp the PWM to its maximum value
			   l_pwm_val = pwmMax;
		}

		return(r_pwm_val);

	}
}

void motor_init(TIM_HandleTypeDef* pwm, TIM_HandleTypeDef* l_enc, TIM_HandleTypeDef* r_enc, I2C_HandleTypeDef * hi2c) {
	// Assign timer pointers
	motor_pwm_tim = pwm;
	l_enc_tim = l_enc;
	r_enc_tim = r_enc;
	gyro_i2c = hi2c;

	// Start Encoders and PWM for L, R motors
	HAL_TIM_Encoder_Start_IT(l_enc, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start_IT(r_enc, TIM_CHANNEL_ALL);
	HAL_TIM_PWM_Start(pwm, L_CHANNEL);
	HAL_TIM_PWM_Start(pwm, R_CHANNEL);

	l_rpm = (int)((1000/no_of_tick) * 60/360);
	r_rpm = (int)((1000/no_of_tick) * 60/360);

	l_speed = 0, r_speed = 0;
	l_position = 0, r_position = 0;  // see SysTick_Handler in stm32f4xx_it.c
	l_oldpos = 0, r_oldpos = 0; // see SysTick_Handler in stm32f4xx_it.c
	l_angle = 0, r_angle = 0;
	l_pwm_val = 0, r_pwm_val = 0;

	start = 0;

	Kp = 8;       // 10
	Ki = 0.0004;   // 0.001
	Kd = 800;
	l_millisOld = HAL_GetTick();
	r_millisOld = HAL_GetTick();

	ICM20948_init(hi2c,0,GYRO_FULL_SCALE_2000DPS);
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
	float wheel_radius = 3.4;							// Wheel radius (cm)
    float circumference = 2 * 3.14159 * wheel_radius;	// Calculate circumference
    uint32_t pulses_per_rev = 1550;						// Encoder's specification: 11 ppr * 30 = 330 (30x reducer)
    float pulses_per_cm = (1.0 * pulses_per_rev) / circumference;
    uint32_t target_pulses = (uint32_t)(distance * pulses_per_cm);
    uint32_t encoder_cnt = 65535;

    servo_set_dir(STRAIGHT);
    HAL_Delay(500);

    OLED_Clear();
//    print_OLED(0, 0, "target_pulse:", false, 0);
//    print_OLED(0, 15, "%u", true, target_pulses);
//    print_OLED(0, 30, "val: ", false, 0);

    // Reset the encoder and initialize encoder count to 65535 after resetting
    reset_encoders();
    motor_forward();
    HAL_Delay(50);

    while (65535 - encoder_cnt < target_pulses) {
		encoder_cnt = __HAL_TIM_GET_COUNTER(l_enc_tim);
//		print_OLED(0, 45, "%u", true, 65535 - encoder_cnt);
    }

    // Stop the motors when the target distance is reached
    motor_stop();
    //HAL_Delay(300);
}

void forward_pid(uint32_t distance) {
	r_millisOld = HAL_GetTick();
	l_millisOld = HAL_GetTick();

	float wheel_radius = 3;							// Wheel radius (cm)
    float circumference = 2 * 3.14159 * wheel_radius;	// Calculate circumference

	target_angle = (distance * 360) / circumference;

	l_rpm = (int)((1000/no_of_tick) * 60/360);
	r_rpm = (int)((1000/no_of_tick) * 60/360);

	l_speed = 0, r_speed = 0;
	l_position = 0, r_position = 0;  // see SysTick_Handler in stm32f4xx_it.c
	l_oldpos = 0, r_oldpos = 0; // see SysTick_Handler in stm32f4xx_it.c
	l_angle = 0, r_angle = 0;
	l_pwm_val = 0, r_pwm_val = 0;

	l_err = 0, r_err = 0;// for checking whether error has settle down near to zero
	l_error_old = 0, r_error_old = 0;
	l_error_area = 0, r_error_area = 0;

	l_error = target_angle - l_angle;
	r_error = target_angle - r_angle;

    servo_set_dir(STRAIGHT);
    HAL_Delay(400);

    // Reset encoder count
    reset_encoders();
    	// Initialize to 65535 after resetting the encoder
    uint32_t l_encoder_cnt = 0;
	uint32_t r_encoder_cnt = 0;
    // Move forward
    motor_forward();
    OLED_Clear();
	print_OLED(0, 0, "ta: %ld", true, target_angle);

	start = 1;

    while (start) {

    	l_encoder_cnt = __HAL_TIM_GET_COUNTER(l_enc_tim);
    	r_encoder_cnt = __HAL_TIM_GET_COUNTER(r_enc_tim);

		r_count = (int16_t)r_encoder_cnt;
		r_position = r_count/4;  //x1 Encoding
		r_angle = r_count/2;

		l_encoder_cnt = 65535 - l_encoder_cnt;
		l_count = (int16_t)l_encoder_cnt; // 1550 - 360deg
		l_position = l_count/4;  //x1 Encoding
		l_angle = l_count/2;

		l_pwm_val = PID_Control_left(0);
		r_pwm_val = PID_Control_right(0);

		print_OLED(0, 15, "l: %ld", true, l_angle);
		print_OLED(0, 30, "r: %ld", true, r_angle);

		__HAL_TIM_SET_COMPARE(motor_pwm_tim, R_CHANNEL, r_pwm_val);
		__HAL_TIM_SET_COMPARE(motor_pwm_tim, L_CHANNEL, l_pwm_val);

		if (abs(l_error) < 15){ // error is less than 3 deg
//	      l_err++; // to keep track how long it has reached steady state
//	      l_angle = (int)(l_position*360/1550);  //calculate the angle
//	      l_error = target_angle - l_angle; // calculate the error
	      l_pwm_val = 0; //stop
	      __HAL_TIM_SET_COMPARE(motor_pwm_tim, L_CHANNEL, l_pwm_val);
		}

		if (abs(r_error) < 15){ // error is less than 3 deg
//	      r_err++; // to keep track how long it has reached steady state
//	      r_angle = (int)(r_position*360/1550);  //calculate the angle
//	      r_error = target_angle - r_angle; // calculate the error
			r_pwm_val = 0; //stop
			__HAL_TIM_SET_COMPARE(motor_pwm_tim, R_CHANNEL, r_pwm_val);
		}

//		if (l_err > 5) { // error has settled to within the acceptance ranges
//	   	 	l_pwm_val = 0; //stop
//   	     	__HAL_TIM_SET_COMPARE(motor_pwm_tim, L_CHANNEL, l_pwm_val);
//		}
//
//		if (r_err > 5) { // error has settled to within the acceptance ranges
//	   	 	r_pwm_val = 0; //stop
//   	     	__HAL_TIM_SET_COMPARE(motor_pwm_tim, R_CHANNEL, r_pwm_val);
//		}

		if (r_pwm_val == 0 && l_pwm_val == 0) {
			start = 0;
		}



        // sprintf(buf, "%u", l_encoder_cnt);
        // OLED_ShowString(0, 30, buf);
		// OLED_Refresh_Gram();
    }

    // Stop the motors when the target distance is reached
    motor_stop();
    servo_set_dir(STRAIGHT);
}


void backward(uint32_t distance) {
	l_millisOld = HAL_GetTick();
	r_millisOld = HAL_GetTick();

	static bool has_run = false;  // Flag to check if the function has already run
    if (has_run) return;  // Exit if the function has already been executed once
	has_run = true;  // Set the flag to true to prevent future runs

    float wheel_radius = 3.4;  // Wheel radius (cm)
    float circumference = 2 * 3.14159 * wheel_radius;  // Calculate circumference
    uint32_t pulses_per_rev = 1550;  // Encoder's specification
    float pulses_per_cm = pulses_per_rev / circumference;
    uint32_t target_pulses = (uint32_t)(distance * pulses_per_cm);
    
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
    servo_set_dir(STRAIGHT);
    has_run = false;
}

void backward_pid(uint32_t distance) {
	r_millisOld = HAL_GetTick();
	l_millisOld = HAL_GetTick();

	float wheel_radius = 3;							// Wheel radius (cm)
    float circumference = 2 * 3.14159 * wheel_radius;	// Calculate circumference

	target_angle = ((distance * 360) / circumference);
	target_angle = -target_angle;

	l_rpm = (int)((1000/no_of_tick) * 60/360);
	r_rpm = (int)((1000/no_of_tick) * 60/360);

	l_speed = 0, r_speed = 0;
	l_position = 0, r_position = 0;  // see SysTick_Handler in stm32f4xx_it.c
	l_oldpos = 0, r_oldpos = 0; // see SysTick_Handler in stm32f4xx_it.c
	l_angle = 0, r_angle = 0;
	l_pwm_val = 0, r_pwm_val = 0;

	l_err = 0, r_err = 0;// for checking whether error has settle down near to zero
	l_error_old = 0, r_error_old = 0;
	l_error_area = 0, r_error_area = 0;

	l_error = target_angle - l_angle;
	r_error = target_angle - r_angle;

    servo_set_dir(STRAIGHT);
    HAL_Delay(400);
    // Reset encoder count
    reset_encoders();
    	// Initialize to 65535 after resetting the encoder
    uint32_t l_encoder_cnt = 0;
	uint32_t r_encoder_cnt = 0;
    // Move forward
    motor_backward();
    OLED_Clear();
	print_OLED(0, 0, "ta: %d", true, target_angle);

	start = 1;

    while (start) {

    	l_encoder_cnt = __HAL_TIM_GET_COUNTER(l_enc_tim);
    	r_encoder_cnt = __HAL_TIM_GET_COUNTER(r_enc_tim);

		r_count = (int16_t)r_encoder_cnt;
		r_position = r_count/4;  //x1 Encoding
		r_angle = r_count/2;

		l_encoder_cnt = 65535 -  l_encoder_cnt;
		l_count = (int16_t)l_encoder_cnt; // 1550 - 360deg
		l_position = l_count/4;  //x1 Encoding
		l_angle = l_count/2;


		l_pwm_val = PID_Control_left(1);
		r_pwm_val = PID_Control_right(1);

		__HAL_TIM_SET_COMPARE(motor_pwm_tim, R_CHANNEL, r_pwm_val);
		__HAL_TIM_SET_COMPARE(motor_pwm_tim, L_CHANNEL, l_pwm_val);

		if (abs(l_error) < 15){ // error is less than 3 deg
	      l_pwm_val = 0; //stop
	      __HAL_TIM_SET_COMPARE(motor_pwm_tim, L_CHANNEL, l_pwm_val);
		}

		if (abs(r_error) < 15){ // error is less than 3 deg
			r_pwm_val = 0; //stop
			__HAL_TIM_SET_COMPARE(motor_pwm_tim, R_CHANNEL, r_pwm_val);
		}

		if (r_pwm_val == 0 && l_pwm_val == 0) {
			start = 0;
		}

		print_OLED(0, 15, "l: %ld", true, l_angle);
		print_OLED(0, 30, "r: %ld", true, r_angle);
    }

    // Stop the motors when the target distance is reached
    motor_stop();
    servo_set_dir(STRAIGHT);
}

void forward_right() {
    static bool has_run = false;  // Flag to check if the function has already run
    if (has_run) return;  // Exit if the function has already been executed once
    has_run = true;  // Set the flag to true to prevent future runs

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
    servo_set_dir(STRAIGHT);
    has_run = false;
}

void forward_right_pid() {
    static bool has_run = false;  // Flag to check if the function has already run
    if (has_run) return;  // Exit if the function has already been executed once
    has_run = true;  // Set the flag to true to prevent future runs

    servo_set_dir(RIGHT);
    HAL_Delay(400);  // Reduced delay after setting servo

    // Move forward
    reset_encoders();
    motor_forward();
    start = 0;
    int16_t target_turn_angle = -90;

	angle = 0;
	l_pwm_val = 0, r_pwm_val = 0;

	angle_err = 0;
	angle_error_old = 0;
	angle_error_area = 0;

//	angle_error = target_turn_angle - angle;

	angleNow = 0;
	start = 1;

	l_pwm_val = 1350;
	__HAL_TIM_SET_COMPARE(motor_pwm_tim, R_CHANNEL, l_pwm_val);
	__HAL_TIM_SET_COMPARE(motor_pwm_tim, L_CHANNEL, l_pwm_val);

	uint32_t last_curTask_tick = HAL_GetTick();



    // Loop until the desired arc length is reached
    while (start) {
    	__Gyro_Read_Z(gyro_i2c, readGyroZData, gyroZ); // polling
		angleNow += gyroZ / GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.2;

		 angle = (int16_t) (angleNow);
		 angle_error = target_turn_angle - angle;

//		 int delta = (int) (angleNow - target_turn_angle);
		 print_OLED(0, 0, "Angle: %d", true, angle);
		 print_OLED(0, 15, "pwm: %ld", true, l_pwm_val);
//		r_pwm_val = PID_Control_right(0);

		if (abs(angle_error) < 6){ // error is less than 3 deg
		  l_pwm_val = 0; //stop
		  r_pwm_val = 0; //stop
		  __HAL_TIM_SET_COMPARE(motor_pwm_tim, L_CHANNEL, l_pwm_val);
		  __HAL_TIM_SET_COMPARE(motor_pwm_tim, R_CHANNEL, r_pwm_val);
		  start = 0;
		  break;
		}
    }

    // Stop the motors when the target distance is reached
    motor_stop();  // Stop moving
    servo_set_dir(STRAIGHT);
    has_run = false;
}

void forward_left() {
	static bool has_run = false;  // Flag to check if the function has already run
	if (has_run) return;  // Exit if the function has already been executed once
	has_run = true;  // Set the flag to true to prevent future runs
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

    while (65535-encoder_cnt < arc_length) {
    	encoder_cnt = __HAL_TIM_GET_COUNTER(l_enc_tim);  // Update encoder count
    	sprintf(buf, "%u", encoder_cnt);  // Format encoder count for display
    	OLED_ShowString(0, 30, buf);  // Display current encoder count
    	OLED_Refresh_Gram();  // Refresh OLED display
    }

    // Stop the motors when the target distance is reached
    motor_stop();
    servo_set_dir(STRAIGHT);
    has_run = false;
    //HAL_Delay(500);
}


void forward_left_pid() {
	servo_set_dir(LEFT);
    HAL_Delay(400);  // Reduced delay after setting servo

    // Move forward
    reset_encoders();
    motor_forward();
    start = 0;
    int16_t target_turn_angle = 90;

	angle = 0;
	l_pwm_val = 0, r_pwm_val = 0;

	angle_err = 0;
	angle_error_old = 0;
	angle_error_area = 0;

//	angle_error = target_turn_angle - angle;

	angleNow = 0;
	start = 1;

	l_pwm_val = 1350;
	__HAL_TIM_SET_COMPARE(motor_pwm_tim, R_CHANNEL, l_pwm_val);
	__HAL_TIM_SET_COMPARE(motor_pwm_tim, L_CHANNEL, l_pwm_val);

	uint32_t last_curTask_tick = HAL_GetTick();



    // Loop until the desired arc length is reached
    while (start) {
    	__Gyro_Read_Z(gyro_i2c, readGyroZData, gyroZ); // polling
		angleNow += gyroZ / GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.2;

		 angle = (int16_t) (angleNow);
		 angle_error = target_turn_angle - angle;

//		 int delta = (int) (angleNow - target_turn_angle);
		 print_OLED(0, 0, "Angle: %d", true, angle);
		 print_OLED(0, 15, "pwm: %ld", true, l_pwm_val);
//		r_pwm_val = PID_Control_right(0);

		if (abs(angle_error) < 6){ // error is less than 3 deg
		  l_pwm_val = 0; //stop
		  r_pwm_val = 0; //stop
		  __HAL_TIM_SET_COMPARE(motor_pwm_tim, L_CHANNEL, l_pwm_val);
		  __HAL_TIM_SET_COMPARE(motor_pwm_tim, R_CHANNEL, r_pwm_val);
		  start = 0;
		  break;
		}
    }

    // Stop the motors when the target distance is reached
    motor_stop();  // Stop moving
    servo_set_dir(STRAIGHT);
}

void backward_move() {
	servo_set_dir(STRAIGHT);
	backward(40);
}

void backward_right() {
	// Servo direction: RIGHT
	servo_set_dir(RIGHT);
	backward(80);
}

void backward_right_pid() {
    servo_set_dir(RIGHT);
    HAL_Delay(400);  // Reduced delay after setting servo

    // Move forward
    reset_encoders();
    motor_backward();
    start = 0;
    int16_t target_turn_angle = 90;

	angle = 0;
	l_pwm_val = 0, r_pwm_val = 0;

	angle_err = 0;
	angle_error_old = 0;
	angle_error_area = 0;

//	angle_error = target_turn_angle - angle;

	angleNow = 0;
	start = 1;

	l_pwm_val = 1350;
	__HAL_TIM_SET_COMPARE(motor_pwm_tim, R_CHANNEL, l_pwm_val);
	__HAL_TIM_SET_COMPARE(motor_pwm_tim, L_CHANNEL, l_pwm_val);

	uint32_t last_curTask_tick = HAL_GetTick();

    // Loop until the desired arc length is reached
    while (start) {
    	__Gyro_Read_Z(gyro_i2c, readGyroZData, gyroZ); // polling
		angleNow += gyroZ / GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.2;

		 angle = (int16_t) (angleNow);
		 angle_error = target_turn_angle - angle;

//		 int delta = (int) (angleNow - target_turn_angle);
		 print_OLED(0, 0, "Angle: %d", true, angle);
		 print_OLED(0, 15, "pwm: %ld", true, l_pwm_val);
//		r_pwm_val = PID_Control_right(0);

		if (abs(angle_error) < 6){ // error is less than 3 deg
		  l_pwm_val = 0; //stop
		  r_pwm_val = 0; //stop
		  __HAL_TIM_SET_COMPARE(motor_pwm_tim, L_CHANNEL, l_pwm_val);
		  __HAL_TIM_SET_COMPARE(motor_pwm_tim, R_CHANNEL, r_pwm_val);
		  start = 0;
		  break;
		}
    }

    // Stop the motors when the target distance is reached
    motor_stop();  // Stop moving
    servo_set_dir(STRAIGHT);
}

void backward_left() {
	// Servo direction: LEFT
	servo_set_dir(LEFT);
	backward(80);
}

void backward_left_pid() {
    servo_set_dir(LEFT);
    HAL_Delay(400);  // Reduced delay after setting servo

    // Move forward
    reset_encoders();
    motor_backward();
    start = 0;
    int16_t target_turn_angle = -90;

	angle = 0;
	l_pwm_val = 0, r_pwm_val = 0;

	angle_err = 0;
	angle_error_old = 0;
	angle_error_area = 0;

//	angle_error = target_turn_angle - angle;

	angleNow = 0;
	start = 1;

	l_pwm_val = 1350;
	__HAL_TIM_SET_COMPARE(motor_pwm_tim, R_CHANNEL, l_pwm_val);
	__HAL_TIM_SET_COMPARE(motor_pwm_tim, L_CHANNEL, l_pwm_val);

	uint32_t last_curTask_tick = HAL_GetTick();

    // Loop until the desired arc length is reached
    while (start) {
    	__Gyro_Read_Z(gyro_i2c, readGyroZData, gyroZ); // polling
		angleNow += gyroZ / GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.2;

		 angle = (int16_t) (angleNow);
		 angle_error = target_turn_angle - angle;

//		 int delta = (int) (angleNow - target_turn_angle);
		 print_OLED(0, 0, "Angle: %d", true, angle);
		 print_OLED(0, 15, "pwm: %ld", true, l_pwm_val);
//		r_pwm_val = PID_Control_right(0);

		if (abs(angle_error) < 6){ // error is less than 3 deg
		  l_pwm_val = 0; //stop
		  r_pwm_val = 0; //stop
		  __HAL_TIM_SET_COMPARE(motor_pwm_tim, L_CHANNEL, l_pwm_val);
		  __HAL_TIM_SET_COMPARE(motor_pwm_tim, R_CHANNEL, r_pwm_val);
		  start = 0;
		  break;
		}
    }

    // Stop the motors when the target distance is reached
    motor_stop();  // Stop moving
    servo_set_dir(STRAIGHT);
}
