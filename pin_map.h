#pragma once // to avoid including the same header file many times

//  GPIOs:
#define LED_PIN 25 //buil-in LED
#define PWM_SERVO 0 

#define PWM_M1 2 //D6
#define PWM_M2 3 //D11
#define AIN1_DIR_M1 8 //D7, CW
#define AIN2_DIR_M1 9 //D8, CCW
#define BIN1_DIR_M2 4 //D9, CW
#define BIN2_DIR_M2 5 //D10, CCW

#define ENCODER_A_M1 10
#define ENCODER_B_M1 12
#define ENCODER_A_M2 11
#define ENCODER_B_M2 13

#define TOOL 6 

#define LIMIT_SWITCH_0_L1 18 //Start, link 1
#define LIMIT_SWITCH_1_L1 19 //End, link 1
#define LIMIT_SWITCH_0_L2 20 //Start, link 2
#define LIMIT_SWITCH_1_L2 21 //End, link 2

// Manual-jog test tuning
#define MANUAL_JOG_DUTY 3000   // PWM duty for M1/M2 
#define SERVO_JOG_STEP_DEG 10  // Degrees per J3_UP/J3_DOWN command
