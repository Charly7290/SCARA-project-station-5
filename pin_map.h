#pragma once //para evitar que un archivo header sea incluido varias veces

//  GPIOs:
#define PWM_SERVO 0 

#define PWM_M1 2 
#define PWM_M2 3 
#define AIN1_DIR_M1 8 
#define AIN2_DIR_M1 9 
#define BIN1_DIR_M2 4 
#define BIN2_DIR_M2 5 

#define ENCODER_A_M1 10
#define ENCODER_B_M1 12
#define ENCODER_A_M2 11
#define ENCODER_B_M2 13

#define TOOL 6 

#define LIMIT_SWITCH_0_L1 18 //Inicio, link 1
#define LIMIT_SWITCH_1_L1 19 //Fin, link 1
#define LIMIT_SWITCH_0_L2 20 //Inicio, link 2
#define LIMIT_SWITCH_1_L2 21 //Fin, link 2