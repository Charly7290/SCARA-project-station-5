#include "system_lib.h"
#include "prismatic.h"
#include "revolute.h"
// Micro-ROS core and RCL (ROS Client Library) includes
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
// Message type from standard ROS 2 message definitions
#include <std_msgs/msg/int32.h>
// Custom transport over UART (for Pico platform)
#include <rmw_microros/rmw_microros.h>
#include "pico_uart_transports.h"

int main(void){
    //  STATE INIT:
    stdio_init_all(); //Initialize UART
    init_servo();
    init_tool();
    init_limitS(LIMIT_SWITCH_0_L1);
    init_limitS(LIMIT_SWITCH_1_L1);
    init_limitS(LIMIT_SWITCH_0_L2);
    init_limitS(LIMIT_SWITCH_1_L2);
    init_encoder(ENCODER_A_M1, ENCODER_B_M1);
    init_encoder(ENCODER_A_M2, ENCODER_B_M2);
    init_motor(AIN1_DIR_M1, AIN2_DIR_M1, PWM_M1);
    init_motor(BIN1_DIR_M2, BIN2_DIR_M2, PWM_M2);
    

    while(1){


        sleep_ms(20);
    }

    return 0; //success
}