#include "system_lib.h"
#include "prismatic.h"
#include "revolute.h"

// Micro-ROS core and RCL (ROS Client Library) includes
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
// Custom transport over UART (for Pico platform)
#include <rmw_microros/rmw_microros.h>
#include "pico_uart_transports.h"

// Declare a global subscriber instance
rcl_subscription_t subscriber;
//A string message for States:
std_msgs__msg__String msg;
char buffer[64];

typedef enum { //States
    STATE_INIT,
    STATE_IDLE,
    STATE_MANUAL,
    STATE_HOMING,
    STATE_TEST_IK,
    STATE_PICK_CASE,
    STATE_PLACE_CASE,
    STATE_PICK_PCB,
    STATE_INSERT_PCB,
    STATE_ALARM
} scara_state_t;
volatile scara_state_t current_state = STATE_INIT;

void cmd_callback(const void *msgin) {
    const std_msgs__msg__String *m = (const std_msgs__msg__String *)msgin;
    const char *cmd = m->data.data;

    if (strcmp(cmd, "HOME") == 0) {
        // Stop any in-progress manual jog before handing control to the homing routine
        move_motor(PWM_M1, 0);
        move_motor(PWM_M2, 0);
        current_state = STATE_HOMING;
    } else if (strcmp(cmd, "MANUAL") == 0) {
        current_state = STATE_MANUAL;
    } else if (strcmp(cmd, "TEST_IK") == 0) {
        move_motor(PWM_M1, 0);
        move_motor(PWM_M2, 0);
        current_state = STATE_TEST_IK;
    // Manual jog commands (only honored while in STATE_MANUAL) 
    } else if (strcmp(cmd, "J1_CW") == 0) {
        if (current_state == STATE_MANUAL) { CW_M1; move_motor(PWM_M1, MANUAL_JOG_DUTY_M1); }
        else printf("Ignored %s: send MANUAL first.\n", cmd);
    } else if (strcmp(cmd, "J1_CCW") == 0) {
        if (current_state == STATE_MANUAL) { CCW_M1; move_motor(PWM_M1, MANUAL_JOG_DUTY_M1); }
        else printf("Ignored %s: send MANUAL first.\n", cmd);
    } else if (strcmp(cmd, "J1_STOP") == 0) {
        if (current_state == STATE_MANUAL) { move_motor(PWM_M1, 0); }
        else printf("Ignored %s: send MANUAL first.\n", cmd);
    } else if (strcmp(cmd, "J2_CW") == 0) {
        if (current_state == STATE_MANUAL) { CW_M2; move_motor(PWM_M2, MANUAL_JOG_DUTY_M2); }
        else printf("Ignored %s: send MANUAL first.\n", cmd);
    } else if (strcmp(cmd, "J2_CCW") == 0) {
        if (current_state == STATE_MANUAL) { CCW_M2; move_motor(PWM_M2, MANUAL_JOG_DUTY_M2); }
        else printf("Ignored %s: send MANUAL first.\n", cmd);
    } else if (strcmp(cmd, "J2_STOP") == 0) {
        if (current_state == STATE_MANUAL) { move_motor(PWM_M2, 0); }
        else printf("Ignored %s: send MANUAL first.\n", cmd);
    } else if (strcmp(cmd, "J3_UP") == 0) {
        if (current_state == STATE_MANUAL) { servo_jog_up(); }
        else printf("Ignored %s: send MANUAL first.\n", cmd);
    } else if (strcmp(cmd, "J3_DOWN") == 0) {
        if (current_state == STATE_MANUAL) { servo_jog_down(); }
        else printf("Ignored %s: send MANUAL first.\n", cmd);
    } else {
        printf("Unknown state: %s\n", cmd);
    }
}

int main(void){
    //  STATE INIT:
    //Setup custom transport for Micro-ROS using UART:
    rmw_uros_set_custom_transport(
		true,                           // Set to true to use custom transport
		NULL,                           // Optional user data (unused here)
		pico_serial_transport_open,     // Open function (UART init)
		pico_serial_transport_close,    // Close function
		pico_serial_transport_write,    // Write function (send data)
		pico_serial_transport_read      // Read function (receive data)
	);
    stdio_init_all(); //Initialize UART

    // Declare Micro-ROS core entities
    rcl_node_t node;            // ROS 2 node (e.g., "pico_node")
    rcl_allocator_t allocator;  // Memory allocator for Micro-ROS
    rclc_support_t support;     // Wrapper that bundles allocator/context/init

    // Use the default allocator (standard memory management)
    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000;
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
    }

    // Initialize Micro-ROS support structure (includes context and options)
    rclc_support_init(&support, 0, NULL, &allocator);

    // Initialize a Micro-ROS node with default options
    rclc_node_init_default(&node, "pico_node", "", &support);

    msg.data.data = buffer;
    msg.data.capacity = sizeof(buffer);
    msg.data.size = 0;

    rclc_subscription_init_default(
        &subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "scara_cmd"); //ROS 2 subcriber name

    rclc_executor_t executor;
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &msg, &cmd_callback, ON_NEW_DATA);

    init_servo();
    init_tool();
    init_limitS(LIMIT_SWITCH_0_L1);
    init_limitS(LIMIT_SWITCH_1_L1);
    init_limitS(LIMIT_SWITCH_0_L2);
    init_limitS(LIMIT_SWITCH_1_L2);
    init_encoder_M1();
    init_encoder_M2();
    init_motor(AIN1_DIR_M1, AIN2_DIR_M1, PWM_M1);
    init_motor(BIN1_DIR_M2, BIN2_DIR_M2, PWM_M2);
    move_motor(PWM_M1, 0);
    move_motor(PWM_M2, 0);
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    bool l1_calib, l1_homed, l2_calib, l2_homed, m1_moved, m2_moved;
    uint64_t last_time = time_us_64();
    scara_state_t last_reported_state = STATE_INIT;
    float encoder_degrees_M1, encoder_degrees_M2;
    //Due to the relationship between the pulleys: consider s=rθ
    const float pulley_ratio_M1 = 60.0f/28.0f; // based on the timing pulley teeth
    const float pulley_ratio_M2 = 60.0f/20.0f; // based on the timing pulley teeth 
    const float target_M2_home = 130.0f * pulley_ratio_M2, target_M1_home = 100.0f * pulley_ratio_M1; 
    float angleM1, angleM2;

    current_state = STATE_IDLE;
    while(1){
        uint64_t current_time = time_us_64();
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)); //each call waits up to 100ms for a new message from publisher
        if(current_state != last_reported_state){ //To run anything once
            switch(current_state){
                case STATE_IDLE:
                    for (int i = 0; i < 4; i++){
                        gpio_put(LED_PIN, 1);
                        sleep_ms(125);
                        gpio_put(LED_PIN, 0);
                        sleep_ms(125);
                    }
                    m1_moved = false;
                    m2_moved = false;
                    TOOL_OFF;
                    //waiting for a message to change state...
                    break;
                case STATE_HOMING:
                    TOOL_OFF;
                    CCW_M1;
                    CW_M2;
                    move_motor(PWM_M1, 0);
                    move_motor(PWM_M2, 625); //5% of duty cycle
                    servo_home();
                    l1_calib = false;
                    l1_homed = false;
                    l2_calib = false;
                    l2_homed = false;
                    m1_moved = false;
                    m2_moved = false;
                    break;
                case STATE_MANUAL:
                    TOOL_OFF;

                    break;
                case STATE_TEST_IK:
                    th anglesM= calculateIK(-10.0f, -20.0f);
                    angleM1 = anglesM.theta1;
                    angleM2 = anglesM.theta2;
                    if(angleM1 == 180.0f && angleM2 == 180.0f){ //Out of joint limitations
                        move_motor(PWM_M1,0);
                        move_motor(PWM_M2,0);
                        current_state = STATE_IDLE;
                    } else {
                        move_motor(PWM_M1, 1000);
                        move_motor(PWM_M2, 625);
                    }
                    set_servo_angle(180);
                    break;
                case STATE_PICK_CASE:
                    
                    
                    break;
                case STATE_PLACE_CASE:


                    break;
                case STATE_PICK_PCB:


                    break;
                case STATE_INSERT_PCB:


                    break;
                default: //just in case
                    break;
            }
            last_reported_state = current_state;
        }
        switch(current_state){ //TO run continuously
            case STATE_MANUAL:
                // Jog commands (J1_CW/J1_CCW/J1_STOP, J2_*, J3_UP/J3_DOWN) are handled directly in cmd_callback() as they arrive.
                //current_state = STATE_IDLE;
                break;
            case STATE_HOMING:
                //Calibration: go to starting position set as 0 by the limit switches
                if (SWITCH_0_L2_ON && l2_calib == false){
                    move_motor(PWM_M2, 0);
                    move_motor(PWM_M1, 1000); //8% of duty cycle
                    l2_calib = true;
                    encoder_count[1] = 0; //Reset value of motor 2's encoder 
                }
                if (SWITCH_0_L1_ON && l1_calib == false && l2_calib == true){
                    move_motor(PWM_M1, 0);
                    l1_calib = true;
                    encoder_count[0] = 0; //Reset value of motor 1's encoder 
                }
                //After reaching this calibration position, start homing:
                if (l1_calib && l2_calib){
                    if(l2_homed == false){
                        if (m2_moved == false){
                            CCW_M2;
                            move_motor(PWM_M2, 625);
                            m2_moved = true;
                        }
                        encoder_degrees_M2 = encoder_count[1] * 360.0f / 3200.0f; // Since it's a Pololu DC motor 50:1, 64 ticks --> 3200 ticks
                        if (fabsf(encoder_degrees_M2) >= target_M2_home){
                            move_motor(PWM_M2, 0);
                            l2_homed = true;
                        }
                    }
                    if (l2_homed == true && l1_homed == false){
                        if (m1_moved == false){
                            CW_M1;
                            move_motor(PWM_M1, 1000); 
                            m1_moved = true;
                        }
                        encoder_degrees_M1 = encoder_count[0] * 360.0f / 3200.0f; // Since it's a Pololu DC motor 50:1, 64 ticks --> 3200 ticks
                        if (fabsf(encoder_degrees_M1) >= target_M1_home){
                            move_motor(PWM_M1, 0);
                            l1_homed = true;
                        }
                    }
                }
                if (l1_homed && l2_homed){
                    encoder_count[0] = 0; //Home will be the new origin
                    encoder_count[1] = 0; //Home will be the new origin
                    current_state = STATE_IDLE;
                }
                break;
            case STATE_TEST_IK:
                encoder_degrees_M1 = encoder_count[0] * 360.0f / 3200.0f; // Since it's a Pololu DC motor 50:1, 64 ticks --> 3200 ticks
                if (m1_moved == false && fabsf(encoder_degrees_M1) >= (angleM1 * pulley_ratio_M1)){
                    move_motor(PWM_M1, 0);
                    m1_moved = true;
                }
                encoder_degrees_M2 = encoder_count[1] * 360.0f / 3200.0f; // Since it's a Pololu DC motor 50:1, 64 ticks --> 3200 ticks
                if (m2_moved == false && fabsf(encoder_degrees_M2) >= (angleM2 * pulley_ratio_M2)){
                    move_motor(PWM_M2, 0);
                    m2_moved = true;
                }
                if(m1_moved && m2_moved){
                    current_state = STATE_IDLE;
                }
                break;
            case STATE_PICK_CASE:
                //current_state = STATE_PLACE_CASE;
                
                break;
            case STATE_PLACE_CASE:

                //current_state = STATE_PICK_PCB;
                break;
            case STATE_PICK_PCB:

                //current_state = STATE_INSERT_PCB;
                break;
            case STATE_INSERT_PCB:

                //current_state = STATE_HOMING;
                break;
            default:
                break;
        }
        sleep_ms(20);
    }

    return 0; //success
}
