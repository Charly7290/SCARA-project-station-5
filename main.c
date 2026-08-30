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
    STATE_PICK_CASE,
    STATE_PLACE_CASE,
    STATE_PICK_PCB,
    STATE_INSERT_PCB,
    STATE_ALARM
} scara_state_t;
volatile scara_state_t current_state = STATE_INIT;

void cmd_callback(const void *msgin) {
    const std_msgs__msg__String *m = (const std_msgs__msg__String *)msgin;
    if (strcmp(m->data.data, "HOME") == 0) {
        current_state = STATE_HOMING;
    } else if (strcmp(m->data.data, "MANUAL") == 0) {
        current_state = STATE_MANUAL;
    } else if (strcmp(m->data.data, "PROCESS") == 0) {
        current_state = STATE_PICK_CASE;
    } else {
        printf("Unknown state: %s\n", m->data.data);
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
    init_encoder(ENCODER_A_M1, ENCODER_B_M1);
    init_encoder(ENCODER_A_M2, ENCODER_B_M2);
    init_motor(AIN1_DIR_M1, AIN2_DIR_M1, PWM_M1);
    init_motor(BIN1_DIR_M2, BIN2_DIR_M2, PWM_M2);
    move_motor(PWM_M1, 0);
    move_motor(PWM_M2, 0);
    
    bool l1_homed, l2_homed;
    scara_state_t last_reported_state = -1;

    current_state = STATE_IDLE;
    while(1){
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)); //each call waits up to 100ms for a new message from publisher
        if(current_state != last_reported_state){ //To run anything once
            switch(current_state){
                case STATE_IDLE:
                    TOOL_OFF;
                    //waiting for a message to change state...
                    break;
                case STATE_HOMING:
                    TOOL_OFF;
                    CCW_M1;
                    CW_M2;
                    servo_home();
                    move_motor(PWM_M1, 6249); //50% of duty cycle
                    l1_homed = false;
                    l2_homed = false;
                    break;
                case STATE_MANUAL:
                    TOOL_OFF;

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


                //current_state = STATE_IDLE;
                break;
            case STATE_HOMING:
                if (SWITCH_0_L1_ON && l1_homed == false){
                    move_motor(PWM_M1, 0);
                    move_motor(PWM_M2, 6249);
                    l1_homed = true;
                }
                if (SWITCH_0_L2_ON && l2_homed == false){
                    move_motor(PWM_M2, 0);
                    l2_homed = true;
                }
                if (l1_homed && l2_homed){
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