#include "system_lib.h"
#include "prismatic.h"
#include "revolute.h"
#include <stdlib.h> //atoi()

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

//Create a publisher for the telemetry IoT system (ROS2 --> MQTT --> server.js --> MariaDB --> Grafana) 
rcl_publisher_t telemetry_pub;
std_msgs__msg__String telemetry_msg;
char telemetry_buf[128];

float coord_x = 0.0f;
float coord_y = 0.0f;
float coord_z = 0.0f;
bool robot_homed = false;
bool zero = false;

typedef enum { //States
    STATE_INIT,
    STATE_IDLE,
    STATE_MANUAL,
    STATE_HOMING,
    STATE_TEST_IK,
    STATE_PICK_CASE,
    STATE_PLACE_CASE,
    STATE_PICK_PCB,
    STATE_INSERT_PCB
} scara_state_t;
volatile scara_state_t current_state = STATE_INIT;

//Dead-man's switch: time (us) of the last jog command in MANUAL
static volatile uint64_t last_manual_cmd_time = 0;

//Smooth J3 (J3_SPEED): servo rate in deg/s, positive = Z down
static volatile float j3_rate_dps = 0.0f;
static uint64_t j3_last_us = 0;

//  LIMIT SWITCH PROTECTION (MANUAL):
//Moving towards a pressed switch is blocked until the operator releases
//Limit switch reached in each direction:
#define M1_CW_LIMIT_HIT   SWITCH_1_L1_ON   //End, link 1
#define M1_CCW_LIMIT_HIT  SWITCH_0_L1_ON   //Start, link 1
#define M2_CW_LIMIT_HIT   SWITCH_0_L2_ON   //Start, link 2
#define M2_CCW_LIMIT_HIT  SWITCH_1_L2_ON   //End, link 2

#define DIR_STOP 0
#define DIR_CW   1
#define DIR_CCW  (-1)

static volatile int  jog_dir[2]       = {DIR_STOP, DIR_STOP}; //[0] = M1, [1] = M2
static volatile bool jog_block_cw[2]  = {false, false};       //CW blocked until released
static volatile bool jog_block_ccw[2] = {false, false};       //CCW blocked until released
bool tool, m1_moved, m2_moved; 
uint64_t goto_start_time = 0;

const float encoder_sign_M1 = -1.0f;
const float encoder_sign_M2 = -1.0f;
//Due to the relationship between the pulleys: consider s=rθ
const float pulley_ratio_M1 = 60.0f/28.0f; // based on the timing pulley teeth and tests made
const float pulley_ratio_M2 = 60.0f/20.0f; // based on the timing pulley teeth 
const float target_M2_home = 130.0f * pulley_ratio_M2, target_M1_home = 106.5f * pulley_ratio_M1; 
bool reach_z = false, tool_used = false;

//Point-to-point
const float tolerance = 0.4f;  // Grados articulares
uint64_t grab_now;
uint64_t drop_now;
const uint64_t goto_timeout_us = 15000000ULL;


float encoder_degrees_M1, encoder_degrees_M2;

static void publish_telemetry(int pcbs, th theta){ // IoT
    // encoder_degrees_M1/M2 are only updated in HOMING and goToPoint, so calculate them again: 
    float degreesM1 = encoder_count[0] * 360.0f / 3200.0f;
    float m1 = encoder_sign_M1 * degreesM1 / pulley_ratio_M1; //actual joint angle1
    float degreesM2 = encoder_count[1] * 360.0f / 3200.0f;
    float m2 = encoder_sign_M2 * degreesM2 / pulley_ratio_M2; //actual joint angle2

    int n = snprintf(telemetry_buf, sizeof(telemetry_buf),
                     "%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f",
                     (int)current_state, pcbs, m1, m2,
                     theta.theta1, theta.theta2, theta.theta_prism);
    if (n <= 0 || n >= (int)sizeof(telemetry_buf)) return; //out of bounds

    telemetry_msg.data.size = n;
    rcl_publish(&telemetry_pub, &telemetry_msg, NULL);
}

static bool jog_limit_hit(int motor, int dir){
    if (motor == 0) return (dir == DIR_CW) ? M1_CW_LIMIT_HIT : M1_CCW_LIMIT_HIT;
    else            return (dir == DIR_CW) ? M2_CW_LIMIT_HIT : M2_CCW_LIMIT_HIT;
}

static void jog_stop_motor(int motor){
    move_motor(motor == 0 ? PWM_M1 : PWM_M2, 0);
    jog_dir[motor] = DIR_STOP;
}

//Moves M1/M2 in MANUAL, checking the limit switches
static void jog_motor(int motor, int dir, int duty){
    if (dir == DIR_STOP || duty <= 0) {
        //Released: unblock both directions
        jog_block_cw[motor] = false;
        jog_block_ccw[motor] = false;
        jog_stop_motor(motor);
        return;
    }
    //Moving one way unblocks the other one
    if (dir == DIR_CW) jog_block_ccw[motor] = false;
    else               jog_block_cw[motor]  = false;

    volatile bool *block = (dir == DIR_CW) ? &jog_block_cw[motor] : &jog_block_ccw[motor];
    if (*block || jog_limit_hit(motor, dir)) {
        *block = true;
        jog_stop_motor(motor);
        return;
    }

    if (motor == 0) { if (dir == DIR_CW) CW_M1; else CCW_M1; }
    else            { if (dir == DIR_CW) CW_M2; else CCW_M2; }
    move_motor(motor == 0 ? PWM_M1 : PWM_M2, duty);
    jog_dir[motor] = dir;
}

//Stops a joint when it reaches a limit switch (called every loop in MANUAL)
static void jog_limit_guard(void){
    for (int motor = 0; motor < 2; motor++) {
        int dir = jog_dir[motor];
        if (dir != DIR_STOP && jog_limit_hit(motor, dir)) {
            if (dir == DIR_CW) jog_block_cw[motor] = true;
            else               jog_block_ccw[motor] = true;
            jog_stop_motor(motor);
        }
    }
}

//Speed from -100 to 100 %
static void jog_motor_percent(int motor, int percent, int max_duty){
    if (percent > 100) percent = 100;
    if (percent < -100) percent = -100;
    if (percent == 0) { jog_motor(motor, DIR_STOP, 0); return; }
    int mag = percent > 0 ? percent : -percent;
    jog_motor(motor, percent > 0 ? DIR_CW : DIR_CCW,
              (int)(((float)mag / 100.0f) * max_duty));
}

static void cmd_callback(const void *msgin) {
    const std_msgs__msg__String *m = (const std_msgs__msg__String *)msgin;
    char cmd[65];
    if (m->data.size >= sizeof(cmd)) {
        return;
    }
    memcpy(cmd, m->data.data, m->data.size);
    cmd[m->data.size] = '\0';

    if (strcmp(cmd, "STOP") == 0) {
        move_motor(PWM_M1, 0);
        STOP_M1;
        move_motor(PWM_M2, 0);
        STOP_M2;
        current_state = STATE_IDLE;
    } else if (strcmp(cmd, "HOME") == 0) {
        // Stop any in-progress manual jog before handing control to the homing routine
        move_motor(PWM_M1, 0);
        move_motor(PWM_M2, 0);
        robot_homed = false;
        current_state = STATE_HOMING;
    } else if (strcmp(cmd, "MANUAL") == 0) {
        current_state = STATE_MANUAL;
    
    } else if (strncmp(cmd, "GO TO:", 6) == 0) {
        float x, y, z;
        char tool_str[4] = {0};

        int result = sscanf(cmd, "GO TO:%f,%f,%f,%3s", &x, &y, &z, tool_str);
        if (result == 4 && isfinite(x) && isfinite(y) && isfinite(z) && robot_homed && current_state == STATE_IDLE) {

            coord_x = x;
            coord_y = y;
            coord_z = z;

            tool = (strcmp(tool_str, "ON") == 0); //strcmp compares tool_str and "ON". If they're the same, it gives 0
            current_state = STATE_TEST_IK;
        }
    } else if (strcmp(cmd, "PICK&PLACE") == 0 && robot_homed && current_state == STATE_IDLE) {
        current_state = STATE_PICK_CASE;
    } else if (strcmp(cmd, "J1_CW") == 0) {
        if (current_state == STATE_MANUAL) { last_manual_cmd_time = time_us_64(); jog_motor(0, DIR_CW, MANUAL_JOG_DUTY_M1); }
        else printf("Ignored %s: send MANUAL first.\n", cmd);
    } else if (strcmp(cmd, "J1_CCW") == 0) {
        if (current_state == STATE_MANUAL) { last_manual_cmd_time = time_us_64(); jog_motor(0, DIR_CCW, MANUAL_JOG_DUTY_M1); }
        else printf("Ignored %s: send MANUAL first.\n", cmd);
    } else if (strcmp(cmd, "J1_STOP") == 0) {
        if (current_state == STATE_MANUAL) { last_manual_cmd_time = time_us_64(); jog_motor(0, DIR_STOP, 0); }
        else printf("Ignored %s: send MANUAL first.\n", cmd);
    } else if (strcmp(cmd, "J2_CW") == 0) {
        if (current_state == STATE_MANUAL) { last_manual_cmd_time = time_us_64(); jog_motor(1, DIR_CW, MANUAL_JOG_DUTY_M2); }
        else printf("Ignored %s: send MANUAL first.\n", cmd);
    } else if (strcmp(cmd, "J2_CCW") == 0) {
        if (current_state == STATE_MANUAL) { last_manual_cmd_time = time_us_64(); jog_motor(1, DIR_CCW, MANUAL_JOG_DUTY_M2); }
        else printf("Ignored %s: send MANUAL first.\n", cmd);
    } else if (strcmp(cmd, "J2_STOP") == 0) {
        if (current_state == STATE_MANUAL) { last_manual_cmd_time = time_us_64(); jog_motor(1, DIR_STOP, 0); }
        else printf("Ignored %s: send MANUAL first.\n", cmd);
    } else if (strcmp(cmd, "J3_UP") == 0) {
        if (current_state == STATE_MANUAL) { last_manual_cmd_time = time_us_64(); servo_jog_up(); }
        else printf("Ignored %s: send MANUAL first.\n", cmd);
    } else if (strcmp(cmd, "J3_DOWN") == 0) {
        if (current_state == STATE_MANUAL) { last_manual_cmd_time = time_us_64(); servo_jog_down(); }
        else printf("Ignored %s: send MANUAL first.\n", cmd);
    //Speed commands from the controller, from -100 to 100 %
    } else if (strncmp(cmd, "J1_SPEED:", 9) == 0) {
        if (current_state == STATE_MANUAL) {
            last_manual_cmd_time = time_us_64();
            jog_motor_percent(0, atoi(cmd + 9), MANUAL_JOG_DUTY_M1);
        } else printf("Ignored %s: send MANUAL first.\n", cmd);
    } else if (strncmp(cmd, "J2_SPEED:", 9) == 0) {
        if (current_state == STATE_MANUAL) {
            last_manual_cmd_time = time_us_64();
            jog_motor_percent(1, atoi(cmd + 9), MANUAL_JOG_DUTY_M2);
        } else printf("Ignored %s: send MANUAL first.\n", cmd);
    } else if (strncmp(cmd, "J3_SPEED:", 9) == 0) {
        //Positive = Z up
        if (current_state == STATE_MANUAL) {
            last_manual_cmd_time = time_us_64();
            int percent = atoi(cmd + 9);
            if (percent > 100) percent = 100;
            if (percent < -100) percent = -100;
            j3_rate_dps = -((float)percent / 100.0f) * SERVO_JOG_SPEED_DPS;
        } else printf("Ignored %s: send MANUAL first.\n", cmd);
    //Vacuum tool (not stopped by the dead-man's switch)
    } else if (strcmp(cmd, "TOOL_ON") == 0) {
        if (current_state == STATE_MANUAL) { TOOL_ON; }
        else printf("Ignored %s: send MANUAL first.\n", cmd);
    } else if (strcmp(cmd, "TOOL_OFF") == 0) {
        if (current_state == STATE_MANUAL) { TOOL_OFF; }
        else printf("Ignored %s: send MANUAL first.\n", cmd);
    } else {
        printf("Unknown state: %s\n", cmd);
    }
}

static th defineAngles(float point_X, float point_Y, float point_Z){ //Get the angles from calculations
    if (zero) set_servo_angle(45);
    zero = (point_Z <= 0);

    th anglesM = calculateIK(point_X, point_Y, point_Z);
    if(!isfinite(anglesM.theta1) || !isfinite(anglesM.theta2) || anglesM.theta1 == 180.0f || anglesM.theta2 == 180.0f){ //Out of joint limitations
        move_motor(PWM_M1,0);
        STOP_M1;
        move_motor(PWM_M2,0);
        STOP_M2;
        current_state = STATE_IDLE;
        return anglesM;
    } else {
        m1_moved = false;
        m2_moved = false;
        reach_z = false;
        goto_start_time = time_us_64();
    }
    return anglesM;
}

static bool goToPoint(float angleM1, float angleM2, float angleServo, bool activate_tool){
    if (time_us_64() - goto_start_time >= goto_timeout_us) {
        move_motor(PWM_M1, 0);
        STOP_M1;
        move_motor(PWM_M2, 0);
        STOP_M2;
        current_state = STATE_IDLE;
        return false;
    }

    // For M1:
    encoder_degrees_M1 = encoder_count[0] * 360.0f / 3200.0f; // Since it's a Pololu DC motor 50:1, 64 ticks --> 3200 ticks
    float current_M1 = encoder_sign_M1 * encoder_degrees_M1 / pulley_ratio_M1;
    float error_M1 = angleM1 - current_M1;

    if (fabsf(error_M1) <= tolerance){
        move_motor(PWM_M1, 0);
        STOP_M1;
        m1_moved = true;
    } else { 
        m1_moved = false;
        if (error_M1 > 0.05f) {
            CW_M1;
        } else {
            CCW_M1;
        }
        move_motor(PWM_M1, 1100);
    }

    //For M2 :D
    encoder_degrees_M2 = encoder_count[1] * 360.0f / 3200.0f; // Since it's a Pololu DC motor 50:1, 64 ticks --> 3200 ticks
    float current_M2 = encoder_sign_M2 * encoder_degrees_M2 / pulley_ratio_M2;
    float error_M2 = angleM2 - current_M2;

    if (fabsf(error_M2) <= tolerance){
        move_motor(PWM_M2, 0);
        STOP_M2;
        m2_moved = true;
    } else {
        m2_moved = false;
        if (error_M2 > 0.0f) {
            CW_M2;
        } else {
            CCW_M2;
        }
        move_motor(PWM_M2, 1100);
    }

    if (m1_moved && m2_moved) {
        move_motor(PWM_M1, 0);
        STOP_M1;
        move_motor(PWM_M2, 0);
        STOP_M2;
        set_servo_angle(angleServo);
        if (reach_z == false){
            if (activate_tool) {
                TOOL_ON;
                tool_used = true;
                grab_now = time_us_64();
            } else {
                drop_now = time_us_64();
            }
            reach_z = true;
        }
        return true;
    }
    return false;
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

    //Publisher for IoT system
    telemetry_msg.data.data = telemetry_buf;
    telemetry_msg.data.capacity = sizeof(telemetry_buf);
    telemetry_msg.data.size = 0;
    rclc_publisher_init_best_effort(   //best effort so that it never waits --> otherwise, it could stall the motor loop if the link is slow or lost.
        &telemetry_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "scara_telemetry");

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

    bool l1_calib, l1_homed, l2_calib, l2_homed;
    th theta = {0};
    int pcbs = 1; //This SCARA's task: Pick&Place of four PCBs with cases;
    
    uint64_t last_time = time_us_64();
    uint64_t last_telemetry_us = 0;
    uint64_t t_tel;
    scara_state_t last_reported_state = STATE_INIT;    

    current_state = STATE_IDLE;
    while(1){
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)); //each call waits up to 10ms for a new message from publisher
        if(current_state != last_reported_state){ //To run anything once
            scara_state_t entered_state = current_state;
            switch(current_state){
                case STATE_IDLE:
                    for (int i = 0; i < 4; i++){
                        gpio_put(LED_PIN, 1);
                        sleep_ms(125);
                        gpio_put(LED_PIN, 0);
                        sleep_ms(125);
                    }
                    if(tool_used == false) TOOL_OFF;
                    pcbs = 1;
                    m1_moved = false;
                    m2_moved = false;
                    //waiting for a message to change state...
                    break;
                case STATE_HOMING:
                    TOOL_OFF;
                    tool_used = false;
                    CCW_M1;
                    CW_M2;
                    move_motor(PWM_M1, 0);
                    STOP_M1;
                    move_motor(PWM_M2, 900);
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
                    tool_used = false;
                    last_manual_cmd_time = time_us_64(); //Reset dead-man's switch
                    //Reset jog and limit switch state
                    for (int i = 0; i < 2; i++) {
                        jog_dir[i] = DIR_STOP;
                        jog_block_cw[i] = false;
                        jog_block_ccw[i] = false;
                    }
                    j3_rate_dps = 0.0f;
                    j3_last_us = time_us_64();
                    break;
                case STATE_TEST_IK:
                    theta = defineAngles(coord_x, coord_y, coord_z);
                    break;
                case STATE_PICK_CASE:
                    theta = defineAngles(28.07f, 5.0f, -1.0f);
                    break;
                case STATE_PLACE_CASE:
                    switch(pcbs){
                        case 1:
                            theta = defineAngles(28.07f, -5.0f, -1.0f);
                            break;
                        case 2:
                            theta = defineAngles(24.79f, -10.0f, -1.0f);
                            break;
                        case 3:
                            theta = defineAngles(20.19f, -13.0f, -1.0f);
                            break;
                        case 4:
                            theta = defineAngles(15.0f, -14.0f, -1.0f);
                            break;
                        default:
                            break;
                    }
                    break;
                case STATE_PICK_PCB:
                    theta = defineAngles(15.0f, 14.0f, -1.0f);
                    break;
                case STATE_INSERT_PCB:
                    switch(pcbs){
                        case 1:
                            theta = defineAngles(28.07f, -5.0f, -1.0f);
                            break;
                        case 2:
                            theta = defineAngles(24.79f, -10.0f, -1.0f);
                            break;
                        case 3:
                            theta = defineAngles(20.19f, -13.0f, -1.0f);
                            break;
                        case 4:
                            theta = defineAngles(15.0f, -14.0f, -1.0f);
                            break;
                        default:
                            break;
                    }
                    break;
                default: //just in case
                    break;
            }
            last_reported_state = entered_state;
        }
        switch(current_state){ //TO run continuously
            case STATE_MANUAL: {
                // Jog commands are handled directly in cmd_callback() as they arrive.
                //Dead-man's switch: stop if no jog command arrived in MANUAL_CMD_TIMEOUT_MS
                uint64_t now = time_us_64(); //read here, not before spin_some (avoids underflow)
                if (now >= last_manual_cmd_time &&
                    (now - last_manual_cmd_time) > (uint64_t)MANUAL_CMD_TIMEOUT_MS * 1000){
                    jog_stop_motor(0);
                    jog_stop_motor(1);
                    j3_rate_dps = 0.0f;
                }
                //Smooth J3: move the servo by rate * elapsed time (dt limited to 50 ms)
                {
                    uint64_t t = time_us_64();
                    float dt = (float)(t - j3_last_us) / 1e6f;
                    j3_last_us = t;
                    if (dt > 0.05f) dt = 0.05f;
                    servo_move_rate(j3_rate_dps, dt);
                }
                jog_limit_guard(); //Limit switch protection
                break;
            }
            case STATE_HOMING:
                //Calibration: go to starting position set as 0 by the limit switches
                if (SWITCH_0_L2_ON && l2_calib == false){
                    move_motor(PWM_M2, 0);
                    STOP_M2;
                    CCW_M1;
                    move_motor(PWM_M1, 900);
                    l2_calib = true;
                    encoder_count[1] = 0; //Reset value of motor 2's encoder
                }
                if (SWITCH_0_L1_ON && l1_calib == false && l2_calib == true){
                    move_motor(PWM_M1, 0);
                    STOP_M1;
                    l1_calib = true;
                    encoder_count[0] = 0; //Reset value of motor 1's encoder
                }
                //After reaching this calibration position, start homing:
                if (l1_calib && l2_calib){
                    if (m2_moved == false){
                        CCW_M2;
                        move_motor(PWM_M2, 1000);
                        m2_moved = true;
                    }
                    if (m1_moved == false){
                        CW_M1;
                        move_motor(PWM_M1, 900); 
                        m1_moved = true;
                    }
                    encoder_degrees_M2 = encoder_count[1] * 360.0f / 3200.0f; // Since it's a Pololu DC motor 50:1, 64 ticks --> 3200 ticks
                    if (fabsf(encoder_degrees_M2) >= target_M2_home){
                        move_motor(PWM_M2, 0);
                        STOP_M2;
                        l2_homed = true;
                    }
                    encoder_degrees_M1 = encoder_count[0] * 360.0f / 3200.0f; // Since it's a Pololu DC motor 50:1, 64 ticks --> 3200 ticks
                    if (fabsf(encoder_degrees_M1) >= target_M1_home){
                        move_motor(PWM_M1, 0);
                        STOP_M1;
                        l1_homed = true;
                    }
                }
                
                if (l1_homed && l2_homed){
                    encoder_count[0] = 0; //Home will be the new origin
                    encoder_count[1] = 0; //Home will be the new origin
                    robot_homed = true;
                    current_state = STATE_IDLE;
                }
                break;
            case STATE_TEST_IK:
                if (goToPoint(theta.theta1, theta.theta2, theta.theta_prism, tool)){
                    if ((time_us_64() - grab_now >= 500000ULL) && tool == true) {
                        current_state = STATE_IDLE;
                    }
                    if ((time_us_64() - drop_now >= 500000ULL) && tool == false) {
                        TOOL_OFF;
                        tool_used = false;
                        current_state = STATE_IDLE;
                    }
                }
                break;
            case STATE_PICK_CASE:
                if (goToPoint(theta.theta1, theta.theta2, theta.theta_prism, 1)){
                    if ((time_us_64() - grab_now >= 2000000ULL)) {
                        current_state = STATE_PLACE_CASE;
                    }
                }
                break;
            case STATE_PLACE_CASE:
                if (goToPoint(theta.theta1, theta.theta2, theta.theta_prism, 0)){
                    if ((time_us_64() - drop_now >= 2000000ULL)) { //Drop the piece after the servo arrives the position
                        TOOL_OFF;
                        tool_used = false;
                        current_state = STATE_PICK_PCB;
                    }
                }
                break;
            case STATE_PICK_PCB:
                if (goToPoint(theta.theta1, theta.theta2, theta.theta_prism, 1)){
                    if ((time_us_64() - grab_now >= 2000000ULL)) {
                        current_state = STATE_INSERT_PCB;
                    }
                } 
                break;
            case STATE_INSERT_PCB:
                if (goToPoint(theta.theta1, theta.theta2, theta.theta_prism, 0)){
                    if ((time_us_64() - drop_now >= 2000000ULL)) { //Drop the piece after the servo arrives the position
                        TOOL_OFF;
                        tool_used = false;
                        pcbs++;
                        if (pcbs <= 4){
                            current_state = STATE_PICK_CASE;
                        } else {
                            current_state = STATE_HOMING;
                        }
                    }
                }
                break;
            default:
                break;
        }
        t_tel = time_us_64();
        if (t_tel - last_telemetry_us >= 200000) {
            last_telemetry_us = t_tel;
            publish_telemetry(pcbs, theta); //publish the data of SCARA's main variables every 200ms to the IoT system, displayed in Grafana  
        }
    }

    return 0; //success
}
