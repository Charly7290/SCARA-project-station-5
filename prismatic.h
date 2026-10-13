#pragma once

// Inverse logic because of the electronic circuit:
#define TOOL_ON gpio_put(TOOL, 0)
#define TOOL_OFF gpio_put(TOOL, 1)

// Servo pulse range (MG996R): narrow an end by ~50us if the servo buzzes there
#define SERVO_PULSE_MIN_US 500   //0 deg (home, Z top)
#define SERVO_PULSE_MAX_US 2500  //180 deg (Z bottom)

#define SERVO_JOG_SPEED_DPS 60.0f //J3 speed at 100 %, in deg/s

void init_servo(void);
void init_tool(void);
int map(int x, int in_min, int in_max, int out_min, int out_max); // Function to map a value from one range to another
void servo_home(void);
void servo_jog_up(void);   // Manual jog: Z up, decrease servo angle by SERVO_JOG_STEP_DEG
void servo_jog_down(void); // Manual jog: Z down, increase servo angle by SERVO_JOG_STEP_DEG
void set_servo_angle(int angle_deg);
void servo_move_rate(float deg_per_s, float dt_s); // Smooth jog: angle += deg_per_s * dt_s


