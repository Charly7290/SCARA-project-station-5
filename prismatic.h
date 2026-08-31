#pragma once

// Inverse logic because of the electronic circuit:
#define TOOL_ON gpio_put(TOOL, 0)
#define TOOL_OFF gpio_put(TOOL, 1)

void init_servo(void);
void init_tool(void);
int map(int x, int in_min, int in_max, int out_min, int out_max); // Function to map a value from one range to another
void servo_home(void);
void servo_jog_up(void);   // Manual jog: increase servo angle by SERVO_JOG_STEP_DEG 
void servo_jog_down(void); // Manual jog: decrease servo angle by SERVO_JOG_STEP_DEG 
