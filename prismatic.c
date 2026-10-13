#include "prismatic.h"
#include "system_lib.h"

static float servo_angle_deg = 0.0f; //Last commanded angle (no readback from the servo)

//PWM clock = 125 MHz / 64 -> 125/64 counts per us
static int servo_us_to_level(float pulse_us){
    return (int)(pulse_us * 125.0f / 64.0f + 0.5f);
}

static void servo_write_angle(float angle_deg){
    if (angle_deg < 0.0f) angle_deg = 0.0f;
    if (angle_deg > 180.0f) angle_deg = 180.0f;
    servo_angle_deg = angle_deg;

    float pulse_us = SERVO_PULSE_MIN_US +
                     (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US) * (angle_deg / 180.0f);
    uint slice_num = pwm_gpio_to_slice_num(PWM_SERVO);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(PWM_SERVO), servo_us_to_level(pulse_us));
}

void set_servo_angle(int angle_deg){
    servo_write_angle((float)angle_deg);
}

void servo_move_rate(float deg_per_s, float dt_s){
    if (deg_per_s == 0.0f || dt_s <= 0.0f) return;
    servo_write_angle(servo_angle_deg + deg_per_s * dt_s);
}

void init_servo(void) {
    gpio_set_function(PWM_SERVO, GPIO_FUNC_PWM);
    // Find which slice is connected to the GPIO pin
    uint slice_num = pwm_gpio_to_slice_num(PWM_SERVO);

    // Set the clock divisor (to slow down the PWM clock to match servo requirements)
    pwm_set_clkdiv(slice_num, 64.0);  // Set clock divisor to 64

    // Set the wrap value for a 50 Hz PWM signal (20 ms)
    pwm_set_wrap(slice_num, 39060);

    // Set the initial position of the servo (0 degrees)
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(PWM_SERVO),
                       servo_us_to_level(SERVO_PULSE_MIN_US));

    // Enable PWM output
    pwm_set_enabled(slice_num, true);
}

void init_tool(void){
    gpio_init(TOOL);
    gpio_set_dir(TOOL, GPIO_OUT);
    gpio_pull_up(TOOL); //enable pull-up resistor
}

int map(int x, int in_min, int in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void servo_home(void){
    set_servo_angle(0); // 0 deg as starting/reference position (Z at the top)
}

//0 deg = Z top, so going up decreases the angle
void servo_jog_up(void){
    servo_write_angle(servo_angle_deg - SERVO_JOG_STEP_DEG);
}

void servo_jog_down(void){
    servo_write_angle(servo_angle_deg + SERVO_JOG_STEP_DEG);
}

