#include "prismatic.h"
#include "system_lib.h"

void init_servo(void) {
    gpio_set_function(PWM_SERVO, GPIO_FUNC_PWM);
    // Find which slice is connected to the GPIO pin
    uint slice_num = pwm_gpio_to_slice_num(PWM_SERVO);

    // Set the clock divisor (to slow down the PWM clock to match servo requirements)
    pwm_set_clkdiv(slice_num, 64.0);  // Set clock divisor to 64
    
    // Set the wrap value for a 50 Hz PWM signal
    // 1 / (50 Hz) = 0.02 s = 20 ms
    // 39060 = 20 ms / (1 / 125 MHz) / 65536
    // 39060 is the maximum value that can be set for the wrap register
    pwm_set_wrap(slice_num, 39060);
    
    // Set the initial position of the servo (0 degrees, corresponding to 1.0 ms pulse width)
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(PWM_SERVO), 1953); 

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
    int angle = map(180, 0, 180, 1953, 3906); // 180° as starting position
    uint slice_num = pwm_gpio_to_slice_num(PWM_SERVO);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(PWM_SERVO), angle);
}