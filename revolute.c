#include "revolute.h"
#include "system_lib.h"

static uint pinA_m1, pinB_m1, pinA_m2, pinB_m2;
volatile int32_t encoder_count[2] = {0, 0}; // Store the number of encoder tick of M1 and M2

void init_limitS(int pin){
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin); //enable pull-up resistor
}

// GPIO interrupt handler for Encoder A pin:
void encoder_irq_handler(uint gpio, uint32_t events) {
    if (gpio == ENCODER_A_M1 || gpio == ENCODER_B_M1) {
        bool encoder_a = gpio_get(ENCODER_A_M1);
        bool encoder_b = gpio_get(ENCODER_B_M1);
        if (gpio == ENCODER_A_M1) {
            if ((encoder_a && !encoder_b) || (!encoder_a && encoder_b)) encoder_count[0]++;
            else encoder_count[0]--;
        } else {
            if ((encoder_a && !encoder_b) || (!encoder_a && encoder_b)) encoder_count[0]--;
            else encoder_count[0]++;
        }
    } else if (gpio == ENCODER_A_M2 || gpio == ENCODER_B_M2) {
        bool encoder_a = gpio_get(ENCODER_A_M2);
        bool encoder_b = gpio_get(ENCODER_B_M2);
        if (gpio == ENCODER_A_M2) {
            if ((encoder_a && !encoder_b) || (!encoder_a && encoder_b)) encoder_count[1]++;
            else encoder_count[1]--;
        } else {
            if ((encoder_a && !encoder_b) || (!encoder_a && encoder_b)) encoder_count[1]--;
            else encoder_count[1]++;
        }
    }
}

void init_encoder_M1(){
    // Initialize the GPIO pins connected to the encoder
    gpio_init(ENCODER_A_M1);
    gpio_set_dir(ENCODER_A_M1, GPIO_IN);
    gpio_pull_up(ENCODER_A_M1);
    // Initialize the GPIO pins connected to the encoder
    gpio_init(ENCODER_B_M1);
    gpio_set_dir(ENCODER_B_M1, GPIO_IN);
    gpio_pull_up(ENCODER_B_M1);

    // Attach interrupt on encoder A pin (rising and falling edge)
    gpio_set_irq_enabled_with_callback(ENCODER_A_M1, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_irq_handler);
    gpio_set_irq_enabled_with_callback(ENCODER_B_M1, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_irq_handler);
}

void init_encoder_M2(){
    // Initialize the GPIO pins connected to the encoder
    gpio_init(ENCODER_A_M2);
    gpio_set_dir(ENCODER_A_M2, GPIO_IN);
    gpio_pull_up(ENCODER_A_M2);
    // Initialize the GPIO pins connected to the encoder
    gpio_init(ENCODER_B_M2);
    gpio_set_dir(ENCODER_B_M2, GPIO_IN);
    gpio_pull_up(ENCODER_B_M2);

    // Attach interrupt on encoder A pin (rising and falling edge)
    gpio_set_irq_enabled_with_callback(ENCODER_A_M2, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_irq_handler);
    gpio_set_irq_enabled_with_callback(ENCODER_B_M2, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_irq_handler);
}

void init_motor(int IN1, int IN2, int PWM_M){
    gpio_init(IN1);
    gpio_set_dir(IN1, GPIO_OUT);
    gpio_put(IN1, 1);

    gpio_init(IN2);
    gpio_set_dir(IN2, GPIO_OUT); 
    gpio_put(IN2, 0);
    
    // Set the GPIO pin function to PWM
    gpio_set_function(PWM_M, GPIO_FUNC_PWM);
    
    // Get the PWM slice number associated with the GPIO pin
    uint slice_num = pwm_gpio_to_slice_num(PWM_M);
    
    // Set the wrap value for a 20 kHz PWM signal (calculated wrap = 6249)
    // PWM frequency = (125 MHz / wrap) - 1
    // 6249 = (125000000 / 20000) - 1
    // 20 kHz PWM signal for a DC motor:
    pwm_set_wrap(slice_num, 6249);
    
    // Start with a 0 % duty cycle
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(PWM_M), 0);
    
    // Enable PWM output on the slice
    pwm_set_enabled(slice_num, true);
}

void move_motor(int pin_pwm, int dutyC){
    // Get the PWM slice number associated with the GPIO pin
    uint slice_num = pwm_gpio_to_slice_num(pin_pwm);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(pin_pwm), dutyC);
}