#include "revolute.h"
#include "system_lib.h"

volatile int32_t encoder_M1_ticks = 0; // Store the number of encoder tick of M1
volatile int32_t encoder_M2_ticks = 0; // Store the number of encoder tick of M2

void init_limitS(int pin){
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin); //enable pull-up resistor
}

// GPIO interrupt handler for Encoder A pin:
void encoder_a_irq_handler(bool motor, int motor_encoderA, int motor_encoderB, uint gpio, uint32_t events) {
    // Read both encoder A and B pins
    bool encoder_a = gpio_get(motor_encoderA);
    bool encoder_b = gpio_get(motor_encoderB);

    // Determine direction based on the quadrature signals
    // Count rising and falling edges of both A and B channels
    if (gpio == motor_encoderA) {
        if ((encoder_a && !encoder_b) || (!encoder_a && encoder_b)) {
            if(motor){ //1 for M1, 0 for M2
                encoder_M1_ticks++; // Forward direction
            } else{
                encoder_M2_ticks++; //Forward direction
            }
        } else {
            if(motor){ //1 for M1, 0 for M2
                encoder_M1_ticks--; // Reverse direction
            } else{
                encoder_M2_ticks--; // Reverse direction
            }
        }
    } else {
        if ((encoder_a && !encoder_b) || (!encoder_a && encoder_b)) {
            if(motor){ //1 for M1, 0 for M2
                encoder_M1_ticks--; // Forward direction
            } else{
                encoder_M2_ticks--; //Forward direction
            }
        } else {
            if(motor){ //1 for M1, 0 for M2
                encoder_M1_ticks++; // Reverse direction
            } else{
                encoder_M2_ticks++; // Reverse direction
            }
        }
    }

}

void init_encoder(int motor_encoderA, int motor_encoderB){
    // Initialize the GPIO pins connected to the encoder
    gpio_init(motor_encoderA);
    gpio_set_dir(motor_encoderA, GPIO_IN);
    gpio_pull_up(motor_encoderA);
    // Initialize the GPIO pins connected to the encoder
    gpio_init(motor_encoderB);
    gpio_set_dir(motor_encoderB, GPIO_IN);
    gpio_pull_up(motor_encoderB);

    // Attach interrupt on encoder A pin (rising and falling edge)
    gpio_set_irq_enabled_with_callback(motor_encoderA, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_a_irq_handler);
    gpio_set_irq_enabled_with_callback(motor_encoderB, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_a_irq_handler);
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