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

th calculateIK(float p_x, float p_y){ //px and py must be in cm
    th anglesIK;
    // th1 is elbow up; th1_p is elbow down 

    //Possible collision with SCARA's base:
    if (p_x < -19.0f && p_x > 0.0f){ 
        p_x = -19.0f;
    }
    if (p_y > -14.0f && p_y < 0.0f){
        p_y = -14.0f; //Right side, not to collide with the power supply on this side
    }
    if (p_y < 11.0f && p_y > 0.0f){ //Left side
        p_y = 11.0f;
    }

    const float l1 = 15.0f, l2 = 15.5f; //Link lenghts, in cm
    float r = sqrtf(powf(p_x, 2.0f) + powf(p_y, 2.0f));
    float alpha = atan2f(p_y,p_x);
    float th1, th2_p, th1_p, th2, phi;

    phi = (powf(r, 2.0f)-powf(l1, 2.0f)-powf(l2, 2.0f))/(2*l1*l2);
    th1 = alpha + acosf((powf(r, 2.0f)+powf(l1, 2.0f)-powf(l2, 2.0f))/(2.0f*l1*r));
    th2_p = -acos(phi);
    th1_p = alpha - acosf((powf(r, 2.0f)+powf(l1, 2.0f)-powf(l2, 2.0f))/(2.0f*l1*r));
    th2 = -th2_p;
    if (isnan(th1) || isnan(th1_p) || isnan(th2) || isnan(th2_p)) {  // target unreachable
      anglesIK.theta1 = 180.0f;
      anglesIK.theta2 = 180.0f;
      return anglesIK;
    } else {
        th1 = th1 * (180.0f / (float)M_PI);
        th2_p = th2_p * (180.0f / (float)M_PI);
        th1_p = th1_p * (180.0f / (float)M_PI);
        th2 = th2 * (180.0f / (float)M_PI);
    }

    //Out of joint limitations:
    if(th1 < -105.9f || th1 > 105.9f){ //link 1
        th1 = 180.0f;
    }
    if(th1_p < -105.9f || th1_p > 105.9f){ 
        th1_p = 180.0f;
    }
    if(th2 > 130.0f){ //link 2
        th2 = 180.0f;
    }
    if(th2_p < -130.0f){
        th2_p = 180.0f;
    }

    //Direction of Motors: (it needs an improvement so that the direction depends on the shortest path)***
    if(p_y < 0) { // Left side of SCARA
        CCW_M1;
        CCW_M2;
        anglesIK.theta2 = fabsf(th2_p);
        anglesIK.theta1 = fabsf(th1);
    } else if(p_y > 0) { // Right side of SCARA
        CW_M1;
        CW_M2;
        anglesIK.theta2 = fabsf(th2);
        anglesIK.theta1 = fabsf(th1_p);
    }

    return anglesIK;
}