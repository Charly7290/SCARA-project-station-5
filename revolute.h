#pragma once
#include <stdbool.h>

#define CCW_M1 (gpio_put(AIN1_DIR_M1, 0), gpio_put(AIN2_DIR_M1, 1))
#define CW_M1 (gpio_put(AIN1_DIR_M1, 1), gpio_put(AIN2_DIR_M1, 0))
#define CCW_M2 (gpio_put(BIN1_DIR_M2, 0), gpio_put(BIN2_DIR_M2, 1))
#define CW_M2 (gpio_put(BIN1_DIR_M2, 1), gpio_put(BIN2_DIR_M2, 0))

// Inverse logic because of pull-up resistor from the switch module:
#define SWITCH_0_L1_ON (gpio_get(LIMIT_SWITCH_0_L1) == 0)
#define SWITCH_0_L1_OFF (gpio_get(LIMIT_SWITCH_0_L1) == 1)
#define SWITCH_1_L1_ON (gpio_get(LIMIT_SWITCH_1_L1) == 0)
#define SWITCH_1_L1_OFF (gpio_get(LIMIT_SWITCH_1_L1) == 1)
#define SWITCH_0_L2_ON (gpio_get(LIMIT_SWITCH_0_L2) == 0)
#define SWITCH_0_L2_OFF (gpio_get(LIMIT_SWITCH_0_L2) == 1)
#define SWITCH_1_L2_ON (gpio_get(LIMIT_SWITCH_1_L2) == 0)
#define SWITCH_1_L2_OFF (gpio_get(LIMIT_SWITCH_1_L2) == 1)

void limitS(int pin);
void encoder_a_irq_handler(bool motor, int motor_encoderA, int motor_encoderB, uint gpio, uint32_t events);
void init_encoder(int motor_encoderA, int motor_encoderB);
void init_motor(int IN1, int IN2, int PWM_M);