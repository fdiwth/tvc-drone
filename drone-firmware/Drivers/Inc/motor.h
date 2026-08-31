#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f7xx_hal.h"

typedef struct {
	TIM_HandleTypeDef *pwm_timer;
    uint16_t           pwm_channel;
} motor_config_t;

void motor_init(motor_config_t *config);

void motor_set(motor_config_t *config, float perc);

void motor_swipe(motor_config_t *config, int start, int max, int min, int duration);

void motor_calibrate_escs(motor_config_t *motor1, motor_config_t *motor2);

#endif
