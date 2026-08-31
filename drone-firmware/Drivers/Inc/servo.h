#ifndef SERVO_H
#define SERVO_H

#include "stm32f7xx_hal.h"

typedef struct {
	TIM_HandleTypeDef *pwm_timer;
    uint16_t           pwm_channel;
    int 			   offset;
} servo_config_t;

void servo_init(servo_config_t *config);

void servo_set(servo_config_t *config, float perc);

void servo_swipe(servo_config_t *config, int start, int max, int min, int duration);

#endif
