#include "servo.h"

void servo_init(servo_config_t *config) {
    HAL_TIM_PWM_Start(config->pwm_timer, config->pwm_channel);
    servo_set(config, 50);
}

void servo_set(servo_config_t *config, float perc) {
    if (perc < 0.0f) perc = 0.0f;
    if (perc > 100.0f) perc = 100.0f;

    uint32_t pulse_width = (uint32_t)(1000.0f + (perc + config->offset) * 10.0f + 0.5f);

    __HAL_TIM_SET_COMPARE(config->pwm_timer, config->pwm_channel, pulse_width);
}

void servo_swipe(servo_config_t *config, int start, int max, int min, int duration) {
    if (config == NULL) return;

    int delay_ms = duration / 2 / (max - min);

    for (int i = start; i <= max; i++) {
        servo_set(config, i);
        HAL_Delay(delay_ms);
    }

    for (int i = max; i >= min; i--) {
        servo_set(config, i);
        HAL_Delay(delay_ms);
    }

    for (int i = min; i <= start; i++) {
        servo_set(config, i);
        HAL_Delay(delay_ms);
    }

    servo_set(config, start);
}
