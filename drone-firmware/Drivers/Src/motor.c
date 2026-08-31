#include "motor.h"

void motor_init(motor_config_t *config) {
    HAL_TIM_PWM_Start(config->pwm_timer, config->pwm_channel);
    motor_set(config, 0);
}

void motor_set(motor_config_t *config, float perc) {
    if (perc < 0) perc = 0;
    if (perc > 100) perc = 100;

    uint32_t pulse_width = 1000 + ((perc) * 1000 / 100);

    __HAL_TIM_SET_COMPARE(config->pwm_timer, config->pwm_channel, pulse_width);
}

void motor_swipe(motor_config_t *config, int start, int max, int min, int duration) {
    if (config == NULL) return;

    int delay_ms = duration / 2 / (max - min);

    for (int i = start; i <= max; i++) {
    	motor_set(config, i);
        HAL_Delay(delay_ms);
    }

    for (int i = max; i >= min; i--) {
    	motor_set(config, i);
        HAL_Delay(delay_ms);
    }

    for (int i = min; i <= start; i++) {
    	motor_set(config, i);
        HAL_Delay(delay_ms);
    }

    motor_set(config, start);
}

void motor_calibrate_escs(motor_config_t *motor1, motor_config_t *motor2) { // this code is meant to be run only in the debug mode
    // Step 1 — send maximum signal with ESC powered off
    // User should power on ESC during this step
    motor_set(motor1, 100);   // 2000µs
    motor_set(motor2, 100);
    HAL_Delay(3000);          // wait for ESC to power on and hear the high signal
                              // ESC will beep to confirm max throttle received

    // Step 2 — send minimum signal
    // ESC will beep again to confirm range is set
    motor_set(motor1, 0);     // 1000µs
    motor_set(motor2, 0);
    HAL_Delay(3000);          // wait for ESC to beep and save the range

    // Step 3 — arm the ESC
    // Most ESCs require sitting at minimum throttle for 1-2 seconds to arm
    motor_set(motor1, 0);
    motor_set(motor2, 0);
    HAL_Delay(2000);          // ESC will beep arming confirmation

}
