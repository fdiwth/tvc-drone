#include <signal.h>

signal_config_t *signal_config;

void signal_init(signal_config_t *config) {
	signal_config = config;
	HAL_TIM_PWM_Start(signal_config->buzzer_htim, signal_config->buzzer_channel);
}

void led_on() {
	HAL_GPIO_WritePin(signal_config->led_port, signal_config->led_pin, SET);
}

void led_off() {
	HAL_GPIO_WritePin(signal_config->led_port, signal_config->led_pin, RESET);
}

void led_toggle() {
	HAL_GPIO_TogglePin(signal_config->led_port, signal_config->led_pin);
}


void buzzer_on(uint32_t freq) {
	if (freq == 0) {
		__HAL_TIM_SET_COMPARE(signal_config->buzzer_htim, signal_config->buzzer_channel, 0);
		return;
	}

	uint32_t arr = (1000000 / freq) - 1;	// auto reload register
	uint32_t ccr = (arr + 1) / 2; // CCR3 register

	__HAL_TIM_SET_AUTORELOAD(signal_config->buzzer_htim, arr);
	__HAL_TIM_SET_COMPARE(signal_config->buzzer_htim, signal_config->buzzer_channel, ccr);
}

void buzzer_off() {
	__HAL_TIM_SET_COMPARE(signal_config->buzzer_htim, signal_config->buzzer_channel, 0);
}

void buzzer_play(note_t *melody, uint8_t len) {
    for (int i = 0; i < len; i++) {
        if (melody[i].frequency == NOTE_REST) {
            buzzer_off();
        } else {
            buzzer_on(melody[i].frequency);
        }
        HAL_Delay(melody[i].duration);
        buzzer_off();
        HAL_Delay(10);
    }
}
