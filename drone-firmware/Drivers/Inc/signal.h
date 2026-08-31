#ifndef SIGNAL_H
#define SIGNAL_H

#include "stm32f7xx_hal.h"

// Frequencies in Hz
#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_D1  37
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_G1  49
#define NOTE_A1  55
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_D2  73
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_G2  98
#define NOTE_A2  110
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_D3  147
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_G3  196
#define NOTE_A3  220
#define NOTE_B3  247
#define NOTE_C4  262  // middle C
#define NOTE_D4  294
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_D5  587
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_G5  784
#define NOTE_A5  880
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_REST 0

#define WHOLE       1000
#define HALF        500
#define QUARTER     250
#define EIGHTH      125
#define SIXTEENTH   62

typedef struct {
    uint32_t frequency;
    uint32_t duration;
} note_t;

typedef struct {
	TIM_HandleTypeDef *buzzer_htim;
	uint16_t          buzzer_channel;
    GPIO_TypeDef     *led_port;
    uint16_t          led_pin;
} signal_config_t;

void signal_init(signal_config_t *config);

void led_on();

void led_off();

void led_toggle();

void buzzer_on();

void buzzer_off();

void buzzer_play(note_t *melody, uint8_t len);

#endif
