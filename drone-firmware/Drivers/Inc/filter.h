#ifndef FILTER_H
#define FILTER_H

#include <stdint.h>

typedef struct {
    float alpha;      // smoothing factor 0-1 (lower = more smoothing)
    float value;      // current filtered value
    uint8_t initialized;
} lpf_t;

void  lpf_init   (lpf_t *f, float alpha, float initial_value);
float lpf_update (lpf_t *f, float input);
void  lpf_reset  (lpf_t *f, float value);

#endif
