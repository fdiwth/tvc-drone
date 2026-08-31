#include "filter.h"

void lpf_init(lpf_t *f, float alpha, float initial_value) {
    f->alpha       = alpha;
    f->value       = initial_value;
    f->initialized = 1;
}

float lpf_update(lpf_t *f, float input) {
    if (!f->initialized) {
        f->value       = input;
        f->initialized = 1;
        return f->value;
    }

    f->value = f->alpha * input + (1.0f - f->alpha) * f->value;
    return f->value;
}

void lpf_reset(lpf_t *f, float value) {
    f->value = value;
}
