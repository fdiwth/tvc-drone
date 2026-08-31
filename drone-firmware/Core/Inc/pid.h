/* pid.h */
#ifndef PID_H
#define PID_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    /* Gains */
    float kp;
    float ki;
    float kd;

    /* Limits */
    float output_min;
    float output_max;
    float integral_min;   /* clamp on the integral *state*, not just output */
    float integral_max;

    uint32_t decimation_factor;   // config: 1 = no decimation, N = output updates every Nth call
    uint32_t decimation_counter;  // state: internal call counter
    float    output_held;

    float d_filter_alpha;

    /* Internal state — do not set these directly, use pid_reset() */
    float integral;
    float prev_error;
    float prev_measurement;
    float d_filtered;
    uint8_t initialized;
} pid_t;

void  pid_reset(pid_t *pid);
float pid_update(pid_t *pid, float setpoint, float measurement, float dt, bool hold_integral);

#endif /* PID_H */
