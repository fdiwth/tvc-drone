/* pid.c */
#include "pid.h"

void pid_reset(pid_t *pid) {
    pid->integral            = 0.0f;
    pid->prev_error          = 0.0f;
    pid->prev_measurement    = 0.0f;
    pid->d_filtered          = 0.0f;
    pid->initialized         = 0;
    pid->decimation_counter  = 0;
    pid->output_held         = 0.0f;
}

float pid_update(pid_t *pid, float setpoint, float measurement, float dt, bool hold_integral) {
    if (dt <= 0.0f) return pid->output_held;

    float error = setpoint - measurement;

    float d_raw;
    if (!pid->initialized) {
        d_raw = 0.0f;
        pid->initialized = 1;
    } else {
        d_raw = -(measurement - pid->prev_measurement) / dt;
    }
    pid->prev_measurement = measurement;

    pid->d_filtered += pid->d_filter_alpha * (d_raw - pid->d_filtered);

    if (!hold_integral) {
        float integral_tentative = pid->integral + error * dt;
        if (integral_tentative >  pid->integral_max) integral_tentative =  pid->integral_max;
        if (integral_tentative < pid->integral_min) integral_tentative = pid->integral_min;
        pid->integral = integral_tentative;
    }

    pid->prev_error = error;

    pid->decimation_counter++;
    if (pid->decimation_counter >= pid->decimation_factor) {
        pid->decimation_counter = 0;

        float p_term = pid->kp * error;
        float i_term = pid->ki * pid->integral;
        float d_term = pid->kd * pid->d_filtered;

        float output = p_term + i_term + d_term;
        if (output > pid->output_max) output = pid->output_max;
        if (output < pid->output_min) output = pid->output_min;

        pid->output_held = output;
    }

    return pid->output_held;
}
