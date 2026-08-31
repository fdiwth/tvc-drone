#ifndef LQR_H
#define LQR_H

#include <stdint.h>

#define LQR_MAX_STATES  12
#define LQR_MAX_INPUTS  4

typedef struct {
    uint8_t n;   // total state dimension including integral states
    uint8_t m;

    float K[LQR_MAX_INPUTS][LQR_MAX_STATES];
    float x_ref[LQR_MAX_STATES];
    float u[LQR_MAX_INPUTS];
    float u_min[LQR_MAX_INPUTS];
    float u_max[LQR_MAX_INPUTS];
} lqr_t;

void lqr_init    (lqr_t *lqr, uint8_t n, uint8_t m,
                  const float K[LQR_MAX_INPUTS][LQR_MAX_STATES],
                  const float *u_min, const float *u_max);
void lqr_set_ref (lqr_t *lqr, const float *x_ref);
void lqr_compute (lqr_t *lqr, const float *x);

#endif
