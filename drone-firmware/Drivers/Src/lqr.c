#include "lqr.h"
#include <string.h>

void lqr_init(lqr_t *lqr, uint8_t n, uint8_t m,
              const float K[LQR_MAX_INPUTS][LQR_MAX_STATES],
              const float *u_min, const float *u_max) {
    lqr->n = n;
    lqr->m = m;

    memcpy(lqr->K, K, sizeof(lqr->K));
    memset(lqr->x_ref, 0, sizeof(lqr->x_ref));
    memset(lqr->u,     0, sizeof(lqr->u));

    for (uint8_t i = 0; i < m; i++) {
        lqr->u_min[i] = u_min[i];
        lqr->u_max[i] = u_max[i];
    }
}

void lqr_set_ref(lqr_t *lqr, const float *x_ref) {
    memcpy(lqr->x_ref, x_ref, lqr->n * sizeof(float));
}

void lqr_compute(lqr_t *lqr, const float *x) {
    uint8_t n = lqr->n;
    uint8_t m = lqr->m;

    // u = -K * (x - x_ref)
    for (uint8_t i = 0; i < m; i++) {
        lqr->u[i] = 0.0f;
        for (uint8_t j = 0; j < n; j++)
            lqr->u[i] -= lqr->K[i][j] * (x[j] - lqr->x_ref[j]);

        if (lqr->u[i] < lqr->u_min[i]) lqr->u[i] = lqr->u_min[i];
        if (lqr->u[i] > lqr->u_max[i]) lqr->u[i] = lqr->u_max[i];
    }
}
