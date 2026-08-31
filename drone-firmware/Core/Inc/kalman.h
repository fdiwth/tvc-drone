#ifndef KALMAN_H
#define KALMAN_H

#include <stdint.h>

#define KALMAN_MAX_DIM   4   // max state dimension
#define KALMAN_MAX_MEAS  4   // max measurement dimension

typedef struct {
    uint8_t dim;    // state dimension n
    uint8_t meas;   // measurement dimension m

    float x[KALMAN_MAX_DIM];                           // state vector (n x 1)
    float P[KALMAN_MAX_DIM][KALMAN_MAX_DIM];           // error covariance (n x n)
    float Q[KALMAN_MAX_DIM][KALMAN_MAX_DIM];           // process noise (n x n)
    float R[KALMAN_MAX_MEAS][KALMAN_MAX_MEAS];         // measurement noise (m x m)
} kalman_t;

// Q_diag: n process noise values
// R_diag: m measurement noise values
// x0:     n initial state values (NULL = zero)
void  kalman_init   (kalman_t *k,
                     uint8_t dim, uint8_t meas,
                     const float *Q_diag,
                     const float *R_diag,
                     const float *x0);

// F:  n x n state transition matrix (row-major)
// B:  n x n control matrix (row-major, NULL if no control input)
// u:  n x 1 control vector (NULL if no control input)
void  kalman_predict(kalman_t *k,
                     const float F[KALMAN_MAX_DIM][KALMAN_MAX_DIM],
                     const float B[KALMAN_MAX_DIM][KALMAN_MAX_DIM],
                     const float *u,
                     float dt);

// H:  m x n observation matrix (row-major)
// z:  m x 1 measurement vector
void  kalman_update (kalman_t *k,
                     const float H[KALMAN_MAX_MEAS][KALMAN_MAX_DIM],
                     const float *z);

float kalman_get_state     (const kalman_t *k, uint8_t idx);
float kalman_get_covariance(const kalman_t *k, uint8_t row, uint8_t col);

// Overrides a single diagonal entry of R between kalman_init and the next
// kalman_update call — for measurements whose noise isn't fixed (e.g.
// optical flow, whose velocity noise scales with height).
void  kalman_set_meas_noise(kalman_t *k, uint8_t idx, float r_value);

#endif
