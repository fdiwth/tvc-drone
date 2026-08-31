#include "kalman.h"
#include <string.h>

// ============================================================
// Scratch buffers
// ============================================================
static float _T1[KALMAN_MAX_DIM][KALMAN_MAX_DIM];
static float _T2[KALMAN_MAX_DIM][KALMAN_MAX_DIM];
static float _T3[KALMAN_MAX_DIM][KALMAN_MAX_DIM];
static float _I [KALMAN_MAX_DIM][KALMAN_MAX_DIM];

// Non-square scratch buffers for H (m x n) operations
static float _HT [KALMAN_MAX_DIM][KALMAN_MAX_MEAS];   // H' is n x m
static float _HP [KALMAN_MAX_MEAS][KALMAN_MAX_DIM];   // H*P is m x n
static float _S  [KALMAN_MAX_MEAS][KALMAN_MAX_MEAS];  // S is m x m
static float _Si [KALMAN_MAX_MEAS][KALMAN_MAX_MEAS];  // S^-1 is m x m
static float _K  [KALMAN_MAX_DIM][KALMAN_MAX_MEAS];   // K is n x m
static float _PHT[KALMAN_MAX_DIM][KALMAN_MAX_MEAS];   // P*H' is n x m
static float _KH [KALMAN_MAX_DIM][KALMAN_MAX_DIM];    // K*H is n x n

static float _vec_n[KALMAN_MAX_DIM];   // n x 1 scratch vector
static float _vec_m[KALMAN_MAX_MEAS];  // m x 1 scratch vector

// ============================================================
// Internal matrix math — now tracks separate row/col dimensions
// ============================================================

static void mat_zero_rc(float *a, uint8_t rows, uint8_t cols) {
    memset(a, 0, rows * cols * sizeof(float));
}

static void mat_identity_n(float a[KALMAN_MAX_DIM][KALMAN_MAX_DIM], uint8_t n) {
    memset(a, 0, KALMAN_MAX_DIM * KALMAN_MAX_DIM * sizeof(float));
    for (uint8_t i = 0; i < n; i++) a[i][i] = 1.0f;
}

// out = A + B  (n x n)
static void mat_add_nn(float out[KALMAN_MAX_DIM][KALMAN_MAX_DIM],
                       const float a[KALMAN_MAX_DIM][KALMAN_MAX_DIM],
                       const float b[KALMAN_MAX_DIM][KALMAN_MAX_DIM],
                       uint8_t n) {
    for (uint8_t i = 0; i < n; i++)
        for (uint8_t j = 0; j < n; j++)
            out[i][j] = a[i][j] + b[i][j];
}

// out = A + B  (m x m)
static void mat_add_mm(float out[KALMAN_MAX_MEAS][KALMAN_MAX_MEAS],
                       const float a[KALMAN_MAX_MEAS][KALMAN_MAX_MEAS],
                       const float b[KALMAN_MAX_MEAS][KALMAN_MAX_MEAS],
                       uint8_t m) {
    for (uint8_t i = 0; i < m; i++)
        for (uint8_t j = 0; j < m; j++)
            out[i][j] = a[i][j] + b[i][j];
}

// out = A * B  (n x n) * (n x n) = (n x n)
static void mat_mul_nn(float out[KALMAN_MAX_DIM][KALMAN_MAX_DIM],
                       const float a[KALMAN_MAX_DIM][KALMAN_MAX_DIM],
                       const float b[KALMAN_MAX_DIM][KALMAN_MAX_DIM],
                       uint8_t n) {
    float tmp[KALMAN_MAX_DIM][KALMAN_MAX_DIM] = {0};
    for (uint8_t i = 0; i < n; i++)
        for (uint8_t k = 0; k < n; k++) {
            if (a[i][k] == 0.0f) continue;
            for (uint8_t j = 0; j < n; j++)
                tmp[i][j] += a[i][k] * b[k][j];
        }
    memcpy(out, tmp, sizeof(tmp));
}

// out = A * B  (m x n) * (n x n) = (m x n)
static void mat_mul_mn_nn(float out[KALMAN_MAX_MEAS][KALMAN_MAX_DIM],
                          const float a[KALMAN_MAX_MEAS][KALMAN_MAX_DIM],
                          const float b[KALMAN_MAX_DIM][KALMAN_MAX_DIM],
                          uint8_t m, uint8_t n) {
    float tmp[KALMAN_MAX_MEAS][KALMAN_MAX_DIM] = {0};
    for (uint8_t i = 0; i < m; i++)
        for (uint8_t k = 0; k < n; k++) {
            if (a[i][k] == 0.0f) continue;
            for (uint8_t j = 0; j < n; j++)
                tmp[i][j] += a[i][k] * b[k][j];
        }
    memcpy(out, tmp, sizeof(tmp));
}

// out = A * B  (m x n) * (n x m) = (m x m)
static void mat_mul_mn_nm(float out[KALMAN_MAX_MEAS][KALMAN_MAX_MEAS],
                          const float a[KALMAN_MAX_MEAS][KALMAN_MAX_DIM],
                          const float b[KALMAN_MAX_DIM][KALMAN_MAX_MEAS],
                          uint8_t m, uint8_t n) {
    float tmp[KALMAN_MAX_MEAS][KALMAN_MAX_MEAS] = {0};
    for (uint8_t i = 0; i < m; i++)
        for (uint8_t k = 0; k < n; k++) {
            if (a[i][k] == 0.0f) continue;
            for (uint8_t j = 0; j < m; j++)
                tmp[i][j] += a[i][k] * b[k][j];
        }
    memcpy(out, tmp, sizeof(tmp));
}

// out = A * B  (n x n) * (n x m) = (n x m)
static void mat_mul_nn_nm(float out[KALMAN_MAX_DIM][KALMAN_MAX_MEAS],
                          const float a[KALMAN_MAX_DIM][KALMAN_MAX_DIM],
                          const float b[KALMAN_MAX_DIM][KALMAN_MAX_MEAS],
                          uint8_t n, uint8_t m) {
    float tmp[KALMAN_MAX_DIM][KALMAN_MAX_MEAS] = {0};
    for (uint8_t i = 0; i < n; i++)
        for (uint8_t k = 0; k < n; k++) {
            if (a[i][k] == 0.0f) continue;
            for (uint8_t j = 0; j < m; j++)
                tmp[i][j] += a[i][k] * b[k][j];
        }
    memcpy(out, tmp, sizeof(tmp));
}

// out = A * B  (n x m) * (m x m) = (n x m)
static void mat_mul_nm_mm(float out[KALMAN_MAX_DIM][KALMAN_MAX_MEAS],
                          const float a[KALMAN_MAX_DIM][KALMAN_MAX_MEAS],
                          const float b[KALMAN_MAX_MEAS][KALMAN_MAX_MEAS],
                          uint8_t n, uint8_t m) {
    float tmp[KALMAN_MAX_DIM][KALMAN_MAX_MEAS] = {0};
    for (uint8_t i = 0; i < n; i++)
        for (uint8_t k = 0; k < m; k++) {
            if (a[i][k] == 0.0f) continue;
            for (uint8_t j = 0; j < m; j++)
                tmp[i][j] += a[i][k] * b[k][j];
        }
    memcpy(out, tmp, sizeof(tmp));
}

// out = A * B  (n x m) * (m x 1) = (n x 1)
static void mat_vec_mul_nm(float out[KALMAN_MAX_DIM],
                           const float a[KALMAN_MAX_DIM][KALMAN_MAX_MEAS],
                           const float v[KALMAN_MAX_MEAS],
                           uint8_t n, uint8_t m) {
    float tmp[KALMAN_MAX_DIM] = {0};
    for (uint8_t i = 0; i < n; i++)
        for (uint8_t j = 0; j < m; j++)
            tmp[i] += a[i][j] * v[j];
    memcpy(out, tmp, n * sizeof(float));
}

// out = A * v  (n x n) * (n x 1) = (n x 1)
static void mat_vec_mul_nn(float out[KALMAN_MAX_DIM],
                           const float a[KALMAN_MAX_DIM][KALMAN_MAX_DIM],
                           const float v[KALMAN_MAX_DIM],
                           uint8_t n) {
    float tmp[KALMAN_MAX_DIM] = {0};
    for (uint8_t i = 0; i < n; i++)
        for (uint8_t j = 0; j < n; j++)
            tmp[i] += a[i][j] * v[j];
    memcpy(out, tmp, n * sizeof(float));
}

// out = A * B  (n x m) * (m x n) = (n x n)
static void mat_mul_nm_mn_to_nn(float out[KALMAN_MAX_DIM][KALMAN_MAX_DIM],
                                 const float a[KALMAN_MAX_DIM][KALMAN_MAX_MEAS],
                                 const float b[KALMAN_MAX_MEAS][KALMAN_MAX_DIM],
                                 uint8_t n, uint8_t m) {
    float tmp[KALMAN_MAX_DIM][KALMAN_MAX_DIM] = {0};
    for (uint8_t i = 0; i < n; i++)
        for (uint8_t k = 0; k < m; k++) {
            if (a[i][k] == 0.0f) continue;
            for (uint8_t j = 0; j < n; j++)
                tmp[i][j] += a[i][k] * b[k][j];
        }
    memcpy(out, tmp, sizeof(tmp));
}

// out = A * v  (m x n) * (n x 1) = (m x 1)
static void mat_vec_mul_mn(float out[KALMAN_MAX_MEAS],
                           const float a[KALMAN_MAX_MEAS][KALMAN_MAX_DIM],
                           const float v[KALMAN_MAX_DIM],
                           uint8_t m, uint8_t n) {
    float tmp[KALMAN_MAX_MEAS] = {0};
    for (uint8_t i = 0; i < m; i++)
        for (uint8_t j = 0; j < n; j++)
            tmp[i] += a[i][j] * v[j];
    memcpy(out, tmp, m * sizeof(float));
}

// out = A^T  (n x n) -> (n x n)
static void mat_trans_nn(float out[KALMAN_MAX_DIM][KALMAN_MAX_DIM],
                         const float a[KALMAN_MAX_DIM][KALMAN_MAX_DIM],
                         uint8_t n) {
    float tmp[KALMAN_MAX_DIM][KALMAN_MAX_DIM];
    for (uint8_t i = 0; i < n; i++)
        for (uint8_t j = 0; j < n; j++)
            tmp[i][j] = a[j][i];
    memcpy(out, tmp, sizeof(tmp));
}

// out = A^T  (m x n) -> (n x m)
static void mat_trans_mn(float out[KALMAN_MAX_DIM][KALMAN_MAX_MEAS],
                         const float a[KALMAN_MAX_MEAS][KALMAN_MAX_DIM],
                         uint8_t m, uint8_t n) {
    for (uint8_t i = 0; i < n; i++)
        for (uint8_t j = 0; j < m; j++)
            out[i][j] = a[j][i];
}

// Gauss-Jordan inversion for m x m matrix
static int mat_inv_mm(float out[KALMAN_MAX_MEAS][KALMAN_MAX_MEAS],
                      const float a[KALMAN_MAX_MEAS][KALMAN_MAX_MEAS],
                      uint8_t m) {
    float tmp[KALMAN_MAX_MEAS][KALMAN_MAX_MEAS * 2] = {0};

    for (uint8_t i = 0; i < m; i++) {
        for (uint8_t j = 0; j < m; j++)
            tmp[i][j] = a[i][j];
        tmp[i][m + i] = 1.0f;
    }

    for (uint8_t col = 0; col < m; col++) {
        float pivot = tmp[col][col];
        if (pivot > -1e-10f && pivot < 1e-10f) return -1;
        float inv_pivot = 1.0f / pivot;
        for (uint8_t j = 0; j < m * 2; j++)
            tmp[col][j] *= inv_pivot;
        for (uint8_t row = 0; row < m; row++) {
            if (row == col) continue;
            float factor = tmp[row][col];
            if (factor == 0.0f) continue;
            for (uint8_t j = 0; j < m * 2; j++)
                tmp[row][j] -= factor * tmp[col][j];
        }
    }

    for (uint8_t i = 0; i < m; i++)
        for (uint8_t j = 0; j < m; j++)
            out[i][j] = tmp[i][m + j];

    return 0;
}

// ============================================================
// Kalman API
// ============================================================

void kalman_init(kalman_t*k,
                 uint8_t dim, uint8_t meas,
                 const float *Q_diag,
                 const float *R_diag,
                 const float *x0) {
    k->dim  = dim;
    k->meas = meas;

    memset(k->x, 0, sizeof(k->x));
    if (x0) memcpy(k->x, x0, dim * sizeof(float));

    // P = identity
    mat_identity_n(k->P, dim);

    // Q diagonal
    memset(k->Q, 0, sizeof(k->Q));
    for (uint8_t i = 0; i < dim; i++)
        k->Q[i][i] = Q_diag[i];

    // R diagonal
    memset(k->R, 0, sizeof(k->R));
    for (uint8_t i = 0; i < meas; i++)
        k->R[i][i] = R_diag[i];
}

void kalman_predict(kalman_t *k,
                    const float F[KALMAN_MAX_DIM][KALMAN_MAX_DIM],
                    const float B[KALMAN_MAX_DIM][KALMAN_MAX_DIM],
                    const float *u,
                    float dt) {
    uint8_t n = k->dim;
    (void)dt;  // dt is baked into F and B by the caller

    // x = F*x
    mat_vec_mul_nn(_vec_n, F, k->x, n);
    memcpy(k->x, _vec_n, n * sizeof(float));

    // x += B*u
    if (B && u) {
        mat_vec_mul_nn(_vec_n, B, u, n);
        for (uint8_t i = 0; i < n; i++)
            k->x[i] += _vec_n[i];
    }

    // P = F*P*F' + Q
    mat_mul_nn (_T1, F,   k->P, n);   // T1 = F*P
    mat_trans_nn(_T2, F,  n);         // T2 = F'
    mat_mul_nn (_T3, _T1, _T2, n);    // T3 = F*P*F'
    mat_add_nn (k->P, _T3, k->Q, n);  // P  = F*P*F' + Q
}

void kalman_update(kalman_t *k,
                   const float H[KALMAN_MAX_MEAS][KALMAN_MAX_DIM],
                   const float *z) {
    uint8_t n = k->dim;
    uint8_t m = k->meas;

    // y = z - H*x  (m x 1)
    mat_vec_mul_mn(_vec_m, H, k->x, m, n);
    float y[KALMAN_MAX_MEAS];
    for (uint8_t i = 0; i < m; i++)
        y[i] = z[i] - _vec_m[i];

    // S = H*P*H' + R  (m x m)
    mat_trans_mn   (_HT,  H,    m, n);       // HT  = H'  (n x m)
    mat_mul_mn_nn  (_HP,  H,  k->P, m, n);   // HP  = H*P (m x n)
    mat_mul_mn_nm  (_S,  _HP,  _HT, m, n);   // S   = H*P*H' (m x m)
    mat_add_mm     (_S,   _S, k->R, m);       // S   = H*P*H' + R

    // Sinv = S^-1  (m x m)
    if (mat_inv_mm(_Si, _S, m) != 0) return;

    // K = P*H' * S^-1  (n x m)
    mat_mul_nn_nm  (_PHT, k->P, _HT, n, m);  // PHT = P*H'  (n x m)
    mat_mul_nm_mm  (_K,  _PHT,  _Si, n, m);  // K   = P*H'*S^-1 (n x m)

    // x = x + K*y  (n x 1)
    mat_vec_mul_nm(_vec_n, _K, y, n, m);
    for (uint8_t i = 0; i < n; i++)
        k->x[i] += _vec_n[i];

    // P = (I - K*H) * P  (n x n)
    mat_identity_n(_I, n);
    mat_mul_nm_mn_to_nn(_KH, _K, H, n, m);   // KH = K*H (n x n)
    for (uint8_t i = 0; i < n; i++)
        for (uint8_t j = 0; j < n; j++)
            _T1[i][j] = _I[i][j] - _KH[i][j];  // T1 = I - K*H
    mat_mul_nn(_T2, _T1, k->P, n);              // T2 = (I-K*H)*P
    memcpy(k->P, _T2, sizeof(_T2));
}

float kalman_get_state(const kalman_t *k, uint8_t idx) {
    return k->x[idx];
}

float kalman_get_covariance(const kalman_t *k, uint8_t row, uint8_t col) {
    return k->P[row][col];
}

void kalman_set_meas_noise(kalman_t *k, uint8_t idx, float r_value) {
    k->R[idx][idx] = r_value;
}
