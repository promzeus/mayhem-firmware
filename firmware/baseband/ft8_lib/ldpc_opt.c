/*
 * Optimized LDPC decoder for FT8 - Fixed-Point Version
 * Memory-optimized for Portapack Mayhem (LPC4330)
 * Uses int16_t Q15 fixed-point instead of float
 *
 * Memory usage: ~58 KB (vs ~115 KB in original)
 */

#include "ldpc_opt.h"
#include "constants.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

// Fast tanh approximation for Q15 fixed-point
// Input/output in Q15 format (-32768 to +32767)
static int16_t fast_tanh_q15(int16_t x) {
    // tanh(x) ≈ x for small x
    // tanh(x) ≈ sign(x) for large x
    // Using piecewise linear approximation

    if (x > 16384) return 32767;  // tanh > 0.5 → saturate to 1.0
    if (x < -16384) return -32768;

    // Linear approximation: tanh(x) ≈ x * 0.8 for |x| < 0.5
    return (int16_t)((x * 26214L) >> 15);  // 0.8 in Q15 = 26214
}

// Fast atanh approximation for Q15 fixed-point
static int16_t fast_atanh_q15(int16_t x) {
    // atanh(x) ≈ x for small x
    // Avoid division by using lookup or approximation

    if (x >= 32767) return 32767;
    if (x <= -32767) return -32768;

    // Simple approximation: atanh(x) ≈ x * 1.2 for |x| < 0.5
    return (int16_t)((x * 39322L) >> 15);  // 1.2 in Q15 = 39322
}

// Check LDPC codeword
static int ldpc_check(uint8_t codeword[]) {
    int errors = 0;

    for (int j = 0; j < FTX_LDPC_M; j++) {
        uint8_t x = 0;
        for (int i = 0; i < kFTX_LDPC_Num_rows[j]; i++) {
            x ^= codeword[kFTX_LDPC_Nm[j][i] - 1];
        }
        if (x != 0) {
            errors++;
        }
    }

    return errors;
}

// Optimized LDPC decoder using Q15 fixed-point
void ldpc_decode_opt(int16_t codeword_q15[], int max_iters, uint8_t plain[], int* ok) {
    // Allocate matrices on stack (reduced size with int16_t)
    static int16_t m[FTX_LDPC_M][FTX_LDPC_N];  // 83 * 174 * 2 = ~28.8 KB
    static int16_t e[FTX_LDPC_M][FTX_LDPC_N];  // 83 * 174 * 2 = ~28.8 KB

    int min_errors = FTX_LDPC_M;

    // Initialize matrices
    for (int j = 0; j < FTX_LDPC_M; j++) {
        for (int i = 0; i < FTX_LDPC_N; i++) {
            m[j][i] = codeword_q15[i];
            e[j][i] = 0;
        }
    }

    // LDPC iterations
    for (int iter = 0; iter < max_iters; iter++) {
        // Update e (check node update)
        for (int j = 0; j < FTX_LDPC_M; j++) {
            for (int ii1 = 0; ii1 < kFTX_LDPC_Num_rows[j]; ii1++) {
                int i1 = kFTX_LDPC_Nm[j][ii1] - 1;
                int16_t a = 32767;  // 1.0 in Q15

                for (int ii2 = 0; ii2 < kFTX_LDPC_Num_rows[j]; ii2++) {
                    int i2 = kFTX_LDPC_Nm[j][ii2] - 1;
                    if (i2 != i1) {
                        // a *= tanh(-m[j][i2] / 2)
                        int16_t neg_m_half = (int16_t)((-m[j][i2]) >> 1);
                        int16_t tanh_val = fast_tanh_q15(neg_m_half);

                        // Multiply Q15 values: (a * tanh_val) >> 15
                        a = (int16_t)(((int32_t)a * tanh_val) >> 15);
                    }
                }

                // e[j][i1] = -2 * atanh(a)
                int16_t atanh_a = fast_atanh_q15(a);
                e[j][i1] = (int16_t)(-2 * atanh_a);
            }
        }

        // Update plain (hard decision)
        for (int i = 0; i < FTX_LDPC_N; i++) {
            int32_t l = codeword_q15[i];  // Use 32-bit accumulator

            for (int j = 0; j < 3; j++) {
                l += e[kFTX_LDPC_Mn[i][j] - 1][i];
            }

            plain[i] = (l > 0) ? 1 : 0;
        }

        // Check for errors
        int errors = ldpc_check(plain);

        if (errors < min_errors) {
            min_errors = errors;

            if (errors == 0) {
                break;  // Perfect decode
            }
        }

        // Update m (variable node update)
        for (int i = 0; i < FTX_LDPC_N; i++) {
            for (int ji1 = 0; ji1 < 3; ji1++) {
                int j1 = kFTX_LDPC_Mn[i][ji1] - 1;
                int32_t l = codeword_q15[i];

                for (int ji2 = 0; ji2 < 3; ji2++) {
                    if (ji1 != ji2) {
                        int j2 = kFTX_LDPC_Mn[i][ji2] - 1;
                        l += e[j2][i];
                    }
                }

                // Saturate to Q15 range
                if (l > 32767) l = 32767;
                if (l < -32768) l = -32768;

                m[j1][i] = (int16_t)l;
            }
        }
    }

    *ok = FTX_LDPC_K - min_errors;
}

// Helper: convert float codeword to Q15 format
void ldpc_convert_to_q15(const float codeword_float[], int16_t codeword_q15[], int length) {
    for (int i = 0; i < length; i++) {
        float val = codeword_float[i];

        // Clamp to Q15 range
        if (val > 1.0f) val = 1.0f;
        if (val < -1.0f) val = -1.0f;

        codeword_q15[i] = FLOAT_TO_Q15(val);
    }
}
