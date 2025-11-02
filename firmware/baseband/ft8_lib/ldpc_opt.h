/*
 * Optimized LDPC decoder for FT8 on memory-constrained systems
 * Uses int16_t instead of float to reduce memory by 50%
 * Adapted for Portapack Mayhem (LPC4330 with 264KB SRAM)
 */

#ifndef _INCLUDE_LDPC_OPT_H_
#define _INCLUDE_LDPC_OPT_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

// Fixed-point Q15 format: 1 sign bit + 15 fractional bits
// Range: -1.0 to +0.999969482421875
#define Q15_SCALE 32768
#define FLOAT_TO_Q15(x) ((int16_t)((x) * Q15_SCALE))
#define Q15_TO_FLOAT(x) (((float)(x)) / Q15_SCALE)

// Optimized LDPC decoder using Q15 fixed-point arithmetic
// Memory usage: ~58 KB instead of ~115 KB
// codeword is 174 log-likelihoods in Q15 format
// plain is a return value, 174 ints, to be 0 or 1
// max_iters is how hard to try (recommend 20-25 for embedded)
// ok == 87 means success
void ldpc_decode_opt(int16_t codeword_q15[], int max_iters, uint8_t plain[], int* ok);

// Helper: convert float codeword to Q15 format
void ldpc_convert_to_q15(const float codeword_float[], int16_t codeword_q15[], int length);

#ifdef __cplusplus
}
#endif

#endif // _INCLUDE_LDPC_OPT_H_
