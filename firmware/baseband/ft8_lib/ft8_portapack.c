/*
 * FT8 Portapack Implementation
 * Core integration layer between ft8_lib and Portapack Mayhem
 */

#include "ft8_portapack.h"
#include "decode.h"
#include "ldpc_opt.h"
#include "constants.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>

// ARM CMSIS-DSP not available in Portapack baseband environment
// We'll use the fallback DFT implementation instead
// #ifdef LPC43XX_M4
// #include "arm_math.h"
// #include "arm_const_structs.h"
// #endif

// Static buffers to avoid dynamic allocation
// Placed in regular RAM (ahb_ram section not available/too small)
static WF_ELEM_T waterfall_buffer[FT8_WATERFALL_SIZE];
static float fft_input[FT8_FFT_SIZE];
static float fft_output[FT8_FFT_SIZE * 2];

// Radix-2 FFT helper functions

// Bit-reversal permutation for FFT
static inline unsigned int reverse_bits(unsigned int x, int log2n) {
    unsigned int result = 0;
    for (int i = 0; i < log2n; i++) {
        result = (result << 1) | (x & 1);
        x >>= 1;
    }
    return result;
}

// In-place Radix-2 Cooley-Tukey FFT
// Input: complex array in fft_output (interleaved real/imag)
// Size: FT8_FFT_SIZE must be power of 2
static void radix2_fft(float* data, int n) {
    const int log2n = 11;  // log2(2048) = 11

    // Bit-reversal permutation
    for (int i = 0; i < n; i++) {
        int j = reverse_bits(i, log2n);
        if (j > i) {
            // Swap complex numbers
            float temp_real = data[i * 2];
            float temp_imag = data[i * 2 + 1];
            data[i * 2] = data[j * 2];
            data[i * 2 + 1] = data[j * 2 + 1];
            data[j * 2] = temp_real;
            data[j * 2 + 1] = temp_imag;
        }
    }

    // Cooley-Tukey FFT
    for (int s = 1; s <= log2n; s++) {
        int m = 1 << s;  // 2^s
        int m2 = m >> 1;  // m/2

        // Twiddle factor: e^(-2πi/m)
        float theta = -2.0f * 3.14159265358979323846f / m;
        float wm_real = cosf(theta);
        float wm_imag = sinf(theta);

        for (int k = 0; k < n; k += m) {
            float w_real = 1.0f;
            float w_imag = 0.0f;

            for (int j = 0; j < m2; j++) {
                int t_idx = (k + j + m2) * 2;
                int u_idx = (k + j) * 2;

                // Complex multiplication: t = w * data[k + j + m/2]
                float t_real = w_real * data[t_idx] - w_imag * data[t_idx + 1];
                float t_imag = w_real * data[t_idx + 1] + w_imag * data[t_idx];

                // Butterfly operation
                float u_real = data[u_idx];
                float u_imag = data[u_idx + 1];

                data[u_idx] = u_real + t_real;
                data[u_idx + 1] = u_imag + t_imag;
                data[t_idx] = u_real - t_real;
                data[t_idx + 1] = u_imag - t_imag;

                // Update twiddle factor: w *= wm
                float w_new_real = w_real * wm_real - w_imag * wm_imag;
                float w_new_imag = w_real * wm_imag + w_imag * wm_real;
                w_real = w_new_real;
                w_imag = w_new_imag;
            }
        }
    }
}

// Initialize FT8 decoder
bool ft8_portapack_init(ft8_decoder_state_t* state) {
    if (!state) return false;

    memset(state, 0, sizeof(ft8_decoder_state_t));

    // Setup waterfall structure
    state->waterfall.max_blocks = FT8_WATERFALL_BLOCKS;
    state->waterfall.num_blocks = 0;
    state->waterfall.num_bins = FT8_NUM_BINS;
    state->waterfall.time_osr = FT8_WATERFALL_TIME_OSR;
    state->waterfall.freq_osr = FT8_WATERFALL_FREQ_OSR;
    state->waterfall.mag = waterfall_buffer;
    state->waterfall.block_stride = FT8_WATERFALL_TIME_OSR * FT8_WATERFALL_FREQ_OSR * FT8_NUM_BINS;
    state->waterfall.protocol = FTX_PROTOCOL_FT8;

    state->waterfall_data = waterfall_buffer;
    state->initialized = true;
    state->decoding_active = false;

    return true;
}

// Free resources
void ft8_portapack_free(ft8_decoder_state_t* state) {
    if (!state) return;

    state->initialized = false;
    state->decoding_active = false;
    // Static buffers don't need freeing
}

// Process audio buffer and compute FFT magnitudes
bool ft8_portapack_process_audio(ft8_decoder_state_t* state,
                                  const float* audio_buffer,
                                  size_t buffer_size) {
    if (!state || !state->initialized) return false;
    if (!audio_buffer || buffer_size == 0) return false;

    // Check if we have space for another block
    if (state->waterfall.num_blocks >= FT8_WATERFALL_BLOCKS) {
        // Slot is complete
        return true;
    }

    // Ensure we have enough samples for FFT
    if (buffer_size < FT8_FFT_SIZE) {
        return false;
    }

    // Copy audio to FFT input buffer with windowing (Hamming)
    for (size_t i = 0; i < FT8_FFT_SIZE && i < buffer_size; i++) {
        // Hamming window: 0.54 - 0.46 * cos(2*pi*i/N)
        float window = 0.54f - 0.46f * cosf(2.0f * 3.14159265f * i / FT8_FFT_SIZE);
        fft_input[i] = audio_buffer[i] * window;
    }

    // Perform FFT: Convert real input to complex, then run Radix-2 FFT
    // Copy windowed real audio to complex buffer (real part only, imag = 0)
    for (int i = 0; i < FT8_FFT_SIZE; i++) {
        fft_output[i * 2] = fft_input[i];      // Real part
        fft_output[i * 2 + 1] = 0.0f;          // Imaginary part = 0
    }

    // Execute optimized Radix-2 FFT (O(n log n) instead of O(n²))
    radix2_fft(fft_output, FT8_FFT_SIZE);

    // Output is now in fft_output[]: [Re0, Im0, Re1, Im1, ..., Re2047, Im2047]
    // We only need the first FT8_FFT_SIZE/2 bins (DC to Nyquist)

    // Calculate magnitudes and store in waterfall
    int block_offset = state->waterfall.num_blocks * state->waterfall.block_stride;

    for (int bin = 0; bin < FT8_NUM_BINS && bin < FT8_FFT_SIZE / 2; bin++) {
        float real = fft_output[bin * 2];
        float imag = fft_output[bin * 2 + 1];
        float magnitude = sqrtf(real * real + imag * imag);

        // Convert to log scale and quantize to uint8_t
        // magnitude in dB = 20 * log10(magnitude)
        float magnitude_db = 20.0f * log10f(magnitude + 1e-10f);

        // Map to 0-255 range (assuming -120dB to 0dB)
        int mag_int = (int)((magnitude_db + 120.0f) * 2.0f);
        if (mag_int < 0) mag_int = 0;
        if (mag_int > 255) mag_int = 255;

        state->waterfall.mag[block_offset + bin] = (WF_ELEM_T)mag_int;
    }

    state->waterfall.num_blocks++;

    // Return true if slot is complete
    return (state->waterfall.num_blocks >= FT8_WATERFALL_BLOCKS);
}

// Decode current waterfall
int ft8_portapack_decode(ft8_decoder_state_t* state) {
    if (!state || !state->initialized) return 0;
    if (state->waterfall.num_blocks < FT8_WATERFALL_BLOCKS) return 0;

    state->decoding_active = true;
    state->num_messages = 0;

    // Find candidates
    state->num_candidates = ftx_find_candidates(&state->waterfall,
                                                  FT8_MAX_CANDIDATES,
                                                  state->candidates,
                                                  50);  // min_score threshold

    // Try to decode each candidate
    for (int i = 0; i < state->num_candidates && state->num_messages < FT8_MAX_MESSAGES; i++) {
        ftx_message_t message;
        ftx_decode_status_t status;

        bool success = ftx_decode_candidate(&state->waterfall,
                                              &state->candidates[i],
                                              FT8_LDPC_ITERATIONS,
                                              &message,
                                              &status);

        if (success && status.ldpc_errors == 0) {
            // Valid decode
            state->messages[state->num_messages] = message;
            state->num_messages++;
            state->decode_count++;
        }
    }

    state->decoding_active = false;
    state->slot_count++;

    return state->num_messages;
}

// Get decoded message
const ftx_message_t* ft8_portapack_get_message(ft8_decoder_state_t* state, int index) {
    if (!state || index < 0 || index >= state->num_messages) {
        return NULL;
    }
    return &state->messages[index];
}

// Reset for new slot
void ft8_portapack_reset_slot(ft8_decoder_state_t* state) {
    if (!state) return;

    state->waterfall.num_blocks = 0;
    state->num_candidates = 0;
    state->num_messages = 0;

    // Clear waterfall buffer
    memset(state->waterfall.mag, 0, FT8_WATERFALL_SIZE);
}
