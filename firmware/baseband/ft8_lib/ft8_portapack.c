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

// Cosine lookup table for Hamming window (256 entries = 1KB, interpolated)
static float cos_lut[256];

// Fast cosine using lookup table with linear interpolation
static float fast_cos(float x) {
    // Normalize x to [0, 1] range (0 to 2*pi -> 0 to 1)
    float norm = x / (2.0f * 3.14159265f);
    norm = norm - (int)norm;  // Keep fractional part
    if (norm < 0.0f) norm += 1.0f;

    // Map to LUT index (0-255)
    float idx_f = norm * 255.0f;
    int idx = (int)idx_f;
    float frac = idx_f - idx;

    // Linear interpolation between two LUT values
    float v0 = cos_lut[idx];
    float v1 = cos_lut[(idx + 1) & 0xFF];  // Wrap around
    return v0 + (v1 - v0) * frac;
}

// Compact Radix-2 FFT for 2048 points
static void radix2_fft(float* d, int n) {
    // Bit-reversal
    for (int i = 1, j = 0; i < n; i++) {
        for (int k = n >> 1; k > (j ^= k); k >>= 1);
        if (j > i) {
            float tr = d[i * 2], ti = d[i * 2 + 1];
            d[i * 2] = d[j * 2]; d[i * 2 + 1] = d[j * 2 + 1];
            d[j * 2] = tr; d[j * 2 + 1] = ti;
        }
    }
    // Cooley-Tukey butterfly
    for (int s = 1; s <= 11; s++) {
        int m = 1 << s, m2 = m >> 1;
        float a = -6.283185307f / m, wr = cosf(a), wi = sinf(a);
        for (int k = 0; k < n; k += m) {
            float w_r = 1.0f, w_i = 0.0f;
            for (int j = 0; j < m2; j++) {
                int ti = (k + j + m2) * 2, ui = (k + j) * 2;
                float tr = w_r * d[ti] - w_i * d[ti + 1];
                float t_i = w_r * d[ti + 1] + w_i * d[ti];
                float u_r = d[ui], u_i = d[ui + 1];
                d[ui] = u_r + tr; d[ui + 1] = u_i + t_i;
                d[ti] = u_r - tr; d[ti + 1] = u_i - t_i;
                float wnr = w_r * wr - w_i * wi;
                w_i = w_r * wi + w_i * wr;
                w_r = wnr;
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

    // Default configuration (like WSJT-X: sensitive but with overflow protection)
    state->min_score_threshold = 30;  // Low threshold for weak signals
    state->skip_count = 0;  // Initialize skip counter

    // Initialize cosine lookup table for fast windowing
    const float pi = 3.14159265f;
    for (int i = 0; i < 256; i++) {
        cos_lut[i] = cosf(2.0f * pi * i / 256.0f);
    }

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

    // Ensure we have at least one FT8 symbol worth of samples (1920)
    // We'll zero-pad to FT8_FFT_SIZE (2048) if needed
    if (buffer_size < FT8_SAMPLES_PER_SYMBOL) {
        return false;
    }

    // Copy audio to FFT input buffer with Hamming window using fast LUT
    size_t copy_size = (buffer_size < FT8_FFT_SIZE) ? buffer_size : FT8_FFT_SIZE;
    const float pi = 3.14159265f;

    // DEBUG: Track peak audio sample
    float audio_peak = 0.0f;
    for (size_t i = 0; i < copy_size; i++) {
        float abs_val = (audio_buffer[i] < 0.0f) ? -audio_buffer[i] : audio_buffer[i];
        if (abs_val > audio_peak) audio_peak = abs_val;

        // Hamming window: 0.54 - 0.46 * cos(2*pi*i/N)
        float window = 0.54f - 0.46f * fast_cos(2.0f * pi * i / FT8_FFT_SIZE);
        fft_input[i] = audio_buffer[i] * window;
    }
    state->debug_audio_peak = audio_peak;  // Store for debugging

    // Zero-pad if needed
    for (size_t i = copy_size; i < FT8_FFT_SIZE; i++) {
        fft_input[i] = 0.0f;
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

    // Calculate power spectrum (r² + i²) and store in waterfall
    // Using power instead of magnitude to avoid slow sqrtf() and log10f()
    int block_offset = state->waterfall.num_blocks * state->waterfall.block_stride;

    // FFT normalization: divide by N (not N²) for power spectrum
    const float fft_norm = 1.0f / FT8_FFT_SIZE;  // 1 / 2048 = ~4.88e-4

    // Adaptive normalization for FT8 decoder (optimized: no power recalculation)
    // This maximizes contrast for signal detection (decoder uses bin differences)

    // Pass 1: Calculate power spectrum and find max AC power (skip DC bin 0)
    float powers[FT8_NUM_BINS];  // 200 floats = 800 bytes stack
    float fft_power_peak = 0.0f;
    float dc_power = 0.0f;
    float ac_power_peak = 0.0f;

    for (int bin = 0; bin < FT8_NUM_BINS && bin < FT8_FFT_SIZE / 2; bin++) {
        float r = fft_output[bin * 2], im = fft_output[bin * 2 + 1];
        float power = (r * r + im * im) * fft_norm;
        powers[bin] = power;

        if (power > fft_power_peak) fft_power_peak = power;

        if (bin == 0) {
            dc_power = power;
        } else {
            if (power > ac_power_peak) ac_power_peak = power;
        }
    }

    // Pass 2: Convert to logarithmic scale with adaptive dynamic range
    // Use peak as ceiling (maps to 127), noise floor 60dB below peak
    float peak_power_db = 10.0f * log10f(fft_power_peak + 1e-10f);

    // Use peak as ceiling (guarantee max waterfall = 127 at peak)
    // Minimum ceiling at -20dB to avoid noise amplification on weak signals
    float signal_ceiling_db = (peak_power_db > -20.0f) ? peak_power_db : -20.0f;

    // Noise floor is 60dB below ceiling (adaptive dynamic range)
    const float noise_floor_db = signal_ceiling_db - 60.0f;
    const float range_db = 60.0f;  // Always 60dB range

    for (int bin = 0; bin < FT8_NUM_BINS && bin < FT8_FFT_SIZE / 2; bin++) {
        int m;

        // Convert power to dB scale
        float power_db = 10.0f * log10f(powers[bin] + 1e-10f);

        // Normalize to 0-1 range using adaptive ceiling
        float normalized = (power_db - noise_floor_db) / range_db;

        // Clamp to valid range
        if (normalized < 0.0f) normalized = 0.0f;
        if (normalized > 1.0f) normalized = 1.0f;

        // Scale to 0-127 for display
        m = (int)(normalized * 127.0f);

        // Final safety clamp
        m = (m < 0) ? 0 : (m > 127) ? 127 : m;
        state->waterfall.mag[block_offset + bin] = (WF_ELEM_T)m;
    }

    state->debug_fft_power = fft_power_peak;  // Store for debugging
    state->debug_dc_power = dc_power;
    state->debug_ac_power = ac_power_peak;

    state->waterfall.num_blocks++;
    state->debug_blocks_written = state->waterfall.num_blocks;

    // Return true if slot is complete
    return (state->waterfall.num_blocks >= FT8_WATERFALL_BLOCKS);
}

// Decode current waterfall
int ft8_portapack_decode(ft8_decoder_state_t* state) {
    if (!state || !state->initialized) return 0;
    if (state->waterfall.num_blocks < FT8_WATERFALL_BLOCKS) return 0;

    // DEBUG: Stage 0 - Start of decode
    state->debug_stage = 0;
    state->debug_value = state->waterfall.num_blocks;

    state->decoding_active = true;
    state->num_messages = 0;

    // Calculate max magnitude for debugging, SKIP bin 0 (DC offset) in each block
    int max_mag = 0;
    int nonzero_count = 0;
    int total_elements = state->waterfall.num_blocks * state->waterfall.block_stride;

    // DEBUG: Save waterfall dimensions to diagnose total_elements issue
    state->debug_num_blocks = state->waterfall.num_blocks;
    state->debug_block_stride = state->waterfall.block_stride;
    state->debug_total_elements = total_elements;

    for (int block = 0; block < state->waterfall.num_blocks; block++) {
        int block_start = block * state->waterfall.block_stride;
        // Skip bin 0 (DC), start from bin 1
        for (int bin = 1; bin < state->waterfall.block_stride; bin++) {
            int idx = block_start + bin;
            WF_ELEM_T mag = state->waterfall.mag[idx];
            if (mag > 0) nonzero_count++;
            if (mag > max_mag) max_mag = mag;
        }
    }
    state->max_magnitude = max_mag;
    state->debug_nonzero_bins = nonzero_count;

    // DEBUG: Collect concrete waterfall samples to diagnose MAG=0
    state->debug_mag_sample_0 = (total_elements > 0) ? state->waterfall.mag[0] : -1;
    state->debug_mag_sample_1 = (total_elements > 1) ? state->waterfall.mag[1] : -1;
    state->debug_mag_sample_100 = (total_elements > 100) ? state->waterfall.mag[100] : -1;

    // Find first nonzero element
    state->debug_mag_first_nonzero = -1;
    state->debug_mag_first_nonzero_idx = -1;
    for (int i = 0; i < total_elements; i++) {
        if (state->waterfall.mag[i] > 0) {
            state->debug_mag_first_nonzero = state->waterfall.mag[i];
            state->debug_mag_first_nonzero_idx = i;
            break;
        }
    }

    // DEBUG: Stage 1 - Before find_candidates
    state->debug_stage = 1;
    state->debug_value = max_mag;

    // Find candidates using configurable threshold
    state->num_candidates = ftx_find_candidates(&state->waterfall,
                                                  FT8_MAX_CANDIDATES,
                                                  state->candidates,
                                                  state->min_score_threshold);  // Adjustable: higher = fewer false positives

    // Overflow protection: Max candidates reached = likely noise
    // ftx_find_candidates returns up to FT8_MAX_CANDIDATES (15)
    // If we got exactly 15, there were probably more that got cut off
    // But we still decode the top candidates we found (don't discard them)
    if (state->num_candidates >= FT8_MAX_CANDIDATES) {
        state->skip_count++;  // Just count the overflow for statistics
        // Continue with decoding the 15 best candidates
        // Don't zero out num_candidates or return early
    }

    // DEBUG: Stage 2 - After find_candidates, before loop
    state->debug_stage = 2;
    state->debug_value = state->num_candidates;

    // Decode candidates
    for (int i = 0; i < state->num_candidates && state->num_messages < FT8_MAX_MESSAGES; i++) {
        // DEBUG: Stage 3+ - Decoding candidate N
        state->debug_stage = 3 + i;
        state->debug_value = (int)state->candidates[i].score;

        ftx_message_t msg;
        ftx_decode_status_t st;
        if (ftx_decode_candidate(&state->waterfall, &state->candidates[i],
                                  FT8_LDPC_ITERATIONS, &msg, &st) && st.ldpc_errors == 0) {
            state->messages[state->num_messages++] = msg;
            state->decode_count++;
        }
    }

    // DEBUG: Stage 100 - Completed successfully
    state->debug_stage = 100;
    state->debug_value = state->num_messages;

    state->decoding_active = false;
    state->slot_count++;

    return state->num_messages;
}

void ft8_portapack_reset_slot(ft8_decoder_state_t* state) {
    if (!state) return;
    state->waterfall.num_blocks = state->num_candidates = state->num_messages = 0;
    memset(state->waterfall.mag, 0, FT8_WATERFALL_SIZE);
}

void ft8_portapack_set_min_score(ft8_decoder_state_t* state, int threshold) {
    if (!state) return;
    // Clamp to reasonable range (30-150)
    if (threshold < 30) threshold = 30;
    if (threshold > 150) threshold = 150;
    state->min_score_threshold = threshold;
}
