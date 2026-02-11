/*
 * FT8 Portapack Implementation
 * Core integration layer between ft8_lib and Portapack Mayhem
 *
 * Uses Goertzel algorithm instead of FFT for exact 6.25 Hz bin spacing.
 * This is critical: ft8_lib's decoder assumes 1 waterfall bin = 1 FT8 tone (6.25 Hz).
 * A 2048-point FFT at 12kHz gives 5.86 Hz/bin which breaks sync detection.
 * Goertzel with N=1920 at 12kHz gives exactly 12000/1920 = 6.25 Hz/bin.
 */

#include "ft8_portapack.h"
#include "decode.h"
#include "ldpc_opt.h"
#include "constants.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>

// --- Static buffers (BSS section) ---

// Waterfall magnitude data: 79 * 200 = 15,800 bytes
static WF_ELEM_T waterfall_buffer[FT8_WATERFALL_SIZE];

// Goertzel precomputed coefficients: 2*cos(2*pi*k/N) for each bin
// 200 bins * 4 bytes = 800 bytes
static float goertzel_coeff[FT8_NUM_BINS];

// Goertzel filter state (persistent across samples within one symbol)
// 200 bins * 4 bytes * 2 = 1,600 bytes
static float goertzel_s1[FT8_NUM_BINS];
static float goertzel_s2[FT8_NUM_BINS];

// Total static RAM: 15,800 + 800 + 1,600 = 18,200 bytes
// (vs old FFT approach: 15,800 + 8,192 + 16,384 + 1,024 = 41,400 bytes)
// Savings: ~23 KB RAM

// --- Internal helpers ---

// Finish one symbol: compute Goertzel power, convert to log scale, store in waterfall
static void finish_goertzel_symbol(ft8_decoder_state_t* state) {
    int block_offset = state->waterfall.num_blocks * state->waterfall.block_stride;

    // Pass 1: Compute power for each bin, find peak (skip log10f for peak finding)
    float peak_power = 0.0f;

    // Temporary power storage on stack (200 * 4 = 800 bytes, fine with 16KB stack)
    float powers[FT8_NUM_BINS];

    for (int b = 0; b < FT8_NUM_BINS; b++) {
        float s1 = goertzel_s1[b];
        float s2 = goertzel_s2[b];
        // Goertzel power: |X[k]|^2 = s1^2 + s2^2 - coeff*s1*s2
        float power = s1 * s1 + s2 * s2 - goertzel_coeff[b] * s1 * s2;
        // Normalize by N (not N^2, since we want power proportional to signal amplitude squared)
        power /= (float)FT8_SAMPLES_PER_SYMBOL;
        if (power < 0.0f) power = 0.0f;  // Numerical safety
        powers[b] = power;
        if (power > peak_power) peak_power = power;
    }

    // Pass 2: Convert to log scale with adaptive dynamic range
    // Use peak as ceiling, 60dB range below peak
    float peak_db = 10.0f * log10f(peak_power + 1e-10f);
    float ceiling_db = (peak_db > -20.0f) ? peak_db : -20.0f;
    float floor_db = ceiling_db - 60.0f;

    for (int b = 0; b < FT8_NUM_BINS; b++) {
        float power_db = 10.0f * log10f(powers[b] + 1e-10f);
        float normalized = (power_db - floor_db) / 60.0f;
        if (normalized < 0.0f) normalized = 0.0f;
        if (normalized > 1.0f) normalized = 1.0f;
        int m = (int)(normalized * 127.0f);
        if (m < 0) m = 0;
        if (m > 127) m = 127;
        state->waterfall.mag[block_offset + b] = (WF_ELEM_T)m;
    }

    // Update debug info
    state->debug_blocks_written = state->waterfall.num_blocks + 1;

    // Reset Goertzel state for next symbol
    memset(goertzel_s1, 0, sizeof(goertzel_s1));
    memset(goertzel_s2, 0, sizeof(goertzel_s2));
    state->samples_in_symbol = 0;

    state->waterfall.num_blocks++;
}

// --- Public API ---

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

    // Default configuration
    state->min_score_threshold = 30;
    state->skip_count = 0;
    state->samples_in_symbol = 0;

    // Precompute Goertzel coefficients for bins [FT8_FREQ_MIN_BIN .. FT8_FREQ_MIN_BIN + FT8_NUM_BINS - 1]
    // Each bin k corresponds to frequency k * 6.25 Hz
    // Goertzel coefficient = 2 * cos(2 * pi * k / N) where N = 1920
    const float two_pi_over_n = 2.0f * 3.14159265358979f / (float)FT8_SAMPLES_PER_SYMBOL;
    for (int b = 0; b < FT8_NUM_BINS; b++) {
        int k = FT8_FREQ_MIN_BIN + b;  // DFT bin index (e.g. 32..231)
        goertzel_coeff[b] = 2.0f * cosf(two_pi_over_n * k);
    }

    // Clear Goertzel state
    memset(goertzel_s1, 0, sizeof(goertzel_s1));
    memset(goertzel_s2, 0, sizeof(goertzel_s2));

    return true;
}

void ft8_portapack_free(ft8_decoder_state_t* state) {
    if (!state) return;
    state->initialized = false;
    state->decoding_active = false;
}

bool ft8_portapack_feed_sample(ft8_decoder_state_t* state, float sample) {
    if (!state || !state->initialized) return false;

    // Slot already complete — signal caller to decode
    if (state->waterfall.num_blocks >= FT8_WATERFALL_BLOCKS) {
        return true;
    }

    // NaN/Inf protection
    if (sample != sample) sample = 0.0f;

    // Track peak for debug
    float abs_s = (sample < 0.0f) ? -sample : sample;
    if (abs_s > state->debug_audio_peak) state->debug_audio_peak = abs_s;

    // Update all Goertzel filters with this sample
    // Per sample: 200 bins * ~5 FP ops = ~1000 ops
    // At 12kHz: 12M FLOPS = ~6% of 204 MHz M4 — acceptable
    for (int b = 0; b < FT8_NUM_BINS; b++) {
        float s0 = sample + goertzel_coeff[b] * goertzel_s1[b] - goertzel_s2[b];
        goertzel_s2[b] = goertzel_s1[b];
        goertzel_s1[b] = s0;
    }

    state->samples_in_symbol++;

    // Check if symbol is complete (1920 samples = 160ms at 12kHz)
    if (state->samples_in_symbol >= FT8_SAMPLES_PER_SYMBOL) {
        finish_goertzel_symbol(state);
        return (state->waterfall.num_blocks >= FT8_WATERFALL_BLOCKS);
    }

    return false;
}

int ft8_portapack_decode(ft8_decoder_state_t* state) {
    if (!state || !state->initialized) return 0;
    if (state->waterfall.num_blocks < FT8_WATERFALL_BLOCKS) return 0;

    state->decoding_active = true;
    state->num_messages = 0;

    // Calculate max magnitude for debug
    int max_mag = 0;
    for (int block = 0; block < state->waterfall.num_blocks; block++) {
        int base = block * state->waterfall.block_stride;
        for (int bin = 1; bin < state->waterfall.block_stride; bin++) {
            WF_ELEM_T mag = state->waterfall.mag[base + bin];
            if (mag > max_mag) max_mag = mag;
        }
    }
    state->max_magnitude = max_mag;

    // Find candidates using configurable threshold
    state->num_candidates = ftx_find_candidates(&state->waterfall,
                                                  FT8_MAX_CANDIDATES,
                                                  state->candidates,
                                                  state->min_score_threshold);

    if (state->num_candidates >= FT8_MAX_CANDIDATES) {
        state->skip_count++;
    }

    // Decode candidates
    for (int i = 0; i < state->num_candidates && state->num_messages < FT8_MAX_MESSAGES; i++) {
        ftx_message_t msg;
        ftx_decode_status_t st;
        if (ftx_decode_candidate(&state->waterfall, &state->candidates[i],
                                  FT8_LDPC_ITERATIONS, &msg, &st) && st.ldpc_errors == 0) {
            state->messages[state->num_messages] = msg;
            state->message_scores[state->num_messages] = state->candidates[i].score;
            state->num_messages++;
            state->decode_count++;
        }
    }

    state->decoding_active = false;
    state->slot_count++;

    return state->num_messages;
}

void ft8_portapack_reset_slot(ft8_decoder_state_t* state) {
    if (!state) return;
    state->waterfall.num_blocks = 0;
    state->num_candidates = 0;
    state->num_messages = 0;
    state->debug_audio_peak = 0.0f;
    memset(state->waterfall.mag, 0, FT8_WATERFALL_SIZE);
}

void ft8_portapack_set_min_score(ft8_decoder_state_t* state, int threshold) {
    if (!state) return;
    if (threshold < 10) threshold = 10;
    if (threshold > 150) threshold = 150;
    state->min_score_threshold = threshold;
}
