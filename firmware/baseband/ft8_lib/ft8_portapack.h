/*
 * FT8 Portapack Adapter
 * Bridges ft8_lib with Portapack Mayhem baseband infrastructure
 *
 * Key adaptations:
 * - Uses CMSIS-DSP arm_rfft_fast_f32 instead of KISSFFT
 * - Optimized LDPC decoder (int16_t, 58KB instead of 115KB)
 * - Memory-mapped buffers in AHB RAM
 * - Integration with existing Portapack DSP chain
 */

#ifndef _FT8_PORTAPACK_H_
#define _FT8_PORTAPACK_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "decode.h"
#include "message.h"
#include "ldpc_opt.h"

// FT8 timing constants
// Note: FT8_SYMBOL_PERIOD is defined in constants.h
#define FT8_SLOT_DURATION      12.64f    // 12.64 seconds per slot
#define FT8_NUM_SYMBOLS        79        // Total symbols in FT8 message
#define FT8_SYNC_SYMBOLS       7         // Costas sync symbols
#define FT8_TONE_SPACING       6.25f     // Hz
#define FT8_SYMBOL_RATE        6.25f     // Symbols per second

// Audio processing parameters
#define FT8_SAMPLE_RATE        12000     // Decimated sample rate for FT8 (from 2457600)
#define FT8_SAMPLES_PER_SYMBOL 1920      // 160ms * 12kHz = 1920 samples per symbol
#define FT8_FFT_SIZE           2048      // FFT size (power of 2 for Radix-2 FFT, 5.86 Hz/bin)
#define FT8_NUM_BINS           200       // Frequency bins (REDUCED from 400 to save RAM)
#define FT8_FREQ_MIN           200.0f    // Minimum frequency (Hz)
#define FT8_FREQ_MAX           1450.0f   // Maximum frequency (Hz) - reduced range

// Memory optimization
#define FT8_MAX_CANDIDATES     50        // Maximum decode candidates
#define FT8_MAX_MESSAGES       10        // Maximum messages per slot
#define FT8_LDPC_ITERATIONS    10        // LDPC iterations (reduced for stability)

// Waterfall buffer configuration
// For one FT8 slot: 79 symbols × 160ms = 12.64 seconds
// Memory-optimized for Portapack's 96KB RAM limit
#define FT8_WATERFALL_BLOCKS   79        // Exactly one FT8 slot
#define FT8_WATERFALL_TIME_OSR 1         // No time oversampling
#define FT8_WATERFALL_FREQ_OSR 1         // No frequency oversampling (REDUCED from 2)

// Calculate waterfall size
// blocks * time_osr * freq_osr * num_bins
// 79 * 1 * 1 * 400 = 31,600 bytes (using uint8_t per magnitude)
#define FT8_WATERFALL_SIZE (FT8_WATERFALL_BLOCKS * FT8_WATERFALL_TIME_OSR * \
                            FT8_WATERFALL_FREQ_OSR * FT8_NUM_BINS)

// FT8 decoder state
typedef struct {
    // Waterfall data
    ftx_waterfall_t waterfall;
    WF_ELEM_T* waterfall_data;  // Allocated in AHB RAM

    // Decode candidates
    ftx_candidate_t candidates[FT8_MAX_CANDIDATES];
    int num_candidates;

    // Decoded messages
    ftx_message_t messages[FT8_MAX_MESSAGES];
    int num_messages;

    // Statistics
    uint32_t slot_count;
    uint32_t decode_count;
    uint32_t error_count;
    int max_magnitude;  // For debugging waterfall magnitude

    // Timing
    uint32_t last_decode_time_ms;

    // Status flags
    bool initialized;
    bool decoding_active;

} ft8_decoder_state_t;

#ifdef __cplusplus
extern "C" {
#endif

// Initialize FT8 decoder
bool ft8_portapack_init(ft8_decoder_state_t* state);

// Free FT8 decoder resources
void ft8_portapack_free(ft8_decoder_state_t* state);

// Process audio buffer and update waterfall
// Returns true if a complete slot has been collected
bool ft8_portapack_process_audio(ft8_decoder_state_t* state,
                                  const float* audio_buffer,
                                  size_t buffer_size);

// Decode current waterfall data
// Returns number of successfully decoded messages
int ft8_portapack_decode(ft8_decoder_state_t* state);

// Reset decoder for new slot
void ft8_portapack_reset_slot(ft8_decoder_state_t* state);

#ifdef __cplusplus
}
#endif

#endif // _FT8_PORTAPACK_H_
