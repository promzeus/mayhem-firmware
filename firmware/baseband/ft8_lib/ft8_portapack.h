/*
 * FT8 Portapack Adapter
 * Bridges ft8_lib with Portapack Mayhem baseband infrastructure
 *
 * Key adaptations:
 * - Uses Goertzel algorithm for exact 6.25 Hz bin spacing (matches FT8 tone spacing)
 * - Incremental sample processing (no burst computation)
 * - Optimized for Portapack's 96KB RAM (M4 core)
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
#define FT8_SLOT_DURATION      12.64f    // 12.64 seconds per slot
#define FT8_NUM_SYMBOLS        79        // Total symbols in FT8 message
#define FT8_SYNC_SYMBOLS       7         // Costas sync symbols
#define FT8_TONE_SPACING       6.25f     // Hz between tones
#define FT8_SYMBOL_RATE        6.25f     // Symbols per second

// Audio processing parameters
// Input is 24kHz, software decimation 2:1 gives 12kHz for Goertzel
#define FT8_SAMPLE_RATE        12000     // Goertzel sample rate
#define FT8_SAMPLES_PER_SYMBOL 1920      // 160ms * 12kHz = 1920 samples per symbol
// DFT size = samples per symbol = 1920
// This gives bin spacing = 12000/1920 = 6.25 Hz — EXACTLY matches FT8 tone spacing
#define FT8_FFT_SIZE           FT8_SAMPLES_PER_SYMBOL

// Frequency range (audio frequencies after USB demodulation)
#define FT8_FREQ_MIN           200.0f    // Minimum frequency (Hz)
#define FT8_FREQ_MAX           1450.0f   // Maximum frequency (Hz)
#define FT8_FREQ_MIN_BIN       32        // 200 Hz / 6.25 Hz = bin 32
#define FT8_NUM_BINS           200       // Frequency bins covering 200-1450 Hz

// Memory optimization
#define FT8_MAX_CANDIDATES     15        // Maximum decode candidates
#define FT8_MAX_MESSAGES       10        // Maximum messages per slot
#define FT8_LDPC_ITERATIONS    10        // LDPC iterations

// Waterfall buffer configuration
// For one FT8 slot: 79 symbols x 160ms = 12.64 seconds
#define FT8_WATERFALL_BLOCKS   79        // Exactly one FT8 slot
#define FT8_WATERFALL_TIME_OSR 1         // No time oversampling
#define FT8_WATERFALL_FREQ_OSR 1         // No frequency oversampling

// Calculate waterfall size: 79 * 1 * 1 * 200 = 15,800 bytes (uint8_t)
#define FT8_WATERFALL_SIZE (FT8_WATERFALL_BLOCKS * FT8_WATERFALL_TIME_OSR * \
                            FT8_WATERFALL_FREQ_OSR * FT8_NUM_BINS)

// FT8 decoder state
typedef struct {
    // Waterfall data
    ftx_waterfall_t waterfall;
    WF_ELEM_T* waterfall_data;

    // Decode candidates
    ftx_candidate_t candidates[FT8_MAX_CANDIDATES];
    int num_candidates;

    // Decoded messages
    ftx_message_t messages[FT8_MAX_MESSAGES];
    int16_t message_scores[FT8_MAX_MESSAGES];  // Sync score per decoded message
    int num_messages;

    // Statistics
    uint32_t slot_count;
    uint32_t decode_count;
    uint32_t error_count;
    uint32_t skip_count;

    // Configuration
    int min_score_threshold;

    // Status flags
    bool initialized;
    bool decoding_active;

    // Goertzel incremental state
    int samples_in_symbol;  // Count of samples processed in current symbol

    // Debug (minimal - keep only useful fields)
    float debug_audio_peak;
    int debug_blocks_written;
    int max_magnitude;

} ft8_decoder_state_t;

#ifdef __cplusplus
extern "C" {
#endif

// Initialize FT8 decoder (precomputes Goertzel coefficients)
bool ft8_portapack_init(ft8_decoder_state_t* state);

// Free FT8 decoder resources
void ft8_portapack_free(ft8_decoder_state_t* state);

// Feed one audio sample at 12kHz to the Goertzel filters (incremental processing)
// Returns true when a complete FT8 slot (79 symbols) has been collected
bool ft8_portapack_feed_sample(ft8_decoder_state_t* state, float sample);

// Decode current waterfall data
// Returns number of successfully decoded messages
int ft8_portapack_decode(ft8_decoder_state_t* state);

// Reset decoder for new slot
void ft8_portapack_reset_slot(ft8_decoder_state_t* state);

// Set minimum score threshold for candidate detection
void ft8_portapack_set_min_score(ft8_decoder_state_t* state, int threshold);

#ifdef __cplusplus
}
#endif

#endif // _FT8_PORTAPACK_H_
