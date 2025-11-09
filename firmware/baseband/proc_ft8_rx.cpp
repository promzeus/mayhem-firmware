/*
 * Copyright (C) 2024
 *
 * This file is part of PortaPack.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include "proc_ft8_rx.hpp"
#include "portapack_shared_memory.hpp"
#include "audio_dma.hpp"
#include "dsp_fir_taps.hpp"
#include "dsp_iir_config.hpp"
#include "event_m4.hpp"

FT8RxProcessor::FT8RxProcessor() {
    // Use same filters as AFSK RX (11kHz, 24kHz output) for better data flow
    decim_0.configure(taps_11k0_decim_0.taps);
    decim_1.configure(taps_11k0_decim_1.taps);
    channel_filter.configure(taps_11k0_channel.taps, 2);  // decimation=2 gives 24kHz

    // Configure channel_filter parameters for spectrum display (like AFSK RX)
    constexpr size_t channel_filter_input_fs = decim_1_output_fs;  // 48 kHz
    channel_filter_low_f = taps_11k0_channel.low_frequency_normalized * channel_filter_input_fs;
    channel_filter_high_f = taps_11k0_channel.high_frequency_normalized * channel_filter_input_fs;
    channel_filter_transition = taps_11k0_channel.transition_normalized * channel_filter_input_fs;

    // Configure audio output without processing - let audio pass through clean
    audio_output.configure(false);

    ft8_portapack_init(&decoder_state);

    configured = true;
}

FT8RxProcessor::~FT8RxProcessor() {
    ft8_portapack_free(&decoder_state);
}

void FT8RxProcessor::execute(const buffer_c8_t& buffer) {
    if (!configured) return;

    const auto decim_0_out = decim_0.execute(buffer, dst_buffer);
    const auto decim_1_out = decim_1.execute(decim_0_out, dst_buffer);
    channel_spectrum.feed(decim_1_out, channel_filter_low_f, channel_filter_high_f, channel_filter_transition);
    // No decim_2 - go directly to channel_filter (like AFSK RX)
    const auto channel_out = channel_filter.execute(decim_1_out, dst_buffer);
    feed_channel_stats(channel_out);
    auto audio = demodulate(channel_out);
    // Pre-attenuate audio from SSB demodulator BEFORE FT8 processing
    // User has MAX=127 saturation even with low FT8_INPUT_GAIN - SSB output is too hot!
    for (size_t i = 0; i < audio.count; i++) {
        audio.p[i] *= 0.1f;  // Reduce SSB output by 10x before FT8 decoder
    }
    // Process FT8 first - FFT needs clean unmodified signal
    process_ft8_audio(audio);
    // Apply gain with limiting for headphones (no AGC to save flash space)
    // 4x gain with soft clip prevents distortion
    for (size_t i = 0; i < audio.count; i++) {
        float sample = audio.p[i] * 4.0f;
        // Soft limiting at ±1.0 to prevent clipping
        if (sample > 1.0f) sample = 1.0f;
        if (sample < -1.0f) sample = -1.0f;
        audio.p[i] = sample;
    }
    audio_output.write(audio);
}

buffer_f32_t FT8RxProcessor::demodulate(const buffer_c16_t& channel) {
    return (modulation_ssb == 0) ? demod_am.execute(channel, audio_buffer)
                                  : demod_ssb.execute(channel, audio_buffer);
}

void FT8RxProcessor::process_ft8_audio(const buffer_f32_t& audio) {
    static bool decimate_phase = false;    // Toggle for 24→12 kHz decimation

    for (size_t i = 0; i < audio.count; i++) {
        float sample = audio.p[i];

        // Software decimation 24kHz → 12kHz: take every 2nd sample to save memory
        decimate_phase = !decimate_phase;
        if (!decimate_phase) continue;  // Skip odd samples (simple 2:1 decimation)

        // Software gain stage: EXTREMELY LOW gain prevents clipping
        // 0.1x gain - user has MAX=127 even with 0.25x at LNA=8 VGA=8!
        constexpr float FT8_INPUT_GAIN = 0.1f;  // 20x less than original 2.0
        float amplified = sample * FT8_INPUT_GAIN;

        // Saturation clipping to prevent FFT overflow
        if (amplified > 1.0f) amplified = 1.0f;
        if (amplified < -1.0f) amplified = -1.0f;

        // NaN/Inf protection (critical for FFT stability)
        if (amplified != amplified) amplified = 0.0f;  // NaN check (NaN != NaN is true)

        audio_accumulator[audio_accumulator_pos++] = amplified;

        // Process FFT every 1920 samples (one FT8 symbol = 160ms @ 12kHz)
        if (audio_accumulator_pos >= FT8_SAMPLES_PER_SYMBOL) {
            // Zero-padding to 2048 is handled inside ft8_portapack_process_audio()
            bool slot_complete = ft8_portapack_process_audio(
                &decoder_state, audio_accumulator.data(), audio_accumulator_pos);
            audio_accumulator_pos = 0;

            if (slot_complete) {
                decode_ft8_slot();
                slot_count++;
            }
        }
    }
}

void FT8RxProcessor::decode_ft8_slot() {
    // Run FT8 decoder
    ft8_portapack_decode(&decoder_state);

    // Minimal debug (3 messages) to fit 32KB limit
    // 1. Audio input debug (with clamping for int8_t/uint8_t safety)
    int audio_peak_x100 = (int)(decoder_state.debug_audio_peak * 100.0f);
    int fft_power_x100 = (int)(decoder_state.debug_fft_power * 100.0f);
    // Clamp to valid ranges: snr=int8_t(-128..127), time_slot=uint8_t(0..255)
    if (audio_peak_x100 > 127) audio_peak_x100 = 127;
    if (audio_peak_x100 < -128) audio_peak_x100 = -128;
    if (fft_power_x100 > 255) fft_power_x100 = 255;
    if (fft_power_x100 < 0) fft_power_x100 = 0;
    FT8PacketMessage audio_debug("AUD", "", "", audio_peak_x100, fft_power_x100);
    shared_memory.application_queue.push(audio_debug);

    // 2. Waterfall magnitude stats
    int total = decoder_state.waterfall.num_blocks * decoder_state.waterfall.block_stride;
    int max_mag = 0;
    int nonzero = 0;
    for (int i = 0; i < total; i++) {
        int mag = decoder_state.waterfall.mag[i];
        if (mag > max_mag) max_mag = mag;
        if (mag > 0) nonzero++;
    }
    // Clamp to valid ranges: snr=int8_t(-128..127), time_slot=uint8_t(0..255)
    if (max_mag > 127) max_mag = 127;
    if (max_mag < -128) max_mag = -128;
    if (nonzero > 255) nonzero = 255;
    FT8PacketMessage wf_stats("MAX", "", "", max_mag, nonzero);
    shared_memory.application_queue.push(wf_stats);

    // 3. Candidates stats
    // Clamp to valid ranges: snr=int8_t(-128..127), time_slot=uint8_t(0..255)
    int num_cand = decoder_state.num_candidates;
    int skip_cnt = decoder_state.skip_count;
    if (num_cand > 127) num_cand = 127;
    if (num_cand < -128) num_cand = -128;
    if (skip_cnt > 255) skip_cnt = 255;
    FT8PacketMessage decode_stats("CND", "", "", num_cand, skip_cnt);
    shared_memory.application_queue.push(decode_stats);

    ft8_portapack_reset_slot(&decoder_state);
}


void FT8RxProcessor::on_message(const Message* const message) {
    switch (message->id) {
        case Message::ID::UpdateSpectrum:
        case Message::ID::SpectrumStreamingConfig:
            channel_spectrum.on_message(message);
            break;

        case Message::ID::CaptureConfig:
            capture_config(*reinterpret_cast<const CaptureConfigMessage*>(message));
            break;

        case Message::ID::FT8Configure:
            configure_threshold(*reinterpret_cast<const FT8ConfigureMessage*>(message));
            break;

        default:
            break;
    }
}

void FT8RxProcessor::capture_config(const CaptureConfigMessage& message) {
    if (message.config) {
        audio_output.set_stream(std::make_unique<StreamInput>(message.config));
    } else {
        audio_output.set_stream(nullptr);
    }
}

void FT8RxProcessor::configure_threshold(const FT8ConfigureMessage& message) {
    // Update the FT8 decoder's min_score threshold
    ft8_portapack_set_min_score(&decoder_state, message.threshold);
}

int main() {
    audio::dma::init_audio_out();  // CRITICAL: Initialize audio DMA for headphone output

    EventDispatcher event_dispatcher{std::make_unique<FT8RxProcessor>()};
    event_dispatcher.run();
    return 0;
}
