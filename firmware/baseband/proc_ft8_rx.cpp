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

    // Configure audio output with 24kHz HPF/deemph filters (no squelch)
    audio_output.configure(audio_24k_hpf_300hz_config, audio_24k_deemph_300_6_config, 0);

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
    // Process FT8 BEFORE AGC - FFT needs clean uncompressed signal
    process_ft8_audio(audio);
    // Apply AGC for audio output only (headphones)
    audio_compressor.execute_in_place(audio);
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

        // Software gain stage: Moderate gain to avoid float overflow in FFT
        // AFSK uses 256x with int32, but FFT with float needs lower gain to prevent NaN/Inf
        constexpr float FT8_INPUT_GAIN = 20.0f;  // Compromise: higher than 10x, lower than 256x
        float amplified = sample * FT8_INPUT_GAIN;

        // Saturation clipping to prevent FFT overflow
        if (amplified > 20.0f) amplified = 20.0f;
        if (amplified < -20.0f) amplified = -20.0f;

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
    // Calculate waterfall statistics manually
    int total_elements = decoder_state.waterfall.num_blocks * decoder_state.waterfall.block_stride;
    int sum_mag = 0;
    int nonzero_count = 0;
    int max_mag = 0;

    for (int i = 0; i < total_elements; i++) {
        int mag = decoder_state.waterfall.mag[i];
        sum_mag += mag;
        if (mag > 0) nonzero_count++;
        if (mag > max_mag) max_mag = mag;
    }

    int avg_mag = (total_elements > 0) ? (sum_mag / total_elements) : 0;

    // Sample waterfall values at specific positions for debugging
    int mag100 = (total_elements > 100) ? decoder_state.waterfall.mag[100] : 0;
    int mag500 = (total_elements > 500) ? decoder_state.waterfall.mag[500] : 0;

    // DECODER DISABLED FOR TESTING - Only output statistics
    // int num_decoded = ft8_portapack_decode(&decoder_state);
    int num_decoded = 0;

    // Debug 1: Max magnitude and average
    FT8PacketMessage mag_stats("MAX", "AVG", "", max_mag, avg_mag);
    shared_memory.application_queue.push(mag_stats);

    // Debug 2: Nonzero bins and decoded count
    FT8PacketMessage nz_debug("NZ", "DCD", "", nonzero_count, num_decoded);
    shared_memory.application_queue.push(nz_debug);

    // Debug 3: Sample waterfall values
    FT8PacketMessage samples_debug("M100", "M500", "", mag100, mag500);
    shared_memory.application_queue.push(samples_debug);

    // Debug 4: Candidates (always 0 when decoder disabled)
    FT8PacketMessage result("CND", "MAG", "", 0, max_mag);
    shared_memory.application_queue.push(result);

    // If decoded messages, send them (never happens when decoder disabled)
    if (num_decoded > 0) {
        send_ft8_messages();
    }

    ft8_portapack_reset_slot(&decoder_state);
}

void FT8RxProcessor::send_ft8_messages() {
    for (int i = 0; i < decoder_state.num_messages; i++) {
        FT8PacketMessage packet("FT8", "", "", 0, slot_count % 2);
        shared_memory.application_queue.push(packet);
    }
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

int main() {
    EventDispatcher event_dispatcher{std::make_unique<FT8RxProcessor>()};
    event_dispatcher.run();
    return 0;
}
