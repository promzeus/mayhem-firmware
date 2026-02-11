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

#include <cstring>

FT8RxProcessor::FT8RxProcessor() {
    // Use same filters as AFSK RX (11kHz, 24kHz output) for better data flow
    decim_0.configure(taps_11k0_decim_0.taps);
    decim_1.configure(taps_11k0_decim_1.taps);
    channel_filter.configure(taps_11k0_channel.taps, 2);  // decimation=2 gives 24kHz

    // Configure channel_filter parameters for spectrum display
    constexpr size_t channel_filter_input_fs = decim_1_output_fs;  // 48 kHz
    channel_filter_low_f = taps_11k0_channel.low_frequency_normalized * channel_filter_input_fs;
    channel_filter_high_f = taps_11k0_channel.high_frequency_normalized * channel_filter_input_fs;
    channel_filter_transition = taps_11k0_channel.transition_normalized * channel_filter_input_fs;

    // Configure audio output without processing
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
    const auto channel_out = channel_filter.execute(decim_1_out, dst_buffer);
    feed_channel_stats(channel_out);
    auto audio = demodulate(channel_out);

    // Feed audio to FT8 decoder BEFORE any gain/clipping for headphones
    process_ft8_audio(audio);

    // Apply 2x gain for headphones and recording
    for (size_t i = 0; i < audio.count; i++) {
        float sample = audio.p[i] * 2.0f;
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
    // Software 2:1 decimation: 24kHz → 12kHz
    // The channel filter (11kHz passband) already bandlimits adequately
    // so aliasing from the decimation is minimal in the FT8 band (200-3000 Hz)
    static bool decimate_phase = false;

    for (size_t i = 0; i < audio.count; i++) {
        float sample = audio.p[i];

        // 2:1 decimation — skip every other sample
        decimate_phase = !decimate_phase;
        if (!decimate_phase) continue;

        // NaN/Inf protection
        if (sample != sample) sample = 0.0f;

        // Feed sample directly to Goertzel filters (incremental, no accumulator needed)
        // Returns true when 79 symbols (one FT8 slot = 12.64s) are complete
        bool slot_complete = ft8_portapack_feed_sample(&decoder_state, sample);

        if (slot_complete) {
            decode_ft8_slot();
            slot_count++;
        }
    }
}

void FT8RxProcessor::decode_ft8_slot() {
    // Run FT8 decoder on the collected waterfall data
    ft8_portapack_decode(&decoder_state);

    // Send each decoded message's raw payload to the application (M0) for decoding & display
    // Message decoding (ftx_message_decode) is done on the M0 side to save baseband flash space
    for (int i = 0; i < decoder_state.num_messages; i++) {
        // Pack raw 10-byte payload into FT8PacketMessage char fields
        // call_from[0..9] = raw payload bytes, call_from[10..12] = 0
        // call_to[0] = 0xFF marker (indicates raw payload, not decoded text)
        // snr = sync score
        char raw_from[13] = {0};
        memcpy(raw_from, decoder_state.messages[i].payload, FTX_PAYLOAD_LENGTH_BYTES);

        char raw_to[13] = {0};
        raw_to[0] = '\xff';  // Raw payload marker

        int8_t score = (int8_t)(decoder_state.message_scores[i] > 127
                                    ? 127
                                    : decoder_state.message_scores[i]);
        FT8PacketMessage msg{raw_from, raw_to, "", score, 0};
        shared_memory.application_queue.push(msg);
    }

    // Also send debug info: candidates found, max magnitude
    if (decoder_state.num_candidates > 0 || decoder_state.max_magnitude > 0) {
        FT8PacketMessage dbg{
            "CND", "",  "",
            (int8_t)(decoder_state.num_candidates > 127 ? 127 : decoder_state.num_candidates),
            (uint8_t)(decoder_state.skip_count > 255 ? 255 : decoder_state.skip_count)};
        shared_memory.application_queue.push(dbg);
    }

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
    ft8_portapack_set_min_score(&decoder_state, message.threshold);
}

int main() {
    audio::dma::init_audio_out();

    EventDispatcher event_dispatcher{std::make_unique<FT8RxProcessor>()};
    event_dispatcher.run();
    return 0;
}
