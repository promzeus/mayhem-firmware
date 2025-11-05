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
#include "event_m4.hpp"

FT8RxProcessor::FT8RxProcessor() {
    // Use same filters as Audio RX for SSB USB 2.8k (am_configs index 2)
    decim_0.configure(taps_6k0_decim_0.taps);
    decim_1.configure(taps_6k0_decim_1.taps);
    decim_2.configure(taps_6k0_decim_2.taps, decim_2_decimation_factor);
    channel_filter.configure(taps_2k8_usb_channel.taps, channel_filter_decimation_factor);
    audio_output.configure(false);

    if (!ft8_portapack_init(&decoder_state)) {
        decoding_enabled = false;
    }

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
    const auto decim_2_out = decim_2.execute(decim_1_out, dst_buffer);
    const auto channel_out = channel_filter.execute(decim_2_out, dst_buffer);
    feed_channel_stats(channel_out);
    auto audio = demodulate(channel_out);
    process_ft8_audio(audio);
    audio_output.write(audio);
}

buffer_f32_t FT8RxProcessor::demodulate(const buffer_c16_t& channel) {
    return (modulation_ssb == 0) ? demod_am.execute(channel, audio_buffer)
                                  : demod_ssb.execute(channel, audio_buffer);
}

void FT8RxProcessor::process_ft8_audio(const buffer_f32_t& audio) {
    if (!decoding_enabled) {
        static size_t test_sample_count = 0;
        test_sample_count += audio.count;
        if (test_sample_count >= FT8_SAMPLES_PER_SLOT) {
            test_sample_count = 0;
            slot_count++;
            send_test_packet();
        }
        return;
    }

    // Calculate audio level for debugging
    static float audio_sum_sq = 0.0f;
    static size_t audio_sample_count = 0;

    for (size_t i = 0; i < audio.count; i++) {
        float sample = audio.p[i];
        audio_sum_sq += sample * sample;
        audio_sample_count++;

        audio_accumulator[audio_accumulator_pos++] = sample;
        // Process FFT every 1920 samples (one FT8 symbol = 160ms)
        if (audio_accumulator_pos >= FT8_SAMPLES_PER_SYMBOL) {
            // Zero-padding to 2048 is handled inside ft8_portapack_process_audio()
            bool slot_complete = ft8_portapack_process_audio(
                &decoder_state, audio_accumulator.data(), audio_accumulator_pos);
            audio_accumulator_pos = 0;

            if (slot_complete) {
                // Calculate mean square audio level (power) for the slot
                // (using mean square instead of RMS to avoid sqrtf which needs libm)
                float mean_sq = audio_sum_sq / audio_sample_count;
                audio_sum_sq = 0.0f;
                audio_sample_count = 0;

                decode_ft8_slot(mean_sq);
                slot_count++;
            }
        }
    }
}

void FT8RxProcessor::send_test_packet() {
    FT8PacketMessage message("CQ", "TEST", "DE1234", -15, slot_count % 2);
    shared_memory.application_queue.push(message);
}

void FT8RxProcessor::decode_ft8_slot(float rms_level) {
    int num_decoded = ft8_portapack_decode(&decoder_state);

    // ALWAYS send debug info: CND=candidates, MAG=max_magnitude
    // SNR field = num_candidates (0-50), time_slot = max_magnitude (0-255)
    int max_mag = decoder_state.max_magnitude > 255 ? 255 : decoder_state.max_magnitude;
    FT8PacketMessage debug("CND", "MAG", "", decoder_state.num_candidates, max_mag);
    shared_memory.application_queue.push(debug);

    // Send audio power level as separate message (SNR field = power*1000, clamped to 127)
    // Note: using mean square (not RMS) to avoid sqrtf dependency
    int power_int = (int)(rms_level * 1000.0f);
    if (power_int > 127) power_int = 127;  // Clamp to signed byte range
    FT8PacketMessage power_debug("AUD", "PWR", "", power_int, slot_count % 2);
    shared_memory.application_queue.push(power_debug);

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
    audio::dma::init_audio_out();
    EventDispatcher event_dispatcher{std::make_unique<FT8RxProcessor>()};
    event_dispatcher.run();
    return 0;
}
