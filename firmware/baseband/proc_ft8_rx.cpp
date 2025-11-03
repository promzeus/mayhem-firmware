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

#include "audio_output.hpp"
#include "audio_dma.hpp"
#include "event_m4.hpp"
#include "portapack_shared_memory.hpp"

#include <array>

FT8RxProcessor::FT8RxProcessor() {
    if (!ft8_portapack_init(&decoder_state)) {
        decoding_enabled = false;
    }
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

    for (size_t i = 0; i < audio.count; i++) {
        audio_accumulator[audio_accumulator_pos++] = audio.p[i];
        if (audio_accumulator_pos >= FT8_FFT_SIZE) {
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

void FT8RxProcessor::send_test_packet() {
    FT8PacketMessage message("CQ", "TEST", "DE1234", -15, slot_count % 2);
    shared_memory.application_queue.push(message);
}

void FT8RxProcessor::decode_ft8_slot() {
    int num_decoded = ft8_portapack_decode(&decoder_state);
    if (num_decoded > 0) {
        send_ft8_messages();
    }
    ft8_portapack_reset_slot(&decoder_state);
}

void FT8RxProcessor::send_ft8_messages() {
    for (int i = 0; i < decoder_state.num_messages; i++) {
        const ftx_message_t* msg = ft8_portapack_get_message(&decoder_state, i);
        if (!msg) continue;
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

        case Message::ID::AMConfigure:
            configure(*reinterpret_cast<const AMConfigureMessage*>(message));
            break;

        case Message::ID::CaptureConfig:
            capture_config(*reinterpret_cast<const CaptureConfigMessage*>(message));
            break;

        default:
            break;
    }
}

void FT8RxProcessor::configure(const AMConfigureMessage& message) {
    constexpr size_t decim_0_output_fs = baseband_fs / decim_0.decimation_factor;
    constexpr size_t decim_1_output_fs = decim_0_output_fs / decim_1.decimation_factor;
    const size_t decim_2_output_fs = decim_1_output_fs / decim_2_decimation_factor;
    decim_0.configure(message.decim_0_filter.taps);
    decim_1.configure(message.decim_1_filter.taps);
    decim_2.configure(message.decim_2_filter.taps, decim_2_decimation_factor);
    channel_filter.configure(message.channel_filter.taps, channel_filter_decimation_factor);
    channel_filter_low_f = message.channel_filter.low_frequency_normalized * decim_2_output_fs;
    channel_filter_high_f = message.channel_filter.high_frequency_normalized * decim_2_output_fs;
    channel_filter_transition = message.channel_filter.transition_normalized * decim_2_output_fs;
    modulation_ssb = (int)message.modulation;
    channel_spectrum.set_decimation_factor(message.channel_spectrum_decimation_factor);
    audio_output.configure(message.audio_hpf_lpf_config);
    configured = true;
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
