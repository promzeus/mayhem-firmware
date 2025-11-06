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
    // No decim_2 - go directly to channel_filter (like AFSK RX)
    const auto channel_out = channel_filter.execute(decim_1_out, dst_buffer);
    feed_channel_stats(channel_out);
    auto audio = demodulate(channel_out);
    audio_compressor.execute_in_place(audio);  // AGC: normalize audio to ±1.0 range
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
    static size_t total_audio_chunks = 0;  // Count how many audio buffers received
    static size_t last_audio_count = 0;    // Track audio.count for debugging
    static bool decimate_phase = false;    // Toggle for 24→12 kHz decimation

    total_audio_chunks++;
    last_audio_count = audio.count;  // Remember last buffer size

    for (size_t i = 0; i < audio.count; i++) {
        float sample = audio.p[i];
        audio_sum_sq += sample * sample;
        audio_sample_count++;

        // Software decimation 24kHz → 12kHz: take every 2nd sample to save memory
        decimate_phase = !decimate_phase;
        if (!decimate_phase) continue;  // Skip odd samples (simple 2:1 decimation)

        // AudioCompressor already normalized audio to ±1.0, no additional gain needed
        audio_accumulator[audio_accumulator_pos++] = sample;

        // Process FFT every 1920 samples (one FT8 symbol = 160ms @ 12kHz)
        if (audio_accumulator_pos >= FT8_SAMPLES_PER_SYMBOL) {
            // Zero-padding to 2048 is handled inside ft8_portapack_process_audio()
            bool slot_complete = ft8_portapack_process_audio(
                &decoder_state, audio_accumulator.data(), audio_accumulator_pos);
            audio_accumulator_pos = 0;

            if (slot_complete) {
                // Calculate mean square audio level (power) for the slot
                // (using mean square instead of RMS to avoid sqrtf which needs libm)
                float mean_sq = audio_sum_sq / audio_sample_count;
                size_t samples_in_slot = audio_sample_count;
                audio_sum_sq = 0.0f;
                audio_sample_count = 0;

                decode_ft8_slot(mean_sq, samples_in_slot, last_audio_count, total_audio_chunks);
                total_audio_chunks = 0;  // Reset chunk counter
                slot_count++;
            }
        }
    }
}

void FT8RxProcessor::send_test_packet() {
    FT8PacketMessage message("CQ", "TEST", "DE1234", -15, slot_count % 2);
    shared_memory.application_queue.push(message);
}

void FT8RxProcessor::decode_ft8_slot(float rms_level, size_t samples_in_slot, size_t chunk_size, size_t num_chunks) {
    // DEBUG: Signal start of decode
    FT8PacketMessage dbg_start("DEC", "START", "", slot_count % 128, 0);
    shared_memory.application_queue.push(dbg_start);

    int num_decoded = ft8_portapack_decode(&decoder_state);

    // DEBUG: Signal end of decode
    FT8PacketMessage dbg_end("DEC", "END", "", num_decoded, decoder_state.num_candidates);
    shared_memory.application_queue.push(dbg_end);

    // DEBUG: Send last stage reached (SNR=stage, time_slot=value)
    FT8PacketMessage dbg_stage("STG", "VAL", "", decoder_state.debug_stage,
                               decoder_state.debug_value > 255 ? 255 : decoder_state.debug_value);
    shared_memory.application_queue.push(dbg_stage);

    // ALWAYS send debug info: CND=candidates, MAG=max_magnitude
    // SNR field = num_candidates (0-50), time_slot = max_magnitude (0-255)
    int max_mag = decoder_state.max_magnitude > 255 ? 255 : decoder_state.max_magnitude;
    FT8PacketMessage debug("CND", "MAG", "", decoder_state.num_candidates, max_mag);
    shared_memory.application_queue.push(debug);

    // Send audio power level (SNR field = power*1000, clamped to 127)
    int power_int = (int)(rms_level * 1000.0f);
    if (power_int > 127) power_int = 127;
    FT8PacketMessage power_debug("AUD", "PWR", "", power_int, slot_count % 2);
    shared_memory.application_queue.push(power_debug);

    // DEBUG: Send audio peak level (SNR = audio_peak * 100, clamped)
    int audio_peak_int = (int)(decoder_state.debug_audio_peak * 100.0f);
    if (audio_peak_int > 127) audio_peak_int = 127;
    FT8PacketMessage audio_peak_debug("AUD", "PEAK", "", audio_peak_int, decoder_state.debug_blocks_written);
    shared_memory.application_queue.push(audio_peak_debug);

    // DEBUG: Send FFT power peak (SNR = fft_power * 10, clamped)
    int fft_power_int = (int)(decoder_state.debug_fft_power * 10.0f);
    if (fft_power_int > 127) fft_power_int = 127;
    FT8PacketMessage fft_power_debug("FFT", "PWR", "", fft_power_int, decoder_state.debug_blocks_written);
    shared_memory.application_queue.push(fft_power_debug);

    // DEBUG: Send DC power vs AC power (SNR = DC*10, time_slot = AC*10)
    int dc_power_int = (int)(decoder_state.debug_dc_power * 10.0f);
    if (dc_power_int > 127) dc_power_int = 127;
    int ac_power_int = (int)(decoder_state.debug_ac_power * 10.0f);
    if (ac_power_int > 127) ac_power_int = 127;
    FT8PacketMessage dc_ac_debug("DC", "AC", "", dc_power_int, ac_power_int);
    shared_memory.application_queue.push(dc_ac_debug);

    // DEBUG: Send nonzero bin count (SNR = count/10, time_slot = blocks)
    int nonzero_int = decoder_state.debug_nonzero_bins / 10;
    if (nonzero_int > 127) nonzero_int = 127;
    FT8PacketMessage nonzero_debug("NZ", "BINS", "", nonzero_int, decoder_state.debug_blocks_written);
    shared_memory.application_queue.push(nonzero_debug);

    // DEBUG: Send waterfall dimensions to diagnose total_elements issue
    int num_blocks = decoder_state.debug_num_blocks > 127 ? 127 : decoder_state.debug_num_blocks;
    int block_stride = decoder_state.debug_block_stride > 255 ? 255 : decoder_state.debug_block_stride;
    FT8PacketMessage dims_debug("BLK", "DIM", "", num_blocks, block_stride);
    shared_memory.application_queue.push(dims_debug);

    // DEBUG: Send total_elements (SNR = total_elements/100, clamped)
    int total_elem_int = decoder_state.debug_total_elements / 100;
    if (total_elem_int > 127) total_elem_int = 127;
    if (total_elem_int < -127) total_elem_int = -127;
    FT8PacketMessage total_elem_debug("TOT", "ELEM", "", total_elem_int, 0);
    shared_memory.application_queue.push(total_elem_debug);

    // DEBUG: Send waterfall samples to diagnose MAG=0 issue (divide by 2 to fit in int8_t range 0-127)
    int wf0_val = decoder_state.debug_mag_sample_0 / 2;
    if (wf0_val < 0) wf0_val = -1;
    FT8PacketMessage wf0_debug("WF", "[0]", "", wf0_val, 0);
    shared_memory.application_queue.push(wf0_debug);

    int wf1_val = decoder_state.debug_mag_sample_1 / 2;
    if (wf1_val < 0) wf1_val = -1;
    FT8PacketMessage wf1_debug("WF", "[1]", "", wf1_val, 0);
    shared_memory.application_queue.push(wf1_debug);

    int wf100_val = decoder_state.debug_mag_sample_100 / 2;
    if (wf100_val < 0) wf100_val = -1;
    FT8PacketMessage wf100_debug("WF", "[100]", "", wf100_val, 0);
    shared_memory.application_queue.push(wf100_debug);

    // DEBUG: Send first nonzero element (SNR = value/2, time_slot = index/10)
    int first_nz_val = decoder_state.debug_mag_first_nonzero / 2;
    if (first_nz_val < 0) first_nz_val = -1;
    int first_nz_idx = decoder_state.debug_mag_first_nonzero_idx / 10;
    if (first_nz_idx > 127) first_nz_idx = 127;
    if (first_nz_idx < 0) first_nz_idx = 0;
    FT8PacketMessage first_nz_debug("1ST", "NZ", "", first_nz_val, first_nz_idx);
    shared_memory.application_queue.push(first_nz_debug);

    // Send audio buffer debug info: chunk_size and num_chunks
    // SNR = chunk_size (how many samples per buffer), time_slot = num_chunks (how many buffers per slot)
    int chunk_dbg = chunk_size > 127 ? 127 : (int)chunk_size;
    int chunks_dbg = num_chunks > 255 ? 255 : (int)num_chunks;
    FT8PacketMessage audio_debug("BUF", "CHK", "", chunk_dbg, chunks_dbg);
    shared_memory.application_queue.push(audio_debug);

    // Send total samples in slot debug (SNR = samples/100, clamped)
    int samples_dbg = (int)(samples_in_slot / 100);
    if (samples_dbg > 127) samples_dbg = 127;
    FT8PacketMessage samples_debug("SMP", "TOT", "", samples_dbg, slot_count % 2);
    shared_memory.application_queue.push(samples_debug);

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
