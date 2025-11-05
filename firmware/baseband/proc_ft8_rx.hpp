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

#ifndef __PROC_FT8_RX_H__
#define __PROC_FT8_RX_H__

#include "baseband_processor.hpp"
#include "baseband_thread.hpp"
#include "rssi_thread.hpp"

#include "dsp_decimate.hpp"
#include "dsp_demodulate.hpp"

#include "audio_output.hpp"
#include "spectrum_collector.hpp"

#include "message.hpp"
#include "ft8_lib/ft8_portapack.h"

class FT8RxProcessor : public BasebandProcessor {
   public:
    FT8RxProcessor();
    ~FT8RxProcessor();

    void execute(const buffer_c8_t& buffer) override;
    void on_message(const Message* const message) override;

   private:
    static constexpr size_t baseband_fs = 3072000;  // 3.072 MHz input

    // Decimation chain: 3.072 MHz → 48 kHz → 24 kHz → 12 kHz
    static constexpr size_t decim_0_output_fs = baseband_fs / 8;  // 384 kHz
    static constexpr size_t decim_1_output_fs = decim_0_output_fs / 8;  // 48 kHz
    static constexpr size_t decim_2_output_fs = decim_1_output_fs / 2;  // 24 kHz
    static constexpr size_t audio_fs = 12000;  // 12 kHz audio for FT8

    // FT8 timing constants (FT8_SLOT_DURATION and FT8_SAMPLES_PER_SYMBOL defined in ft8_portapack.h)
    static constexpr size_t FT8_SAMPLES_PER_SLOT = static_cast<size_t>(audio_fs * FT8_SLOT_DURATION);  // ~151680 samples

    // Buffers
    std::array<complex16_t, 512> dst{};
    const buffer_c16_t dst_buffer{
        dst.data(),
        dst.size()};

    std::array<float, 32> audio{};
    const buffer_f32_t audio_buffer{
        audio.data(),
        audio.size()};

    // Decimation chain
    dsp::decimate::FIRC8xR16x24FS4Decim8 decim_0{};
    dsp::decimate::FIRC16xR16x32Decim8 decim_1{};
    dsp::decimate::FIRAndDecimateComplex decim_2{};
    dsp::decimate::FIRAndDecimateComplex channel_filter{};

    // Demodulators
    dsp::demodulate::AM demod_am{};
    dsp::demodulate::SSB demod_ssb{};

    // Audio processing
    AudioOutput audio_output{};
    SpectrumCollector channel_spectrum{};

    // Configuration
    bool configured{false};
    int modulation_ssb{1};  // 1 = SSB mode for FT8
    uint32_t decim_2_decimation_factor{2};
    uint32_t channel_filter_decimation_factor{2};  // Changed 1→2 to get 12 kHz (24/2)
    int32_t channel_filter_low_f{0};
    int32_t channel_filter_high_f{0};
    int32_t channel_filter_transition{0};

    // FT8 decoder state and buffers
    ft8_decoder_state_t decoder_state{};
    std::array<float, FT8_FFT_SIZE> audio_accumulator{};  // 1920 samples for FFT
    size_t audio_accumulator_pos{0};
    uint32_t slot_count{0};
    bool decoding_enabled{true};  // Enable/disable FT8 decoding

    // Message handlers
    void capture_config(const CaptureConfigMessage& message);
    buffer_f32_t demodulate(const buffer_c16_t& channel);

    // FT8 processing
    void process_ft8_audio(const buffer_f32_t& audio);
    void decode_ft8_slot(float rms_level);  // Decode complete FT8 slot with audio RMS
    void send_ft8_messages();        // Send decoded messages to M0
    void send_test_packet();         // For fallback testing

    /* NB: Threads should be the last members in the class definition. */
    BasebandThread baseband_thread{baseband_fs, this, baseband::Direction::Receive};
    RSSIThread rssi_thread{};
};

#endif  // __PROC_FT8_RX_H__
