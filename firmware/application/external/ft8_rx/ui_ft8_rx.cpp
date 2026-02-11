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

#include "ui_ft8_rx.hpp"
#include "baseband_api.hpp"
#include "audio.hpp"
#include "rtc_time.hpp"
#include "string_format.hpp"
#include "hal.h"
#include "portapack_shared_memory.hpp"
#include <cstring>

// FT8 message decoding (compiled on M0 side to save M4 baseband flash)
extern "C" {
#include "../baseband/ft8_lib/message.h"
}

using namespace portapack;

namespace ui::external_app::ft8_rx {

void FT8RxView::focus() {
    field_frequency.focus();
}

FT8RxView::~FT8RxView() {
    receiver_model.disable();
    audio::output::stop();
    baseband::shutdown();
}

FT8RxView::FT8RxView(NavigationView& nav)
    : nav_{nav} {

    add_children({
        &field_rf_amp,
        &field_lna,
        &field_vga,
        &rssi,
        &audio,
        &field_threshold,
        &field_volume,
        &field_frequency,
        &text_status,
        &console
    });

    // Set threshold field initial value and change handler
    // Start with minimum squelch (10) so audio is audible by default
    field_threshold.set_value(10);  // Minimum squelch - user can hear everything
    field_threshold.on_change = [this](int32_t value) {
        this->on_threshold_change(value);
    };

    // Send initial threshold to baseband and set squelch
    on_threshold_change(10);

    // Configure frequency field
    field_frequency.set_step(100);  // 100 Hz steps for FT8

    // Load FT8 RX baseband image (already prepared by external app loader)
    // baseband::run_prepared_image() already done by external app loader
    baseband::run_prepared_image(portapack::memory::map::m4_code.base());

    // Set sampling rates for FT8 (DO NOT call set_modulation - it replaces FT8 baseband!)
    receiver_model.set_sampling_rate(3072000);  // 3.072 MHz
    receiver_model.set_baseband_bandwidth(15000);  // 15 kHz - narrow filter for FT8 (was 1.75 MHz!)

    // Enable receiver
    receiver_model.enable();

    // Start audio output - FT8 baseband already configured audio_output in constructor
    audio::output::start();

    text_status.set("SSB USB 2.8k");

    // Test console output
    console.writeln("=== FT8 RX Ready ===");
    console.writeln("Waiting for signals...");
}

void FT8RxView::on_statistics_update(const ChannelStatistics& statistics) {
    // RSSI widget automatically updates from RSSIStatistics messages
    // Don't use set_db() here - that's reserved for threshold line
    (void)statistics;
}

void FT8RxView::on_threshold_change(int32_t value) {
    // Send threshold configuration to baseband for FT8 decoder
    FT8ConfigureMessage message{static_cast<uint8_t>(value)};
    shared_memory.application_queue.push(message);

    // NOTE: AM/SSB modes do NOT support squelch (only NBFM does)
    // Audio is always sent to output - user can adjust volume manually
    // The threshold only affects FT8 decoder sensitivity (min_score)

    // Update Audio widget to show threshold line (yellow line on lower bar like in POCSAG)
    // Convert threshold 10-99 to approximate dB scale (-80 to +10 dB)
    // Lower threshold = more sensitive = appears lower on indicator
    // Map: 10 (most sensitive) -> -70dB, 99 (least sensitive) -> -30dB
    int16_t threshold_db = -70 + ((value - 10) * 40 / 89);
    audio.set_db(threshold_db);

    // Update UI to show current threshold
    text_status.set("Threshold: " + to_string_dec_uint(value));
}

void FT8RxView::on_ft8_packet(const FT8PacketMessage* message) {
    std::string line;

    // Check for raw payload marker (0xFF in call_to[0])
    // Baseband sends raw 10-byte ftx_message_t payload; we decode it here on M0
    if (message->call_to[0] == '\xff') {
        // Reconstruct ftx_message_t from raw bytes in call_from[0..9]
        ftx_message_t ftx_msg;
        memcpy(ftx_msg.payload, message->call_from, FTX_PAYLOAD_LENGTH_BYTES);
        ftx_msg.hash = 0;

        // Decode to human-readable text
        char text[FTX_MAX_MESSAGE_LENGTH];
        memset(text, 0, sizeof(text));
        ftx_message_offsets_t offsets;

        ftx_message_rc_t rc = ftx_message_decode(
            &ftx_msg,
            NULL,   // No hash interface — hashed calls show as <...>
            text,
            &offsets);

        if (rc != FTX_MESSAGE_RC_OK) {
            // Try free-text decode as fallback
            ftx_message_decode_free(&ftx_msg, text);
            if (text[0] == '\0') return;
        }

        // Format: "score text"
        char snr_str[8];
        snprintf(snr_str, sizeof(snr_str), "%+3d", message->snr);
        line = snr_str;
        line += " ";
        line += text;

        console.writeln(line);
        return;
    }

    // Debug message types (sent with plain text in call_from)
    if (strcmp(message->call_from, "CND") == 0) {
        line = "CND: ";
        line += to_string_dec_uint(message->snr, 2);

        if (message->time_slot > 0) {
            line += " skip:";
            line += to_string_dec_uint(message->time_slot, 3);
        }

        line += " thr:";
        line += to_string_dec_uint(field_threshold.value(), 2);
        console.writeln(line);
    }
}

}  // namespace ui::external_app::ft8_rx
