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
    receiver_model.set_baseband_bandwidth(1750000);  // 1.75 MHz

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

    // Check debug message types
    if (strcmp(message->call_from, "AUD") == 0) {
        // Audio/FFT debug: AUD peak=x.xx power=y.yy
        // FT8PacketMessage(from, to, grid, snr=audio_peak_x100, time_slot=fft_power_x100)
        // Show RAW values (before /100) to debug why they're empty
        line = "AUD raw[";
        line += to_string_dec_int(message->snr, 4);      // snr (int8_t) raw value
        line += ",";
        line += to_string_dec_uint(message->time_slot, 3); // time_slot (uint8_t) raw value
        line += "] pk=";

        float audio_peak = message->snr / 100.0f;
        float fft_power = message->time_slot / 100.0f;

        char pk_buf[8], pwr_buf[8];
        snprintf(pk_buf, sizeof(pk_buf), "%.2f", audio_peak);
        snprintf(pwr_buf, sizeof(pwr_buf), "%.2f", fft_power);
        line += pk_buf;
        line += " pwr=";
        line += pwr_buf;
    }
    else if (strcmp(message->call_from, "MAX") == 0) {
        // Show waterfall maximum with visual bar
        // FT8PacketMessage(from, to, grid, snr=max_mag, time_slot=nonzero)
        char bar[12];
        int bar_len = (message->snr * 10) / 127;  // snr = max_mag
        for (int i = 0; i < 10; i++) {
            bar[i] = (i < bar_len) ? '#' : '-';
        }
        bar[10] = '\0';

        line = "MAX +";
        line += to_string_dec_uint(message->snr, 3);  // snr = max_mag
        line += " [";
        line += bar;
        line += "] NZ:";
        line += to_string_dec_uint(message->time_slot, 3);  // time_slot = nonzero
    }
    else if (strcmp(message->call_from, "CND") == 0) {
        // Show candidates found and skip counter
        // FT8PacketMessage(from, to, grid, snr=num_candidates, time_slot=skip_count)
        line = "CND: ";
        line += to_string_dec_uint(message->snr, 2);  // snr = num_candidates

        if (message->time_slot > 0) {  // time_slot = skip_count
            line += " skip:";
            line += to_string_dec_uint(message->time_slot, 3);
        }

        line += " thr:";
        line += to_string_dec_uint(field_threshold.value(), 2);
    }
    else {
        // Regular FT8 message
        char time_str[8];
        char snr_str[8];

        // Format time
        snprintf(time_str, sizeof(time_str), "%02d:%02d",
                 message->time_slot / 60, message->time_slot % 60);

        // Format SNR with sign
        snprintf(snr_str, sizeof(snr_str), "%+3d", message->snr);

        // Build the line
        line = time_str;
        line += " ";
        line += snr_str;
        line += " ";
        line += message->call_from;

        if (message->call_to[0] != '\0') {
            line += " ";
            line += message->call_to;
        }

        if (message->grid[0] != '\0') {
            line += " ";
            line += message->grid;
        }
    }

    // Display in console
    console.writeln(line);
}

}  // namespace ui::external_app::ft8_rx
