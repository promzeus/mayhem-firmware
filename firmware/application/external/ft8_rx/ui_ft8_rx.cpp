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
        &field_volume,
        &field_frequency,
        &text_status,
        &console
    });

    // Configure frequency field
    field_frequency.set_step(100);  // 100 Hz steps for FT8

    // Load FT8 RX baseband image (already prepared by external app loader)
    // baseband::run_image(portapack::spi_flash::image_tag_ft8_rx);
    baseband::run_prepared_image(portapack::memory::map::m4_code.base());

    // Set modulation mode to AM Audio (this will call update_modulation)
    receiver_model.set_modulation(ReceiverModel::Mode::AMAudio);

    // Set sampling rates (same as Audio RX for AM mode)
    receiver_model.set_sampling_rate(3072000);  // 3.072 MHz
    receiver_model.set_baseband_bandwidth(1750000);  // 1.75 MHz

    // Enable receiver
    receiver_model.enable();

    // Configure USB filter AFTER enable (index 2 = USB 2.8 kHz - same as Audio RX "USB+3k")
    // This will call update_modulation again and apply the correct filter
    receiver_model.set_am_configuration(2);

    // Start audio output with 12 kHz rate (same as Audio RX for AM)
    audio::set_rate(audio::Rate::Hz_12000);
    audio::output::start();

    text_status.set("SSB USB 2.8k");

    // Test console output
    console.writeln("=== FT8 RX Ready ===");
    console.writeln("Waiting for signals...");
}

void FT8RxView::on_statistics_update(const ChannelStatistics& statistics) {
    rssi.set_db(statistics.max_db);
}

void FT8RxView::on_ft8_packet(const FT8PacketMessage* message) {
    // Format: "TIME SNR FROM TO GRID"
    // Example: "1545 -12 CQ W1ABC FN42"

    std::string line;
    char time_str[8];
    char snr_str[8];

    // Format time (HH:MM format, using time_slot as minutes)
    snprintf(time_str, sizeof(time_str), "%02d:%02d",
             message->time_slot / 60, message->time_slot % 60);

    // Format SNR with sign
    snprintf(snr_str, sizeof(snr_str), "%+3d", message->snr);

    // Build the line: "TIME SNR FROM TO GRID"
    line = time_str;
    line += " ";
    line += snr_str;
    line += " ";
    line += message->call_from;

    // Only add call_to if not empty
    if (message->call_to[0] != '\0') {
        line += " ";
        line += message->call_to;
    }

    // Only add grid if not empty
    if (message->grid[0] != '\0') {
        line += " ";
        line += message->grid;
    }

    // Display in console
    console.writeln(line);
}

}  // namespace ui::external_app::ft8_rx
