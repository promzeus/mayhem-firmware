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
#include "oversample.hpp"
#include "ui_spectrum.hpp"

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
        &text_status
    });

    // Configure frequency field
    field_frequency.set_step(100);  // 100 Hz steps for FT8

    // Start receiver
    audio::set_rate(audio::Rate::Hz_12000);
    baseband::run_image(portapack::spi_flash::image_tag_capture);
    receiver_model.set_modulation(ReceiverModel::Mode::Capture);

    const uint32_t bandwidth = 3000;
    baseband::set_sample_rate(bandwidth, get_oversample_rate(bandwidth));

    auto actual_sampling_rate = get_actual_sample_rate(bandwidth);
    receiver_model.set_sampling_rate(actual_sampling_rate);
    receiver_model.set_baseband_bandwidth(filter_bandwidth_for_sampling_rate(actual_sampling_rate));

    audio::output::start();
    receiver_model.enable();
}

void FT8RxView::on_statistics_update(const ChannelStatistics& statistics) {
    rssi.set_db(statistics.max_db);
}

}  // namespace ui::external_app::ft8_rx
