/*
 * Copyright (C) 2015 Jared Boone, ShareBrained Technology, Inc.
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

#include "ui_audio.hpp"

#include "utility.hpp"

#include <algorithm>

namespace ui {

void Audio::paint(Painter& painter) {
    const auto r = screen_rect();

    constexpr int db_min = -96;
    constexpr int db_max = 0;
    constexpr int db_delta = db_max - db_min;
    const range_t<int> x_rms_range{0, r.width() - 1};
    const auto x_rms = x_rms_range.clip((rms_db_ - db_min) * r.width() / db_delta);
    const range_t<int> x_max_range{x_rms + 1, r.width()};
    const auto x_max = x_max_range.clip((max_db_ - db_min) * r.width() / db_delta);

    const Rect r0{r.left(), r.top(), x_rms, r.height()};
    painter.fill_rectangle(
        r0,
        Color::green());

    const Rect r1{r.left() + x_rms, r.top(), 1, r.height()};
    painter.fill_rectangle(
        r1,
        Color::black());

    const Rect r2{r.left() + x_rms + 1, r.top(), x_max - (x_rms + 1), r.height()};
    painter.fill_rectangle(
        r2,
        Color::red());

    const Rect r3{r.left() + x_max, r.top(), r.width() - x_max, r.height()};
    painter.fill_rectangle(
        r3,
        Color::black());

    // Draw threshold line (like in RSSI widget)
    if (threshold_db_) {
        constexpr int db_threshold_min = -80;
        constexpr int db_threshold_max = 10;
        constexpr int db_threshold_delta = db_threshold_max - db_threshold_min;
        const range_t<int> x_threshold_range{0, r.width() - 1};
        const int16_t x_threshold = x_threshold_range.clip((threshold_db_ - db_threshold_min) * r.width() / db_threshold_delta);

        const Rect r_threshold{r.left() + x_threshold, r.top(), 1, r.height()};
        painter.fill_rectangle(r_threshold, Color::yellow());  // Yellow line for threshold
    }
}

void Audio::on_statistics_update(const AudioStatistics& statistics) {
    rms_db_ = statistics.rms_db;
    max_db_ = statistics.max_db;
    set_dirty();
}

void Audio::set_db(int16_t db) {
    threshold_db_ = db;
}

} /* namespace ui */
