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

#ifndef __UI_FT8_RX_H__
#define __UI_FT8_RX_H__

#include "app_settings.hpp"
#include "radio_state.hpp"
#include "ui.hpp"
#include "ui_receiver.hpp"
#include "ui_rssi.hpp"
#include "ui_audio.hpp"
#include "ui_freq_field.hpp"
#include "ui_widget.hpp"
#include "receiver_model.hpp"
#include "message.hpp"

namespace ui::external_app::ft8_rx {

class FT8RxView : public View {
   public:
    FT8RxView(NavigationView& nav);
    ~FT8RxView();

    void focus() override;
    std::string title() const override { return "FT8 RX"; }

   private:
    NavigationView& nav_;

    // FT8 на 80m: 3.574 MHz (можно также 7.074 MHz для 40m)
    RxRadioState radio_state_{
        3574000 /* frequency */,
        3000 /* bandwidth */,
        12000 /* sampling rate */
    };

    app_settings::SettingsManager settings_{
        "rx_ft8",
        app_settings::Mode::RX};

    void on_statistics_update(const ChannelStatistics& statistics);
    void on_ft8_packet(const FT8PacketMessage* message);
    void on_threshold_change(int32_t value);

    // All controls on line 0 (like AFSK RX and POCSAG)
    RFAmpField field_rf_amp{
        {11 * 8, 0}};

    LNAGainField field_lna{
        {13 * 8, 0}};

    VGAGainField field_vga{
        {16 * 8, 0}};

    // DUAL RSSI+Audio indicators like POCSAG (two bars stacked vertically)
    // Upper bar: RSSI signal strength
    // X: 19*8-4=148px, Width: 52px (ends at 200px)
    RSSI rssi{
        {19 * 8 - 4, 3, 52, 4}};  // Like POCSAG: Y=3, height=4

    // Lower bar: Audio level (shows threshold line via set_db)
    Audio audio{
        {19 * 8 - 4, 8, 52, 4}};  // Like POCSAG: Y=8, height=4

    // Threshold adjustment field (min_score for FT8 decoder)
    // Position from right edge: 5 chars (40px) from right = 200px
    // Ends at 216px (200 + 16), then 8px space, then volume at 224px
    NumberField field_threshold{
        {UI_POS_X_RIGHT(5), 0},
        2,        // 2 digits
        {10, 99}, // Range 10-99
        1,        // Step
        ' '       // Filler
    };

    // Volume field: 2 chars (16px) from right = 224px
    AudioVolumeField field_volume{
        {UI_POS_X_RIGHT(2), 0}};

    RxFrequencyField field_frequency{
        {0, 0},
        nav_};

    // Status text on line 1
    Text text_status{
        {0, 1 * 16, 240, 16},
        "Listening..."};

    // Console for decoded FT8 messages (lines 2-14)
    Console console{
        {0, 2 * 16, 240, 240 - 2 * 16}};

    MessageHandlerRegistration message_handler_stats{
        Message::ID::ChannelStatistics,
        [this](const Message* const p) {
            this->on_statistics_update(static_cast<const ChannelStatisticsMessage*>(p)->statistics);
        }};

    MessageHandlerRegistration message_handler_ft8_packet{
        Message::ID::FT8Packet,
        [this](const Message* const p) {
            this->on_ft8_packet(static_cast<const FT8PacketMessage*>(p));
        }};
};

}  // namespace ui::external_app::ft8_rx

#endif  // __UI_FT8_RX_H__
