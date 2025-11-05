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

    // All controls on line 0 (like AFSK RX)
    RFAmpField field_rf_amp{
        {13 * 8, 0}};

    LNAGainField field_lna{
        {15 * 8, 0}};

    VGAGainField field_vga{
        {18 * 8, 0}};

    RSSI rssi{
        {21 * 8, 0, 6 * 8, 8}};

    AudioVolumeField field_volume{
        {28 * 8, 0}};

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
