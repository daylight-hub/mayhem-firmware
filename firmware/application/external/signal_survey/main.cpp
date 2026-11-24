/*
 * Copyright (C) 2026
 *
 * This file is part of PortaPack.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "ui.hpp"
#include "ui_signal_survey.hpp"
#include "ui_navigation.hpp"
#include "external_app.hpp"

namespace ui::external_app::signal_survey {
void initialize_app(ui::NavigationView& nav) {
    nav.push<SignalSurveyView>();
}
}  // namespace ui::external_app::signal_survey

extern "C" {

__attribute__((section(".external_app.app_signal_survey.application_information"), used)) application_information_t _application_information_signal_survey = {
    /*.memory_location = */ (uint8_t*)0x00000000,
    /*.externalAppEntry = */ ui::external_app::signal_survey::initialize_app,
    /*.header_version = */ CURRENT_HEADER_VERSION,
    /*.app_version = */ VERSION_MD5,

    /*.app_name = */ "SigSurvey",
    /*.bitmap_data = */
    {
        0x00, 0x00,
        0xF8, 0x01,
        0x04, 0x02,
        0x02, 0x04,
        0x02, 0x04,
        0x02, 0x04,
        0x04, 0x02,
        0xF8, 0x01,
        0x10, 0x00,
        0x20, 0x00,
        0x40, 0x00,
        0x80, 0x00,
        0x00, 0x01,
        0x00, 0x00,
        0x00, 0x00,
        0x00, 0x00},
    /*.icon_color = */ ui::Color::cyan().v,
    /*.menu_location = */ app_location_t::RX,
    /*.desired_menu_position = */ -1,

    /*.m4_app_tag = */ {'P', 'C', 'A', 'P'},  // image_tag_capture (spectrum/FFT stream)
    /*.m4_app_offset = */ 0x00000000,         // filled at compile time
};
}
