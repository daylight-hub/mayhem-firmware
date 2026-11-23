/*
 * ui_jammer_settings.cpp - Settings for jammer system
 */

#include "ui_jammer_settings.hpp"

namespace ui {

JammerSettingsView::JammerSettingsView(NavigationView& nav) : nav_(nav) {
    add_children({
        &label_interval_,
        &field_interval_,
        &label_threshold_,
        &field_threshold_,
        &label_ranges_,
        &check_range1_,
        &check_range2_,
        &check_range3_,
        &button_save_,
        &button_cancel_,
    });
    
    // Load current settings
    field_interval_.set_value(jammer::Scanner::getUpdateInterval() / 60);
    
    // Range checkboxes
    check_range1_.set_value(true);
    check_range2_.set_value(true);
    check_range3_.set_value(true);
    
    button_save_.on_select = [this](Button&) {
        saveSettings();
        nav_.pop();
    };
    
    button_cancel_.on_select = [this](Button&) {
        nav_.pop();
    };
}

void JammerSettingsView::saveSettings() {
    // Save scan interval (minutes to seconds)
    uint32_t minutes = field_interval_.value();
    jammer::Scanner::setUpdateInterval(minutes * 60);
    
    // Save range settings
    // (would persist to config file)
}

} // namespace ui
