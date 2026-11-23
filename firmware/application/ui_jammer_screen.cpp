/*
 * ui_jammer_screen.cpp - Visual interface for jammer system
 */

#include "ui_jammer_screen.hpp"
#include "portapack.hpp"
#include "event_m0.hpp"

using namespace portapack;

namespace ui {

JammerScreen::JammerScreen(NavigationView& nav) : nav_(nav) {
    add_children({
        &labels_,
        &status_view_,
        &button_scan_,
        &button_stop_,
        &button_settings_,
        &frequency_list_,
        &block_view_,
        &signal_info_,
        &progress_bar_,
    });
    
    // Initialize UI elements
    button_scan_.on_select = [this](Button&) {
        startScan();
    };
    
    button_stop_.on_select = [this](Button&) {
        stopScan();
    };
    
    button_settings_.on_select = [this](Button&) {
        nav_.push<JammerSettingsView>();
    };
    
    // Update display periodically
    refresh_timer_ = std::make_unique<Timer>([this]() {
        updateDisplay();
    });
    refresh_timer_->start(std::chrono::milliseconds(1000));
}

void JammerScreen::startScan() {
    jammer::Scanner::start();
    status_view_.set_text("Scanning...");
    button_scan_.set_text("Scanning");
    button_scan_.set_style(&Styles::red);
}

void JammerScreen::stopScan() {
    jammer::Scanner::stop();
    status_view_.set_text("Stopped");
    button_scan_.set_text("Start Scan");
    button_scan_.set_style(&Styles::default_style);
}

void JammerScreen::updateDisplay() {
    const auto& results = jammer::Scanner::getResults();
    const auto& blocks = jammer::Scanner::getBlocks();
    
    // Update frequency list
    frequency_list_.clear();
    for (const auto& freq : results.frequencies) {
        char buffer[64];
        snprintf(buffer, sizeof(buffer), 
                 "%6.1f MHz  %s  %3.1f dBm",
                 freq.frequency_mhz, freq.modulation.c_str(), freq.power_db);
        frequency_list_.add_item(buffer);
    }
    
    // Update block view
    block_view_.clear();
    for (const auto& block : blocks) {
        char buffer[64];
        snprintf(buffer, sizeof(buffer),
                 "%6.1f-%6.1f MHz  %s",
                 block.start_mhz, block.end_mhz, block.modulation.c_str());
        block_view_.add_item(buffer);
    }
    
    // Update signal info
    if (!results.frequencies.empty()) {
        const auto& strongest = results.frequencies[0];
        char buffer[128];
        snprintf(buffer, sizeof(buffer),
                 "Strongest: %.1f MHz\n"
                 "Modulation: %s\n"
                 "Power: %.1f dBm\n"
                 "Bandwidth: %.0f kHz",
                 strongest.frequency_mhz,
                 strongest.modulation.c_str(),
                 strongest.power_db,
                 strongest.bandwidth_hz / 1000.0);
        signal_info_.set_text(buffer);
    }
    
    // Update progress
    uint32_t elapsed = getCurrentTime() - results.last_scan_time;
    uint32_t interval = jammer::Scanner::getUpdateInterval();
    if (interval > 0) {
        progress_bar_.set_value((elapsed * 100) / interval);
    }
}

void JammerScreen::paint(Painter& painter) {
    // Custom drawing for nice visual
    painter.fill_rectangle(
        {0, 0, screen_width, screen_height},
        Color::black()
    );
    
    // Draw header
    painter.draw_string(
        {20, 10},
        Styles::white,
        "Smart Jammer System"
    );
    
    // Draw frequency spectrum placeholder
    drawSpectrum(painter);
}

void JammerScreen::drawSpectrum(Painter& painter) {
    // Simple spectrum visualization
    const auto& results = jammer::Scanner::getResults();
    
    int y_base = 150;
    int bar_width = 10;
    int bar_gap = 2;
    
    for (size_t i = 0; i < results.frequencies.size() && i < 20; i++) {
        const auto& freq = results.frequencies[i];
        
        // Normalize power to bar height
        int height = static_cast<int>(
            (freq.power_db + 100.0) * 2.0
        );
        height = std::max(10, std::min(100, height));
        
        // Color based on priority
        Color bar_color = Color::green();
        if (freq.priority_score > 5.0) {
            bar_color = Color::red();
        } else if (freq.priority_score > 3.0) {
            bar_color = Color::yellow();
        }
        
        painter.fill_rectangle(
            {20 + static_cast<int>(i * (bar_width + bar_gap)),
             y_base - height,
             bar_width,
             height},
            bar_color
        );
    }
}

} // namespace ui
