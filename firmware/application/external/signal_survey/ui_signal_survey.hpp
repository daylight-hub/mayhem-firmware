/*
 * Copyright (C) 2026
 *
 * This file is part of PortaPack.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Signal Survey: a receive-only band scanner with spectral classification
 * and a coarse hop-pair detector.
 *
 * It sweeps the configured ranges, and at each channel pulls averaged FFT
 * frames (ChannelSpectrum, 256 bins) from the capture baseband. From each
 * averaged frame it extracts spectral features (noise floor, occupied
 * bandwidth, crest factor, in-band variance, carrier symmetry) and makes a
 * modulation guess (AM / NFM / WFM / Digital / CW / Unknown). Unknowns are
 * kept and exported to a separate list. A second pass tracks per-candidate
 * occupancy over time and flags anti-correlated pairs as candidate
 * frequency-hopping pairs.
 *
 * RECEIVE ONLY. No transmit path, not linked to any transmitter/jammer.
 * FreqMan output is a plain frequency list.
 *
 * STATUS: untested scaffolding. The DSP is written for the hardware model
 * (M0 integer math, capture-baseband FFT), but every threshold marked
 * TUNE must be calibrated on a real device, and it must be compiled with
 * the ARM toolchain and bench-tested before any PR (see AGENT.md).
 */

#ifndef _UI_SIGNAL_SURVEY_H
#define _UI_SIGNAL_SURVEY_H

#include "ui.hpp"
#include "ui_menu.hpp"
#include "ui_navigation.hpp"
#include "ui_receiver.hpp"
#include "receiver_model.hpp"
#include "radio_state.hpp"
#include "baseband_api.hpp"
#include "freqman_db.hpp"
#include "app_settings.hpp"
#include "message.hpp"

#include <array>
#include <cstdint>

namespace ui::external_app::signal_survey {

static constexpr size_t kMaxResults = 24;
static constexpr size_t kSpecBins = 256;         // ChannelSpectrum::db size
static constexpr int64_t kMergeToleranceHz = 50'000;

// Occupancy history depth (passes) used for hop-pair correlation.
static constexpr size_t kOccupancyPasses = 32;

enum class SurveyMod : uint8_t { Unknown,
                                 AM,
                                 NFM,
                                 WFM,
                                 Digital,
                                 CW };

const char* mod_name(SurveyMod m);

// Spectral features derived from one averaged 256-bin FFT frame.
// All values are in the ChannelSpectrum dB code space (0..255) or Hz.
struct SpectrumFeatures {
    uint8_t noise_floor;     // low-percentile bin level
    uint8_t peak;            // max bin level
    uint8_t mean;            // mean bin level
    uint16_t peak_bin;       // index of the max bin
    uint16_t occupied_bins;  // bins above (noise_floor + margin)
    int64_t occupied_bw_hz;  // occupied_bins * bin_hz
    uint8_t crest;           // peak - mean (peak-to-average, dB code)
    uint8_t band_var;        // spread of in-band bins (flat vs peaky)
    uint8_t symmetry;        // sideband balance around the peak (0=asym..255=sym)
    bool valid;
};

struct SurveyRange {
    int64_t start_hz;
    int64_t end_hz;
    int64_t step_hz;
    bool enabled;
};

struct SurveyResult {
    int64_t frequency_hz;
    int16_t power_db;        // best power seen (peak bin, dB code)
    uint16_t persistence;    // number of sweeps this signal was seen in
    int64_t bandwidth_hz;    // estimated occupied bandwidth
    int32_t score;           // ranking score (integer)
    SurveyMod mod;           // classification guess
    uint32_t occupancy_mask; // low kOccupancyPasses bits: active per recent pass
    bool used;
};

// A detected candidate frequency-hopping pair.
struct HopPair {
    int64_t freq_a;
    int64_t freq_b;
    uint8_t anticorr;        // 0..100 %: passes where exactly one was active
    bool used;
};

class SignalSurveyView : public View {
   public:
    explicit SignalSurveyView(NavigationView& nav);
    ~SignalSurveyView();

    SignalSurveyView(const SignalSurveyView&) = delete;
    SignalSurveyView& operator=(const SignalSurveyView&) = delete;

    void focus() override;
    std::string title() const override { return "SigSurvey"; }

   private:
    NavigationView& nav_;
    RxRadioState radio_state_{};

    // ---- scan configuration ----
    SurveyRange ranges_[3]{
        {88'000'000, 108'000'000, 100'000, true},
        {430'000'000, 440'000'000, 25'000, true},
        {0, 0, 25'000, false},
    };

    int32_t threshold_db_ = 80;    // min peak (dB code) above noise to log  [TUNE]
    int32_t target_sweeps_ = 4;    // 0 = continuous
    int32_t frames_per_ch_ = 4;    // FFT frames averaged per channel dwell
    int32_t detect_hops_ = 1;      // 1 = run hop-pair correlation

    // ---- runtime scan state ----
    bool scanning_ = false;
    size_t range_idx_ = 0;
    int64_t cur_freq_ = 0;
    uint32_t sweep_count_ = 0;

    // per-channel FFT accumulation (integer; averaged over frames_per_ch_)
    uint32_t accum_[kSpecBins]{};
    uint32_t accum_sr_ = 0;
    int32_t frames_acc_ = 0;

    SurveyResult results_[kMaxResults]{};
    HopPair pairs_[kMaxResults]{};

    // ---- scan engine ----
    void start_scan();
    void stop_scan();
    void advance_channel();
    void finish_sweep();
    void rank_results();
    void refresh_list();
    void configure_radio();
    void retune(int64_t freq_hz);
    void select_frequency(int64_t freq_hz);  // park receiver on a tapped row

    // ---- spectrum intake + DSP ----
    void drain_spectrum();
    void process_frame(const ChannelSpectrum& frame);
    void finish_channel();                               // called after frames averaged
    SpectrumFeatures extract_features(const uint32_t* bins, size_t n, uint32_t sr);
    SurveyMod classify(const SpectrumFeatures& f);
    void record_detection(int64_t freq_hz, const SpectrumFeatures& f, SurveyMod mod);

    // ---- hop-pair correlation ----
    void mark_pass_occupancy();     // shift occupancy history by one pass
    void correlate_pairs();         // fill pairs_ from occupancy masks

    // ---- FreqMan export (plain lists; no TX) ----
    void save_results();
    uint32_t write_freqman(const std::string& stem, bool unknown_only);

    // ---- widgets ----
    Labels labels_{
        {{0 * 8, 0 * 16}, "LNA   VGA   AMP  Thr:", Theme::getInstance()->fg_light->foreground},
    };
    LNAGainField field_lna_{{4 * 8, 0 * 16}};
    VGAGainField field_vga_{{10 * 8, 0 * 16}};
    RFAmpField field_rf_amp_{{16 * 8, 0 * 16}};
    NumberField field_threshold_{{22 * 8, 0 * 16}, 3, {0, 200}, 2, ' '};

    Text text_status_{{0, 1 * 16 + 4, screen_width, 16}, "Idle"};

    Button button_start_{{0 * 8, 2 * 16, 7 * 8, 2 * 16}, "Start"};
    Button button_stop_{{8 * 8, 2 * 16, 7 * 8, 2 * 16}, "Stop"};
    Button button_save_{{16 * 8, 2 * 16, 13 * 8, 2 * 16}, "Save FreqMan"};

    // Scrollable, tappable results list. Hop pairs are listed first (tagged
    // HOP), then ranked signals. Selecting a row parks the receiver on that
    // frequency. The menu is rebuilt at sweep boundaries, not every frame.
    MenuView menu_view_{
        {0, 4 * 16, screen_width, screen_height - (4 * 16)},
        true /* keep_highlight */};

    app_settings::SettingsManager settings_{
        "rx_signal_survey",
        app_settings::Mode::RX,
        {
            {"threshold_db"sv, &threshold_db_},
            {"target_sweeps"sv, &target_sweeps_},
            {"frames_per_ch"sv, &frames_per_ch_},
            {"detect_hops"sv, &detect_hops_},
        }};

    // Baseband hands us the FFT FIFO via a config message; we drain it on
    // each DisplayFrameSync (same pattern as the spectrum waterfall).
    ChannelSpectrumFIFO* channel_fifo_{nullptr};

    MessageHandlerRegistration message_handler_spectrum_config_{
        Message::ID::ChannelSpectrumConfig,
        [this](const Message* const p) {
            channel_fifo_ = reinterpret_cast<const ChannelSpectrumConfigMessage*>(p)->fifo;
        }};

    MessageHandlerRegistration message_handler_frame_sync_{
        Message::ID::DisplayFrameSync,
        [this](const Message* const) { this->drain_spectrum(); }};
};

}  // namespace ui::external_app::signal_survey

#endif  // _UI_SIGNAL_SURVEY_H
