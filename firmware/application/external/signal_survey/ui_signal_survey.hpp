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
 * Blocks: after each sweep the ranked hits are arranged into up to
 * block_count_ blocks (default 3). A block holds one modulation, and no two
 * of its hits are more than block_gap_khz_ apart (default 6 MHz). Each block
 * is anchored on the best-ranked hit not yet placed and takes the window of
 * block_gap around it that holds the most same-modulation hits. Its range is
 * padded by half a channel (block_spread_hz_, default 12.5 kHz) each side,
 * and that spread is also the channel step of the exported block list.
 * Unknown-modulation hits are not blocked.
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
static constexpr size_t kMaxBlocks = 8;
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
    uint8_t noise_floor;     // local noise floor (dB code) at detection
    uint16_t persistence;    // number of sweeps this signal was seen in
    int64_t bandwidth_hz;    // estimated occupied bandwidth
    int32_t score;           // ranking score (integer)
    SurveyMod mod;           // classification guess
    uint32_t occupancy_mask; // low kOccupancyPasses bits: active per recent pass
    bool used;
    int8_t block{-1};        // index into blocks_, -1 if in none
};

// Hits of one modulation lying within block_gap of each other.
struct SurveyBlock {
    SurveyMod mod;
    int64_t f_min;     // lowest member
    int64_t f_max;     // highest member
    int64_t range_lo;  // f_min - spread / 2
    int64_t range_hi;  // f_max + spread / 2
    uint8_t count;
    int16_t best_power;
    bool used;
};

// Which hits a FreqMan export takes.
enum class ExportFilter : uint8_t { All,
                                    Unknown,
                                    FM };

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

    // ---- blocks ----
    int32_t block_count_ = 3;          // blocks to arrange the hits into
    int32_t block_spread_hz_ = 12'500; // channel bandwidth / step of a block
    int32_t block_gap_khz_ = 6'000;    // max distance between hits of a block

    // ---- runtime scan state ----
    bool scanning_ = false;
    size_t range_idx_ = 0;
    int64_t cur_freq_ = 0;
    uint32_t sweep_count_ = 0;

    // individual frequencies scanned in addition to the ranges
    static constexpr size_t kMaxSingles = 8;
    int64_t single_freqs_[kMaxSingles]{};
    size_t single_count_ = 0;
    bool in_singles_ = false;   // true while sweeping the single-freq list
    size_t single_idx_ = 0;

    // per-channel FFT accumulation (integer; averaged over frames_per_ch_)
    uint32_t accum_[kSpecBins]{};
    uint32_t accum_sr_ = 0;
    int32_t frames_acc_ = 0;

    SurveyResult results_[kMaxResults]{};
    HopPair pairs_[kMaxResults]{};
    SurveyBlock blocks_[kMaxBlocks]{};
    size_t block_total_ = 0;

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
    void add_single_frequency(int64_t freq_hz);  // add a manual watch frequency

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

    // ---- blocks ----
    void build_blocks();            // fill blocks_ from the ranked results
    void show_block(size_t index);  // list a block's members
    void on_block_setting();        // re-group after a block setting change

    // ---- FreqMan export (plain lists; no TX) ----
    void save_results();
    uint32_t write_freqman(const std::string& stem, ExportFilter filter);
    uint32_t write_blocks_freqman(const std::string& stem);

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

    // Manual watch-frequency entry: scanned in addition to the ranges.
    FrequencyField field_add_freq_{{0 * 8, 4 * 16}};
    Button button_add_freq_{{12 * 8, 4 * 16, 6 * 8, 16}, "Add"};
    Button button_clear_freq_{{19 * 8, 4 * 16, 6 * 8, 16}, "Clr"};

    // Block settings: count, channel spread (Hz), max gap (kHz).
    Labels labels_blocks_{
        {{0 * 8, 5 * 16}, "Blk:  Spr:       Gap:      k", Theme::getInstance()->fg_light->foreground},
    };
    NumberField field_block_count_{{4 * 8, 5 * 16}, 1, {1, (int32_t)kMaxBlocks}, 1, ' '};
    NumberField field_block_spread_{{10 * 8, 5 * 16}, 6, {1'000, 500'000}, 500, ' '};
    NumberField field_block_gap_{{21 * 8, 5 * 16}, 5, {100, 20'000}, 100, ' '};

    // Scrollable, tappable results list. Hop pairs are listed first (tagged
    // HOP), then the blocks (B1..), then ranked signals with their block.
    // Selecting a signal parks the receiver on it, a block lists its members.
    // The menu is rebuilt at sweep boundaries, not every frame.
    MenuView menu_view_{
        {0, 6 * 16, screen_width, screen_height - (6 * 16)},
        true /* keep_highlight */};

    app_settings::SettingsManager settings_{
        "rx_signal_survey",
        app_settings::Mode::RX,
        {
            {"threshold_db"sv, &threshold_db_},
            {"target_sweeps"sv, &target_sweeps_},
            {"frames_per_ch"sv, &frames_per_ch_},
            {"detect_hops"sv, &detect_hops_},
            {"block_count"sv, &block_count_},
            {"block_spread_hz"sv, &block_spread_hz_},
            {"block_gap_khz"sv, &block_gap_khz_},
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
