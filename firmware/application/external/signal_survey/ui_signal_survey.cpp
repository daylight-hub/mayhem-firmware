/*
 * Copyright (C) 2026
 *
 * This file is part of PortaPack.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "ui_signal_survey.hpp"

#include "string_format.hpp"
#include "portapack.hpp"
#include "file_path.hpp"  // freqman_dir

using namespace portapack;

namespace ui::external_app::signal_survey {

// Thresholds marked [TUNE] are first-guess values and MUST be calibrated
// on real hardware against known signals. They are intentionally exposed
// here rather than buried.
static constexpr uint8_t kBinMargin = 12;   // dB code above noise floor = "occupied" [TUNE]
static constexpr uint8_t kCrestCW = 40;     // peak-to-avg for a bare carrier   [TUNE]
static constexpr uint8_t kCrestLowFM = 30;  // WFM has a low crest              [TUNE]
static constexpr uint8_t kSymAM = 150;      // AM sideband symmetry             [TUNE]
static constexpr uint8_t kFlatDigital = 12; // low in-band variance = flat      [TUNE]
static constexpr uint8_t kAnticorrHop = 60; // % anti-correlation for a pair    [TUNE]

const char* mod_name(SurveyMod m) {
    switch (m) {
        case SurveyMod::AM: return "AM";
        case SurveyMod::NFM: return "NFM";
        case SurveyMod::WFM: return "WFM";
        case SurveyMod::Digital: return "DIG";
        case SurveyMod::CW: return "CW";
        default: return "UNK";
    }
}

/* ------------------------------------------------------------------ */
/* construction / teardown                                             */
/* ------------------------------------------------------------------ */

SignalSurveyView::SignalSurveyView(NavigationView& nav)
    : nav_{nav} {
    add_children({
        &labels_, &field_lna_, &field_vga_, &field_rf_amp_, &field_threshold_,
        &text_status_, &button_start_, &button_stop_, &button_save_,
        &menu_view_,
    });

    field_threshold_.set_value(threshold_db_);
    field_threshold_.on_change = [this](int32_t v) { threshold_db_ = v; };

    button_start_.on_select = [this](Button&) { start_scan(); };
    button_stop_.on_select = [this](Button&) { stop_scan(); };
    button_save_.on_select = [this](Button&) { save_results(); };

    configure_radio();
    refresh_list();
}

SignalSurveyView::~SignalSurveyView() {
    baseband::spectrum_streaming_stop();
    receiver_model.disable();
    baseband::shutdown();
}

void SignalSurveyView::focus() {
    button_start_.focus();
}

// Use the capture baseband, which streams ChannelSpectrum (256 FFT bins).
// NOTE: exact capture sample-rate/oversample setup mirrors the SPEC path
// in external/level/ui_level.cpp (change_mode, SPEC_MODULATION). Replicate
// baseband::set_sample_rate()/get_oversample_rate() there if bins look
// mis-scaled on hardware.
void SignalSurveyView::configure_radio() {
    receiver_model.disable();
    baseband::shutdown();

    baseband::run_image(portapack::spi_flash::image_tag_capture);
    receiver_model.set_modulation(ReceiverModel::Mode::Capture);
    receiver_model.set_sampling_rate(3'072'000);
    receiver_model.set_baseband_bandwidth(1'750'000);
}

/* ------------------------------------------------------------------ */
/* scan control                                                        */
/* ------------------------------------------------------------------ */

void SignalSurveyView::retune(int64_t freq_hz) {
    cur_freq_ = freq_hz;
    frames_acc_ = 0;
    accum_sr_ = 0;
    for (auto& a : accum_) a = 0;
    receiver_model.set_target_frequency(freq_hz);
}

void SignalSurveyView::start_scan() {
    if (scanning_) return;

    for (auto& r : results_) r = SurveyResult{};
    for (auto& p : pairs_) p = HopPair{};
    sweep_count_ = 0;
    range_idx_ = 0;
    while (range_idx_ < 3 && !ranges_[range_idx_].enabled) range_idx_++;
    if (range_idx_ >= 3) {
        text_status_.set("No ranges enabled");
        return;
    }

    scanning_ = true;
    receiver_model.enable();
    baseband::spectrum_streaming_start();
    retune(ranges_[range_idx_].start_hz);
    text_status_.set("Scanning...");
}

void SignalSurveyView::stop_scan() {
    if (!scanning_) return;
    scanning_ = false;
    baseband::spectrum_streaming_stop();
    receiver_model.disable();
    rank_results();
    if (detect_hops_) correlate_pairs();
    refresh_list();
    text_status_.set("Stopped (" + to_string_dec_uint(sweep_count_) + " sweeps)");
}

// Pull every queued FFT frame; average frames_per_ch_ of them per channel.
void SignalSurveyView::drain_spectrum() {
    if (!scanning_ || !channel_fifo_) return;

    ChannelSpectrum frame;
    while (channel_fifo_->out(frame)) {
        process_frame(frame);
        if (frames_acc_ >= frames_per_ch_) {
            finish_channel();
            return;  // one channel settled per drain keeps the UI responsive
        }
    }
}

void SignalSurveyView::process_frame(const ChannelSpectrum& frame) {
    for (size_t i = 0; i < kSpecBins; i++)
        accum_[i] += frame.db[i];
    accum_sr_ = frame.sampling_rate;
    frames_acc_++;
}

void SignalSurveyView::finish_channel() {
    // average the accumulated bins back into an 8-bit working buffer
    uint32_t avg[kSpecBins];
    const int32_t n = frames_acc_ > 0 ? frames_acc_ : 1;
    for (size_t i = 0; i < kSpecBins; i++)
        avg[i] = accum_[i] / n;

    SpectrumFeatures f = extract_features(avg, kSpecBins, accum_sr_);

    if (f.valid && (int32_t)(f.peak - f.noise_floor) >= threshold_db_) {
        SurveyMod mod = classify(f);
        record_detection(cur_freq_, f, mod);
    }

    advance_channel();
}

void SignalSurveyView::advance_channel() {
    SurveyRange& range = ranges_[range_idx_];
    int64_t next = cur_freq_ + range.step_hz;

    if (next > range.end_hz) {
        do {
            range_idx_++;
        } while (range_idx_ < 3 && !ranges_[range_idx_].enabled);

        if (range_idx_ >= 3) {
            finish_sweep();
            return;
        }
        next = ranges_[range_idx_].start_hz;
    }
    retune(next);
}

void SignalSurveyView::finish_sweep() {
    sweep_count_++;
    rank_results();
    if (detect_hops_) correlate_pairs();
    refresh_list();

    if (target_sweeps_ > 0 && sweep_count_ >= (uint32_t)target_sweeps_) {
        stop_scan();
        text_status_.set("Done: " + to_string_dec_uint(sweep_count_) + " sweeps");
        return;
    }

    // begin a new pass: shift occupancy history left, then restart sweep
    mark_pass_occupancy();
    range_idx_ = 0;
    while (range_idx_ < 3 && !ranges_[range_idx_].enabled) range_idx_++;
    retune(ranges_[range_idx_].start_hz);
}

/* ------------------------------------------------------------------ */
/* feature extraction + classification                                 */
/* ------------------------------------------------------------------ */

// Derive spectral features from one averaged 256-bin frame. Bins are dB
// codes (0..255). All integer: no FPU on the M0.
SpectrumFeatures SignalSurveyView::extract_features(const uint32_t* bins, size_t n, uint32_t sr) {
    SpectrumFeatures f{};
    if (n == 0 || sr == 0) { f.valid = false; return f; }

    // histogram over the 0..255 code space -> cheap percentile + peak + mean
    uint16_t hist[256] = {0};
    uint32_t sum = 0, peak = 0, peak_bin = 0;
    for (size_t i = 0; i < n; i++) {
        uint32_t v = bins[i] > 255 ? 255 : bins[i];
        hist[v]++;
        sum += v;
        if (v > peak) { peak = v; peak_bin = i; }
    }
    f.peak = (uint8_t)peak;
    f.peak_bin = (uint16_t)peak_bin;
    f.mean = (uint8_t)(sum / n);

    // noise floor = 25th percentile
    uint32_t target = n / 4, cum = 0, floor = 0;
    for (uint32_t v = 0; v < 256; v++) { cum += hist[v]; if (cum >= target) { floor = v; break; } }
    f.noise_floor = (uint8_t)floor;

    const uint32_t thr = floor + kBinMargin;

    // occupied bins + in-band mean
    uint32_t occ = 0, inband_sum = 0;
    for (size_t i = 0; i < n; i++) {
        if (bins[i] > thr) { occ++; inband_sum += bins[i]; }
    }
    f.occupied_bins = (uint16_t)occ;
    const int64_t bin_hz = (int64_t)sr / (int64_t)n;
    f.occupied_bw_hz = (int64_t)occ * bin_hz;
    f.crest = (uint8_t)(peak > f.mean ? peak - f.mean : 0);

    // in-band variance proxy: mean absolute deviation of occupied bins
    if (occ > 0) {
        uint32_t ib_mean = inband_sum / occ, mad = 0;
        for (size_t i = 0; i < n; i++)
            if (bins[i] > thr) {
                uint32_t d = bins[i] > ib_mean ? bins[i] - ib_mean : ib_mean - bins[i];
                mad += d;
            }
        f.band_var = (uint8_t)(mad / occ > 255 ? 255 : mad / occ);
    } else {
        f.band_var = 0;
    }

    // carrier symmetry: compare energy on each side of the peak bin
    const int W = 16;
    uint32_t left = 0, right = 0;
    for (int k = 1; k <= W; k++) {
        if ((int)peak_bin - k >= 0) left += bins[peak_bin - k];
        if (peak_bin + k < n) right += bins[peak_bin + k];
    }
    uint32_t tot = left + right;
    if (tot > 0) {
        uint32_t diff = left > right ? left - right : right - left;
        f.symmetry = (uint8_t)(255 - (diff * 255 / tot));
    } else {
        f.symmetry = 0;
    }

    f.valid = true;
    return f;
}

// Decision tree over the extracted features. Rough by design; the [TUNE]
// thresholds decide the boundaries and need hardware calibration.
SurveyMod SignalSurveyView::classify(const SpectrumFeatures& f) {
    if (!f.valid) return SurveyMod::Unknown;

    const int64_t bw = f.occupied_bw_hz;

    if (f.occupied_bins <= 2 && f.crest >= kCrestCW)
        return SurveyMod::CW;

    if (bw > 0 && bw < 20'000 && f.crest >= kCrestCW && f.symmetry >= kSymAM)
        return SurveyMod::AM;

    if (bw >= 8'000 && bw <= 30'000)
        return SurveyMod::NFM;

    if (bw >= 80'000 && bw <= 260'000 && f.crest < kCrestLowFM)
        return SurveyMod::WFM;

    if (bw > 30'000 && f.band_var < kFlatDigital)
        return SurveyMod::Digital;

    return SurveyMod::Unknown;
}

/* ------------------------------------------------------------------ */
/* result table                                                        */
/* ------------------------------------------------------------------ */

void SignalSurveyView::record_detection(int64_t freq_hz, const SpectrumFeatures& f, SurveyMod mod) {
    const int16_t power = (int16_t)f.peak;

    // merge with a nearby existing entry
    for (auto& r : results_) {
        if (r.used && llabs(r.frequency_hz - freq_hz) <= kMergeToleranceHz) {
            if (power > r.power_db) r.power_db = power;
            if (r.persistence < 0xFFFF) r.persistence++;
            if (r.mod == SurveyMod::Unknown) r.mod = mod;  // upgrade if we learn more
            r.bandwidth_hz = f.occupied_bw_hz;
            r.occupancy_mask |= 1u;  // active this pass
            return;
        }
    }
    // free slot
    for (auto& r : results_) {
        if (!r.used) {
            r.used = true;
            r.frequency_hz = freq_hz;
            r.power_db = power;
            r.persistence = 1;
            r.bandwidth_hz = f.occupied_bw_hz;
            r.mod = mod;
            r.occupancy_mask = 1u;
            return;
        }
    }
    // replace weakest if stronger
    SurveyResult* weakest = &results_[0];
    for (auto& r : results_)
        if (r.power_db < weakest->power_db) weakest = &r;
    if (power > weakest->power_db) {
        weakest->frequency_hz = freq_hz;
        weakest->power_db = power;
        weakest->persistence = 1;
        weakest->bandwidth_hz = f.occupied_bw_hz;
        weakest->mod = mod;
        weakest->occupancy_mask = 1u;
    }
}

void SignalSurveyView::rank_results() {
    for (auto& r : results_) {
        if (!r.used) { r.score = -100000; continue; }
        // strength + persistence, with a nudge for classified (non-unknown) hits
        r.score = (r.power_db) + (int32_t)r.persistence * 15 +
                  (r.mod != SurveyMod::Unknown ? 20 : 0);
    }
    for (size_t i = 1; i < kMaxResults; i++) {
        SurveyResult key = results_[i];
        size_t j = i;
        while (j > 0 && results_[j - 1].score < key.score) {
            results_[j] = results_[j - 1];
            j--;
        }
        results_[j] = key;
    }
}

/* ------------------------------------------------------------------ */
/* hop-pair correlation                                                */
/* ------------------------------------------------------------------ */

// Shift every occupancy history left by one pass (bit 0 = most recent).
void SignalSurveyView::mark_pass_occupancy() {
    for (auto& r : results_)
        if (r.used) r.occupancy_mask <<= 1;
}

// Flag anti-correlated candidate pairs: two signals that tend to be active
// in alternate passes (A on when B off) are a candidate hop pair.
//
// LIMIT: a channel-stepping sweep has seconds-scale time resolution, so
// this catches only slow hoppers. Fast FHSS (ms) needs the whole span
// observed at once -- feed this same correlator from the wideband sweep
// (Looking Glass / SpectrumSweep) instead of the stepper for that.
void SignalSurveyView::correlate_pairs() {
    for (auto& p : pairs_) p = HopPair{};
    const uint32_t passes = sweep_count_ < kOccupancyPasses ? sweep_count_ : kOccupancyPasses;
    if (passes < 4) return;  // need a few passes before correlation means anything
    const uint32_t mask = (passes >= 32) ? 0xFFFFFFFFu : ((1u << passes) - 1);

    size_t out = 0;
    for (size_t i = 0; i < kMaxResults && out < kMaxResults; i++) {
        if (!results_[i].used) continue;
        for (size_t j = i + 1; j < kMaxResults && out < kMaxResults; j++) {
            if (!results_[j].used) continue;

            uint32_t a = results_[i].occupancy_mask & mask;
            uint32_t b = results_[j].occupancy_mask & mask;
            uint32_t either = a | b;
            if (!either) continue;

            uint32_t xor_count = __builtin_popcount(a ^ b);
            uint32_t either_count = __builtin_popcount(either);
            uint32_t ai = __builtin_popcount(a), bi = __builtin_popcount(b);
            if (ai < 2 || bi < 2) continue;  // both must actually appear

            uint8_t anticorr = (uint8_t)(xor_count * 100 / either_count);
            if (anticorr >= kAnticorrHop) {
                pairs_[out].used = true;
                pairs_[out].freq_a = results_[i].frequency_hz;
                pairs_[out].freq_b = results_[j].frequency_hz;
                pairs_[out].anticorr = anticorr;
                out++;
            }
        }
    }
}

/* ------------------------------------------------------------------ */
/* display                                                             */
/* ------------------------------------------------------------------ */

// Rebuild the scrollable list: hop pairs first (tagged HOP), then the
// ranked signals. Called at sweep boundaries and on stop, not per frame.
void SignalSurveyView::refresh_list() {
    menu_view_.clear();

    for (const auto& p : pairs_) {
        if (!p.used) continue;
        const int64_t fa = p.freq_a, fb = p.freq_b;
        menu_view_.add_item(
            {"HOP " + to_string_short_freq(fa) + "<>" +
                 to_string_short_freq(fb) + " " +
                 to_string_dec_uint(p.anticorr) + "%",
             Theme::getInstance()->fg_orange->foreground,
             nullptr,
             [this, fa](KeyEvent) { this->select_frequency(fa); }});
    }

    for (const auto& r : results_) {
        if (!r.used) continue;
        const int64_t f = r.frequency_hz;
        menu_view_.add_item(
            {to_string_short_freq(f) + " " + mod_name(r.mod) + " " +
                 to_string_dec_int(r.power_db) + " " +
                 to_string_dec_uint((uint32_t)(r.bandwidth_hz / 1000)) + "k x" +
                 to_string_dec_uint(r.persistence),
             r.mod == SurveyMod::Unknown
                 ? Theme::getInstance()->fg_light->foreground
                 : Theme::getInstance()->fg_green->foreground,
             nullptr,
             [this, f](KeyEvent) { this->select_frequency(f); }});
    }

    menu_view_.set_dirty();
}

// Park the receiver on a selected frequency. Stops the sweep and tunes
// there so you can inspect it. NOTE: this uses the capture baseband, so
// there is no audio; to actually listen, switch to an audio demod
// baseband (see external/level/ui_level.cpp change_mode) -- left out here
// to keep this a pure survey tool.
void SignalSurveyView::select_frequency(int64_t freq_hz) {
    if (scanning_) stop_scan();
    receiver_model.set_target_frequency(freq_hz);
    text_status_.set("Parked " + to_string_short_freq(freq_hz) + " MHz");
}

/* ------------------------------------------------------------------ */
/* FreqMan export (plain lists; no TX)                                 */
/* ------------------------------------------------------------------ */

// NOTE: confirm the persistence call against firmware/common/freqman_db.cpp
// on your checkout -- some revisions flush on close(), others need an
// explicit save. The structure below targets the open/append/close API.
uint32_t SignalSurveyView::write_freqman(const std::string& stem, bool unknown_only) {
    create_freqman_file(stem);

    freqman_db db;
    if (!db.open(freqman_dir / (stem + ".TXT"), /*create=*/true))
        return 0;

    uint32_t written = 0;
    for (const auto& r : results_) {
        if (!r.used) continue;
        if (unknown_only && r.mod != SurveyMod::Unknown) continue;

        freqman_entry e{};
        e.type = freqman_type::Single;
        e.frequency_a = r.frequency_hz;
        e.description =
            std::string(mod_name(r.mod)) + " " +
            to_string_dec_int(r.power_db) + " bw" +
            to_string_dec_uint((uint32_t)(r.bandwidth_hz / 1000)) + "k";
        db.append_entry(e);
        written++;
    }
    db.close();
    return written;
}

void SignalSurveyView::save_results() {
    uint32_t all = write_freqman("SIGNAL_SURVEY", false);
    uint32_t unk = write_freqman("SIGNAL_SURVEY_UNK", true);
    text_status_.set("Saved " + to_string_dec_uint(all) + " (" +
                     to_string_dec_uint(unk) + " unk)");
}

}  // namespace ui::external_app::signal_survey
