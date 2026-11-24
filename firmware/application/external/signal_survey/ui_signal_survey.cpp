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
        &field_add_freq_, &button_add_freq_, &button_clear_freq_,
        &labels_blocks_, &field_block_count_, &field_block_spread_, &field_block_gap_,
        &menu_view_,
    });

    field_block_count_.set_value(block_count_);
    field_block_count_.on_change = [this](int32_t v) { block_count_ = v; on_block_setting(); };
    field_block_spread_.set_value(block_spread_hz_);
    field_block_spread_.on_change = [this](int32_t v) { block_spread_hz_ = v; on_block_setting(); };
    field_block_gap_.set_value(block_gap_khz_);
    field_block_gap_.on_change = [this](int32_t v) { block_gap_khz_ = v; on_block_setting(); };

    field_threshold_.set_value(threshold_db_);
    field_threshold_.on_change = [this](int32_t v) { threshold_db_ = v; };

    button_start_.on_select = [this](Button&) { start_scan(); };
    button_stop_.on_select = [this](Button&) { stop_scan(); };
    button_save_.on_select = [this](Button&) { save_results(); };

    field_add_freq_.set_value(433'000'000);
    button_add_freq_.on_select = [this](Button&) {
        add_single_frequency(field_add_freq_.value());
    };
    button_clear_freq_.on_select = [this](Button&) {
        single_count_ = 0;
        text_status_.set("Watch freqs cleared");
    };

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
    for (auto& b : blocks_) b = SurveyBlock{};
    block_total_ = 0;
    sweep_count_ = 0;
    in_singles_ = false;
    single_idx_ = 0;
    range_idx_ = 0;
    while (range_idx_ < 3 && !ranges_[range_idx_].enabled) range_idx_++;
    const bool have_ranges = (range_idx_ < 3);

    if (!have_ranges && single_count_ == 0) {
        text_status_.set("No ranges or watch freqs");
        return;
    }

    scanning_ = true;
    receiver_model.enable();
    baseband::spectrum_streaming_start();
    if (have_ranges) {
        retune(ranges_[range_idx_].start_hz);
    } else {
        in_singles_ = true;  // watch-frequency-only scan
        retune(single_freqs_[0]);
    }
    text_status_.set("Scanning...");
}

void SignalSurveyView::stop_scan() {
    if (!scanning_) return;
    scanning_ = false;
    baseband::spectrum_streaming_stop();
    receiver_model.disable();
    rank_results();
    build_blocks();
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
    // Watch-frequency phase: step through the manual list, then end the sweep.
    if (in_singles_) {
        single_idx_++;
        if (single_idx_ >= single_count_) {
            finish_sweep();
            return;
        }
        retune(single_freqs_[single_idx_]);
        return;
    }

    SurveyRange& range = ranges_[range_idx_];
    int64_t next = cur_freq_ + range.step_hz;

    if (next > range.end_hz) {
        do {
            range_idx_++;
        } while (range_idx_ < 3 && !ranges_[range_idx_].enabled);

        if (range_idx_ >= 3) {
            // ranges done -> sweep the watch frequencies, or finish
            if (single_count_ > 0) {
                in_singles_ = true;
                single_idx_ = 0;
                retune(single_freqs_[0]);
                return;
            }
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
    build_blocks();
    if (detect_hops_) correlate_pairs();
    refresh_list();

    if (target_sweeps_ > 0 && sweep_count_ >= (uint32_t)target_sweeps_) {
        stop_scan();
        text_status_.set("Done: " + to_string_dec_uint(sweep_count_) + " sweeps");
        return;
    }

    // begin a new pass: shift occupancy history left, then restart sweep
    mark_pass_occupancy();
    in_singles_ = false;
    single_idx_ = 0;
    range_idx_ = 0;
    while (range_idx_ < 3 && !ranges_[range_idx_].enabled) range_idx_++;
    if (range_idx_ < 3) {
        retune(ranges_[range_idx_].start_hz);
    } else if (single_count_ > 0) {
        in_singles_ = true;  // watch-frequency-only scan
        retune(single_freqs_[0]);
    } else {
        stop_scan();
    }
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
            r.noise_floor = f.noise_floor;
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
            r.noise_floor = f.noise_floor;
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
        weakest->noise_floor = f.noise_floor;
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
/* blocks                                                              */
/* ------------------------------------------------------------------ */

// "462.550"
static std::string block_mhz(int64_t f) {
    return to_string_dec_uint((uint32_t)(f / 1'000'000)) + "." +
           to_string_dec_uint((uint32_t)((f / 1000) % 1000), 3, '0');
}

// Arrange the hits into blocks of one modulation whose hits are at most
// block_gap apart. Each block is anchored on the best-ranked hit not placed
// yet and takes the window [lo, lo + gap] around it that holds the most
// same-modulation hits (stronger hits break ties). Unknowns are not blocked.
void SignalSurveyView::build_blocks() {
    for (auto& b : blocks_) b = SurveyBlock{};
    for (auto& r : results_) r.block = -1;
    block_total_ = 0;

    const int64_t gap = (int64_t)block_gap_khz_ * 1000;
    const int64_t half_spread = block_spread_hz_ / 2;
    const size_t max_blocks = block_count_ < 1 ? 1 : (block_count_ > (int32_t)kMaxBlocks ? kMaxBlocks : (size_t)block_count_);

    auto placeable = [](const SurveyResult& r) {
        return r.used && r.block < 0 && r.mod != SurveyMod::Unknown;
    };

    while (block_total_ < max_blocks) {
        // Anchor: best-ranked hit still free.
        int32_t anchor = -1;
        for (size_t i = 0; i < kMaxResults; i++)
            if (placeable(results_[i]) && (anchor < 0 || results_[i].score > results_[anchor].score))
                anchor = (int32_t)i;
        if (anchor < 0) break;

        const SurveyMod mod = results_[anchor].mod;
        const int64_t fa = results_[anchor].frequency_hz;

        // Windows holding the anchor start at a same-modulation hit within gap below it.
        int64_t best_lo = fa;
        int32_t best_weight = -1;
        for (const auto& c : results_) {
            if (!placeable(c) || c.mod != mod || c.frequency_hz > fa || fa - c.frequency_hz > gap) continue;
            int32_t weight = 0;
            for (const auto& m : results_)
                if (placeable(m) && m.mod == mod && m.frequency_hz >= c.frequency_hz && m.frequency_hz - c.frequency_hz <= gap)
                    weight += 1000 + m.power_db;
            if (weight > best_weight) {
                best_weight = weight;
                best_lo = c.frequency_hz;
            }
        }

        SurveyBlock& b = blocks_[block_total_];
        b.used = true;
        b.mod = mod;
        b.f_min = fa;
        b.f_max = fa;
        b.best_power = -32000;
        for (auto& m : results_) {
            if (!placeable(m) || m.mod != mod || m.frequency_hz < best_lo || m.frequency_hz - best_lo > gap) continue;
            m.block = (int8_t)block_total_;
            b.count++;
            if (m.frequency_hz < b.f_min) b.f_min = m.frequency_hz;
            if (m.frequency_hz > b.f_max) b.f_max = m.frequency_hz;
            if (m.power_db > b.best_power) b.best_power = m.power_db;
        }
        b.range_lo = b.f_min > half_spread ? b.f_min - half_spread : 0;
        b.range_hi = b.f_max + half_spread;
        block_total_++;
    }
}

void SignalSurveyView::on_block_setting() {
    build_blocks();
    refresh_list();
}

void SignalSurveyView::show_block(size_t index) {
    if (index >= block_total_) return;
    const SurveyBlock& b = blocks_[index];
    std::string msg = std::string(mod_name(b.mod)) + ", " + to_string_dec_uint(b.count) + " hits\n" +
                      block_mhz(b.range_lo) + "-" + block_mhz(b.range_hi) + " MHz\n" +
                      "Spread " + to_string_dec_uint(block_spread_hz_) + " Hz\n";
    size_t listed = 0;
    for (const auto& r : results_) {
        if (!r.used || r.block != (int8_t)index) continue;
        if (++listed > 8) {
            msg += "...";
            break;
        }
        msg += to_string_short_freq(r.frequency_hz) + " p" + to_string_dec_int(r.power_db) +
               " nf" + to_string_dec_uint(r.noise_floor) + "\n";
    }
    nav_.display_modal("Block " + to_string_dec_uint(index + 1), msg);
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

    for (size_t i = 0; i < block_total_; i++) {
        const SurveyBlock& b = blocks_[i];
        menu_view_.add_item(
            {"B" + to_string_dec_uint(i + 1) + " " + mod_name(b.mod) + " x" +
                 to_string_dec_uint(b.count) + " " + block_mhz(b.range_lo) + "-" +
                 block_mhz(b.range_hi),
             Theme::getInstance()->fg_yellow->foreground,
             nullptr,
             [this, i](KeyEvent) { this->show_block(i); }});
    }

    for (const auto& r : results_) {
        if (!r.used) continue;
        const int64_t f = r.frequency_hz;
        menu_view_.add_item(
            {to_string_short_freq(f) + " " + mod_name(r.mod) + " " +
                 to_string_dec_int(r.power_db) + " " +
                 to_string_dec_uint((uint32_t)(r.bandwidth_hz / 1000)) + "k x" +
                 to_string_dec_uint(r.persistence) +
                 (r.block >= 0 ? " B" + to_string_dec_uint(r.block + 1) : std::string{}),
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

// Add a manual watch frequency; these are swept in addition to the ranges.
void SignalSurveyView::add_single_frequency(int64_t freq_hz) {
    if (single_count_ >= kMaxSingles) {
        text_status_.set("Watch list full");
        return;
    }
    single_freqs_[single_count_++] = freq_hz;
    text_status_.set("Watch " + to_string_short_freq(freq_hz) + " (" +
                     to_string_dec_uint((uint32_t)single_count_) + ")");
}

/* ------------------------------------------------------------------ */
/* FreqMan export (plain lists; no TX)                                 */
/* ------------------------------------------------------------------ */

// NOTE: confirm the persistence call against firmware/common/freqman_db.cpp
// on your checkout -- some revisions flush on close(), others need an
// explicit save. The structure below targets the open/append/close API.
uint32_t SignalSurveyView::write_freqman(const std::string& stem, ExportFilter filter) {
    create_freqman_file(stem);

    FreqmanDB db;
    if (!db.open(freqman_dir / (stem + ".TXT"), /*create=*/true))
        return 0;

    uint32_t written = 0;
    for (const auto& r : results_) {
        if (!r.used) continue;
        if (filter == ExportFilter::Unknown && r.mod != SurveyMod::Unknown) continue;
        if (filter == ExportFilter::FM && r.mod != SurveyMod::NFM && r.mod != SurveyMod::WFM) continue;

        freqman_entry e{};
        e.type = freqman_type::Single;
        e.frequency_a = r.frequency_hz;
        // Store the full capture data for each hit: modulation, signal power,
        // noise floor, and occupied bandwidth.
        e.description =
            std::string(mod_name(r.mod)) +
            " p" + to_string_dec_int(r.power_db) +
            " nf" + to_string_dec_uint(r.noise_floor) +
            " bw" + to_string_dec_uint((uint32_t)(r.bandwidth_hz / 1000)) + "k" +
            (r.block >= 0 ? " B" + to_string_dec_uint(r.block + 1) : std::string{});
        db.append_entry(e);
        written++;
    }
    db.close();
    return written;
}

// Each block as a FreqMan range: modulation, bandwidth and channel step from
// block_spread_hz_, ready for Recon. Digital/unset modulations are left out
// of the entry so the loading app keeps its own.
uint32_t SignalSurveyView::write_blocks_freqman(const std::string& stem) {
    create_freqman_file(stem);

    FreqmanDB db;
    if (!db.open(freqman_dir / (stem + ".TXT"), /*create=*/true))
        return 0;

    // freqman indices are positions in its option lists (see freqman_db.cpp)
    freqman_index_t step = freqman_invalid_index;
    for (size_t i = 0; i < freqman_steps.size(); i++)
        if (freqman_steps[i].second == block_spread_hz_) step = (freqman_index_t)i;

    uint32_t written = 0;
    for (size_t i = 0; i < block_total_; i++) {
        const SurveyBlock& b = blocks_[i];
        freqman_entry e{};
        e.type = freqman_type::Range;
        e.frequency_a = b.range_lo;
        e.frequency_b = b.range_hi;
        e.step = step;
        switch (b.mod) {
            case SurveyMod::AM:
                e.modulation = 0;                                  // AM
                e.bandwidth = block_spread_hz_ >= 9'000 ? 0 : 1;  // DSB 9k / DSB 6k
                break;
            case SurveyMod::CW:
                e.modulation = 0;  // AM
                e.bandwidth = 4;   // CW
                break;
            case SurveyMod::NFM:
                e.modulation = 1;  // NFM: 8k5 / 11k / 12k5 / 16k
                e.bandwidth = block_spread_hz_ <= 8'500 ? 0 : block_spread_hz_ <= 11'000 ? 1
                                                          : block_spread_hz_ <= 12'500   ? 2
                                                                                         : 3;
                break;
            case SurveyMod::WFM:
                e.modulation = 2;  // WFM
                e.bandwidth = 2;   // 200k
                break;
            default:
                break;
        }
        e.description = "B" + to_string_dec_uint(i + 1) + " " + mod_name(b.mod) + " x" +
                        to_string_dec_uint(b.count) + " p" + to_string_dec_int(b.best_power);
        db.append_entry(e);
        written++;
    }
    db.close();
    return written;
}

void SignalSurveyView::save_results() {
    uint32_t all = write_freqman("SIGNAL_SURVEY", ExportFilter::All);
    uint32_t unk = write_freqman("SIGNAL_SURVEY_UNK", ExportFilter::Unknown);
    uint32_t fm = write_freqman("SIGNAL_SURVEY_FM", ExportFilter::FM);
    uint32_t blk = write_blocks_freqman("SIGNAL_SURVEY_BLK");
    text_status_.set("Saved " + to_string_dec_uint(all) + " (" +
                     to_string_dec_uint(unk) + " unk " + to_string_dec_uint(fm) + " FM " +
                     to_string_dec_uint(blk) + " blk)");
}

}  // namespace ui::external_app::signal_survey
