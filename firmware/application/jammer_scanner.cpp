/*
 * jammer_scanner.cpp - Frequency scanning and priority detection
 * Part of mayhem-firmware jammer system
 * 
 * This module handles:
 * - Hourly frequency sweeps across configured ranges
 * - Signal detection and classification
 * - Priority frequency list generation
 * - Hopping pair detection
 */

#include "jammer_scanner.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>

namespace jammer {

// Configuration defaults (overridable via UI)
static ScannerConfig g_config = {
    .update_interval_sec = 3600,      // 1 hour default
    .scan_timeout_sec = 1500,
    .power_threshold_db = -72.0,
    .top_n_per_range = 4,
    .capture_seconds = 3,
    .lna_gain = 24,
    .vga_gain = 36,
    .hop_block_ms = 20,
    .hop_on_threshold_ratio = 3.0,
    .min_hops_for_flag = 4,
    .pair_sweeps = 12,
    .pair_correlation_threshold = -0.4,
    .high_power_db = -58.0,
    .tone_min_hz = 200,
    .tone_max_hz = 3000,
    .tone_prominence_db = 12
};

// Frequency ranges for scanning
static std::vector<FrequencyRange> g_ranges = {
    { "manual1", 88.0, 108.0, true },    // FM broadcast
    { "manual2", 400.0, 470.0, true },   // UHF
    { "manual3", 800.0, 900.0, true }    // Cellular bands
};

// Results storage
static ScanResults g_scan_results;
static SmartList g_smart_list;
static std::vector<TransmitBlock> g_active_blocks;

// Mutex for thread safety
static std::mutex g_results_mutex;

void Scanner::init() {
    g_scan_results.last_scan_time = 0;
    g_scan_results.is_scanning = false;
    g_scan_results.frequencies.clear();
    g_smart_list.signals.clear();
    g_active_blocks.clear();
    
    // Load any saved state
    loadState();
}

void Scanner::start() {
    if (g_scan_results.is_scanning) return;
    
    g_scan_results.is_scanning = true;
    g_scan_results.last_scan_time = getCurrentTime();
    
    // Start scanning thread
    std::thread([this]() {
        while (g_scan_results.is_scanning) {
            performScan();
            generateSmartList();
            generateTransmitBlocks();
            saveState();
            
            // Sleep until next scan
            std::this_thread::sleep_for(
                std::chrono::seconds(g_config.update_interval_sec)
            );
        }
    }).detach();
}

void Scanner::stop() {
    g_scan_results.is_scanning = false;
}

void Scanner::performScan() {
    std::lock_guard<std::mutex> lock(g_results_mutex);
    
    g_scan_results.frequencies.clear();
    
    for (const auto& range : g_ranges) {
        if (!range.enabled) continue;
        
        // Sweep the range in segments
        double freq = range.start_mhz;
        while (freq < range.end_mhz) {
            // Capture and analyze signal at this frequency
            SignalInfo signal = analyzeFrequency(freq);
            
            if (signal.power_db > g_config.power_threshold_db) {
                g_scan_results.frequencies.push_back(signal);
                
                // Check for hopping patterns
                detectHoppingPairs(signal);
            }
            
            // Move to next frequency
            freq += range.step_mhz;
        }
    }
    
    // Sort by priority (power * persistence)
    std::sort(g_scan_results.frequencies.begin(),
              g_scan_results.frequencies.end(),
              [](const SignalInfo& a, const SignalInfo& b) {
                  return a.priority_score > b.priority_score;
              });
    
    // Keep only top N
    if (g_scan_results.frequencies.size() > g_config.top_n_per_range * g_ranges.size()) {
        g_scan_results.frequencies.resize(
            g_config.top_n_per_range * g_ranges.size()
        );
    }
}

SignalInfo Scanner::analyzeFrequency(double freq_mhz) {
    SignalInfo signal;
    signal.frequency_mhz = freq_mhz;
    
    // Configure HackRF for this frequency
    configureReceiver(freq_mhz);
    
    // Capture samples
    std::vector<ComplexSample> samples = captureSamples(
        g_config.capture_seconds
    );
    
    if (samples.empty()) {
        signal.power_db = -100.0;
        signal.modulation = "unknown";
        return signal;
    }
    
    // Calculate power
    signal.power_db = calculatePower(samples);
    
    // Detect modulation type
    signal.modulation = detectModulation(samples);
    
    // Detect bandwidth
    signal.bandwidth_hz = estimateBandwidth(samples);
    
    // Check for tones
    signal.has_tone = detectTone(samples);
    
    // Calculate priority score
    signal.priority_score = calculatePriority(signal);
    
    return signal;
}

std::string Scanner::detectModulation(const std::vector<ComplexSample>& samples) {
    // FFT analysis
    auto spectrum = computeFFT(samples);
    
    // Check for AM (carrier + sidebands)
    if (isAMSignal(spectrum)) {
        return "am";
    }
    
    // Check for FM (constant envelope, varying frequency)
    if (isFMSignal(samples)) {
        return "fm";
    }
    
    // Check for digital (wideband, high crest factor)
    if (isDigitalSignal(spectrum)) {
        return "digital";
    }
    
    // Check for CW (narrow, strong carrier)
    if (isCWSignal(spectrum)) {
        return "cw";
    }
    
    // Check for hopping (multiple peaks over time)
    if (isHoppingSignal(samples)) {
        return "hopping";
    }
    
    // Check for pulsed
    if (isPulsedSignal(samples)) {
        return "pulsed";
    }
    
    return "unknown";
}

void Scanner::detectHoppingPairs(const SignalInfo& signal) {
    // Look for frequencies that alternate with this one
    for (auto& existing : g_scan_results.frequencies) {
        // Check if signals are complementary (hopping pair)
        double correlation = calculateCorrelation(signal, existing);
        
        if (correlation < g_config.pair_correlation_threshold) {
            HoppingPair pair;
            pair.freq_a = signal.frequency_mhz;
            pair.freq_b = existing.frequency_mhz;
            pair.correlation = correlation;
            pair.last_seen = getCurrentTime();
            
            g_smart_list.hopping_pairs.push_back(pair);
        }
    }
}

void Scanner::generateSmartList() {
    std::lock_guard<std::mutex> lock(g_results_mutex);
    
    g_smart_list.signals.clear();
    
    // Weight and rank all detected signals
    for (const auto& signal : g_scan_results.frequencies) {
        SmartSignal smart;
        smart.frequency_mhz = signal.frequency_mhz;
        smart.modulation = signal.modulation;
        smart.power_db = signal.power_db;
        smart.bandwidth_hz = signal.bandwidth_hz;
        
        // Calculate weighted score
        smart.score = calculateSmartScore(signal);
        
        g_smart_list.signals.push_back(smart);
    }
    
    // Sort by score
    std::sort(g_smart_list.signals.begin(),
              g_smart_list.signals.end(),
              [](const SmartSignal& a, const SmartSignal& b) {
                  return a.score > b.score;
              });
    
    // Keep top N
    if (g_smart_list.signals.size() > SMART_TOP_N) {
        g_smart_list.signals.resize(SMART_TOP_N);
    }
    
    // Add hopping pairs to smart list
    for (const auto& pair : g_smart_list.hopping_pairs) {
        SmartSignal hop_a;
        hop_a.frequency_mhz = pair.freq_a;
        hop_a.modulation = "hopping";
        hop_a.score = 10.0; // High priority for hopping
        g_smart_list.signals.push_back(hop_a);
        
        SmartSignal hop_b;
        hop_b.frequency_mhz = pair.freq_b;
        hop_b.modulation = "hopping";
        hop_b.score = 10.0;
        g_smart_list.signals.push_back(hop_b);
    }
}

double Scanner::calculateSmartScore(const SignalInfo& signal) {
    double score = 0.0;
    
    // Persistence bonus
    score += signal.persistence_count * W_PERSISTENCE;
    
    // Hopping bonus
    if (signal.is_hopping) {
        score += W_HOPPING;
    }
    
    // Modulation weights
    auto mod_it = W_MOD.find(signal.modulation);
    if (mod_it != W_MOD.end()) {
        score += mod_it->second;
    }
    
    // Power contribution
    score += (signal.power_db + 100.0) * W_POWER;
    
    // Tone bonus
    if (signal.has_tone) {
        score += W_TONE;
    }
    
    return score;
}

void Scanner::generateTransmitBlocks() {
    std::lock_guard<std::mutex> lock(g_results_mutex);
    
    g_active_blocks.clear();
    
    // Combine all sources
    std::vector<SignalInfo> all_signals;
    
    // Add manual frequencies
    if (USE_MANUAL) {
        for (const auto& manual : MANUAL_FREQUENCIES) {
            SignalInfo sig;
            sig.frequency_mhz = manual.mhz;
            sig.bandwidth_hz = manual.bandwidth * 1e6;
            sig.modulation = "manual";
            sig.priority_score = 100.0; // Always highest
            all_signals.push_back(sig);
        }
    }
    
    // Add scanned frequencies
    if (USE_SCANNED) {
        all_signals.insert(all_signals.end(),
            g_scan_results.frequencies.begin(),
            g_scan_results.frequencies.end());
    }
    
    // Add smart list
    if (USE_SMART) {
        for (const auto& smart : g_smart_list.signals) {
            SignalInfo sig;
            sig.frequency_mhz = smart.frequency_mhz;
            sig.bandwidth_hz = smart.bandwidth_hz;
            sig.modulation = smart.modulation;
            sig.priority_score = smart.score;
            all_signals.push_back(sig);
        }
    }
    
    // Sort by frequency for clustering
    std::sort(all_signals.begin(), all_signals.end(),
              [](const SignalInfo& a, const SignalInfo& b) {
                  return a.frequency_mhz < b.frequency_mhz;
              });
    
    // Cluster into blocks
    if (all_signals.empty()) return;
    
    TransmitBlock current_block;
    current_block.start_mhz = all_signals[0].frequency_mhz - BLOCK_MARGIN_MHZ;
    current_block.end_mhz = all_signals[0].frequency_mhz + BLOCK_MARGIN_MHZ;
    current_block.center_mhz = all_signals[0].frequency_mhz;
    current_block.frequencies.push_back(all_signals[0]);
    
    for (size_t i = 1; i < all_signals.size(); i++) {
        double gap = all_signals[i].frequency_mhz - 
                    all_signals[i-1].frequency_mhz;
        
        if (gap <= BLOCK_GAP_MHZ) {
            // Add to current block
            current_block.end_mhz = all_signals[i].frequency_mhz + BLOCK_MARGIN_MHZ;
            current_block.frequencies.push_back(all_signals[i]);
        } else {
            // Finalize current block
            finalizeBlock(current_block);
            g_active_blocks.push_back(current_block);
            
            // Start new block
            current_block = TransmitBlock();
            current_block.start_mhz = all_signals[i].frequency_mhz - BLOCK_MARGIN_MHZ;
            current_block.end_mhz = all_signals[i].frequency_mhz + BLOCK_MARGIN_MHZ;
            current_block.center_mhz = all_signals[i].frequency_mhz;
            current_block.frequencies.push_back(all_signals[i]);
        }
    }
    
    // Add last block
    finalizeBlock(current_block);
    g_active_blocks.push_back(current_block);
    
    // Save blocks to file
    saveBlocks();
}

void Scanner::finalizeBlock(TransmitBlock& block) {
    // Calculate block width
    block.width_mhz = block.end_mhz - block.start_mhz;
    
    // Enforce min/max width
    if (block.width_mhz < BLOCK_MIN_WIDTH_MHZ) {
        block.width_mhz = BLOCK_MIN_WIDTH_MHZ;
        block.end_mhz = block.start_mhz + BLOCK_MIN_WIDTH_MHZ;
    }
    
    if (block.width_mhz > BLOCK_MAX_WIDTH_MHZ) {
        block.width_mhz = BLOCK_MAX_WIDTH_MHZ;
        block.end_mhz = block.start_mhz + BLOCK_MAX_WIDTH_MHZ;
    }
    
    // Determine modulation for block
    if (BLOCK_MATCH_MODULATION && !block.frequencies.empty()) {
        // Use most common modulation
        std::map<std::string, int> mod_count;
        for (const auto& freq : block.frequencies) {
            mod_count[freq.modulation]++;
        }
        
        auto max_it = std::max_element(mod_count.begin(), mod_count.end(),
            [](const auto& a, const auto& b) {
                return a.second < b.second;
            });
        
        block.modulation = max_it->first;
        
        // Map to transmit waveform
        auto wave_it = MOD_TO_WAVE.find(block.modulation);
        if (wave_it != MOD_TO_WAVE.end() && wave_it->second) {
            block.waveform = wave_it->second;
        } else {
            block.waveform = TRANSMIT_TYPE;
        }
    }
}

void Scanner::saveState() {
    // Save scan results to JSON
    saveToJson(JSON_PATH, g_scan_results);
    
    // Save smart list
    saveToJson(SMART_LIST_PATH, g_smart_list);
    
    // Save active blocks
    saveToJson(BLOCKS_FILE, g_active_blocks);
}

void Scanner::loadState() {
    // Load previous state if exists
    loadFromJson(JSON_PATH, g_scan_results);
    loadFromJson(SMART_LIST_PATH, g_smart_list);
    loadFromJson(BLOCKS_FILE, g_active_blocks);
}

} // namespace jammer
