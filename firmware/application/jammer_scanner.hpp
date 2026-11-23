/*
 * jammer_scanner.hpp - Header for jammer scanning system
 */

#pragma once

#include <vector>
#include <string>
#include <map>
#include <mutex>
#include <thread>
#include <chrono>
#include <cstdint>

namespace jammer {

// Configuration structure
struct ScannerConfig {
    uint32_t update_interval_sec;
    uint32_t scan_timeout_sec;
    double power_threshold_db;
    uint32_t top_n_per_range;
    uint32_t capture_seconds;
    uint32_t lna_gain;
    uint32_t vga_gain;
    uint32_t hop_block_ms;
    double hop_on_threshold_ratio;
    uint32_t min_hops_for_flag;
    uint32_t pair_sweeps;
    double pair_correlation_threshold;
    double high_power_db;
    uint32_t tone_min_hz;
    uint32_t tone_max_hz;
    double tone_prominence_db;
};

// Frequency range for scanning
struct FrequencyRange {
    std::string name;
    double start_mhz;
    double end_mhz;
    bool enabled;
    double step_mhz = 0.1;
};

// Signal information
struct SignalInfo {
    double frequency_mhz;
    double power_db;
    std::string modulation;
    double bandwidth_hz;
    bool has_tone = false;
    bool is_hopping = false;
    uint32_t persistence_count = 0;
    double priority_score = 0.0;
};

// Hopping pair detection
struct HoppingPair {
    double freq_a;
    double freq_b;
    double correlation;
    uint64_t last_seen;
};

// Smart signal ranking
struct SmartSignal {
    double frequency_mhz;
    std::string modulation;
    double power_db;
    double bandwidth_hz;
    double score;
};

// Smart list container
struct SmartList {
    std::vector<SmartSignal> signals;
    std::vector<HoppingPair> hopping_pairs;
};

// Transmit block
struct TransmitBlock {
    double start_mhz;
    double end_mhz;
    double center_mhz;
    double width_mhz;
    std::string modulation;
    std::string waveform;
    std::vector<SignalInfo> frequencies;
};

// Scan results
struct ScanResults {
    std::vector<SignalInfo> frequencies;
    uint64_t last_scan_time;
    bool is_scanning;
};

// Complex sample type
struct ComplexSample {
    float i;
    float q;
};

// Main scanner class
class Scanner {
public:
    static void init();
    static void start();
    static void stop();
    
    static const ScanResults& getResults() { return g_scan_results; }
    static const SmartList& getSmartList() { return g_smart_list; }
    static const std::vector<TransmitBlock>& getBlocks() { return g_active_blocks; }
    
    static void setUpdateInterval(uint32_t seconds) {
        g_config.update_interval_sec = seconds;
    }
    
    static uint32_t getUpdateInterval() {
        return g_config.update_interval_sec;
    }
    
private:
    static void performScan();
    static SignalInfo analyzeFrequency(double freq_mhz);
    static std::string detectModulation(const std::vector<ComplexSample>& samples);
    static void detectHoppingPairs(const SignalInfo& signal);
    static void generateSmartList();
    static void generateTransmitBlocks();
    static void finalizeBlock(TransmitBlock& block);
    static double calculateSmartScore(const SignalInfo& signal);
    
    static void configureReceiver(double freq_mhz);
    static std::vector<ComplexSample> captureSamples(uint32_t seconds);
    static double calculatePower(const std::vector<ComplexSample>& samples);
    static double estimateBandwidth(const std::vector<ComplexSample>& samples);
    static bool detectTone(const std::vector<ComplexSample>& samples);
    static double calculatePriority(const SignalInfo& signal);
    static double calculateCorrelation(const SignalInfo& a, const SignalInfo& b);
    
    static std::vector<float> computeFFT(const std::vector<ComplexSample>& samples);
    static bool isAMSignal(const std::vector<float>& spectrum);
    static bool isFMSignal(const std::vector<ComplexSample>& samples);
    static bool isDigitalSignal(const std::vector<float>& spectrum);
    static bool isCWSignal(const std::vector<float>& spectrum);
    static bool isHoppingSignal(const std::vector<ComplexSample>& samples);
    static bool isPulsedSignal(const std::vector<ComplexSample>& samples);
    
    static void saveState();
    static void loadState();
    static void saveBlocks();
    
    static uint64_t getCurrentTime();
    
    static ScannerConfig g_config;
    static std::vector<FrequencyRange> g_ranges;
    static ScanResults g_scan_results;
    static SmartList g_smart_list;
    static std::vector<TransmitBlock> g_active_blocks;
    static std::mutex g_results_mutex;
};

// Configuration constants (matching your Python config)
constexpr bool USE_MANUAL = true;
constexpr bool USE_SCANNED = true;
constexpr bool USE_SMART = true;

constexpr double BLOCK_GAP_MHZ = 5.0;
constexpr double BLOCK_MARGIN_MHZ = 1.0;
constexpr double BLOCK_MIN_WIDTH_MHZ = 1.0;
constexpr double BLOCK_MAX_WIDTH_MHZ = 24.0;
constexpr bool BLOCK_MATCH_MODULATION = true;

constexpr uint32_t SMART_TOP_N = 15;
constexpr double SMART_MIN_SCORE = 1.0;

constexpr double W_PERSISTENCE = 3.0;
constexpr double W_HOPPING = 4.0;
constexpr double W_WAVES = 3.0;
constexpr double W_POWER = 0.02;
constexpr double W_PAIR = 2.0;
constexpr double W_TONE = 1.0;

const std::map<std::string, double> W_MOD = {
    {"digital", 2.5}, {"fm", 2.0}, {"am", 2.0},
    {"hopping", 3.0}, {"pulsed", 1.5}, {"cw", 0.5},
    {"noise", -1.0}
};

const std::map<std::string, std::string> MOD_TO_WAVE = {
    {"cw", "carrier"},
    {"am", "sine"},
    {"fm", "fm_tone"},
    {"nbfm", "fm_tone"},
    {"fm_capture", "fm_tone"},
    {"fsk", "rand_fsk"},
    {"digital", "rand_fsk"},
    {"hopping", "brute"},
    {"pulsed", "carrier"},
    {"noise", "noise"},
    {"unknown", "noise"}
};

} // namespace jammer
