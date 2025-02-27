#pragma once

#include <array>
#include <cstdint>

namespace etc {

// Total range (resolution) of the ADC (12 bits).
constexpr std::uint16_t k_adc_range = 2u << 11u;

// Maximum tolerated delta between sensor ADC counts and the absolute endpoints [0, k_adc_range] before a sensor error
// is reported.
constexpr std::uint16_t k_absolute_delta = 1000;

// Maximum tolerated delta between sensor ADC counts and the calibrated endpoints before the system is deemed
// uncalibrated.
constexpr std::uint16_t k_relative_delta = 0;

// Minimum accepted total sensor range of the pedal in ADC counts.
constexpr std::uint16_t k_minimum_range = 300;

class CalibrationData {
    // Store the worst case lookup table size, which is the full ADC range minus the null zones at the endpoints. Most
    // entries will end up being zero since in practice the sensor range is ~600 ADC counts.
    std::array<std::uint16_t, k_adc_range - k_absolute_delta * 2u> m_lookup_table{};

    // Recorded calibrated endpoint values.
    std::uint16_t m_min_value{UINT16_MAX};
    std::uint16_t m_max_value{0};

    // Transient data used during calibration.
    std::uint16_t m_start_value{UINT16_MAX};
    std::uint16_t m_ring_index{0};

public:
    std::uint16_t map_value(std::uint16_t value);
    bool update_calibration(std::uint16_t value);

    std::uint16_t min_value() const { return m_min_value; }
    std::uint16_t max_value() const { return m_max_value; }
};

enum class State {
    CanOffline,
    Uncalibrated,
    Calibrating,
    CalibrationHold,
    Running,
};

} // namespace etc
