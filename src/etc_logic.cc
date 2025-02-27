#include <etc_logic.hh>

#include <cmath>
#include <cstdint>
#include <numeric>

namespace etc {
namespace {

std::uint16_t adc_distance(std::uint16_t a, std::uint16_t b) {
    return a > b ? a - b : b - a;
}

float sigmoid(float x, float k, float x0) {
    return 1.0f / (1.0f + std::exp(-k * (x - x0)));
}

float sigmoid_normal(float x, float k, float x0) {
    float s0 = sigmoid(0, k, x0);
    return (sigmoid(x, k, x0) - s0) / (1.0f - s0);
}

} // namespace

std::uint16_t CalibrationData::map_value(const std::uint16_t value) {
    // Check absolute plausibility.
    if (value < k_absolute_delta || value > (k_adc_range - k_absolute_delta)) {
        // TODO
    }

    // Check plausibility against calibrated endpoints.
    // if (

    // Offset value to start of calibrated range and calculated a linear factor scaled by xxxx.
    const auto offset_value = static_cast<std::uint32_t>(std::max(value, m_min_value) - m_min_value) * 2096u;
    auto normalised = std::min(offset_value / (m_max_value - m_min_value), 2095ul);

    // Flip sensor reading.
    // TODO: Should be done in calibration step?
    // normalised = 2096u - normalised;

    return m_lookup_table[normalised];
}

bool CalibrationData::update_calibration(const std::uint16_t value) {
    if (m_start_value == UINT16_MAX) {
        m_start_value = value;
    }
    m_min_value = std::min(m_min_value, value);
    m_max_value = std::max(m_max_value, value);
    
    // Use the lookup table as a ring buffer.
    m_lookup_table[m_ring_index] = value;
    m_ring_index = (m_ring_index + 1) % 20;

    // Check if the pedal has moved enough from its starting position (the position of the pedal when calibration was
    // initiated). If not, don't complete calibration yet.
    if (adc_distance(value, m_start_value) < k_minimum_range) {
        return false;
    }

    // Calculate the average sensor value to check if the pedal is still moving. If so, don't complete calibration yet.
    const auto ring_sum = std::accumulate(m_lookup_table.begin(), std::next(m_lookup_table.begin(), 20), 0u);
    const auto average = static_cast<std::uint16_t>(ring_sum / 20u);
    if (adc_distance(value, average) > 10) {
        return false;
    }

    auto sigmoid = [](float x) {
        return 1.0f / (1.0f + std::exp(-12.0f * (x - 0.45f)));
    };
    auto sigmoid_zero = sigmoid(0.0f);

    float f = 

    // Otherwise, calibration is complete, so we can construct the lookup table.
    for (std::uint32_t i = 0; i < m_lookup_table.size(); i++) {
        auto normalised = static_cast<float>(i) / 2096.0f;
        normalised = 1.0f - normalised;

        float curve = (sigmoid(normalised) - sigmoid_zero) / (1.0f - sigmoid_zero);
        m_lookup_table[i] = static_cast<std::uint16_t>(curve * 1000.0f);
    }
    return true;
}

} // namespace etc
