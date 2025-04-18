#pragma once

#include <array>
#include <cstdint>

namespace bms {

enum class Request : std::uint8_t {
    Disable = 0x10,
    Enable = 0x20,
    MeasureRail = 0x30,
    Sample = 0x40,
};

struct I2cResponse {
    // For a given thermistor index, the bit corresponding to that index is:
    //   0 if the thermistor is disconnected or otherwise reading out of range.
    //   1 if the thermistor is connected and reading properly;
    std::uint32_t thermistor_bitset;

    // For a given cell index, the bit corresponding to that index is:
    //   0 if the cell tap is connected;
    //   1 if the cell tap is disconnected.
    // std::uint16_t bad_cell_tap_bitset;

    // For a given cell index, the bit corresponding to that index is:
    //   0 if the cell voltage reading is as expected;
    //   1 if the reading is noisy or otherwise less reliable than normal.
    // std::uint16_t degraded_bitset;

    // 3V3 rail voltage in 100 uV resolution.
    std::uint16_t rail_voltage;

    // Cell voltages in 100 uV resolution.
    std::array<std::uint16_t, 12> voltages;

    // Thermistor temperatures to the nearest degree.
    std::array<std::uint8_t, 23> temperatures;
};

} // namespace bms
