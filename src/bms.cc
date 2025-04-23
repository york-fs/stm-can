#include <bms.hh>
#include <hal.hh>
#include <stm32f103xb.h>
#include <util.hh>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <tuple>
#include <utility>

namespace {

// Number of ADC samples to perform for rail voltage, cell voltage, and thermistor measurements respectively.
constexpr std::size_t k_rail_sample_count = 1024;
constexpr std::size_t k_cell_sample_count = 64;
constexpr std::size_t k_thermistor_sample_count = 8;

// Cell degraded threshold in ADC counts.
constexpr std::uint16_t k_cell_degraded_threshold = 10;

// Open cell tap voltage threshold in 100 uV resolution.
constexpr std::uint16_t k_cell_open_threshold = 1000;

// Maximum number of connected thermistors, including the onboard ones.
constexpr std::uint8_t k_max_thermistor_count = 23;

// Voltage threshold from the absolute endpoints (0 and Vref) in 100 uV resolution from when to consider a thermistor as
// being either open or short circuit.
constexpr std::uint32_t k_thermistor_range_threshold = 3000;

// Hard-coded value of the on-board precision voltage reference in 100 uV resolution.
constexpr std::uint16_t k_reference_voltage = 45000;

// MAX14920 product and die version bits.
constexpr std::uint8_t k_afe_version_bits = 0b1010;

enum class AfeStatus {
    Ready,
    NotReady,
    BadSpi,
    Shutdown,
};

std::array s_thermistor_enable{
    hal::Gpio(hal::GpioPort::A, 0), hal::Gpio(hal::GpioPort::A, 1), hal::Gpio(hal::GpioPort::A, 2),
    hal::Gpio(hal::GpioPort::A, 3), hal::Gpio(hal::GpioPort::A, 4), hal::Gpio(hal::GpioPort::A, 5),
    hal::Gpio(hal::GpioPort::A, 6), hal::Gpio(hal::GpioPort::A, 7),
};
static_assert(s_thermistor_enable.size() * 3 >= k_max_thermistor_count);

hal::Gpio s_ref_en(hal::GpioPort::B, 1);
hal::Gpio s_led(hal::GpioPort::B, 5);
hal::Gpio s_scl(hal::GpioPort::B, 6);
hal::Gpio s_sda(hal::GpioPort::B, 7);
hal::Gpio s_adc_cs(hal::GpioPort::B, 9);
hal::Gpio s_afe_cs(hal::GpioPort::B, 10);
hal::Gpio s_afe_en(hal::GpioPort::B, 11);
hal::Gpio s_sck(hal::GpioPort::B, 13);
hal::Gpio s_miso(hal::GpioPort::B, 14);
hal::Gpio s_mosi(hal::GpioPort::B, 15);

std::optional<bms::Command> i2c_handle(const bms::SegmentData &data) {
    if ((I2C1->SR1 & I2C_SR1_ADDR) == 0u) {
        // Address not matched.
        return std::nullopt;
    }

    // Clear address matched status by reading SR2.
    const std::uint32_t sr2 = I2C1->SR2;

    // Check if master is sending us a command.
    if ((sr2 & I2C_SR2_TRA) == 0u) {
        // Receive the single command byte.
        std::uint32_t timeout = 200;
        while ((I2C1->SR1 & I2C_SR1_RXNE) == 0u) {
            if (--timeout == 0) {
                return std::nullopt;
            }
        }
        return static_cast<bms::Command>(I2C1->DR);
    }

    // Otherwise master wants to read our data.
    std::array<std::uint8_t, sizeof(bms::SegmentData)> bytes;
    auto it = bytes.begin();
    auto append_value = [&](auto value) {
        auto value_bytes = util::write_be(value);
        it = std::copy(value_bytes.begin(), value_bytes.end(), it);
    };
    append_value(data.thermistor_bitset);
    append_value(data.cell_tap_bitset);
    append_value(data.degraded_bitset);
    append_value(data.rail_voltage);
    for (std::uint16_t voltage : data.voltages) {
        append_value(voltage);
    }
    for (std::uint8_t temperature : data.temperatures) {
        *it++ = temperature;
    }

    // TODO: Rewrite this.
    std::uint32_t index = 0;
    std::uint32_t timeout = 100;
    while (true) {
        if ((I2C1->SR1 & I2C_SR1_AF) != 0u) {
            // NACK received.
            I2C1->SR1 &= ~I2C_SR1_AF;
            break;
        }

        if (--timeout == 0) {
            break;
        }

        if ((I2C1->SR1 & I2C_SR1_TXE) != 0u) {
            if (index < bytes.size()) {
                I2C1->DR = bytes[index++];
                timeout = 100;
            } else {
                I2C1->DR = 0xff;
            }
        }
    }
    return std::nullopt;
}

void spi_transfer(const hal::Gpio &cs, std::span<std::uint8_t> data) {
    // Pull CS low and create a scope guard to pull it high again on return.
    // TODO: Add timeouts.
    util::ScopeGuard cs_guard([&cs] {
        hal::gpio_set(cs);
    });
    hal::gpio_reset(cs);

    // Transmit each byte one-by-one.
    for (auto &byte : data) {
        // Transmit byte.
        while ((SPI2->SR & SPI_SR_TXE) != SPI_SR_TXE) {
            __NOP();
        }
        SPI2->DR = byte;

        // Receive byte.
        while ((SPI2->SR & SPI_SR_RXNE) != SPI_SR_RXNE) {
            __NOP();
        }
        byte = SPI2->DR;
    }

    // Wait for busy to be clear and then reset CS to high.
    while ((SPI2->SR & SPI_SR_BSY) != 0u) {
        __NOP();
    }
}

std::uint16_t adc_sample_raw() {
    // Trigger conversion.
    hal::gpio_reset(s_adc_cs);
    hal::gpio_set(s_adc_cs);

    // Read value over SPI.
    std::array<std::uint8_t, 2> bytes{};
    spi_transfer(s_adc_cs, bytes);

    // Return assembled value.
    return (static_cast<std::uint16_t>(bytes[0]) << 8u) | bytes[1];
}

std::pair<std::uint16_t, std::uint16_t> adc_sample_voltage(std::size_t sample_count) {
    std::uint16_t min_value = std::numeric_limits<std::uint16_t>::max();
    std::uint16_t max_value = 0;
    std::uint32_t sum = 0;
    for (std::size_t i = 0; i < sample_count; i++) {
        const auto value = adc_sample_raw();
        min_value = std::min(min_value, value);
        max_value = std::max(max_value, value);
        sum += value;
    }

    const auto average = sum / sample_count;
    const auto voltage = (average * k_reference_voltage) >> 16u;
    return std::make_pair(voltage, max_value - min_value);
}

[[nodiscard]] AfeStatus afe_command(std::uint16_t balance_bits, std::uint8_t control_bits) {
    std::array<std::uint8_t, 3> data{
        static_cast<std::uint8_t>(balance_bits >> 8u),
        static_cast<std::uint8_t>(balance_bits),
        control_bits,
    };
    spi_transfer(s_afe_cs, data);

    // Check version bits are correct.
    if ((data[2] >> 4u) != k_afe_version_bits) {
        return AfeStatus::BadSpi;
    }

    // Check UVLO and thermal shutdown bits.
    if ((data[2] & 0b1101u) != 0u) {
        return AfeStatus::Shutdown;
    }

    // Check ready bit.
    if ((data[2] & 0b10u) != 0u) {
        return AfeStatus::NotReady;
    }
    return AfeStatus::Ready;
}

std::optional<std::pair<std::uint16_t, bool>> sample_cell_voltage(std::uint8_t index) {
    // Select cell for output on AOUT in hold mode. The level shift and AOUT settle delay should pass before the first
    // ADC acquisition occurs.
    constexpr std::array<std::uint8_t, 12> index_table{
        0b10000000, 0b11000000, 0b10100000, 0b11100000, 0b10010000, 0b11010000,
        0b10110000, 0b11110000, 0b10001000, 0b11001000, 0b10101000, 0b11101000,
    };
    if (afe_command(0u, index_table[index] | 0b100u) != AfeStatus::Ready) {
        return std::nullopt;
    }

    // Take successive ADC samples to obtain an average voltage reading. Check whether the cell tap is open by checking
    // closeness to the ADC reading endpoints.
    const auto [voltage, adc_range] = adc_sample_voltage(k_cell_sample_count);
    if (voltage < k_cell_open_threshold || voltage > (k_reference_voltage - k_cell_open_threshold)) {
        return std::nullopt;
    }
    return std::make_pair(voltage, adc_range > k_cell_degraded_threshold);
}

std::optional<std::int8_t> sample_thermistor(std::uint16_t rail_voltage, std::uint8_t index) {
    // Enable the relevant MOSFET and create a scope guard to turn it off at the end.
    const auto &mosfet = s_thermistor_enable[index / 3];
    util::ScopeGuard cs_guard([&mosfet] {
        hal::gpio_reset(mosfet);
    });
    hal::gpio_set(mosfet);

    // Route the relevant Tx input to AOUT.
    constexpr std::array<std::uint8_t, 3> index_table{
        0b01011000u, // T1 (buffered)
        0b00111000u, // T2 (buffered)
        0b01111000u, // T3 (buffered)
    };
    if (afe_command(0u, index_table[index % 3]) != AfeStatus::Ready) {
        return std::nullopt;
    }

    // Take ADC samples and check if the voltage reading is viable. The min ensures that a bad rail voltage doesn't
    // result in false readings.
    const auto voltage = std::min(adc_sample_voltage(k_thermistor_sample_count).first, rail_voltage);
    if (voltage < k_thermistor_range_threshold || voltage > (rail_voltage - k_thermistor_range_threshold)) {
        return std::nullopt;
    }

    // Thermistor is connected, so we can calculate the temperature.
    // TODO: Use a lookup table/don't use floats here.
    const auto resistance = (voltage * 10000) / (rail_voltage - voltage);
    float beta = 1.0f / 3950.0f;
    if (index < 3) {
        // The onboard thermistors have a different beta.
        beta = 1.0f / 3350.0f;
    }
    const float t0 = 1.0f / 298.15f;
    const float temperature = 1.0f / (t0 + beta * std::log(static_cast<float>(resistance) / 10000.0f)) - 273.15f;
    return static_cast<std::int8_t>(temperature);
}

} // namespace

bool hal_low_power() {
    // Use the 8 MHz internal clock.
    return true;
}

void app_main() {
    // Configure general GPIOs.
    for (const auto &gpio : s_thermistor_enable) {
        gpio.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    }
    s_ref_en.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    s_led.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    s_afe_en.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);

    // Configure CS pin and enable a pull-up on MISO to avoid floating when no slave is selected.
    s_adc_cs.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    s_afe_cs.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    s_miso.configure(hal::GpioInputMode::PullUp);

    // Enable clocks for I2C1 and SPI2.
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN | RCC_APB1ENR_SPI2EN;

    // Enable external interrupt on SCL (PB6).
    AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI6_PB;
    EXTI->EMR |= EXTI_EMR_MR6;
    EXTI->FTSR |= EXTI_FTSR_TR6;

    // TODO: Don't hardcode the I2C address.
    const std::uint8_t i2c_address = 0x58u;
    bms::SegmentData data{};
    while (true) {
        // Wait a bit to allow a repeated start to be captured.
        hal::delay_us(100);
        i2c_handle(data);

        // Reconfigure SCK and MOSI as regular GPIOs before going to sleep.
        s_sck.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
        s_mosi.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);

        // Pull CS lines high by default (active-low) and put the ADC into shutdown.
        hal::gpio_set(s_adc_cs, s_afe_cs, s_sck);
        hal::gpio_reset(s_adc_cs);
        hal::gpio_set(s_adc_cs);

        // Reconfigure SCL as a regular input for use as an external event and enter stop mode. Also reconfigure SDA to
        // avoid the STM driving it low and upsetting the isolator.
        s_scl.configure(hal::GpioInputMode::Floating);
        s_sda.configure(hal::GpioInputMode::Floating);
        hal::enter_stop_mode();

        // Configure SCL and SDA for use with the I2C peripheral.
        s_scl.configure(hal::GpioOutputMode::AlternateOpenDrain, hal::GpioOutputSpeed::Max2);
        s_sda.configure(hal::GpioOutputMode::AlternateOpenDrain, hal::GpioOutputSpeed::Max2);

        // Reset the I2C and SPI peripherals just in case.
        RCC->APB1RSTR = RCC_APB1RSTR_I2C1RST | RCC_APB1RSTR_SPI2RST;
        RCC->APB1RSTR = 0u;

        // Configure I2C peripheral.
        I2C1->OAR1 = i2c_address << 1u;
        I2C1->CR2 = 8u;
        I2C1->CR1 = I2C_CR1_ACK | I2C_CR1_PE;

        // Wait a maximum of 10 ms for address match.
        hal::wait_equal(I2C1->SR1, I2C_SR1_ADDR, I2C_SR1_ADDR, 10);

        const auto request = i2c_handle(data);
        if (!request) {
            // Data not for us or spurious wakeup - go back to sleep.
            continue;
        }

        switch (*request) {
            using enum bms::Command;
        case Enable:
            hal::gpio_set(s_afe_en, s_ref_en);
            break;
        case Disable:
            hal::gpio_reset(s_afe_en, s_ref_en);
            break;
        case MeasureRail:
        case Sample:
            break;
        default:
            // Bad request.
            continue;
        }

        // Wake the ADC.
        hal::gpio_reset(s_sck, s_adc_cs);
        hal::gpio_set(s_adc_cs, s_afe_cs);

        // Configure SCK and MOSI for use with the SPI peripheral.
        s_sck.configure(hal::GpioOutputMode::AlternatePushPull, hal::GpioOutputSpeed::Max10);
        s_mosi.configure(hal::GpioOutputMode::AlternatePushPull, hal::GpioOutputSpeed::Max10);

        // Enable SPI2 in master mode at 2 MHz (4x divider).
        SPI2->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_SPE | SPI_CR1_BR_0 | SPI_CR1_MSTR;

        // Wait for AFE startup to complete. Route T1 (buffered) by default.
        while (afe_command(0, 0b01011000u) == AfeStatus::NotReady) {
            __NOP();
        }

        if (*request == bms::Command::MeasureRail) {
            // All thermistors are switched off so we can measure the 3V3 rail voltage directly.
            std::tie(data.rail_voltage, std::ignore) = adc_sample_voltage(k_rail_sample_count);
            continue;
        }

        // Sample all thermistors. Doing this first allows the sampling capacitors to top up a bit.
        data.thermistor_bitset = 0;
        std::fill(data.temperatures.begin(), data.temperatures.end(), 0);
        for (std::uint8_t index = 0; index < k_max_thermistor_count; index++) {
            const auto temperature = sample_thermistor(data.rail_voltage, index);
            if (temperature) {
                // Temperature reading is viable.
                data.thermistor_bitset |= 1u << index;
                data.temperatures[index] = *temperature;
            }
        }

        // Clear previously stored cell data.
        data.cell_tap_bitset = 0;
        data.degraded_bitset = 0;
        std::fill(data.voltages.begin(), data.voltages.end(), 0);

        // Sample all cells in order of most potential to least potential (w.r.t. ground).
        for (std::size_t cell = 12; cell > 0; cell--) {
            const auto index = static_cast<std::uint8_t>(cell - 1);
            const auto sample = sample_cell_voltage(index);
            if (sample) {
                // AFE is working and cell tap is connected.
                data.cell_tap_bitset |= 1u << index;
                data.voltages[index] = sample->first;
                if (sample->second) {
                    data.degraded_bitset |= 1u << index;
                }
            }
        }

        // Put the AFE into diagnostic mode.
        static_cast<void>(afe_command(0u, 0b01011010u));
    }
}
