#include <bms.hh>
#include <hal.hh>
#include <max_adc.hh>
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

// Thermistor acceptable noise threshold in ADC counts. This value corresponds to around 100 mV.
constexpr std::uint16_t k_thermistor_noise_threshold = 1500;

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

// TCA9535 I2C registers.
enum class ExpanderRegister : std::uint8_t {
    InputPort0 = 0x00,
    InputPort1 = 0x01,
    OutputPort0 = 0x02,
    OutputPort1 = 0x03,
    PolarityPort0 = 0x04,
    PolarityPort1 = 0x05,
    ConfigurationPort0 = 0x06,
    ConfigurationPort1 = 0x07,
};

std::array s_address_pins{
    hal::Gpio(hal::GpioPort::A, 8),
    hal::Gpio(hal::GpioPort::A, 9),
    hal::Gpio(hal::GpioPort::A, 10),
    hal::Gpio(hal::GpioPort::A, 11),
};

// Thermistors connected directly to the STM.
std::array s_mcu_thermistor_enable{
    hal::Gpio(hal::GpioPort::B, 9), hal::Gpio(hal::GpioPort::B, 8), hal::Gpio(hal::GpioPort::B, 12),
    hal::Gpio(hal::GpioPort::A, 1), hal::Gpio(hal::GpioPort::A, 2), hal::Gpio(hal::GpioPort::A, 3),
    hal::Gpio(hal::GpioPort::A, 4),
};

hal::Gpio s_adc_cs(hal::GpioPort::A, 5);
hal::Gpio s_afe_cs(hal::GpioPort::A, 7);
hal::Gpio s_afe_en(hal::GpioPort::B, 0);
hal::Gpio s_ref_en(hal::GpioPort::B, 1);
hal::Gpio s_led(hal::GpioPort::B, 5);
hal::Gpio s_sck(hal::GpioPort::B, 13);
hal::Gpio s_miso(hal::GpioPort::B, 14);
hal::Gpio s_mosi(hal::GpioPort::B, 15);

// I2C pins.
hal::Gpio s_scl_1(hal::GpioPort::B, 6);
hal::Gpio s_sda_1(hal::GpioPort::B, 7);
hal::Gpio s_scl_2(hal::GpioPort::B, 10);
hal::Gpio s_sda_2(hal::GpioPort::B, 11);

[[nodiscard]] AfeStatus afe_command(std::uint16_t balance_bits, std::uint8_t control_bits) {
    std::array<std::uint8_t, 3> data{
        static_cast<std::uint8_t>(balance_bits >> 8u),
        static_cast<std::uint8_t>(balance_bits),
        control_bits,
    };
    if (!hal::spi_transfer(SPI2, s_afe_cs, data, 1)) {
        return AfeStatus::BadSpi;
    }

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
    const auto sample = max_adc::sample_voltage(SPI2, s_adc_cs, k_reference_voltage, k_cell_sample_count);
    if (!sample) {
        // Failed to sample ADC.
        return std::nullopt;
    }
    const auto [voltage, adc_range] = *sample;
    if (voltage < k_cell_open_threshold || voltage > (k_reference_voltage - k_cell_open_threshold)) {
        // Cell tap is bad.
        return std::nullopt;
    }
    return std::make_pair(voltage, adc_range > k_cell_degraded_threshold);
}

hal::I2cStatus set_expander_register(ExpanderRegister reg, std::uint8_t value) {
    std::array data{
        static_cast<std::uint8_t>(reg),
        value,
    };
    if (auto status = hal::i2c_wait_idle(I2C2); status != hal::I2cStatus::Ok) {
        return status;
    }
    if (auto status = hal::i2c_master_write(I2C2, 0x20, data); status != hal::I2cStatus::Ok) {
        return status;
    }
    hal::i2c_stop(I2C2);
    return hal::I2cStatus::Ok;
}

std::optional<std::int8_t> sample_thermistor(std::uint16_t rail_voltage, std::uint8_t index) {
    auto configuration_register = ExpanderRegister::ConfigurationPort0;
    if (index >= s_mcu_thermistor_enable.size() + 8) {
        configuration_register = ExpanderRegister::ConfigurationPort1;
    }

    // Enable the thermistor.
    if (index < s_mcu_thermistor_enable.size()) {
        // Pull the MCU pin high.
        s_mcu_thermistor_enable[index].configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
        hal::gpio_set(s_mcu_thermistor_enable[index]);
    } else {
        // The thermistor order is reversed on port 1 compared to port 0, i.e. the pins go in a clockwise fashion.
        const auto pin_index = index - s_mcu_thermistor_enable.size();
        const auto pin_bit =
            configuration_register == ExpanderRegister::ConfigurationPort0 ? pin_index : 15 - pin_index;
        if (set_expander_register(configuration_register, ~(1u << pin_bit)) != hal::I2cStatus::Ok) {
            // Failed to configure expander.
            return std::nullopt;
        }
    }

    // Allow some settling time.
    hal::delay_us(5);

    // Sample the voltage on the ADC. The min ensures that a bad rail voltage doesn't result in false readings.
    const auto sample = max_adc::sample_voltage(SPI2, s_adc_cs, k_reference_voltage, k_thermistor_sample_count);
    if (!sample) {
        // Failed to sample ADC.
        return std::nullopt;
    }
    auto [voltage, adc_range] = *sample;
    voltage = std::min(voltage, rail_voltage);

    // Disable the thermistor.
    if (index < s_mcu_thermistor_enable.size()) {
        // Reconfigure MCU pin to high impedance.
        s_mcu_thermistor_enable[index].configure(hal::GpioInputMode::Floating);
    } else if (set_expander_register(configuration_register, 0xff) != hal::I2cStatus::Ok) {
        // Failed to configure expander.
        return std::nullopt;
    }

    // Check if the voltage measurement is viable.
    if (voltage < k_thermistor_range_threshold || voltage > (rail_voltage - k_thermistor_range_threshold)) {
        return std::nullopt;
    }

    // Check if the voltage measurement is too noisy.
    if (adc_range > k_thermistor_noise_threshold) {
        return std::nullopt;
    }

    // Thermistor is connected, so we can calculate the temperature.
    // TODO: Use a lookup table/don't use floats here.
    const auto resistance = (rail_voltage * 10000) / voltage - 10000;
    float beta = 1.0f / 3950.0f;
    if (index < 3) {
        // The onboard thermistors have a different beta.
        beta = 1.0f / 3350.0f;
    }
    const float t0 = 1.0f / 298.15f;
    const float temperature = 1.0f / (t0 + beta * std::log(static_cast<float>(resistance) / 10000.0f)) - 273.15f;
    return static_cast<std::int8_t>(temperature);
}

std::optional<bms::Command> i2c_check(const bms::SegmentData &data, std::uint32_t timeout) {
    // TODO: Record bus errors and timeouts.
    const auto status = hal::i2c_slave_accept(I2C1, timeout);
    if (status != hal::I2cStatus::OkRead && status != hal::I2cStatus::OkWrite) {
        return std::nullopt;
    }

    if (status == hal::I2cStatus::OkRead) {
        // Receive the command byte.
        std::uint8_t byte = 0;
        if (hal::i2c_slave_read(I2C1, std::span(&byte, 1), 1) != hal::I2cStatus::Ok) {
            return std::nullopt;
        }
        return static_cast<bms::Command>(byte);
    }

    // Otherwise the master wants our data.
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

    static_cast<void>(hal::i2c_slave_write(I2C1, bytes, 1));
    return std::nullopt;
}

} // namespace

bool hal_low_power() {
    // Use the 8 MHz internal clock.
    return true;
}

void app_main() {
    // Configure general GPIOs.
    for (const auto &gpio : s_address_pins) {
        gpio.configure(hal::GpioInputMode::PullUp);
    }
    for (const auto &gpio : s_mcu_thermistor_enable) {
        gpio.configure(hal::GpioInputMode::Floating);
    }
    s_ref_en.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    s_led.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    s_afe_en.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);

    // Configure CS pin and enable a pull-up on MISO to avoid floating when no slave is selected.
    s_adc_cs.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    s_afe_cs.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    s_miso.configure(hal::GpioInputMode::PullUp);

    // Enable external interrupt on SCL (PB6).
    AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI6_PB;
    EXTI->EMR |= EXTI_EMR_MR6;
    EXTI->FTSR |= EXTI_FTSR_TR6;

    // Compute I2C address from on-board solder jumpers.
    const auto i2c_address = 0x40u | ~(GPIOA->IDR >> 8u) & 0xfu;

    bms::SegmentData data{};
    while (true) {
        // Wait a bit to allow a repeated start to be captured.
        hal::delay_us(100);
        i2c_check(data, 0);

        // Reconfigure SCK and MOSI as regular GPIOs before going to sleep.
        s_sck.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
        s_mosi.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);

        // Pull CS lines high by default (active-low) and put the ADC into shutdown.
        hal::gpio_set(s_adc_cs, s_afe_cs, s_sck);
        hal::gpio_reset(s_adc_cs);
        hal::gpio_set(s_adc_cs);

        // Reconfigure SCL as a regular input for use as an external event and enter stop mode. Also reconfigure SDA to
        // avoid the STM driving it low and upsetting the isolator.
        for (const auto &pin : {s_scl_1, s_sda_1, s_scl_2, s_sda_2}) {
            pin.configure(hal::GpioInputMode::Floating);
        }
        hal::enter_stop_mode();

        // Configure SCL and SDA for use with the I2C peripheral.
        for (const auto &pin : {s_scl_1, s_sda_1, s_scl_2, s_sda_2}) {
            pin.configure(hal::GpioOutputMode::AlternateOpenDrain, hal::GpioOutputSpeed::Max2);
        }

        // Reset the I2C and SPI peripherals just in case.
        RCC->APB1RSTR = RCC_APB1RSTR_I2C1RST | RCC_APB1RSTR_I2C2RST | RCC_APB1RSTR_SPI2RST;
        RCC->APB1RSTR = 0u;

        // Configure the I2C peripherals.
        hal::i2c_init(I2C1, i2c_address);
        hal::i2c_init(I2C2, std::nullopt);

        // Wait a maximum of 10 ms for address match.
        const auto command = i2c_check(data, 10);

        // Make sure GPIO expander is in a good state.
        static_cast<void>(set_expander_register(ExpanderRegister::OutputPort0, 0xff));
        static_cast<void>(set_expander_register(ExpanderRegister::OutputPort1, 0xff));
        static_cast<void>(set_expander_register(ExpanderRegister::PolarityPort0, 0x00));
        static_cast<void>(set_expander_register(ExpanderRegister::PolarityPort1, 0x00));
        static_cast<void>(set_expander_register(ExpanderRegister::ConfigurationPort0, 0xff));
        static_cast<void>(set_expander_register(ExpanderRegister::ConfigurationPort1, 0xff));

        if (!command) {
            // Data not for us or spurious wakeup - go back to sleep.
            continue;
        }

        switch (*command) {
            using enum bms::Command;
        case Enable:
            hal::gpio_set(s_afe_en, s_ref_en);
            continue;
        case Disable:
            hal::gpio_reset(s_afe_en, s_ref_en);
            continue;
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
        hal::spi_init_master(SPI2, SPI_CR1_BR_0);

        // Wait for AFE startup to complete. Route T2 (buffered) by default to measure thermistors.
        while (afe_command(0, 0b00111000u) == AfeStatus::NotReady) {
            __NOP();
        }

        if (*command == bms::Command::MeasureRail) {
            // All thermistors are switched off so we can measure the 3V3 rail voltage directly.
            const auto voltage = max_adc::sample_voltage(SPI2, s_adc_cs, k_reference_voltage, k_rail_sample_count);
            if (voltage) {
                std::tie(data.rail_voltage, std::ignore) = *voltage;
            }

            // TODO: This is broken in hardware revision D.
            data.rail_voltage = 33330;
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

        // Give a bit more time for the cell voltages to sample just in case.
        hal::delay_us(1000);

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
