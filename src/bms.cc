#include <hal.hh>
#include <stm32f103xb.h>

#include <array>
#include <cmath>
#include <cstdint>

namespace {

// MAX14920 product and die version bits.
constexpr std::uint8_t k_afe_version_bits = 0b1010;

class CsGuard {
    hal::Gpio m_pin;

public:
    explicit CsGuard(hal::Gpio pin) : m_pin(pin) {
        // Pull CS low.
        hal::gpio_reset(pin);
    }
    CsGuard(const CsGuard &) = delete;
    CsGuard(CsGuard &&) = delete;
    ~CsGuard() {
        // Pull CS high again.
        hal::gpio_set(m_pin);
    }

    CsGuard &operator=(const CsGuard &) = delete;
    CsGuard &operator=(CsGuard &&) = delete;
};

std::array s_thermistor_enable{
    hal::Gpio(hal::GpioPort::A, 0), hal::Gpio(hal::GpioPort::A, 1), hal::Gpio(hal::GpioPort::A, 2),
    hal::Gpio(hal::GpioPort::A, 3), hal::Gpio(hal::GpioPort::A, 4), hal::Gpio(hal::GpioPort::A, 5),
    hal::Gpio(hal::GpioPort::A, 6), hal::Gpio(hal::GpioPort::A, 7),
};

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

std::uint8_t cell_selection_bits(std::uint8_t index) {
    return std::array<std::uint8_t, 12>{0b10000000, 0b11000000, 0b10100000, 0b11100000, 0b10010000, 0b11010000,
                                        0b10110000, 0b11110000, 0b10001000, 0b11001000, 0b10101000, 0b11101000}[index];
}

void spi_transfer(const hal::Gpio &cs, std::span<std::uint8_t> data) {
    // TODO: Add timeouts.
    CsGuard cs_guard(cs);
    for (auto &byte : data) {
        // Transmit byte.
        hal::wait_equal(SPI2->SR, SPI_SR_TXE, SPI_SR_TXE);
        SPI2->DR = byte;

        // Receive byte.
        hal::wait_equal(SPI2->SR, SPI_SR_RXNE, SPI_SR_RXNE);
        byte = SPI2->DR;
    }

    // Wait for busy to be clear and then reset CS to high.
    hal::wait_equal(SPI2->SR, SPI_SR_BSY, 0u);
}

void afe_transfer(std::uint16_t balance_bits, std::uint8_t control_bits) {
    std::array<std::uint8_t, 3> data{
        static_cast<std::uint8_t>(balance_bits >> 8u),
        static_cast<std::uint8_t>(balance_bits),
        control_bits,
    };
    spi_transfer(s_afe_cs, data);

    // Check version bits are correct.
    if ((data[2] >> 4u) != k_afe_version_bits) {
        // TODO: Return SPI communication error.
    }

    // Check UVLO and thermal shutdown bits.
    if ((data[2] & 0b1101u) != 0u) {
        // TODO: Return AFE error.
    }

    (data[2] & 0b10u) != 0u;

    hal::swd_printf("[0x%x, 0x%x, 0x%x]\n", data[0], data[1], data[2]);
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

    // Enable external interrupt on SCL (PB6).
    AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI6_PB;
    EXTI->EMR |= EXTI_EMR_MR6;
    EXTI->FTSR |= EXTI_FTSR_TR6;

    while (true) {
        // Reconfigure SCK and MOSI as regular GPIOs before going to sleep.
        s_sck.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
        s_mosi.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);

        // Pull CS lines high by default (active-low) and put the ADC, AFE, and reference into shutdown.
        hal::gpio_set(s_adc_cs, s_afe_cs, s_sck);
        hal::gpio_reset(s_adc_cs, s_afe_en, s_ref_en);
        hal::gpio_set(s_adc_cs);

        // Reconfigure SCL as a regular input for use as an external event and enter stop mode.
        s_scl.configure(hal::GpioInputMode::Floating);
        hal::enter_stop_mode();

        // Wake the ADC and enable the AFE and reference.
        hal::gpio_reset(s_sck, s_adc_cs);
        hal::gpio_set(s_adc_cs, s_afe_cs, s_afe_en, s_ref_en);

        // Configure SCK and MOSI for use with SPI peripheral.
        s_sck.configure(hal::GpioOutputMode::AlternatePushPull, hal::GpioOutputSpeed::Max10);
        s_mosi.configure(hal::GpioOutputMode::AlternatePushPull, hal::GpioOutputSpeed::Max10);

        // Enable SPI2 in master mode at 1 MHz (8x divider).
        RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
        SPI2->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_SPE | SPI_CR1_BR_1 | SPI_CR1_MSTR;

        for (std::uint32_t i = 0;; i++) {
            hal::swd_printf("%u: ", i);
            afe_transfer(0, 0);
            for (std::uint32_t i = 0; i < 2000; i++) {
                __NOP();
            }
        }

        // Initial AFE transfer to start sampling.
        // TODO: Check OT and product bits.
        {
            std::array<std::uint8_t, 3> data{
                0,
                0,
                0b010,
            };
            spi_transfer(s_afe_cs, data);
            hal::swd_printf("AFE initial data: [0x%x, 0x%x, 0x%x]\n", data[0], data[1], data[2]);
        }

        // Wait 60 ms for sampling.
        for (std::uint32_t i = 0; i < 120000; i++) {
            __NOP();
        }

        // Hold voltages.
        // TODO: Check C[1, 12], UV_VA, UV_VP, RDY, OT bits.
        {
            std::array<std::uint8_t, 3> data{
                0,
                0,
                0b110,
            };
            spi_transfer(s_afe_cs, data);
        }

        // Wait for level shift (50 us).
        for (std::uint32_t i = 0; i < 100; i++) {
            __NOP();
        }

        for (std::uint32_t cell = 12; cell > 0; cell--) {
            std::array<std::uint8_t, 3> data{
                0,
                0,
                0b10000100,
            };
            // spi_transfer(s_afe_cs, data);

            data[0] = 0;
            data[1] = 0;
            data[2] = cell_selection_bits(cell - 1) | 0b100;
            spi_transfer(s_afe_cs, data);

            // Wait for AOUT settle (10 us).
            for (std::uint32_t i = 0; i < 20; i++) {
                __NOP();
            }

            // Sample ADC.
            std::array<std::uint16_t, 8> adc_values{};
            for (auto &value : adc_values) {
                // Trigger conversion.
                hal::gpio_reset(s_adc_cs);
                hal::gpio_set(s_adc_cs);

                // Wait minimum conversion time.
                for (std::uint32_t i = 0; i < 20; i++) {
                    __NOP();
                }

                std::array<std::uint8_t, 2> spi_data{};
                spi_transfer(s_adc_cs, spi_data);
                value = (static_cast<std::uint16_t>(spi_data[0]) << 8u) | spi_data[1];

                for (std::uint32_t i = 0; i < 20000; i++) {
                    __NOP();
                }
            }

            std::uint16_t min_adc = UINT16_MAX;
            std::uint16_t max_adc = 0;
            for (auto value : adc_values) {
                min_adc = std::min(min_adc, value);
                max_adc = std::max(max_adc, value);
            }

            const auto min_voltage = (min_adc * 45000u) >> 16u;
            const auto max_voltage = (max_adc * 45000u) >> 16u;
            hal::swd_printf("v%u: [%u, %u, %u]\n", cell, min_voltage, max_voltage, max_voltage - min_voltage);
        }

        hal::gpio_reset(s_led);
        break;
    }
}
