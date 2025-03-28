#include <hal.hh>
#include <stm32f103xb.h>

#include <array>
#include <cmath>
#include <cstdint>

namespace {

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

hal::Gpio s_led(hal::GpioPort::A, 15);
hal::Gpio s_scl(hal::GpioPort::B, 6);
hal::Gpio s_sda(hal::GpioPort::B, 7);
hal::Gpio s_adc_cs(hal::GpioPort::B, 9);
hal::Gpio s_afe_cs(hal::GpioPort::B, 10);
hal::Gpio s_afe_en(hal::GpioPort::B, 11);
hal::Gpio s_afe_sampl(hal::GpioPort::B, 12);
hal::Gpio s_sck(hal::GpioPort::B, 13);
hal::Gpio s_miso(hal::GpioPort::B, 14);
hal::Gpio s_mosi(hal::GpioPort::B, 15);

std::uint8_t cell_selection_bits(std::uint8_t index) {
    return std::array<std::uint8_t, 12>{0b10000000, 0b11000000, 0b10100000, 0b11100000, 0b10010000, 0b11010000,
                                        0b10110000, 0b11110000, 0b10001000, 0b11001000, 0b10101000, 0b11101000}[index];
}

[[gnu::noinline]] void wfe() {
    __WFE();
    __NOP();
}

void enter_stop_mode() {
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    __SEV();
    wfe();
    wfe();
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
    s_led.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    s_afe_en.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    s_afe_sampl.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);

    // Configure CS pin and enable a pull-up on MISO to avoid floating when no slave is selected.
    s_adc_cs.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    s_afe_cs.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    s_miso.configure(hal::GpioInputMode::PullUp);

    // Keep AFE SAMPL pin always high and use SPI control instead.
    // TODO: This will be removed in a future hardware revision.
    hal::gpio_set(s_afe_sampl);

    // Enable external interrupt on SCL (PB6).
    AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI6_PB;
    EXTI->EMR |= EXTI_EMR_MR6;
    EXTI->FTSR |= EXTI_FTSR_TR6;

    while (true) {
        // Reconfigure SCK and MOSI as regular GPIOs before going to sleep.
        s_sck.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
        s_mosi.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);

        // Pull CS lines high by default (active-low) and put ADC and AFE into shutdown.
        hal::gpio_set(s_adc_cs, s_afe_cs, s_sck);
        hal::gpio_reset(s_adc_cs, s_afe_en);
        hal::gpio_set(s_adc_cs);

        // Reconfigure SCL as a regular input for use as an external event and enter stop mode.
        s_scl.configure(hal::GpioInputMode::Floating);
        enter_stop_mode();

        // Wake ADC and enable AFE.
        hal::gpio_reset(s_sck, s_adc_cs);
        hal::gpio_set(s_adc_cs, s_afe_cs, s_afe_en, s_led);

        // Configure SCK and MOSI for use with SPI peripheral.
        s_sck.configure(hal::GpioOutputMode::AlternatePushPull, hal::GpioOutputSpeed::Max10);
        s_mosi.configure(hal::GpioOutputMode::AlternatePushPull, hal::GpioOutputSpeed::Max10);

        // Enable SPI2 in master mode at 1 MHz (8x divider).
        RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
        SPI2->CR1 |= SPI_CR1_SPE | SPI_CR1_BR_1 | SPI_CR1_MSTR;

        // Initial AFE transfer to start sampling.
        // TODO: Check OT and product bits.
        {
            std::array<std::uint8_t, 3> data{};
            spi_transfer(s_afe_cs, data);
            hal::swd_printf("AFE initial data: [0x%x, 0x%x, 0x%x]\n", data[0], data[1], data[2]);
        }

        // Wait 60 ms for sampling.
        for (std::uint32_t i = 0; i < 120000; i++) {
            __NOP();
        }

        // TODO: Check C[1, 12], UV_VA, UV_VP, RDY, OT bits.
        // {
        //     std::array<std::uint8_t, 3> data{
        //         0,
        //         0,
        //         0b10000100,
        //     };
        //     spi_transfer(s_afe_cs, data);
        //     hal::swd_printf("AFE hold data1: [0x%x, 0x%x, 0x%x]\n", data[0], data[1], data[2]);
        // }

        // Wait for settle.
        // for (std::uint32_t i = 0; i < 256; i++) {
        //     __NOP();
        // }

        // {
        //     std::array<std::uint8_t, 3> data{
        //         0,
        //         0,
        //         0b10000100,
        //     };
        //     spi_transfer(s_afe_cs, data);
        //     hal::swd_printf("AFE hold data2: [0x%x, 0x%x, 0x%x]\n", data[0], data[1], data[2]);
        // }

        // SAMPL switch to hold time - 0.5 us
        // AOUT settle - 50 us
        // sample time - 60 ms

        {
            std::array<std::uint8_t, 3> data{
                0,
                0b00000000,
                0b00000100,
            };
            spi_transfer(s_afe_cs, data);
        }

        // Wait for level shift (50 us).
        for (std::uint32_t i = 0; i < 100; i++) {
            __NOP();
        }

        for (std::uint32_t cell = 12; cell > 0; cell--) {
            std::array<std::uint8_t, 3> data{
                0b00000000,
                0b00000000,
                0b10000000,
            };
            spi_transfer(s_afe_cs, data);

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
            }

            std::uint16_t min_adc = UINT16_MAX;
            std::uint16_t max_adc = 0;
            for (auto value : adc_values) {
                min_adc = std::min(min_adc, value);
                max_adc = std::max(max_adc, value);
            }

            const auto min_voltage = (min_adc * 40960u) >> 16u;
            const auto max_voltage = (max_adc * 40960u) >> 16u;
            hal::swd_printf("v%u: [%u, %u, %u]\n", cell, min_voltage, max_voltage, max_voltage - min_voltage);
        }

        hal::gpio_reset(s_led);
    }

    // s_scl.configure(hal::GpioOutputMode::AlternateOpenDrain, hal::GpioOutputSpeed::Max2);
    // s_sda.configure(hal::GpioOutputMode::AlternateOpenDrain, hal::GpioOutputSpeed::Max2);
    // RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    // I2C1->OAR1 = 0x58u << 1u;
    // I2C1->CR2 |= 0b1000u;
    // I2C1->CR1 |= I2C_CR1_ACK | I2C_CR1_PE;

    // while (true) {
    //     hal::swd_printf("wait address match\n");
    //     while ((I2C1->SR1 & I2C_SR1_ADDR) == 0u) {
    //         __NOP();
    //     }
    //     hal::swd_printf("address match\n");

    //     I2C1->SR1;
    //     I2C1->SR2;

    //     while (true) {
    //         while ((I2C1->SR1 & I2C_SR1_RXNE) == 0u) {
    //             __NOP();
    //         }
    //         hal::swd_printf("received 0x%x\n", I2C1->DR);

    //         if ((I2C1->SR1 & I2C_SR1_STOPF) != 0u) {
    //             I2C1->CR1 |= I2C_CR1_PE;
    //             break;
    //         }
    //     }
    // }

    hal::gpio_set(s_thermistor_enable[0]);
    while (true) {
        for (std::uint32_t i = 0; i < 12; i++) {
            std::array<std::uint8_t, 3> data{
                0,
                0b00000000,
                0b10000000,
            };

            // for (std::uint32_t j = 0; j < 5; j++) {
            //     const auto index = (i + j) % 12;
            //     if (index < 8) {
            //         data[0] |= 1u << (7 - index);
            //     } else {
            //         data[1] |= 1u << (7 - (index - 8));
            //     }
            // }

            spi_transfer(s_afe_cs, data);

            for (std::uint32_t i = 0; i < 1000000; i++) {
                __NOP();
            }

            data[0] = 0;
            data[1] = 0b00000000;
            data[2] = cell_selection_bits(i) | 0b100;
            spi_transfer(s_afe_cs, data);

            __NOP();
            __NOP();
            __NOP();
            __NOP();
            __NOP();
            __NOP();
            __NOP();
            __NOP();

            // Trigger ADC conversion.
            hal::gpio_reset(s_adc_cs);
            __NOP();
            hal::gpio_set(s_adc_cs);

            // Wait minimum conversion time.
            for (std::uint32_t j = 0; j < 1024; j++) {
                __NOP();
            }

            std::array<std::uint8_t, 2> adc_data{};
            spi_transfer(s_adc_cs, adc_data);

            const auto adc_value = (static_cast<std::uint32_t>(adc_data[0]) << 8u) | adc_data[1];
            const auto voltage = (adc_value * 40960u) >> 16u;
            hal::swd_printf("v%u: %u\n", i, voltage);

            // float v0 = static_cast<float>(adc_value) / 65536.0f * 4.096f;
            // float resistance = (v0) / (3.3f - v0);
            // float beta = 1.0f / 3950.0f;
            // float t0 = 1.0f / 298.15f;
            // float temperature = 1.0f / (t0 + beta * std::log(resistance)) - 273.15f;
            // hal::swd_printf("temperature: %u\n", static_cast<std::uint32_t>(temperature * 10.0f));

            // for (std::uint32_t j = 0; j < 1250000; j++) {
            //     __NOP();
            // }
        }
    }
}
