#include <hal.hh>
#include <stm32f103xb.h>

#include <array>
#include <cmath>
#include <cstdint>

namespace {

std::array s_thermistor_enable{
    hal::Gpio(hal::GpioPort::A, 0), hal::Gpio(hal::GpioPort::A, 1), hal::Gpio(hal::GpioPort::A, 2),
    hal::Gpio(hal::GpioPort::A, 3), hal::Gpio(hal::GpioPort::A, 4), hal::Gpio(hal::GpioPort::A, 5),
    hal::Gpio(hal::GpioPort::A, 6), hal::Gpio(hal::GpioPort::A, 7),
};

hal::Gpio s_led(hal::GpioPort::A, 15);
hal::Gpio s_adc_cs(hal::GpioPort::B, 9);
hal::Gpio s_afe_cs(hal::GpioPort::B, 10);
hal::Gpio s_afe_en(hal::GpioPort::B, 11);
hal::Gpio s_afe_sampl(hal::GpioPort::B, 12);
hal::Gpio s_sck(hal::GpioPort::B, 13);
hal::Gpio s_miso(hal::GpioPort::B, 14);
hal::Gpio s_mosi(hal::GpioPort::B, 15);

} // namespace

static std::uint32_t s_index = 0;
static std::array<std::uint8_t, 16> s_bytes;

extern "C" void SPI2_IRQHandler() {
    if ((SPI2->SR & SPI_SR_RXNE) != 0u) {
        const auto byte = SPI2->DR;
        s_bytes[s_index] = byte;
        // hal::swd_printf("received: 0x%x (%u)\n", byte, s_index);
        s_index++;
    }
    // if ((SPI2->SR & SPI_SR_TXE) != 0u) {
    //     SPI2->DR = 0x0;
    // }
}

std::uint32_t i = 0;
bool done_afe = false;

extern "C" void TIM2_IRQHandler() {
    TIM2->SR = ~TIM_SR_UIF;
    if (i == 0) {
        s_index = 0;
    }

    if (!done_afe) {
        switch (i) {
        case 0:
            GPIOB->ODR &= ~(1u << 10u);
            break;
        case 1:
        case 2:
            SPI2->DR = 0x0;
            break;
        case 3:
            // SPI2->DR = 0b00011100u;
            SPI2->DR = 0b00111000u;
            break;
        case 4:
            GPIOB->ODR |= 1u << 10u;
            done_afe = true;
            break;
        }
        i = (i + 1) % 5;
    } else {
        switch (i) {
        case 0:
            GPIOB->ODR &= ~(1u << 9u);
            break;
        case 1:
        case 2:
            SPI2->DR = 0x0;
            break;
        case 3:
            GPIOB->ODR |= 1u << 9u;
            break;
        }
        i = (i + 1) % 4;
    }

    std::uint16_t adc_count = (static_cast<std::uint16_t>(s_bytes[0]) << 8u) | s_bytes[1];
    float voltage = static_cast<float>(adc_count) / 65536 * 4.096f;

    float resistance = (voltage) / (3.3f - voltage);

    float beta = 1.0f / 3950.0f;
    float t0 = 1.0f / 298.15f;
    float temperature = 1.0f / (t0 + beta * std::log(resistance)) - 273.15f;

    // hal::swd_printf("resistance: %u\n", static_cast<std::uint32_t>(10000.0f * resistance));
    hal::swd_printf("temperature: %u, voltage: %u\n", static_cast<std::uint32_t>(temperature * 10.0f),
                    static_cast<std::uint32_t>(voltage * 1000.0f));
}

bool hal_low_power() {
    return true;
}

void app_main() {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN;

    // Configure general GPIOs.
    for (const auto &gpio : s_thermistor_enable) {
        gpio.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    }
    s_led.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    s_afe_en.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    s_afe_sampl.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);

    // Configure SPI pins.
    s_adc_cs.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    s_afe_cs.configure(hal::GpioOutputMode::PushPull, hal::GpioOutputSpeed::Max2);
    s_sck.configure(hal::GpioOutputMode::AlternatePushPull, hal::GpioOutputSpeed::Max10);
    s_mosi.configure(hal::GpioOutputMode::AlternatePushPull, hal::GpioOutputSpeed::Max10);
    s_miso.configure(hal::GpioInputMode::PullUp);

    // Pull CS lines high by default (active-low).
    hal::gpio_set(s_adc_cs, s_afe_cs);

    // Keep AFE SAMPL pin always high and use SPI control instead.
    // TODO: This will be removed in a future hardware revision.
    // s_afe_sampl.write(true);
    hal::gpio_set(s_afe_sampl);

    // Enable AFE.
    hal::gpio_set(s_afe_en);

    // RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    // SPI2->CR2 |= SPI_CR2_RXNEIE;
    // SPI2->CR1 |= SPI_CR1_BR_2 | SPI_CR1_MSTR;
    // SPI2->CR1 |= SPI_CR1_SPE;
    // hal::enable_irq(SPI2_IRQn, 3);

    // RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    // TIM2->DIER |= TIM_DIER_UIE;
    // TIM2->PSC = 99;
    // TIM2->ARR = 3499;
    // TIM2->CR1 |= TIM_CR1_CEN;
    // hal::enable_irq(TIM2_IRQn, 4);

    while (true) {
        GPIOA->ODR ^= 1u << 15u;
        for (int i = 0; i < 1000000; i++) {
            __NOP();
        }
    }
}
