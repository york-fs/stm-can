#include <hal.hh>

#include <miniprintf.h>
#include <stm32f103xb.h>

#include <bit>
#include <cstdarg>
#include <cstdint>
#include <span>

[[gnu::weak]] bool hal_low_power() {
    return false;
}

namespace hal {

static void set_gpio(GPIO_TypeDef *port, std::uint32_t pin, std::uint32_t cnf, std::uint32_t mode) {
    const auto shift = (pin % 8) * 4;
    auto &reg = pin > 7 ? port->CRH : port->CRL;
    reg &= ~(0xf << shift);
    reg |= cnf << (shift + 2);
    reg |= mode << shift;
}

static GPIO_TypeDef *gpio_port(GpioPort port) {
    return std::array{
        GPIOA, GPIOB, GPIOC, GPIOD, GPIOE,
    }[static_cast<std::uint32_t>(port)];
}

Gpio::Gpio(GpioPort port, std::uint8_t pin) : m_port(gpio_port(port)), m_pin(pin) {}

void Gpio::configure(GpioInputMode mode) const {
    auto cnf_bits = static_cast<std::uint32_t>(mode);
    if (mode == GpioInputMode::PullDown || mode == GpioInputMode::PullUp) {
        cnf_bits = 0b10u;
    }
    set_gpio(m_port, m_pin, cnf_bits, 0b00u);
    if (mode == GpioInputMode::PullUp) {
        hal::gpio_set(*this);
    } else if (mode == GpioInputMode::PullDown) {
        hal::gpio_reset(*this);
    }
}

void Gpio::configure(GpioOutputMode mode, GpioOutputSpeed speed) const {
    set_gpio(m_port, m_pin, static_cast<std::uint32_t>(mode), static_cast<std::uint32_t>(speed));
    hal::gpio_reset(*this);
}

void enable_irq(IRQn_Type irq, std::uint32_t priority) {
    NVIC_SetPriority(irq, priority);
    NVIC_EnableIRQ(irq);
}

void disable_irq(IRQn_Type irq) {
    NVIC_DisableIRQ(irq);
}

// Workaround for STM32F103 erratum affecting stop debug mode.
[[gnu::noinline]] static void wfe() {
    __WFE();
    __NOP();
}

void enter_stop_mode() {
    // Clear the PDDS bit to ensure stop mode, not standby mode, is selected.
    PWR->CR &= ~PWR_CR_PDDS;

    // Set the SLEEPDEEP bit to set stop mode rather than sleep mode.
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    // Set event and invoke WFE twice to clear any stale events.
    __SEV();
    wfe();
    wfe();

    // Clear the SLEEPDEEP bit.
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
}

void adc_init(ADC_TypeDef *adc, std::uint32_t channel_count) {
    // Enable clock for ADC.
    RCC->APB2ENR |= (adc == ADC1 ? RCC_APB2ENR_ADC1EN : RCC_APB2ENR_ADC2EN);

    // Wait for ADC to settle.
    adc->CR2 |= ADC_CR2_ADON;
    hal::delay_us(100);

    // Perform calibration.
    adc->CR2 |= ADC_CR2_RSTCAL;
    hal::wait_equal(adc->CR2, ADC_CR2_RSTCAL, 0u);
    adc->CR2 |= ADC_CR2_CAL;
    hal::wait_equal(adc->CR2, ADC_CR2_CAL, 0u);

    // Default to external trigger via software start.
    adc->CR2 |= 0b111u << ADC_CR2_EXTSEL_Pos;

    // If we have more than one channel, enable scan mode, since we probably always want it.
    adc->SQR1 |= (channel_count - 1) << ADC_SQR1_L_Pos;
    if (channel_count > 1) {
        adc->CR1 |= ADC_CR1_SCAN;
    }
}

void adc_init_dma(std::span<std::uint16_t> data) {
    // Enable DMA peripheral clock.
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    // Enable DMA mode on ADC1.
    ADC1->CR2 |= ADC_CR2_DMA;

    // Configure DMA channel 1.
    DMA1_Channel1->CPAR = std::bit_cast<std::uint32_t>(&ADC1->DR);
    DMA1_Channel1->CMAR = std::bit_cast<std::uint32_t>(data.data());
    DMA1_Channel1->CNDTR = data.size();
    DMA1_Channel1->CCR = DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_EN;
}

void adc_sequence_channel(ADC_TypeDef *adc, std::uint32_t index, std::uint32_t channel, std::uint32_t sample_time) {
    // Enable temperature/VREF channel.
    if (adc == ADC1 && (channel == 16 || channel == 17)) {
        adc->CR2 |= ADC_CR2_TSVREFE;
    }

    // Configure sample time.
    if (channel >= 10) {
        adc->SMPR1 |= sample_time << ((channel - 10) * 3);
    } else {
        adc->SMPR2 |= sample_time << (channel * 3);
    }

    // Map sequence index to channel.
    if (index >= 13) {
        adc->SQR1 |= channel << ((index - 13) * 5);
    } else if (index >= 7) {
        adc->SQR2 |= channel << ((index - 7) * 5);
    } else {
        adc->SQR3 |= channel << ((index - 1) * 5);
    }
}

void adc_start(ADC_TypeDef *adc) {
    // Clear EOC flag.
    adc->SR |= ADC_SR_EOC;

    // Issue software start.
    adc->CR2 |= ADC_CR2_SWSTART | ADC_CR2_EXTTRIG;
}

void delay_us(std::size_t us) {
    const auto count = us * (hal_low_power() ? 2 : 14);
    for (std::size_t i = 0; i < us * 2; i++) {
        __NOP();
    }
}

void swd_putc(char ch) {
    ITM_SendChar(ch);
}

int swd_printf(const char *format, ...) {
    va_list args;
    va_start(args, format);
    int rc = mini_vprintf_cooked(swd_putc, format, args);
    va_end(args);
    return rc;
}

bool wait_equal(const volatile std::uint32_t &reg, std::uint32_t mask, std::uint32_t desired, std::uint32_t timeout) {
    auto timeout_scaled = timeout * 10;
    for (; (reg & mask) != desired; timeout_scaled--) {
        if (timeout_scaled == 0) {
            return false;
        }
        hal::delay_us(100);
    }
    return true;
}

} // namespace hal

extern void app_main();

int main() {
    // Enable 56 MHz system clock via an 8 MHz external crystal if low power mode is not desired.
    if (!hal_low_power()) {
        // Increase flash latency for use with a 56 MHz AHB clock.
        FLASH->ACR |= FLASH_ACR_LATENCY_2;

        // Enable HSE (8 MHz crystal) and wait for readiness.
        RCC->CR |= RCC_CR_HSEON;
        hal::wait_equal(RCC->CR, RCC_CR_HSERDY, RCC_CR_HSERDY);

        // Configure PLL to HSE * 7 = 56 MHz.
        uint32_t rcc_cfgr = RCC->CFGR;
        rcc_cfgr &= ~RCC_CFGR_PLLMULL;
        rcc_cfgr |= RCC_CFGR_PLLMULL7;
        rcc_cfgr |= RCC_CFGR_PLLSRC;
        RCC->CFGR = rcc_cfgr;

        // Enable PLL and wait for readiness.
        RCC->CR |= RCC_CR_PLLON;
        hal::wait_equal(RCC->CR, RCC_CR_PLLRDY, RCC_CR_PLLRDY);

        // Switch system clock to PLL. HSI is default, so no need to mask.
        RCC->CFGR |= RCC_CFGR_SW_PLL;
        hal::wait_equal(RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_PLL);

        // Done with the HSI, disable it.
        RCC->CR &= ~RCC_CR_HSION;
        hal::wait_equal(RCC->CR, RCC_CR_HSIRDY, 0u);

        // Set a 2x divider on APB1 clock as to not exceed 36 MHz limit.
        RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;

        // Set a 4x divider on the ADC clock to achieve the maximum 14 MHz.
        RCC->CFGR |= RCC_CFGR_ADCPRE_DIV4;
    }

    // Default to setting the internal LDO to a low-power mode in stop mode. This incurs a small startup time penalty.
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_LPDS;

    // Disable JTAG interface.
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
    AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;

    // Enable clocks for all GPIO ports. For cases where we care about power usage, stop and standby mode will disable
    // them anyway. Otherwise, where we don't care about power usage, this simplifies things.
    RCC->APB2ENR |=
        RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_IOPDEN | RCC_APB2ENR_IOPEEN;

    // Default all pins to pull-down.
    for (auto *gpio : {GPIOA, GPIOB, GPIOC, GPIOD, GPIOE}) {
        gpio->CRL = 0x88888888;
        gpio->CRH = 0x88888888;
    }

    // Jump to user code.
    app_main();
}
