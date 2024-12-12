#include <can.hh>
#include <config.hh>
#include <dti.hh>
#include <hal.hh>
#include <stm32f103xb.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <utility>
#include <variant>

namespace {

class DtiState {
    // TODO: Record more data such as actual RPM to crosscheck it.
    std::atomic<bool> m_drive_enabled;

public:
    void operator()(const dti::GeneralData1 &) {}

    void operator()(const dti::GeneralData2 &) {}

    void operator()(const dti::GeneralData3 &) {}

    void operator()(const dti::GeneralData5 &gd5) {
        m_drive_enabled.store(gd5.drive_enable, std::memory_order_relaxed);
    }

    void operator()(const dti::UnknownData &) {
        // TODO: Log this as a warning.
    }

    bool is_drive_enabled() const { return m_drive_enabled.load(std::memory_order_relaxed); }
} s_dti_state;

bool s_should_send_status = false;
bool s_should_update_dti = false;

void dti_message_callback(const can::Message &message) {
    const auto dti_packet = dti::parse_packet(message.identifier, message.data_low, message.data_high);
    std::visit(s_dti_state, dti_packet);
}

void update_dti() {
    // TODO: Compute based on RTD and inverter data sanity checking.
    bool drive_enabled_desired = true;

    // Inverter's drive enable state doesn't match what we want, attempt to update it.
    if (s_dti_state.is_drive_enabled() != drive_enabled_desired) {
        // Send drive enable message.
        can::Message message{
            .identifier = (0x0cul << 8u) | config::k_dti_can_id,
            .data_low = drive_enabled_desired ? 0x1u : 0x0u,
            .length = 1,
        };
        if (!can::transmit(message)) {
            // TODO: Handle this?
        }

        // Early return so we avoid sending throttle data when drive is not enabled.
        return;
    }

    // TODO: Calculate desired RPM based on throttle pedal input.
    std::int32_t desired_rpm = 500;

    // TODO: Sanity check RPM.

    // Send set speed (ERPM) message.
    can::Message message{
        .identifier = (0x03ul << 8u) | config::k_dti_can_id,
        .data_low = static_cast<std::uint32_t>(desired_rpm * config::k_erpm_factor),
        .length = 4,
    };
    if (!can::transmit(message)) {
        // TODO: Handle this?
    }
}

} // namespace

extern "C" void TIM2_IRQHandler() {
    // Clear update interrupt flag.
    TIM2->SR = ~TIM_SR_UIF;
    s_should_send_status = true;
}

extern "C" void TIM3_IRQHandler() {
    // Clear update interrupt flag.
    TIM3->SR = ~TIM_SR_UIF;
    s_should_update_dti = true;
}

int main() {
    hal::init_clocks();

    // Initialise CAN peripheral and route all DTI messages to FIFO 0.
    can::init();
    can::route_filter(0, 0, 0x7feu, (config::k_dti_can_id << 3u) | 0b100u);

    // Install FIFO message pending callback and enable IRQ with a high priority.
    can::set_fifo_callback(0, dti_message_callback);
    hal::enable_irq(USB_LP_CAN1_RX0_IRQn, 2);

    // Enable timer 2 and 3 clocks.
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN;

    // Enable update event interrupt generation.
    TIM2->DIER |= TIM_DIER_UIE;
    TIM3->DIER |= TIM_DIER_UIE;

    // Configure prescaler.
    TIM2->PSC = 1999;
    TIM3->PSC = 1999;

    // Configure auto reload registers.
    TIM2->ARR = 15999;
    TIM3->ARR = 7999;

    TIM2->ARR = 999;

    // Enable timers and IRQs.
    TIM2->CR1 |= TIM_CR1_CEN;
    TIM3->CR1 |= TIM_CR1_CEN;
    hal::enable_irq(TIM2_IRQn, 4);
    hal::enable_irq(TIM3_IRQn, 3);

    // Enable and calibrate ADC.
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_IOPAEN;
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    // PA5 - ADC1_IN5, PA6 - ADC1_IN6
    hal::configure_gpio(GPIOA, 5, hal::GpioInputMode::Analog);
    hal::configure_gpio(GPIOA, 6, hal::GpioInputMode::Analog);
    ADC1->SQR1 |= ADC_SQR1_L_0;
    ADC1->SQR3 = (6u << ADC_SQR3_SQ2_Pos) | (5u << ADC_SQR3_SQ1_Pos);
    ADC1->SMPR2 |= (0b010u << ADC_SMPR2_SMP6_Pos) | (0b010u << ADC_SMPR2_SMP5_Pos);

    ADC1->CR2 |= ADC_CR2_ADON;
    for (std::uint32_t i = 0; i < 1000; i++) {
        asm volatile("" ::: "memory");
    }

    // Perform calibration.
    ADC1->CR2 |= ADC_CR2_RSTCAL;
    hal::wait_equal(ADC1->CR2, ADC_CR2_RSTCAL, 0u);
    ADC1->CR2 |= ADC_CR2_CAL;
    hal::wait_equal(ADC1->CR2, ADC_CR2_CAL, 0u);

    std::array<std::uint16_t, 12> adc_buffer{};
    DMA1_Channel1->CPAR = reinterpret_cast<std::uint32_t>(&ADC1->DR);
    DMA1_Channel1->CMAR = reinterpret_cast<std::uint32_t>(adc_buffer.data());
    DMA1_Channel1->CNDTR = adc_buffer.size();
    DMA1_Channel1->CCR = DMA_CCR_PL_1 | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_EN;

    ADC1->CR1 |= ADC_CR1_SCAN;
    ADC1->CR2 |= ADC_CR2_DMA | ADC_CR2_CONT;
    ADC1->CR2 |= ADC_CR2_ADON;

    // Main control loop.
    while (true) {
        if (std::exchange(s_should_send_status, false)) {
            // TODO: Send status report.
            // const auto adc_value = adc_buffer[0];
            // std::uint32_t throttle = (adc_value - 1300) * 100;
            // throttle /= (3000 - 1300);
            // if (throttle > 100) {
            //     throttle = 0;
            // }

            // hal::swd_printf("throttle: %u (raw: [%u, %u])\n", throttle, adc_value, adc_buffer[1]);

            std::uint32_t left_avg = 0;
            std::uint32_t right_avg = 0;
            for (std::uint32_t i = 0; i < adc_buffer.size(); i += 2) {
                left_avg += adc_buffer[i];
            }
            for (std::uint32_t i = 1; i < adc_buffer.size(); i += 2) {
                right_avg += adc_buffer[i];
            }
            left_avg /= adc_buffer.size() / 2;
            right_avg /= adc_buffer.size() / 2;
            std::uint32_t left = (left_avg + 2) / 5 * 5;
            std::uint32_t right = (right_avg + 2) / 5 * 5;
            hal::swd_printf("left: %u, right: %u\n", left, right);
        }

        if (std::exchange(s_should_update_dti, false)) {
            update_dti();
        }

        // Sleep until next interrupt.
        // __WFI();
    }
}
