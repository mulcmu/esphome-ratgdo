#pragma once

// Hardware UART RX + RMT TX implementation for SEC+ v1 on ESP32.
// Selected when PROTOCOL_SECPLUSV1 and PROTOCOL_SECPLUSV1_ESP32_RMT are both defined.

#if defined(USE_ESP32) && defined(PROTOCOL_SECPLUSV1) && defined(PROTOCOL_SECPLUSV1_ESP32_RMT)

#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/optional.h"

#include "callbacks.h"
#include "common.h"
#include "observable.h"
#include "protocol.h"
#include "ratgdo_state.h"
#include "secplus1.h" // reuse RxCommand, RxPacket, CommandType, etc.

namespace esphome {

class Scheduler;
class InternalGPIOPin;

namespace ratgdo {
    namespace secplus1 {

        // RMT symbol array upper bound for a single 8E1 byte:
        //   11 bits (START + 8 data + 1 parity + 1 STOP); two bits packed per symbol word.
        //   Worst-case without merging: ceil(11/2) = 6 symbol words.
        static constexpr size_t SEC1_RMT_SYMBOLS_PER_BYTE = 8;

        // RMT resolution: 1 µs tick (1 MHz clock)
        static constexpr uint32_t SEC1_RMT_RESOLUTION_HZ = 1000000;

        // 1200 baud → 833.333 µs per bit; truncated to 833 µs (0.04% error, well within UART tolerance)
        static constexpr uint32_t SEC1_RMT_BIT_DURATION_US = 833;

        class Secplus1Esp32 : public Secplus1 {
        public:
            // Called from ratgdo.cpp before setup() when uart_id is configured
            void set_uart_parent(uart::UARTComponent* uart_parent) { this->uart_parent_ = uart_parent; }

            // Override Protocol interface
            void setup(RATGDOComponent* ratgdo, Scheduler* scheduler, InternalGPIOPin* rx_pin, InternalGPIOPin* tx_pin) override;
            void dump_config() override;

        protected:
            // read_command hides base so loop() (inherited) picks it up via name hiding;
            // loop() is NOT overridden — wall_panel_emulation callbacks call transmit_byte
            // which is now virtual, so Secplus1Esp32::transmit_byte dispatches correctly.
            optional<RxCommand> read_command();
            void transmit_byte(uint32_t value) override;

            uart::UARTComponent* uart_parent_ { nullptr };

            rmt_channel_handle_t rmt_chan_ { nullptr };
            rmt_encoder_handle_t rmt_encoder_ { nullptr };

            // Re-used per-byte symbol buffer
            rmt_symbol_word_t rmt_symbols_[SEC1_RMT_SYMBOLS_PER_BYTE];

            size_t build_rmt_byte(uint32_t value);
        };

    } // namespace secplus1
} // namespace ratgdo
} // namespace esphome

#endif // USE_ESP32 && PROTOCOL_SECPLUSV1 && PROTOCOL_SECPLUSV1_ESP32_RMT
