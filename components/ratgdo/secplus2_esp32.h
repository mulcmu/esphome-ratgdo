#pragma once

// Hardware UART RX + RMT TX implementation for SEC+ v2 on ESP32.
// Selected when PROTOCOL_SECPLUSV2 and PROTOCOL_SECPLUSV2_ESP32_RMT are both defined.

#if defined(USE_ESP32) && defined(PROTOCOL_SECPLUSV2) && defined(PROTOCOL_SECPLUSV2_ESP32_RMT)

#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/optional.h"

#include "callbacks.h"
#include "common.h"
#include "observable.h"
#include "protocol.h"
#include "ratgdo_state.h"
#include "secplus2.h" // reuse CommandType, Command, WirePacket, PACKET_LENGTH

namespace esphome {

class Scheduler;
class InternalGPIOPin;

namespace ratgdo {
    namespace secplus2 {

        // Total number of RMT symbols for one SEC+ v2 frame (upper bound before merging):
        //   1 preamble symbol (LOW+HIGH packed) + up to 19 bytes × 10 bits / 2 = 95 symbols
        //   Worst case without any merging: 97 symbols. Array sized generously at 192.
        static constexpr size_t RMT_SYMBOLS_PER_FRAME = 192;

        // RMT resolution: 1 µs tick (1 MHz clock)
        static constexpr uint32_t RMT_RESOLUTION_HZ = 1000000;

        // 9600 baud → 104.167 µs per bit — round to nearest integer µs
        static constexpr uint32_t RMT_BIT_DURATION_US = 104;

        // Preamble timings (µs) — matches the original transmit_packet() logic
        static constexpr uint16_t RMT_PREAMBLE_LOW_US  = 1300;
        static constexpr uint16_t RMT_PREAMBLE_HIGH_US = 130;

        class Secplus2Esp32 : public Secplus2 {
        public:
            // Called from ratgdo.cpp before setup() when uart_id is configured
            void set_uart_parent(uart::UARTComponent* uart_parent) { this->uart_parent_ = uart_parent; }

            // Override Protocol interface
            void setup(RATGDOComponent* ratgdo, Scheduler* scheduler, InternalGPIOPin* rx_pin, InternalGPIOPin* tx_pin) override;
            void loop() override;
            void dump_config() override;

        protected:
            // Override the two methods that touch SoftwareSerial so the parent
            // class helpers (encode_packet, decode_packet, handle_command, etc.)
            // remain usable unchanged.
            optional<Command> read_command() override;
            bool transmit_packet() override;

            uart::UARTComponent* uart_parent_ { nullptr };

            rmt_channel_handle_t rmt_chan_ { nullptr };
            rmt_encoder_handle_t rmt_encoder_ { nullptr };

            // Pre-built symbol array reused for each transmission
            rmt_symbol_word_t rmt_symbols_[RMT_SYMBOLS_PER_FRAME];

            size_t build_rmt_frame(const WirePacket& packet);
        };

    } // namespace secplus2
} // namespace ratgdo
} // namespace esphome

#endif // USE_ESP32 && PROTOCOL_SECPLUSV2 && PROTOCOL_SECPLUSV2_ESP32_RMT
