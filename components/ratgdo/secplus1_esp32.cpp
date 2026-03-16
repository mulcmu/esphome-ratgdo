#if defined(USE_ESP32) && defined(PROTOCOL_SECPLUSV1) && defined(PROTOCOL_SECPLUSV1_ESP32_RMT)

#include "secplus1_esp32.h"
#include "ratgdo.h"

#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "esphome/core/gpio.h"
#include "esphome/core/log.h"
#include "esphome/core/scheduler.h"

namespace esphome {
namespace ratgdo {
    namespace secplus1 {

        static const char* const TAG = "ratgdo_secplus1_esp32";

        // ---------------------------------------------------------------------------
        // Secplus1Esp32::setup
        // ---------------------------------------------------------------------------

        void Secplus1Esp32::setup(RATGDOComponent* ratgdo, Scheduler* scheduler, InternalGPIOPin* rx_pin, InternalGPIOPin* tx_pin)
        {
            this->ratgdo_    = ratgdo;
            this->scheduler_ = scheduler;
            this->tx_pin_    = tx_pin;
            this->rx_pin_    = rx_pin;

            if (!this->uart_parent_) {
                ESP_LOGE(TAG, "uart_id is required for ESP32 RMT TX mode");
                return;
            }

            // TX pin: configure as output with GPIO idle LOW. The external
            // transistor inversion keeps the GDO bus idle HIGH (released).
            tx_pin->setup();
            tx_pin->pin_mode(gpio::FLAG_OUTPUT);
            tx_pin->digital_write(false);

            // Allocate RMT TX channel
            rmt_tx_channel_config_t chan_cfg = {};
            chan_cfg.clk_src            = RMT_CLK_SRC_DEFAULT;
            chan_cfg.gpio_num           = static_cast<gpio_num_t>(tx_pin->get_pin());
            chan_cfg.mem_block_symbols  = 64;   // one block; 8E1 byte needs at most 6 symbols
            chan_cfg.resolution_hz      = SEC1_RMT_RESOLUTION_HZ;
            chan_cfg.trans_queue_depth  = 1;
            // Keep logical RMT levels non-inverted at GPIO. The external
            // transistor stage provides the physical inversion to the bus.
            chan_cfg.flags.invert_out   = false;
            chan_cfg.flags.with_dma     = false;
            chan_cfg.flags.io_loop_back = false;
            chan_cfg.flags.io_od_mode   = false;

            esp_err_t err = rmt_new_tx_channel(&chan_cfg, &this->rmt_chan_);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to allocate RMT TX channel: %s", esp_err_to_name(err));
                if (this->ratgdo_ != nullptr) {
                    this->ratgdo_->mark_failed();
                }
                return;
            }

            rmt_copy_encoder_config_t copy_enc_cfg = {};
            err = rmt_new_copy_encoder(&copy_enc_cfg, &this->rmt_encoder_);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to create RMT copy encoder: %s", esp_err_to_name(err));
                rmt_del_channel(this->rmt_chan_);
                this->rmt_chan_ = nullptr;
                if (this->ratgdo_ != nullptr) {
                    this->ratgdo_->mark_failed();
                }
                return;
            }

            err = rmt_enable(this->rmt_chan_);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to enable RMT TX channel: %s", esp_err_to_name(err));
                if (this->ratgdo_ != nullptr) {
                    this->ratgdo_->mark_failed();
                }
                return;
            }

            this->traits_.set_features(HAS_DOOR_STATUS | HAS_LIGHT_TOGGLE | HAS_LOCK_TOGGLE);
            ESP_LOGCONFIG(TAG, "Secplus1Esp32: hardware UART RX + RMT TX initialised on GPIO %d",
                tx_pin->get_pin());
        }

        // ---------------------------------------------------------------------------
        // Secplus1Esp32::dump_config
        // ---------------------------------------------------------------------------

        void Secplus1Esp32::dump_config()
        {
            ESP_LOGCONFIG(TAG, "  Protocol: SEC+ v1 (ESP32 UART RX + RMT TX)");
        }

        // ---------------------------------------------------------------------------
        // read_command — uses hardware UART instead of SoftwareSerial.
        // Logic matches Secplus1::read_command() but reads from uart_parent_.
        // Called from the inherited loop() via name hiding.
        // ---------------------------------------------------------------------------

        optional<RxCommand> Secplus1Esp32::read_command()
        {
            static bool reading_msg = false;
            static uint16_t byte_count = 0;
            static RxPacket rx_packet;

            if (!reading_msg) {
                while (this->uart_parent_->available()) {
                    uint8_t ser_byte;
                    this->uart_parent_->read_byte(&ser_byte);
                    this->last_rx_ = millis();

                    if (ser_byte < 0x30 || ser_byte > 0x3A) {
                        ESP_LOG2(TAG, "[%d] Ignoring byte [%02X]", millis(), ser_byte);
                        byte_count = 0;
                        continue;
                    }
                    rx_packet[byte_count++] = ser_byte;
                    ESP_LOG2(TAG, "[%d] Received byte: [%02X]", millis(), ser_byte);
                    reading_msg = true;

                    // Single-byte commands that carry no response byte
                    if (ser_byte == 0x37 || (ser_byte >= 0x30 && ser_byte <= 0x35)) {
                        rx_packet[byte_count++] = 0;
                        reading_msg = false;
                        byte_count = 0;
                        ESP_LOG2(TAG, "[%d] Received command: [%02X]", millis(), rx_packet[0]);
                        return this->decode_packet(rx_packet);
                    }
                    break;
                }
            }
            if (reading_msg) {
                while (this->uart_parent_->available()) {
                    uint8_t ser_byte;
                    this->uart_parent_->read_byte(&ser_byte);
                    this->last_rx_ = millis();
                    rx_packet[byte_count++] = ser_byte;
                    ESP_LOG2(TAG, "[%d] Received byte: [%02X]", millis(), ser_byte);

                    if (byte_count == RX_LENGTH) {
                        reading_msg = false;
                        byte_count = 0;
                        this->print_rx_packet(rx_packet);
                        return this->decode_packet(rx_packet);
                    }
                }

                if (millis() - this->last_rx_ > 100) {
                    ESP_LOGW(TAG, "[%d] Discard incomplete packet: [%02X ...]", millis(), rx_packet[0]);
                    reading_msg = false;
                    byte_count = 0;
                }
            }

            return { };
        }

        // ---------------------------------------------------------------------------
        // build_rmt_byte
        //
        // Converts a single byte to an 8E1 UART waveform as RMT symbols.
        //
        // RMT channel configured with invert_out = false:
        //   RMT level 0 -> GPIO LOW  -> transistor OFF -> GDO bus HIGH (pulled up)
        //   RMT level 1 -> GPIO HIGH -> transistor ON  -> GDO bus LOW
        //
        // 8E1 UART bit mapping (11 bits total):
        //   IDLE / STOP / data '1' (mark)  = GDO HIGH = RMT level 0
        //   START       / data '0' (space) = GDO LOW  = RMT level 1
        //   EVEN parity bit: set to make total 1-count in data bits even
        //
        // Returns the number of rmt_symbol_word_t entries written.
        // ---------------------------------------------------------------------------

        size_t Secplus1Esp32::build_rmt_byte(uint32_t value)
        {
            uint8_t byte_val = static_cast<uint8_t>(value);

            // EVEN parity: number of 1 bits in byte_val; parity bit makes total even
            uint8_t parity = static_cast<uint8_t>(__builtin_popcount(byte_val) & 1);

            // Build 11 bit-levels: START, D0-D7, PARITY, STOP
            uint8_t levels[11];
            uint16_t durs[11];
            levels[0] = 1; durs[0] = SEC1_RMT_BIT_DURATION_US; // START: GDO LOW = level 1
            for (int i = 0; i < 8; i++) {
                // data '1' = GDO HIGH = level 0; data '0' = GDO LOW = level 1
                levels[i + 1] = ((byte_val >> i) & 1) ? 0 : 1;
                durs[i + 1]   = SEC1_RMT_BIT_DURATION_US;
            }
            levels[9]  = parity ? 0 : 1; durs[9]  = SEC1_RMT_BIT_DURATION_US; // EVEN parity
            levels[10] = 0;             durs[10] = SEC1_RMT_BIT_DURATION_US;   // STOP: GDO HIGH = level 0

            // Pack bits into rmt_symbol_word_t pairs, merging consecutive same-level bits
            size_t si = 0;
            int bit = 0;
            while (bit < 11) {
                uint8_t cur_level = levels[bit];
                uint32_t combined = 0;
                while (bit < 11 && levels[bit] == cur_level && combined + durs[bit] <= 32767) {
                    combined += durs[bit];
                    bit++;
                }
                uint16_t dur0 = static_cast<uint16_t>(combined);
                uint8_t lvl0  = cur_level;

                if (bit >= 11) {
                    this->rmt_symbols_[si++] = { { dur0, lvl0, 0, 0 } };
                    break;
                }

                uint8_t cur_level2 = levels[bit];
                uint32_t combined2 = 0;
                while (bit < 11 && levels[bit] == cur_level2 && combined2 + durs[bit] <= 32767) {
                    combined2 += durs[bit];
                    bit++;
                }
                this->rmt_symbols_[si++] = { { dur0, lvl0,
                                               static_cast<uint16_t>(combined2), cur_level2 } };
            }

            return si;
        }

        // ---------------------------------------------------------------------------
        // transmit_byte — virtual override; called from do_transmit_if_pending() and
        // wall_panel_emulation() callbacks, both inherited from Secplus1.
        // ---------------------------------------------------------------------------

        void Secplus1Esp32::transmit_byte(uint32_t value)
        {
            if (!this->rmt_chan_) {
                return;
            }

            size_t symbol_count = this->build_rmt_byte(value);

            rmt_transmit_config_t tx_cfg = {};
            tx_cfg.loop_count = 0;
            // Keep GPIO output low after TX; transistor stage keeps bus idle high.
            tx_cfg.flags.eot_level = 0;

            esp_err_t err = rmt_transmit(
                this->rmt_chan_,
                this->rmt_encoder_,
                this->rmt_symbols_,
                symbol_count * sizeof(rmt_symbol_word_t),
                &tx_cfg);

            if (err != ESP_OK) {
                ESP_LOGE(TAG, "rmt_transmit failed: %s", esp_err_to_name(err));
                return;
            }

            // Wait for byte transmission to finish (~9.2 ms at 1200 baud 8E1).
            // Block here to preserve the original transmit_byte() synchronous contract
            // (caller records last_tx_ immediately after this returns).
            err = rmt_tx_wait_all_done(this->rmt_chan_, 20);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "rmt_tx_wait_all_done failed: %s", esp_err_to_name(err));
                return;
            }

            this->last_tx_ = millis();
            ESP_LOGD(TAG, "[%d] Sent byte: [%02X]", millis(), static_cast<uint8_t>(value));
        }

    } // namespace secplus1
} // namespace ratgdo
} // namespace esphome

#endif // USE_ESP32 && PROTOCOL_SECPLUSV1 && PROTOCOL_SECPLUSV1_ESP32_RMT
