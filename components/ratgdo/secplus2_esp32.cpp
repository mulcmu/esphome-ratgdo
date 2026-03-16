#if defined(USE_ESP32) && defined(PROTOCOL_SECPLUSV2) && defined(PROTOCOL_SECPLUSV2_ESP32_RMT)

#include "secplus2_esp32.h"
#include "ratgdo.h"

#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "esphome/core/gpio.h"
#include "esphome/core/log.h"
#include "esphome/core/scheduler.h"

extern "C" {
#include "secplus.h"
}

namespace esphome {
namespace ratgdo {
    namespace secplus2 {

        static const char* const TAG = "ratgdo_secplus2_esp32";

        // ---------------------------------------------------------------------------
        // Secplus2Esp32::setup
        // ---------------------------------------------------------------------------

        void Secplus2Esp32::setup(RATGDOComponent* ratgdo, Scheduler* scheduler, InternalGPIOPin* rx_pin, InternalGPIOPin* tx_pin)
        {
            this->ratgdo_    = ratgdo;
            this->scheduler_ = scheduler;
            this->tx_pin_    = tx_pin;
            this->rx_pin_    = rx_pin;

            // Hardware UART is configured via the ESPHome uart component.
            // We only call uart_parent_ read/available here; setup is done by the
            // uart component itself (baud_rate: 9600, 8N1).
            // Validate that the uart parent is configured.
            if (!this->uart_parent_) {
                ESP_LOGE(TAG, "uart_id is required for ESP32 RMT TX mode");
                return;
            }

            // SEC+ v2 frames are 19 bytes long. With the default ESPHome IDF
            // RX full threshold of 8, the trailing 3 bytes of the last frame in
            // a burst can be delayed until later traffic arrives. Lower the
            // threshold in this transport mode so frame tails are surfaced
            // promptly and parsed in one pass.
            if (this->uart_parent_->get_rx_full_threshold() > 3) {
                this->uart_parent_->set_rx_full_threshold(1);
                ESP_LOGD(TAG, "Adjusted UART RX full threshold to 1 for SEC+ v2 frame assembly");
            }

            // TX pin: configure as output with GPIO idle LOW. The external
            // transistor inversion keeps the GDO bus idle HIGH (released).
            tx_pin->setup();
            tx_pin->pin_mode(gpio::FLAG_OUTPUT);
            tx_pin->digital_write(false);

            // RMT TX channel — allocate one of the 8 available ESP32 RMT channels
            rmt_tx_channel_config_t chan_cfg = {};
            chan_cfg.clk_src                = RMT_CLK_SRC_DEFAULT;
            chan_cfg.gpio_num               = static_cast<gpio_num_t>(tx_pin->get_pin());
            chan_cfg.mem_block_symbols      = 128;  // two 64-symbol blocks; fits the full 19-byte 8N1 frame
            chan_cfg.resolution_hz          = RMT_RESOLUTION_HZ;
            chan_cfg.trans_queue_depth      = 1;
            // Keep GPIO idle low; external transistor inversion then keeps the
            // GDO bus idle high.
            chan_cfg.flags.invert_out       = false;
            chan_cfg.flags.with_dma         = false;
            chan_cfg.flags.io_loop_back     = false;
            chan_cfg.flags.io_od_mode       = false;

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
            }

            this->traits_.set_features(Traits::all());
            ESP_LOGCONFIG(TAG, "Secplus2Esp32: hardware UART RX + RMT TX initialised on GPIO %d",
                tx_pin->get_pin());
        }

        // ---------------------------------------------------------------------------
        // Secplus2Esp32::loop
        // ---------------------------------------------------------------------------

        void Secplus2Esp32::loop()
        {
            if (this->flags_.transmit_pending) {
                if (!this->transmit_packet()) {
                    return;
                }
            }

            auto cmd = this->read_command();
            if (cmd) {
                this->handle_command(*cmd);
            }
        }

        // ---------------------------------------------------------------------------
        // Secplus2Esp32::dump_config
        // ---------------------------------------------------------------------------

        void Secplus2Esp32::dump_config()
        {
            ESP_LOGCONFIG(TAG, "  Rolling Code Counter: %d", *this->rolling_code_counter_);
            ESP_LOGCONFIG(TAG, "  Client ID: %d", this->client_id_);
            ESP_LOGCONFIG(TAG, "  Protocol: SEC+ v2 (ESP32 UART RX + RMT TX)");
        }

        // ---------------------------------------------------------------------------
        // read_command — uses hardware UART instead of SoftwareSerial
        // Logic is identical to Secplus2::read_command() but reads from uart_parent_.
        // ---------------------------------------------------------------------------

        optional<Command> Secplus2Esp32::read_command()
        {
            static bool reading_msg = false;
            static uint32_t msg_start = 0;
            static uint16_t byte_count = 0;
            static WirePacket rx_packet;
            static uint32_t last_read = 0;

            if (!reading_msg) {
                while (this->uart_parent_->available()) {
                    uint8_t ser_byte;
                    this->uart_parent_->read_byte(&ser_byte);
                    last_read = millis();

                    if (ser_byte != 0x55 && ser_byte != 0x01 && ser_byte != 0x00) {
                        ESP_LOG2(TAG, "Ignoring byte (%d): %02X", byte_count, ser_byte);
                        byte_count = 0;
                        continue;
                    }
                    msg_start = ((msg_start << 8) | ser_byte) & 0xffffff;
                    byte_count++;

                    if (msg_start == 0x550100) {
                        rx_packet[0] = 0x55;
                        rx_packet[1] = 0x01;
                        rx_packet[2] = 0x00;
                        // Restart payload indexing at byte 3. bytes consumed while
                        // searching for sync can include extra candidate values and
                        // must not shift packet assembly.
                        byte_count = 3;

                        reading_msg = true;
                        break;
                    }
                }
            }
            if (reading_msg) {
                while (this->uart_parent_->available()) {
                    uint8_t ser_byte;
                    this->uart_parent_->read_byte(&ser_byte);
                    last_read = millis();
                    rx_packet[byte_count] = ser_byte;
                    byte_count++;

                    if (byte_count == PACKET_LENGTH) {
                        reading_msg = false;
                        byte_count = 0;
                        this->print_packet(LOG_STR("Received packet: "), rx_packet);
                        return this->decode_packet(rx_packet);
                    }
                }

                if (millis() - last_read > 100) {
                    ESP_LOGW(TAG, "Discard incomplete packet, length: %d", byte_count);
                    reading_msg = false;
                    byte_count = 0;
                }
            }

            return { };
        }

        // ---------------------------------------------------------------------------
        // build_rmt_frame
        //
        // Converts a 19-byte WirePacket into RMT symbols:
        //   [0]   preamble LOW  1300 µs  (line pulled low via transistor)
        //   [1]   preamble HIGH  130 µs  (line released high)
        //   [2..] 19 bytes, each serialised as 8N1:
        //           START bit (LOW)
        //           8 data bits LSB first
        //           STOP bit  (HIGH)
        //
        // With invert_out disabled and the transistor stage:
        //   RMT level 0 -> GPIO LOW  -> transistor OFF -> GDO bus HIGH
        //   RMT level 1 -> GPIO HIGH -> transistor ON  -> GDO bus LOW
        //
        // Therefore the UART mapping used here is:
        //   IDLE / STOP / data '1' (mark)  = GDO HIGH = RMT level 0
        //   START       / data '0' (space) = GDO LOW  = RMT level 1
        // ---------------------------------------------------------------------------

        // Returns the number of rmt_symbol_word_t entries written
        size_t Secplus2Esp32::build_rmt_frame(const WirePacket& packet)
        {
            size_t si = 0;

            // RMT channel is configured with invert_out = false, so:
            //   RMT level 0 → GPIO LOW  → transistor OFF → GDO bus HIGH (pulled up)
            //   RMT level 1 → GPIO HIGH → transistor ON  → GDO bus LOW
            //
            // SEC+ v2 preamble (original transmit_packet logic):
            //   GDO LOW  for 1300 µs  → RMT level 1
            //   GDO HIGH for  130 µs  → RMT level 0
            //
            // 8N1 UART mapping:
            //   IDLE / STOP / data '1' (mark)  = GDO HIGH = RMT level 0
            //   START       / data '0' (space) = GDO LOW  = RMT level 1
            this->rmt_symbols_[si++] = { { RMT_PREAMBLE_LOW_US,  1,
                                           RMT_PREAMBLE_HIGH_US, 0 } };

            // 19 bytes of 8N1 UART data
            for (size_t b = 0; b < PACKET_LENGTH; b++) {
                uint8_t byte_val = packet[b];

                // Build an array of 10 bit levels (START, D0-D7, STOP)
                // and merge consecutive same-level bits into packed RMT symbol words.
                uint8_t levels[10];
                uint16_t durs[10];
                levels[0] = 1; durs[0] = RMT_BIT_DURATION_US; // START: GDO LOW = level 1
                for (int i = 0; i < 8; i++) {
                    // data '1' = GDO HIGH = level 0; data '0' = GDO LOW = level 1
                    levels[i + 1] = ((byte_val >> i) & 1) ? 0 : 1;
                    durs[i + 1]   = RMT_BIT_DURATION_US;
                }
                levels[9] = 0; durs[9] = RMT_BIT_DURATION_US; // STOP: GDO HIGH = level 0

                // Merge consecutive same-level bits into combined duration symbols
                // to minimise RMT symbol count.
                int bit = 0;
                while (bit < 10) {
                    uint8_t cur_level = levels[bit];
                    uint32_t combined = 0;
                    while (bit < 10 && levels[bit] == cur_level && combined + durs[bit] <= 32767) {
                        combined += durs[bit];
                        bit++;
                    }
                    uint16_t dur0 = static_cast<uint16_t>(combined);
                    uint8_t lvl0 = cur_level;

                    if (bit >= 10) {
                        // Pad second half of symbol word with a 0-duration entry
                        this->rmt_symbols_[si++] = { { dur0, lvl0, 0, 0 } };
                        break;
                    }

                    uint8_t cur_level2 = levels[bit];
                    uint32_t combined2 = 0;
                    while (bit < 10 && levels[bit] == cur_level2 && combined2 + durs[bit] <= 32767) {
                        combined2 += durs[bit];
                        bit++;
                    }
                    this->rmt_symbols_[si++] = { { dur0, lvl0,
                                                   static_cast<uint16_t>(combined2), cur_level2 } };
                }
            }

            ESP_LOG2(TAG, "RMT frame built: %d symbols", (int)si);
            return si;
        }

        // ---------------------------------------------------------------------------
        // transmit_packet — checks bus idle then fires RMT
        // ---------------------------------------------------------------------------

        bool Secplus2Esp32::transmit_packet()
        {
            if (!this->rmt_chan_) {
                return false;
            }

            // Check bus is idle for 1300 µs (same as original collision detection).
            // rx_pin_ is still wired to the GDO bus for this purpose.
            auto now = micros();
            while (micros() - now < 1300) {
                if (this->rx_pin_->digital_read()) {
                    if (!this->flags_.transmit_pending) {
                        this->flags_.transmit_pending = true;
                        this->transmit_pending_start_ = millis();
                        ESP_LOGD(TAG, "Collision detected, waiting to send packet");
                    } else if (millis() - this->transmit_pending_start_ >= 5000) {
                        this->transmit_pending_start_ = 0;
                    }
                    return false;
                }
                delayMicroseconds(100);
            }

            this->print_packet(LOG_STR("Sending packet"), this->tx_packet_);

            size_t symbol_count = this->build_rmt_frame(this->tx_packet_);

            rmt_transmit_config_t tx_cfg = {};
            tx_cfg.loop_count = 0; // no loop
            // Keep GPIO output low after TX. The external transistor then keeps
            // the GDO bus in its idle-high (released) state.
            tx_cfg.flags.eot_level = 0;

            esp_err_t err = rmt_transmit(
                this->rmt_chan_,
                this->rmt_encoder_,
                this->rmt_symbols_,
                symbol_count * sizeof(rmt_symbol_word_t),
                &tx_cfg);

            if (err != ESP_OK) {
                ESP_LOGE(TAG, "rmt_transmit failed: %s", esp_err_to_name(err));
                return false;
            }

            // Wait for transmission completion so on_command_sent_ preserves the
            // original synchronous semantics of the SoftwareSerial path.
            // Timing: 19 bytes * 10 bits / 9600 baud ~= 19.8 ms, plus preamble
            // (1300 us + 130 us) ~= 21.2 ms total. Use a 50 ms timeout margin.
            err = rmt_tx_wait_all_done(this->rmt_chan_, 50);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "rmt_tx_wait_all_done failed: %s", esp_err_to_name(err));
                return false;
            }

            this->flags_.transmit_pending = false;
            this->transmit_pending_start_ = 0;
            this->on_command_sent_.trigger();
            return true;
        }

    } // namespace secplus2
} // namespace ratgdo
} // namespace esphome

#endif // USE_ESP32 && PROTOCOL_SECPLUSV2 && PROTOCOL_SECPLUSV2_ESP32_RMT
