/************************************
 * Rage
 * Against
 * The
 * Garage
 * Door
 * Opener
 *
 * Copyright (C) 2022  Paul Wieland
 *
 * GNU GENERAL PUBLIC LICENSE
 ************************************/

#pragma once
#include "SoftwareSerial.h" // Using espsoftwareserial https://github.com/plerup/espsoftwareserial
#include "callbacks.h"
#include "enum.h"
#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/core/log.h"
#include "esphome/core/preferences.h"
#include "observable.h"

extern "C" {
#include "secplus.h"
}

// query_status_flags
#define QSF_STATUS 0b0000001
#define QSF_EXT_STATUS 0b0000010
#define QSF_TCC_DUR 0b0000100
#define QSF_OPENINGS 0b0001000

#include "ratgdo_state.h"

namespace esphome {
namespace ratgdo {

    class RATGDOComponent;
    typedef Parented<RATGDOComponent> RATGDOClient;

    static const uint8_t PACKET_LENGTH = 19;
    typedef uint8_t WirePacket[PACKET_LENGTH];

    const float DOOR_POSITION_UNKNOWN = -1.0;
    const float DOOR_DELTA_UNKNOWN = -2.0;

    namespace data {
        const uint32_t LIGHT_OFF = 0;
        const uint32_t LIGHT_ON = 1;
        const uint32_t LIGHT_TOGGLE = 2;
        const uint32_t LIGHT_TOGGLE2 = 3;

        const uint32_t LOCK_OFF = 0;
        const uint32_t LOCK_ON = 1;
        const uint32_t LOCK_TOGGLE = 2;

        const uint32_t DOOR_CLOSE = 0;
        const uint32_t DOOR_OPEN = 1;
        const uint32_t DOOR_TOGGLE = 2;
        const uint32_t DOOR_STOP = 3;

        const uint32_t TTC_GET_DURATION = 1;
        const uint32_t TTC_CANCEL_OFF = 0x000501; // Unknown meaning for these bytes, mimic wall pad
        const uint32_t TTC_CANCEL_TOGGLE_HOLD = 0x000401; // Unknown meaning for these bytes, mimic wall pad

        const uint32_t GET_EXT_STATUS = 1;
    }

    ENUM(Command, uint16_t,
        (UNKNOWN, 0x000),

        (GET_STATUS, 0x080),
        (STATUS, 0x081),
        (GET_EXT_STATUS, 0x0a0), // Extended status has TTC state in bit0 to bit2 of byte1. Bit3 has something but not sure what it is
        (EXT_STATUS, 0x0a1),

        (OBST_1, 0x084), // sent when an obstruction happens?
        (OBST_2, 0x085), // sent when an obstruction happens?

        (LEARN_2, 0x181),
        (LOCK, 0x18c),
        (DOOR_ACTION, 0x280),
        (LIGHT, 0x281),
        (MOTOR_ON, 0x284),
        (MOTION, 0x285),

        (LEARN_1, 0x391),
        (PING, 0x392),
        (PING_RESP, 0x393),

        (TTC_GET_DURATION, 0x400),
        (TTC_DURATION, 0x401), // data appears to contain the current TTC setting in gdo
        (TTC_SET_DURATION, 0x402), // Set time to close in seconds = (byte1<<8)+byte2
        (TTC_CANCEL, 0x408), // OFF or TOGGLE_HOLD are options in data
        (TTC_COUNTDOWN, 0x40a), // Time to close countdown in seconds

        (GET_OPENINGS, 0x48b),
        (OPENINGS, 0x48c), // openings = (byte1<<8)+byte2
    )

    inline bool operator==(const uint16_t cmd_i, const Command& cmd_e) { return cmd_i == static_cast<uint16_t>(cmd_e); }
    inline bool operator==(const Command& cmd_e, const uint16_t cmd_i) { return cmd_i == static_cast<uint16_t>(cmd_e); }

    struct RATGDOStore {
        int obstruction_low_count = 0; // count obstruction low pulses

        static void IRAM_ATTR HOT isr_obstruction(RATGDOStore* arg)
        {
            arg->obstruction_low_count++;
        }
    };

    class RATGDOComponent : public Component {
    public:
        void setup() override;
        void loop() override;
        void dump_config() override;

        observable<uint32_t> rolling_code_counter { 0 };

        float start_opening { -1 };
        observable<float> opening_duration { 0 };
        float start_closing { -1 };
        observable<float> closing_duration { 0 };

        observable<uint16_t> openings { 0 }; // number of times the door has been opened

        observable<DoorState> door_state { DoorState::UNKNOWN };
        observable<float> door_position { DOOR_POSITION_UNKNOWN };

        unsigned long door_start_moving { 0 };
        float door_start_position { DOOR_POSITION_UNKNOWN };
        float door_move_delta { DOOR_DELTA_UNKNOWN };

        observable<LightState> light_state { LightState::UNKNOWN };
        observable<LockState> lock_state { LockState::UNKNOWN };
        observable<HoldState> hold_state { HoldState::UNKNOWN };
        observable<uint16_t> ttc_time_seconds { 0xFFFF };
        observable<ObstructionState> obstruction_state { ObstructionState::UNKNOWN };
        observable<MotorState> motor_state { MotorState::UNKNOWN };
        observable<ButtonState> button_state { ButtonState::UNKNOWN };
        observable<MotionState> motion_state { MotionState::UNKNOWN };

        OnceCallbacks<void(DoorState)> door_state_received;
        OnceCallbacks<void()> command_sent;

        observable<bool> sync_failed { false };

        void set_output_gdo_pin(InternalGPIOPin* pin) { this->output_gdo_pin_ = pin; }
        void set_input_gdo_pin(InternalGPIOPin* pin) { this->input_gdo_pin_ = pin; }
        void set_input_obst_pin(InternalGPIOPin* pin) { this->input_obst_pin_ = pin; }
        void set_client_id(uint64_t client_id) { this->client_id_ = client_id & 0xffffff; } // not sure how large client_id can be, assuming not more than 24 bits

        void gdo_state_loop();
        uint16_t decode_packet(const WirePacket& packet);
        void obstruction_loop();
        void send_command(Command command, uint32_t data = 0, bool increment = true);
        void send_command(Command command, uint32_t data, bool increment, std::function<void()>&& on_sent);
        bool transmit_packet();
        void encode_packet(Command command, uint32_t data, bool increment, WirePacket& packet);
        void print_packet(const WirePacket& packet) const;

        void increment_rolling_code_counter(int delta = 1);
        void set_rolling_code_counter(uint32_t code);

        // door
        void door_command(uint32_t data);
        void ensure_door_command(uint32_t data, uint32_t delay = 1500);
        void toggle_door();
        void open_door();
        void close_door();
        void stop_door();
        void door_move_to_position(float position);
        void set_door_position(float door_position) { this->door_position = door_position; }
        void set_opening_duration(float duration);
        void set_closing_duration(float duration);
        void schedule_door_position_sync(float update_period = 500);
        void door_position_update();
        void cancel_position_sync_callbacks();

        // light
        void toggle_light();
        void light_on();
        void light_off();
        LightState get_light_state() const;

        // lock
        void toggle_lock();
        void lock();
        void unlock();

        // hold
        void toggle_hold();
        void hold_enable();
        void hold_disable();

        // TTC
        void turn_ttc_off();
        void ttc_toggle_hold();
        void set_ttc_sec(uint16_t duration);

        // button functionality
        void query_status();
        [[deprecated("query_status() now requests the opening count.")]] void query_openings();
        void sync();
        void close_with_alert();

        // children subscriptions
        void subscribe_rolling_code_counter(std::function<void(uint32_t)>&& f);
        void subscribe_opening_duration(std::function<void(float)>&& f);
        void subscribe_closing_duration(std::function<void(float)>&& f);
        void subscribe_openings(std::function<void(uint16_t)>&& f);
        void subscribe_door_state(std::function<void(DoorState, float)>&& f);
        void subscribe_light_state(std::function<void(LightState)>&& f);
        void subscribe_lock_state(std::function<void(LockState)>&& f);
        void subscribe_hold_state(std::function<void(HoldState)>&& f);
        void subscribe_ttc_seconds(std::function<void(uint16_t)>&& f);
        void subscribe_obstruction_state(std::function<void(ObstructionState)>&& f);
        void subscribe_motor_state(std::function<void(MotorState)>&& f);
        void subscribe_button_state(std::function<void(ButtonState)>&& f);
        void subscribe_motion_state(std::function<void(MotionState)>&& f);
        void subscribe_sync_failed(std::function<void(bool)>&& f);

    protected:
        // tx data
        bool transmit_pending_ { false };
        WirePacket tx_packet_;

        RATGDOStore isr_store_ {};
        SoftwareSerial sw_serial_;

        bool obstruction_from_status_ { false };
        bool restore_ttc_ { false };
        bool restore_hold_state_ { false };

        InternalGPIOPin* output_gdo_pin_;
        InternalGPIOPin* input_gdo_pin_;
        InternalGPIOPin* input_obst_pin_;
        uint64_t client_id_ { 0x539 };

        uint16_t query_status_flags_;

    }; // RATGDOComponent

} // namespace ratgdo
} // namespace esphome
