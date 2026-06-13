#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/number/number.h"

namespace esphome
{
  namespace ld2410s
  {

    static const uint8_t NUM_GATES = 16;

    enum class CmdFlow : uint8_t
    {
      IDLE,
      QUERY_PARAMS,
      SET_DISTANCES,
      SWITCH_OUTPUT
    };

    class LD2410SComponent;

    class LD2410SNumber : public number::Number, public Component
    {
    public:
      void set_parent(LD2410SComponent *parent) { this->parent_ = parent; }
      void set_role(uint8_t role) { this->role_ = role; }
      void setup() override {}
      void dump_config() override {}

    protected:
      void control(float value) override;
      LD2410SComponent *parent_{nullptr};
      uint8_t role_{0};
    };

    class LD2410SComponent : public PollingComponent, public uart::UARTDevice
    {
    public:
      void set_has_target_binary_sensor(binary_sensor::BinarySensor *s) { this->has_target_ = s; }
      void set_last_cmd_ok_binary_sensor(binary_sensor::BinarySensor *s) { this->last_cmd_ok_ = s; }
      void set_distance_sensor(sensor::Sensor *s) { this->distance_ = s; }
      void set_max_gate_number(LD2410SNumber *n) { this->max_gate_ = n; }
      void set_min_gate_number(LD2410SNumber *n) { this->min_gate_ = n; }
      void set_none_duration_number(LD2410SNumber *n) { this->none_duration_ = n; }

      // gate energy sensors — set individually by codegen
      void set_gate_energy_sensor(uint8_t gate, sensor::Sensor *s)
      {
        if (gate < NUM_GATES)
          this->gate_energy_[gate] = s;
      }

      void setup() override;
      void loop() override;
      void update() override {}
      void dump_config() override;

      void set_config_mode(bool enable);
      void query_parameters();
      void set_distances_and_none_duration(int max_gate, int min_gate, int none_s);
      void switch_output_mode(bool standard);

      // public: accessed by LD2410SNumber::control()
      LD2410SNumber *max_gate_{nullptr};
      LD2410SNumber *min_gate_{nullptr};
      LD2410SNumber *none_duration_{nullptr};

    protected:
      void send_command_(uint8_t cmd_lo, uint8_t cmd_hi, const uint8_t *value, size_t value_len);
      void readline_(int readch);
      void handle_minimal_frame_(const uint8_t *buf, int len);
      void handle_standard_frame_(const uint8_t *buf, int len);
      void handle_ack_frame_(const uint8_t *buf, int len);
      void publish_presence_(bool detected);
      void advance_flow_();

      // ring-buffer helpers
      uint8_t rx_peek_(size_t offset) const;      // byte at distance `offset` from head, no removal
      void rx_drop_(size_t n);                    // advance head by n, decrement count (resync)
      void rx_extract_(uint8_t *out, size_t len); // copy len bytes out (unwrapping the ring) and drop them

      static int16_t two_byte_to_int_(uint8_t lo, uint8_t hi)
      {
        return static_cast<int16_t>((hi << 8) | lo);
      }

      // -------------------------------------------------------------------------
      // Framing protocol constants
      // -------------------------------------------------------------------------
      // Frame headers / footers
      static constexpr uint8_t FRAME_HDR_MINIMAL = 0x6E;
      static constexpr uint8_t FRAME_FTR_MINIMAL = 0x62;

      static constexpr uint8_t FRAME_HDR_ACK0 = 0xFD, FRAME_HDR_ACK1 = 0xFC,
                               FRAME_HDR_ACK2 = 0xFB, FRAME_HDR_ACK3 = 0xFA;
      static constexpr uint8_t ACK_FTR0 = 0x04, ACK_FTR1 = 0x03, ACK_FTR2 = 0x02, ACK_FTR3 = 0x01;

      static constexpr uint8_t FRAME_HDR_STD0 = 0xF4, FRAME_HDR_STD1 = 0xF3,
                               FRAME_HDR_STD2 = 0xF2, FRAME_HDR_STD3 = 0xF1;
      static constexpr uint8_t STD_FTR0 = 0xF8, STD_FTR1 = 0xF7, STD_FTR2 = 0xF6, STD_FTR3 = 0xF5;

      // Frame lengths
      static constexpr size_t MINIMAL_FRAME_LEN = 5;
      static constexpr size_t ACK_FRAME_MIN_LEN = 10;
      static constexpr size_t ACK_FRAME_MAX_LEN = 64; // resync bound
      static constexpr size_t STANDARD_FRAME_MIN_LEN = 80;
      static constexpr size_t STANDARD_FRAME_MAX_LEN = 96; // resync bound

      // Target-state byte values (presence)
      static constexpr uint8_t TARGET_STATE_MOVING = 0x02;
      static constexpr uint8_t TARGET_STATE_STATIC = 0x03;

      // Minimal-frame field offsets
      static constexpr int MIN_OFF_STATE = 1;
      static constexpr int MIN_OFF_DIST_LO = 2;
      static constexpr int MIN_OFF_DIST_HI = 3;

      // Standard-frame field offsets
      static constexpr int STD_OFF_FRAME_TYPE = 6;
      static constexpr uint8_t STD_FRAME_TYPE_TARGET = 0x01;
      static constexpr int STD_OFF_STATE = 7;
      static constexpr int STD_OFF_DIST_LO = 8;
      static constexpr int STD_OFF_DIST_HI = 9;
      static constexpr int STD_OFF_GATE_ENERGY_START = 12;
      static constexpr int STD_GATE_ENERGY_STRIDE = 4;

      // ACK-frame field offsets
      static constexpr int ACK_OFF_CMD = 6;
      static constexpr uint8_t ACK_FRAME_TYPE_REPLY = 0x01;
      static constexpr int ACK_OFF_FRAME_TYPE = 7;
      static constexpr int ACK_OFF_STATUS_LO = 8;
      static constexpr int ACK_OFF_STATUS_HI = 9;
      static constexpr uint8_t CMD_QUERY_PARAMS = 0x71;
      static constexpr size_t ACK_QUERY_PARAMS_LEN = 22;
      static constexpr int ACK_OFF_MAX_GATE = 10;
      static constexpr int ACK_OFF_MIN_GATE = 14;
      static constexpr int ACK_OFF_NONE_DELAY = 18;

      binary_sensor::BinarySensor *has_target_{nullptr};
      binary_sensor::BinarySensor *last_cmd_ok_{nullptr};
      sensor::Sensor *distance_{nullptr};
      sensor::Sensor *gate_energy_[NUM_GATES]{};

      bool target_state_{false};

      static const size_t RX_BUF_SIZE = 512; // must be a power of two
      static const size_t RX_BUF_MASK = RX_BUF_SIZE - 1;
      uint8_t rx_buf_[RX_BUF_SIZE]{};
      size_t rx_head_{0};
      size_t rx_tail_{0};
      size_t rx_count_{0};

      // async command state machine
      CmdFlow cmd_flow_{CmdFlow::IDLE};
      uint8_t cmd_step_{0};
      int pending_max_gate_{0};
      int pending_min_gate_{0};
      int pending_none_s_{0};
      bool pending_standard_{false};
    };

  } // namespace ld2410s
} // namespace esphome
