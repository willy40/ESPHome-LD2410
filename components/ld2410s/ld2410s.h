#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/number/number.h"

namespace esphome {
namespace ld2410s {

// Forward declare the main component so Number subclasses can call back into it
class LD2410SComponent;

class LD2410SNumber : public number::Number, public Component {
 public:
  void set_parent(LD2410SComponent *parent) { this->parent_ = parent; }
  void set_role(uint8_t role) { this->role_ = role; }  // 0=max_gate 1=min_gate 2=none_duration
 protected:
  void control(float value) override;
  LD2410SComponent *parent_{nullptr};
  uint8_t role_{0};
};

/*
 * LD2410S protocol differs from LD2410:
 *  - Default baud rate: 115200  (LD2410: 256000)
 *  - Minimal frame:  6E <state> <dist_lo> <dist_hi> 62  (default output mode)
 *  - Standard frame: F4 F3 F2 F1 <len 2B> 0x01 <state> <dist 2B> <res 2B> <64B gate energy> F8 F7 F6 F5
 *  - Enable config:  cmd 0x00FF  val 0x0001
 *  - End config:     cmd 0x00FE  (no value)
 *  - Write params:   cmd 0x0070  format: (2B param_word + 4B value) × N
 *  - Read params:    cmd 0x0071  format: (2B param_word) × N  → (4B value) × N
 *  - Switch mode:    cmd 0x007A  val: 000001000000=standard  000000000000=minimal
 *  - 16 distance gates  (LD2410: 9), gate = 0.75 m
 *  - State: 0/1 = no one,  2/3 = someone present  (LD2410: 0x00–0x03 moving/still)
 */

class LD2410SComponent : public PollingComponent, public uart::UARTDevice {
 public:
  // ----- setters called from Python codegen -----
  void set_has_target_binary_sensor(binary_sensor::BinarySensor *s)  { this->has_target_ = s; }
  void set_last_cmd_ok_binary_sensor(binary_sensor::BinarySensor *s) { this->last_cmd_ok_ = s; }
  void set_distance_sensor(sensor::Sensor *s)       { this->distance_ = s; }
  void set_max_gate_number(LD2410SNumber *n)        { this->max_gate_ = n; }
  void set_min_gate_number(LD2410SNumber *n)        { this->min_gate_ = n; }
  void set_none_duration_number(LD2410SNumber *n)   { this->none_duration_ = n; }

  // ----- ESPHome lifecycle -----
  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;

  // ----- public actions (called from YAML buttons / number set_action) -----
  void set_config_mode(bool enable);
  void query_parameters();
  void set_distances_and_none_duration(int max_gate, int min_gate, int none_s);
  void switch_output_mode(bool standard);  // true=standard, false=minimal

 protected:
  // ----- UART framing -----
  void send_command_(uint8_t cmd_lo, uint8_t cmd_hi, const uint8_t *value, size_t value_len);
  void readline_(int readch);
  void handle_minimal_frame_(const uint8_t *buf, int len);
  void handle_standard_frame_(const uint8_t *buf, int len);
  void handle_ack_frame_(const uint8_t *buf, int len);

  static int16_t two_byte_to_int_(uint8_t lo, uint8_t hi) {
    return static_cast<int16_t>((hi << 8) | lo);
  }

  // ----- entities -----
  binary_sensor::BinarySensor *has_target_{nullptr};
  binary_sensor::BinarySensor *last_cmd_ok_{nullptr};
  sensor::Sensor *distance_{nullptr};
  LD2410SNumber *max_gate_{nullptr};
  LD2410SNumber *min_gate_{nullptr};
  LD2410SNumber *none_duration_{nullptr};

  // ----- rx buffer -----
  static const int RX_BUF_SIZE = 96;
  uint8_t rx_buf_[RX_BUF_SIZE]{};
  int rx_pos_{0};

  uint32_t last_periodic_ms_{0};
};

}  // namespace ld2410s
}  // namespace esphome