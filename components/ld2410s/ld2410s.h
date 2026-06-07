#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/number/number.h"

namespace esphome {
namespace ld2410s {

static const uint8_t NUM_GATES = 16;

class LD2410SComponent;

class LD2410SNumber : public number::Number, public Component {
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

class LD2410SComponent : public PollingComponent, public uart::UARTDevice {
 public:
  void set_has_target_binary_sensor(binary_sensor::BinarySensor *s)  { this->has_target_ = s; }
  void set_last_cmd_ok_binary_sensor(binary_sensor::BinarySensor *s) { this->last_cmd_ok_ = s; }
  void set_distance_sensor(sensor::Sensor *s)    { this->distance_ = s; }
  void set_max_gate_number(LD2410SNumber *n)      { this->max_gate_ = n; }
  void set_min_gate_number(LD2410SNumber *n)      { this->min_gate_ = n; }
  void set_none_duration_number(LD2410SNumber *n) { this->none_duration_ = n; }
  void set_off_delay(uint32_t ms)                 { this->off_delay_ms_ = ms; }

  // gate energy sensors — set individually by codegen
  void set_gate_energy_sensor(uint8_t gate, sensor::Sensor *s) {
    if (gate < NUM_GATES) this->gate_energy_[gate] = s;
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

  static int16_t two_byte_to_int_(uint8_t lo, uint8_t hi) {
    return static_cast<int16_t>((hi << 8) | lo);
  }

  binary_sensor::BinarySensor *has_target_{nullptr};
  binary_sensor::BinarySensor *last_cmd_ok_{nullptr};
  sensor::Sensor *distance_{nullptr};
  sensor::Sensor *gate_energy_[NUM_GATES]{};

  uint32_t last_detection_ms_{0};
  uint32_t off_delay_ms_{5000};
  bool target_state_{false};

  static const int RX_BUF_SIZE = 96;
  uint8_t rx_buf_[RX_BUF_SIZE]{};
  int rx_pos_{0};
  uint32_t last_periodic_ms_{0};
};

}  // namespace ld2410s
}  // namespace esphome
