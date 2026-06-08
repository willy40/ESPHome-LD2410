#include "ld2410s.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ld2410s {

static const char *const TAG = "ld2410s";

// ---------------------------------------------------------------------------
// LD2410SNumber::control
// ---------------------------------------------------------------------------
void LD2410SNumber::control(float value) {
  this->publish_state(value);
  if (this->parent_ == nullptr)
    return;
  float max_g = (this->parent_->max_gate_      != nullptr) ? this->parent_->max_gate_->state      : 16;
  float min_g = (this->parent_->min_gate_      != nullptr) ? this->parent_->min_gate_->state      : 0;
  float none  = (this->parent_->none_duration_ != nullptr) ? this->parent_->none_duration_->state : 10;
  switch (this->role_) {
    case 0: max_g = value; break;
    case 1: min_g = value; break;
    case 2: none  = value; break;
  }
  this->parent_->set_distances_and_none_duration((int) max_g, (int) min_g, (int) none);
}

// ---------------------------------------------------------------------------
// LD2410SComponent — lifecycle
// ---------------------------------------------------------------------------
void LD2410SComponent::setup() {
  ESP_LOGCONFIG(TAG, "LD2410S setup done");
  // query parameters after 2s so the sensor UART is ready and loop() is running
  this->set_timeout(2000, [this]() {
    this->query_parameters();
  });
}

void LD2410SComponent::loop() {
  while (this->available())
    this->readline_(this->read());

  // OFF debounce: if no detection for off_delay_ms, publish OFF
  if (this->target_state_ && this->has_target_ != nullptr) {
    if (millis() - this->last_detection_ms_ > this->off_delay_ms_) {
      this->target_state_ = false;
      this->has_target_->publish_state(false);
    }
  }
}

void LD2410SComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "HLK-LD2410S:");
  ESP_LOGCONFIG(TAG, "  Off delay: %u ms", this->off_delay_ms_);
  LOG_BINARY_SENSOR("  ", "Has Target",  this->has_target_);
  LOG_BINARY_SENSOR("  ", "Last Cmd OK", this->last_cmd_ok_);
  LOG_SENSOR("  ",        "Distance",    this->distance_);
}

// ---------------------------------------------------------------------------
// Presence publisher with debounce
// ---------------------------------------------------------------------------
void LD2410SComponent::publish_presence_(bool detected) {
  if (detected) {
    this->last_detection_ms_ = millis();
    if (!this->target_state_) {
      this->target_state_ = true;
      if (this->has_target_ != nullptr)
        this->has_target_->publish_state(true);
    }
  }
  // OFF is handled by the loop() timeout — never publish OFF here directly
}

// ---------------------------------------------------------------------------
// UART framing
// ---------------------------------------------------------------------------
void LD2410SComponent::send_command_(uint8_t cmd_lo, uint8_t cmd_hi,
                                     const uint8_t *value, size_t value_len) {
  if (this->last_cmd_ok_ != nullptr)
    this->last_cmd_ok_->publish_state(false);

  this->write_byte(0xFD); this->write_byte(0xFC);
  this->write_byte(0xFB); this->write_byte(0xFA);

  uint16_t len = 2 + (uint16_t) value_len;
  this->write_byte(lowByte(len));
  this->write_byte(highByte(len));
  this->write_byte(cmd_lo);
  this->write_byte(cmd_hi);

  if (value != nullptr) {
    for (size_t i = 0; i < value_len; i++)
      this->write_byte(value[i]);
  }

  this->write_byte(0x04); this->write_byte(0x03);
  this->write_byte(0x02); this->write_byte(0x01);
  delay(50);
}

void LD2410SComponent::readline_(int readch) {
  if (readch < 0)
    return;

  if (this->rx_pos_ < RX_BUF_SIZE - 1) {
    this->rx_buf_[this->rx_pos_++] = static_cast<uint8_t>(readch);
  } else {
    this->rx_pos_ = 0;
    return;
  }

  int pos = this->rx_pos_;

  if (pos == 5 && this->rx_buf_[0] == 0x6E && this->rx_buf_[4] == 0x62) {
    this->handle_minimal_frame_(this->rx_buf_, pos);
    this->rx_pos_ = 0;
    return;
  }

  if (pos >= 4 &&
      this->rx_buf_[pos-4] == 0x04 && this->rx_buf_[pos-3] == 0x03 &&
      this->rx_buf_[pos-2] == 0x02 && this->rx_buf_[pos-1] == 0x01) {
    this->handle_ack_frame_(this->rx_buf_, pos);
    this->rx_pos_ = 0;
    return;
  }

  if (pos >= 4 &&
      this->rx_buf_[pos-4] == 0xF8 && this->rx_buf_[pos-3] == 0xF7 &&
      this->rx_buf_[pos-2] == 0xF6 && this->rx_buf_[pos-1] == 0xF5) {
    this->handle_standard_frame_(this->rx_buf_, pos);
    this->rx_pos_ = 0;
    return;
  }

  if (this->rx_pos_ >= RX_BUF_SIZE - 1)
    this->rx_pos_ = 0;
}

// ---------------------------------------------------------------------------
// Frame handlers
// ---------------------------------------------------------------------------
void LD2410SComponent::handle_minimal_frame_(const uint8_t *buf, int len) {
  if (len != 5 || buf[0] != 0x6E || buf[4] != 0x62)
    return;

  bool detected = (buf[1] == 0x02 || buf[1] == 0x03);
  this->publish_presence_(detected);

  if (!detected)
    return;

  uint32_t now = millis();
  if (now - this->last_periodic_ms_ < 1000)
    return;
  this->last_periodic_ms_ = now;

  int dist = two_byte_to_int_(buf[2], buf[3]);
  if (this->distance_ != nullptr && (int) this->distance_->get_state() != dist)
    this->distance_->publish_state(dist);
}

void LD2410SComponent::handle_standard_frame_(const uint8_t *buf, int len) {
  if (len < 80) return;
  if (buf[0] != 0xF4 || buf[1] != 0xF3 || buf[2] != 0xF2 || buf[3] != 0xF1) return;
  if (buf[6] != 0x01) return;
  if (buf[len-4] != 0xF8 || buf[len-3] != 0xF7 || buf[len-2] != 0xF6 || buf[len-1] != 0xF5) return;

  bool detected = (buf[7] == 0x02 || buf[7] == 0x03);
  this->publish_presence_(detected);

  if (!detected)
    return;

  uint32_t now = millis();
  if (now - this->last_periodic_ms_ < 1000)
    return;
  this->last_periodic_ms_ = now;

  int dist = two_byte_to_int_(buf[8], buf[9]);
  if (this->distance_ != nullptr && (int) this->distance_->get_state() != dist)
    this->distance_->publish_state(dist);

  // gate energy: 4 bytes per gate (lo, hi, lo, hi) starting at buf[12]
  // standard frame layout: buf[12..75] = 16 gates × 4 bytes
  // each gate value = buf[12 + gate*4] | (buf[13 + gate*4] << 8)  (16-bit, hi bytes unused)
  for (uint8_t g = 0; g < NUM_GATES; g++) {
    if (this->gate_energy_[g] == nullptr) continue;
    uint16_t energy = buf[12 + g * 4] | (buf[13 + g * 4] << 8);
    if ((int) this->gate_energy_[g]->get_state() != energy)
      this->gate_energy_[g]->publish_state(energy);
  }
}

void LD2410SComponent::handle_ack_frame_(const uint8_t *buf, int len) {
  if (len < 10) return;
  if (buf[0] != 0xFD || buf[1] != 0xFC || buf[2] != 0xFB || buf[3] != 0xFA) return;
  if (buf[7] != 0x01) return;

  uint16_t ack = static_cast<uint16_t>((buf[9] << 8) | buf[8]);
  if (ack != 0x0000) {
    ESP_LOGW(TAG, "Command ACK error: 0x%04X", ack);
    if (this->last_cmd_ok_ != nullptr)
      this->last_cmd_ok_->publish_state(false);
    return;
  }
  if (this->last_cmd_ok_ != nullptr)
    this->last_cmd_ok_->publish_state(true);

  if (buf[6] == 0x71 && len >= 22) {
    uint32_t max_gate   = buf[10] | (buf[11]<<8) | (buf[12]<<16) | (buf[13]<<24);
    uint32_t min_gate   = buf[14] | (buf[15]<<8) | (buf[16]<<16) | (buf[17]<<24);
    uint32_t none_delay = buf[18] | (buf[19]<<8) | (buf[20]<<16) | (buf[21]<<24);
    ESP_LOGD(TAG, "Params: max_gate=%u min_gate=%u none_delay=%us", max_gate, min_gate, none_delay);
    if (this->max_gate_ != nullptr)      this->max_gate_->publish_state(max_gate);
    if (this->min_gate_ != nullptr)      this->min_gate_->publish_state(min_gate);
    if (this->none_duration_ != nullptr) this->none_duration_->publish_state(none_delay);
  }
}

// ---------------------------------------------------------------------------
// Public actions
// ---------------------------------------------------------------------------
void LD2410SComponent::set_config_mode(bool enable) {
  if (enable) {
    uint8_t val[2] = {0x01, 0x00};
    this->send_command_(0xFF, 0x00, val, 2);
  } else {
    this->send_command_(0xFE, 0x00, nullptr, 0);
  }
}

void LD2410SComponent::query_parameters() {
  uint8_t val[6] = {0x05, 0x00,  0x0A, 0x00,  0x06, 0x00};
  this->send_command_(0x71, 0x00, val, 6);
}

void LD2410SComponent::set_distances_and_none_duration(int max_gate, int min_gate, int none_s) {
  uint8_t val[18] = {
    0x05, 0x00, (uint8_t)(max_gate & 0xFF), (uint8_t)(max_gate >> 8), 0x00, 0x00,
    0x0A, 0x00, (uint8_t)(min_gate & 0xFF), (uint8_t)(min_gate >> 8), 0x00, 0x00,
    0x06, 0x00, (uint8_t)(none_s   & 0xFF), (uint8_t)(none_s   >> 8), 0x00, 0x00,
  };
  this->set_config_mode(true);
  this->send_command_(0x70, 0x00, val, 18);
  this->query_parameters();
  this->set_config_mode(false);
}

void LD2410SComponent::switch_output_mode(bool standard) {
  uint8_t val[6] = {0x00, 0x00, (uint8_t)(standard ? 0x01 : 0x00), 0x00, 0x00, 0x00};
  this->set_config_mode(true);
  this->send_command_(0x7A, 0x00, val, 6);
  this->set_config_mode(false);
}

}  // namespace ld2410s
}  // namespace esphome