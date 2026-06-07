#include "ld2410s.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ld2410s {

static const char *const TAG = "ld2410s";

// ---------------------------------------------------------------------------
// ESPHome lifecycle
// ---------------------------------------------------------------------------

void LD2410SComponent::setup() {
  this->set_update_interval(15000);
  ESP_LOGCONFIG(TAG, "LD2410S setup done");
}

void LD2410SComponent::loop() {
  while (this->available()) {
    this->readline_(this->read());
  }
}

void LD2410SComponent::update() {
  // Periodic query can be added here if needed
}

void LD2410SComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "HLK-LD2410S:");
  LOG_BINARY_SENSOR("  ", "Has Target",         this->has_target_);
  LOG_BINARY_SENSOR("  ", "Last Command OK",     this->last_cmd_ok_);
  LOG_SENSOR("  ",        "Distance",            this->distance_);
}

// ---------------------------------------------------------------------------
// UART low-level
// ---------------------------------------------------------------------------

void LD2410SComponent::send_command_(uint8_t cmd_lo, uint8_t cmd_hi,
                                     const uint8_t *value, size_t value_len) {
  if (this->last_cmd_ok_ != nullptr)
    this->last_cmd_ok_->publish_state(false);

  // frame start
  this->write_byte(0xFD); this->write_byte(0xFC);
  this->write_byte(0xFB); this->write_byte(0xFA);
  // length = 2 (cmd word) + value_len
  uint16_t len = 2 + value_len;
  this->write_byte(lowByte(len)); this->write_byte(highByte(len));
  // command word (little-endian)
  this->write_byte(cmd_lo); this->write_byte(cmd_hi);
  // value bytes
  if (value != nullptr) {
    for (size_t i = 0; i < value_len; i++)
      this->write_byte(value[i]);
  }
  // frame end
  this->write_byte(0x04); this->write_byte(0x03);
  this->write_byte(0x02); this->write_byte(0x01);
  delay(50);
}

void LD2410SComponent::readline_(int readch) {
  if (readch < 0) return;

  if (this->rx_pos_ < RX_BUF_SIZE - 1) {
    this->rx_buf_[this->rx_pos_++] = static_cast<uint8_t>(readch);
  } else {
    this->rx_pos_ = 0;
    return;
  }

  int pos = this->rx_pos_;

  // --- minimal frame: exactly 5 bytes 6E ?? ?? ?? 62 ---
  if (pos == 5 && this->rx_buf_[0] == 0x6E && this->rx_buf_[4] == 0x62) {
    this->handle_minimal_frame_(this->rx_buf_, pos);
    this->rx_pos_ = 0;
    return;
  }

  // --- ACK / config frame end: 04 03 02 01 ---
  if (pos >= 4 &&
      this->rx_buf_[pos - 4] == 0x04 && this->rx_buf_[pos - 3] == 0x03 &&
      this->rx_buf_[pos - 2] == 0x02 && this->rx_buf_[pos - 1] == 0x01) {
    this->handle_ack_frame_(this->rx_buf_, pos);
    this->rx_pos_ = 0;
    return;
  }

  // --- standard / threshold data frame end: F8 F7 F6 F5 ---
  if (pos >= 4 &&
      this->rx_buf_[pos - 4] == 0xF8 && this->rx_buf_[pos - 3] == 0xF7 &&
      this->rx_buf_[pos - 2] == 0xF6 && this->rx_buf_[pos - 1] == 0xF5) {
    this->handle_standard_frame_(this->rx_buf_, pos);
    this->rx_pos_ = 0;
    return;
  }

  // safety reset
  if (this->rx_pos_ >= RX_BUF_SIZE - 1)
    this->rx_pos_ = 0;
}

// ---------------------------------------------------------------------------
// Frame handlers
// ---------------------------------------------------------------------------

void LD2410SComponent::handle_minimal_frame_(const uint8_t *buf, int len) {
  // 6E <state 1B> <dist_lo> <dist_hi> 62
  if (len != 5) return;
  if (buf[0] != 0x6E || buf[4] != 0x62) return;

  uint8_t state = buf[1];
  bool presence = (state == 0x02 || state == 0x03);

  if (this->has_target_ != nullptr)
    this->has_target_->publish_state(presence);

  uint32_t now = millis();
  if (now - this->last_periodic_ms_ < 1000) return;
  this->last_periodic_ms_ = now;

  int dist = two_byte_to_int_(buf[2], buf[3]);
  if (this->distance_ != nullptr && (int) this->distance_->get_state() != dist)
    this->distance_->publish_state(dist);
}

void LD2410SComponent::handle_standard_frame_(const uint8_t *buf, int len) {
  // F4 F3 F2 F1 <len_lo> <len_hi> 0x01 <state> <dist_lo> <dist_hi>
  // <res_lo> <res_hi> <64B gate energy> F8 F7 F6 F5  → total 80 bytes
  if (len < 80) return;
  if (buf[0] != 0xF4 || buf[1] != 0xF3 || buf[2] != 0xF2 || buf[3] != 0xF1) return;
  if (buf[6] != 0x01) return;
  if (buf[len - 4] != 0xF8 || buf[len - 3] != 0xF7 ||
      buf[len - 2] != 0xF6 || buf[len - 1] != 0xF5) return;

  uint8_t state = buf[7];
  bool presence = (state == 0x02 || state == 0x03);

  if (this->has_target_ != nullptr)
    this->has_target_->publish_state(presence);

  uint32_t now = millis();
  if (now - this->last_periodic_ms_ < 1000) return;
  this->last_periodic_ms_ = now;

  int dist = two_byte_to_int_(buf[8], buf[9]);
  if (this->distance_ != nullptr && (int) this->distance_->get_state() != dist)
    this->distance_->publish_state(dist);

  // Gate energy bytes at buf[12..75] available for future extension
}

void LD2410SComponent::handle_ack_frame_(const uint8_t *buf, int len) {
  // FD FC FB FA <len_lo> <len_hi> <cmd_lo> <cmd_hi> 0x01 <ack_lo> <ack_hi> [payload] 04 03 02 01
  if (len < 10) return;
  if (buf[0] != 0xFD || buf[1] != 0xFC || buf[2] != 0xFB || buf[3] != 0xFA) return;
  if (buf[7] != 0x01) return;  // response indicator

  uint16_t ack = static_cast<uint16_t>((buf[9] << 8) | buf[8]);
  if (ack != 0x0000) {
    ESP_LOGW(TAG, "Command ACK error: 0x%04X", ack);
    if (this->last_cmd_ok_ != nullptr)
      this->last_cmd_ok_->publish_state(false);
    return;
  }
  if (this->last_cmd_ok_ != nullptr)
    this->last_cmd_ok_->publish_state(true);

  uint8_t cmd_lo = buf[6];

  switch (cmd_lo) {
    case 0x71: {  // Read common parameters response (0x0071)
      // Response payload (after 0x0000 ACK): 4 bytes per requested parameter
      // We request: 0x05 (max gate), 0x0A (min gate), 0x06 (none delay)
      // Offsets in ACK: [10..13] max_gate, [14..17] min_gate, [18..21] none_delay
      if (len < 22) break;
      uint32_t max_gate   = buf[10] | (buf[11] << 8) | (buf[12] << 16) | (buf[13] << 24);
      uint32_t min_gate   = buf[14] | (buf[15] << 8) | (buf[16] << 16) | (buf[17] << 24);
      uint32_t none_delay = buf[18] | (buf[19] << 8) | (buf[20] << 16) | (buf[21] << 24);
      ESP_LOGD(TAG, "Params: max_gate=%u min_gate=%u none_delay=%us", max_gate, min_gate, none_delay);
      if (this->max_gate_ != nullptr)      this->max_gate_->publish_state(max_gate);
      if (this->min_gate_ != nullptr)      this->min_gate_->publish_state(min_gate);
      if (this->none_duration_ != nullptr) this->none_duration_->publish_state(none_delay);
      break;
    }
    default:
      ESP_LOGD(TAG, "ACK for cmd 0x%02X OK", cmd_lo);
      break;
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
  // Read params: cmd 0x0071, request 3 param words: 0x05, 0x0A, 0x06
  uint8_t val[6] = {0x05, 0x00,  0x0A, 0x00,  0x06, 0x00};
  this->send_command_(0x71, 0x00, val, 6);
}

void LD2410SComponent::set_distances_and_none_duration(int max_gate, int min_gate, int none_s) {
  // Write params: cmd 0x0070, each param = 2B word + 4B value
  uint8_t val[18] = {
    0x05, 0x00, (uint8_t)(max_gate & 0xFF), (uint8_t)(max_gate >> 8), 0x00, 0x00,
    0x0A, 0x00, (uint8_t)(min_gate & 0xFF), (uint8_t)(min_gate >> 8), 0x00, 0x00,
    0x06, 0x00, (uint8_t)(none_s  & 0xFF),  (uint8_t)(none_s  >> 8),  0x00, 0x00,
  };
  this->set_config_mode(true);
  this->send_command_(0x70, 0x00, val, 18);
  this->query_parameters();
  this->set_config_mode(false);
}

void LD2410SComponent::switch_output_mode(bool standard) {
  // cmd 0x007A, value: 00 00 01 00 00 00 = standard,  00 00 00 00 00 00 = minimal
  uint8_t val[6] = {0x00, 0x00, (uint8_t)(standard ? 0x01 : 0x00), 0x00, 0x00, 0x00};
  this->set_config_mode(true);
  this->send_command_(0x7A, 0x00, val, 6);
  this->set_config_mode(false);
}

}  // namespace ld2410s
}  // namespace esphome