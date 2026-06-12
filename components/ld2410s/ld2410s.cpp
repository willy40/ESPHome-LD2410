#include "ld2410s.h"
#include "esphome/core/log.h"

#include <cstring>

namespace esphome
{
  namespace ld2410s
  {

    static const char *const TAG = "ld2410s";

    // ---------------------------------------------------------------------------
    // LD2410SNumber::control
    // ---------------------------------------------------------------------------
    void LD2410SNumber::control(float value)
    {
      this->publish_state(value);
      if (this->parent_ == nullptr)
        return;
      float max_g = (this->parent_->max_gate_ != nullptr) ? this->parent_->max_gate_->state : 16;
      float min_g = (this->parent_->min_gate_ != nullptr) ? this->parent_->min_gate_->state : 0;
      float none = (this->parent_->none_duration_ != nullptr) ? this->parent_->none_duration_->state : 10;
      switch (this->role_)
      {
      case 0:
        max_g = value;
        break;
      case 1:
        min_g = value;
        break;
      case 2:
        none = value;
        break;
      }
      this->parent_->set_distances_and_none_duration((int)max_g, (int)min_g, (int)none);
    }

    // ---------------------------------------------------------------------------
    // LD2410SComponent — lifecycle
    // ---------------------------------------------------------------------------
    void LD2410SComponent::setup()
    {
      ESP_LOGCONFIG(TAG, "LD2410S setup done");
      // query parameters after 2s so the sensor UART is ready and loop() is running
      this->set_timeout(2000, [this]()
                        { this->query_parameters(); });
    }

    void LD2410SComponent::loop()
    {
      while (this->available())
        this->readline_(this->read());

      // OFF debounce: if no detection for off_delay_ms, publish OFF
      if (this->target_state_ && this->has_target_ != nullptr)
      {
        if (millis() - this->last_detection_ms_ > this->off_delay_ms_)
        {
          this->target_state_ = false;
          this->has_target_->publish_state(false);
        }
      }
    }

    void LD2410SComponent::dump_config()
    {
      ESP_LOGCONFIG(TAG, "HLK-LD2410S:");
      ESP_LOGCONFIG(TAG, "  Off delay: %u ms", this->off_delay_ms_);
      LOG_BINARY_SENSOR("  ", "Has Target", this->has_target_);
      LOG_BINARY_SENSOR("  ", "Last Cmd OK", this->last_cmd_ok_);
      LOG_SENSOR("  ", "Distance", this->distance_);
    }

    // ---------------------------------------------------------------------------
    // Presence publisher with debounce
    // ---------------------------------------------------------------------------
    void LD2410SComponent::publish_presence_(bool detected)
    {
      if (detected)
      {
        this->last_detection_ms_ = millis();
        if (!this->target_state_)
        {
          this->target_state_ = true;
          if (this->has_target_ != nullptr)
            this->has_target_->publish_state(true);
        }
      }
      // OFF is handled by the loop() timeout — never publish OFF here directly
    }

    // ---------------------------------------------------------------------------
    // UART framing — no delay(), bytes are sent and we wait for ACK via loop()
    // ---------------------------------------------------------------------------
    void LD2410SComponent::send_command_(uint8_t cmd_lo, uint8_t cmd_hi,
                                         const uint8_t *value, size_t value_len)
    {
      if (this->last_cmd_ok_ != nullptr)
        this->last_cmd_ok_->publish_state(false);

      this->write_byte(0xFD);
      this->write_byte(0xFC);
      this->write_byte(0xFB);
      this->write_byte(0xFA);

      uint16_t len = 2 + (uint16_t)value_len;
      this->write_byte(static_cast<uint8_t>(len & 0xFF));
      this->write_byte(static_cast<uint8_t>(len >> 8));
      this->write_byte(cmd_lo);
      this->write_byte(cmd_hi);

      if (value != nullptr)
      {
        for (size_t i = 0; i < value_len; i++)
          this->write_byte(value[i]);
      }

      this->write_byte(0x04);
      this->write_byte(0x03);
      this->write_byte(0x02);
      this->write_byte(0x01);
    }

    void LD2410SComponent::readline_(int readch)
    {
      if (readch < 0)
        return;

      auto drop_first = [this]()
      {
        this->rx_pos_--;
        memmove(this->rx_buf_, this->rx_buf_ + 1, this->rx_pos_);
      };

      // buffer full without a complete frame: drop the oldest byte so a frame
      // hidden inside the remaining data can still be found
      if (this->rx_pos_ >= RX_BUF_SIZE)
        drop_first();
      this->rx_buf_[this->rx_pos_++] = static_cast<uint8_t>(readch);

      ESP_LOGVV(TAG, "RX buf: %s", format_hex_pretty(this->rx_buf_, this->rx_pos_).c_str());

      while (this->rx_pos_ > 0)
      {
        const uint8_t b0 = this->rx_buf_[0];

        // resync: only 0x6E (minimal), 0xFD (ACK) and 0xF4 (standard) can start a frame
        if (b0 != 0x6E && b0 != 0xFD && b0 != 0xF4)
        {
          drop_first();
          continue;
        }

        const int pos = this->rx_pos_;

        if (b0 == 0x6E)
        {
          if (pos < 5)
            return; // wait for more data
          if (this->rx_buf_[4] == 0x62)
          {
            ESP_LOGD(TAG, "Minimal frame: %s", format_hex_pretty(this->rx_buf_, 5).c_str());
            this->handle_minimal_frame_(this->rx_buf_, 5);
            this->rx_pos_ = 0;
            return;
          }
          drop_first(); // false header, resync on remaining bytes
          continue;
        }

        if (b0 == 0xFD)
        {
          if ((pos >= 2 && this->rx_buf_[1] != 0xFC) ||
              (pos >= 3 && this->rx_buf_[2] != 0xFB) ||
              (pos >= 4 && this->rx_buf_[3] != 0xFA))
          {
            drop_first();
            continue;
          }
          if (pos >= 10 &&
              this->rx_buf_[pos - 4] == 0x04 && this->rx_buf_[pos - 3] == 0x03 &&
              this->rx_buf_[pos - 2] == 0x02 && this->rx_buf_[pos - 1] == 0x01)
          {
            ESP_LOGD(TAG, "ACK frame: %s", format_hex_pretty(this->rx_buf_, pos).c_str());
            this->handle_ack_frame_(this->rx_buf_, pos);
            this->rx_pos_ = 0;
          }
          return; // wait for more data
        }

        // b0 == 0xF4 — standard frame
        if ((pos >= 2 && this->rx_buf_[1] != 0xF3) ||
            (pos >= 3 && this->rx_buf_[2] != 0xF2) ||
            (pos >= 4 && this->rx_buf_[3] != 0xF1))
        {
          drop_first();
          continue;
        }
        if (pos >= 80 &&
            this->rx_buf_[pos - 4] == 0xF8 && this->rx_buf_[pos - 3] == 0xF7 &&
            this->rx_buf_[pos - 2] == 0xF6 && this->rx_buf_[pos - 1] == 0xF5)
        {
          ESP_LOGD(TAG, "Standard frame: %s", format_hex_pretty(this->rx_buf_, pos).c_str());
          this->handle_standard_frame_(this->rx_buf_, pos);
          this->rx_pos_ = 0;
        }
        return; // wait for more data
      }
    }

    // ---------------------------------------------------------------------------
    // Frame handlers
    // ---------------------------------------------------------------------------
    void LD2410SComponent::handle_minimal_frame_(const uint8_t *buf, int len)
    {
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
      if (this->distance_ != nullptr && (int)this->distance_->get_state() != dist)
        this->distance_->publish_state(dist);
    }

    void LD2410SComponent::handle_standard_frame_(const uint8_t *buf, int len)
    {
      if (len < 80)
        return;
      if (buf[0] != 0xF4 || buf[1] != 0xF3 || buf[2] != 0xF2 || buf[3] != 0xF1)
        return;
      if (buf[6] != 0x01)
        return;
      if (buf[len - 4] != 0xF8 || buf[len - 3] != 0xF7 || buf[len - 2] != 0xF6 || buf[len - 1] != 0xF5)
        return;

      bool detected = (buf[7] == 0x02 || buf[7] == 0x03);
      this->publish_presence_(detected);

      if (!detected)
        return;

      uint32_t now = millis();
      if (now - this->last_periodic_ms_ < 1000)
        return;
      this->last_periodic_ms_ = now;

      int dist = two_byte_to_int_(buf[8], buf[9]);
      if (this->distance_ != nullptr && (int)this->distance_->get_state() != dist)
        this->distance_->publish_state(dist);

      // gate energy: 4 bytes per gate (lo, hi, lo, hi) starting at buf[12]
      // standard frame layout: buf[12..75] = 16 gates × 4 bytes
      // each gate value = buf[12 + gate*4] | (buf[13 + gate*4] << 8)  (16-bit, hi bytes unused)
      for (uint8_t g = 0; g < NUM_GATES; g++)
      {
        if (this->gate_energy_[g] == nullptr)
          continue;
        uint16_t energy = buf[12 + g * 4] | (buf[13 + g * 4] << 8);
        if ((int)this->gate_energy_[g]->get_state() != energy)
          this->gate_energy_[g]->publish_state(energy);
      }
    }

    void LD2410SComponent::handle_ack_frame_(const uint8_t *buf, int len)
    {
      if (len < 10)
        return;
      if (buf[0] != 0xFD || buf[1] != 0xFC || buf[2] != 0xFB || buf[3] != 0xFA)
        return;
      if (buf[7] != 0x01)
        return;

      uint16_t ack = static_cast<uint16_t>((buf[9] << 8) | buf[8]);
      if (ack != 0x0000)
      {
        ESP_LOGD(TAG, "Command ACK error: 0x%04X", ack);
        if (this->last_cmd_ok_ != nullptr)
          this->last_cmd_ok_->publish_state(false);
        this->cmd_flow_ = CmdFlow::IDLE;
        return;
      }
      if (this->last_cmd_ok_ != nullptr)
        this->last_cmd_ok_->publish_state(true);

      // parse query-params response before advancing the flow
      if (buf[6] == 0x71 && len >= 22)
      {
        uint32_t max_gate = buf[10] | (buf[11] << 8) | (buf[12] << 16) | (buf[13] << 24);
        uint32_t min_gate = buf[14] | (buf[15] << 8) | (buf[16] << 16) | (buf[17] << 24);
        uint32_t none_delay = buf[18] | (buf[19] << 8) | (buf[20] << 16) | (buf[21] << 24);
        ESP_LOGD(TAG, "Params: max_gate=%u min_gate=%u none_delay=%us", max_gate, min_gate, none_delay);
        if (this->max_gate_ != nullptr)
          this->max_gate_->publish_state(max_gate);
        if (this->min_gate_ != nullptr)
          this->min_gate_->publish_state(min_gate);
        if (this->none_duration_ != nullptr)
          this->none_duration_->publish_state(none_delay);
      }

      this->advance_flow_();
    }

    // ---------------------------------------------------------------------------
    // Async command state machine
    //
    // Each flow has numbered steps. On every successful ACK, advance_flow_() is
    // called and sends the next command in the sequence, or resets to IDLE when done.
    //
    //  QUERY_PARAMS:   step 0 enter_config → step 1 query(0x71) → step 2 exit_config → IDLE
    //  SET_DISTANCES:  step 0 enter_config → step 1 set(0x70) → step 2 query(0x71) → step 3 exit_config → IDLE
    //  SWITCH_OUTPUT:  step 0 enter_config → step 1 switch(0x7A) → step 2 exit_config → IDLE
    // ---------------------------------------------------------------------------
    void LD2410SComponent::advance_flow_()
    {
      this->cmd_step_++;

      switch (this->cmd_flow_)
      {

      case CmdFlow::QUERY_PARAMS:
        if (this->cmd_step_ == 1)
        {
          uint8_t val[6] = {0x05, 0x00, 0x0A, 0x00, 0x06, 0x00};
          this->send_command_(0x71, 0x00, val, 6);
        }
        else if (this->cmd_step_ == 2)
        {
          this->send_command_(0xFE, 0x00, nullptr, 0);
        }
        else
        {
          this->cmd_flow_ = CmdFlow::IDLE;
        }
        break;

      case CmdFlow::SET_DISTANCES:
        if (this->cmd_step_ == 1)
        {
          uint8_t val[18] = {
              0x05,
              0x00,
              (uint8_t)(this->pending_max_gate_ & 0xFF),
              (uint8_t)(this->pending_max_gate_ >> 8),
              0x00,
              0x00,
              0x0A,
              0x00,
              (uint8_t)(this->pending_min_gate_ & 0xFF),
              (uint8_t)(this->pending_min_gate_ >> 8),
              0x00,
              0x00,
              0x06,
              0x00,
              (uint8_t)(this->pending_none_s_ & 0xFF),
              (uint8_t)(this->pending_none_s_ >> 8),
              0x00,
              0x00,
          };
          this->send_command_(0x70, 0x00, val, 18);
        }
        else if (this->cmd_step_ == 2)
        {
          uint8_t val[6] = {0x05, 0x00, 0x0A, 0x00, 0x06, 0x00};
          this->send_command_(0x71, 0x00, val, 6);
        }
        else if (this->cmd_step_ == 3)
        {
          this->send_command_(0xFE, 0x00, nullptr, 0);
        }
        else
        {
          this->cmd_flow_ = CmdFlow::IDLE;
        }
        break;

      case CmdFlow::SWITCH_OUTPUT:
        if (this->cmd_step_ == 1)
        {
          uint8_t val[6] = {0x00, 0x00, (uint8_t)(this->pending_standard_ ? 0x01 : 0x00), 0x00, 0x00, 0x00};
          this->send_command_(0x7A, 0x00, val, 6);
        }
        else if (this->cmd_step_ == 2)
        {
          this->send_command_(0xFE, 0x00, nullptr, 0);
        }
        else
        {
          this->cmd_flow_ = CmdFlow::IDLE;
        }
        break;

      default:
        break;
      }
    }

    // ---------------------------------------------------------------------------
    // Public actions — each starts a flow by entering config mode (step 0)
    // ---------------------------------------------------------------------------
    void LD2410SComponent::set_config_mode(bool enable)
    {
      if (enable)
      {
        uint8_t val[2] = {0x01, 0x00};
        this->send_command_(0xFF, 0x00, val, 2);
      }
      else
      {
        this->send_command_(0xFE, 0x00, nullptr, 0);
      }
    }

    void LD2410SComponent::query_parameters()
    {
      if (this->cmd_flow_ != CmdFlow::IDLE)
      {
        ESP_LOGD(TAG, "query_parameters: command in progress, ignoring");
        return;
      }
      this->cmd_flow_ = CmdFlow::QUERY_PARAMS;
      this->cmd_step_ = 0;
      uint8_t val[2] = {0x01, 0x00};
      this->send_command_(0xFF, 0x00, val, 2); // enter config, flow advances on ACK
    }

    void LD2410SComponent::set_distances_and_none_duration(int max_gate, int min_gate, int none_s)
    {
      if (this->cmd_flow_ != CmdFlow::IDLE)
      {
        ESP_LOGD(TAG, "set_distances: command in progress, ignoring");
        return;
      }
      this->pending_max_gate_ = max_gate;
      this->pending_min_gate_ = min_gate;
      this->pending_none_s_ = none_s;
      this->cmd_flow_ = CmdFlow::SET_DISTANCES;
      this->cmd_step_ = 0;
      uint8_t val[2] = {0x01, 0x00};
      this->send_command_(0xFF, 0x00, val, 2);
    }

    void LD2410SComponent::switch_output_mode(bool standard)
    {
      if (this->cmd_flow_ != CmdFlow::IDLE)
      {
        ESP_LOGD(TAG, "switch_output_mode: command in progress, ignoring");
        return;
      }
      this->pending_standard_ = standard;
      this->cmd_flow_ = CmdFlow::SWITCH_OUTPUT;
      this->cmd_step_ = 0;
      uint8_t val[2] = {0x01, 0x00};
      this->send_command_(0xFF, 0x00, val, 2);
    }

  } // namespace ld2410s
} // namespace esphome
