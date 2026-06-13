#include "ld2410s.h"
#include "esphome/core/log.h"

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
      this->set_timeout(500, [this]()
                        { this->query_parameters(); });

      // TEST: switch to standard output mode after a timeout
      // this->set_timeout(100, [this]()
      //                   { this->switch_output_mode(false); });
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

    // ---------------------------------------------------------------------------
    // Ring-buffer helpers
    // ---------------------------------------------------------------------------
    uint8_t LD2410SComponent::rx_peek_(size_t offset) const
    {
      return this->rx_buf_[(this->rx_head_ + offset) & RX_BUF_MASK];
    }

    void LD2410SComponent::rx_drop_(size_t n)
    {
      this->rx_head_ = (this->rx_head_ + n) & RX_BUF_MASK;
      this->rx_count_ -= n;
    }

    void LD2410SComponent::rx_extract_(uint8_t *out, size_t len)
    {
      for (size_t i = 0; i < len; i++)
        out[i] = this->rx_buf_[(this->rx_head_ + i) & RX_BUF_MASK];
      this->rx_drop_(len);
    }

    void LD2410SComponent::readline_(int readch)
    {
      if (readch < 0)
        return;

      // buffer full without a complete frame: drop the oldest byte so a frame
      // hidden inside the remaining data can still be found
      if (this->rx_count_ >= RX_BUF_SIZE)
        this->rx_drop_(1);

      this->rx_buf_[this->rx_tail_] = static_cast<uint8_t>(readch);
      this->rx_tail_ = (this->rx_tail_ + 1) & RX_BUF_MASK;
      this->rx_count_++;

      while (this->rx_count_ > 0)
      {
        const uint8_t b0 = this->rx_peek_(0);

        // resync: only minimal, ACK and standard frame headers can start a frame
        if (b0 != FRAME_HDR_MINIMAL && b0 != FRAME_HDR_ACK0 && b0 != FRAME_HDR_STD0)
        {
          this->rx_drop_(1);
          continue;
        }

        const size_t pos = this->rx_count_;

        if (b0 == FRAME_HDR_MINIMAL)
        {
          if (pos < MINIMAL_FRAME_LEN)
            return; // wait for more data
          if (this->rx_peek_(4) == FRAME_FTR_MINIMAL)
          {
            uint8_t frame[MINIMAL_FRAME_LEN];
            this->rx_extract_(frame, MINIMAL_FRAME_LEN);
            ESP_LOGD(TAG, "Minimal frame: %s", format_hex_pretty(frame, MINIMAL_FRAME_LEN).c_str());
            this->handle_minimal_frame_(frame, MINIMAL_FRAME_LEN);
            return;
          }
          this->rx_drop_(1); // false header, resync on remaining bytes
          continue;
        }

        if (b0 == FRAME_HDR_ACK0)
        {
          if ((pos >= 2 && this->rx_peek_(1) != FRAME_HDR_ACK1) ||
              (pos >= 3 && this->rx_peek_(2) != FRAME_HDR_ACK2) ||
              (pos >= 4 && this->rx_peek_(3) != FRAME_HDR_ACK3))
          {
            this->rx_drop_(1);
            continue;
          }
          if (pos >= ACK_FRAME_MIN_LEN &&
              this->rx_peek_(pos - 4) == ACK_FTR0 && this->rx_peek_(pos - 3) == ACK_FTR1 &&
              this->rx_peek_(pos - 2) == ACK_FTR2 && this->rx_peek_(pos - 1) == ACK_FTR3)
          {
            uint8_t frame[ACK_FRAME_MAX_LEN];
            size_t len = (pos < ACK_FRAME_MAX_LEN) ? pos : ACK_FRAME_MAX_LEN;
            this->rx_extract_(frame, len);
            ESP_LOGD(TAG, "ACK frame: %s", format_hex_pretty(frame, len).c_str());
            this->handle_ack_frame_(frame, len);
            return;
          }
          if (pos >= ACK_FRAME_MAX_LEN)
          {
            this->rx_drop_(1); // give up resync on this header
            continue;
          }
          return; // wait for more data
        }

        // b0 == FRAME_HDR_STD0 — standard frame
        if ((pos >= 2 && this->rx_peek_(1) != FRAME_HDR_STD1) ||
            (pos >= 3 && this->rx_peek_(2) != FRAME_HDR_STD2) ||
            (pos >= 4 && this->rx_peek_(3) != FRAME_HDR_STD3))
        {
          this->rx_drop_(1);
          continue;
        }
        if (pos >= STANDARD_FRAME_MIN_LEN &&
            this->rx_peek_(pos - 4) == STD_FTR0 && this->rx_peek_(pos - 3) == STD_FTR1 &&
            this->rx_peek_(pos - 2) == STD_FTR2 && this->rx_peek_(pos - 1) == STD_FTR3)
        {
          uint8_t frame[STANDARD_FRAME_MAX_LEN];
          size_t len = (pos < STANDARD_FRAME_MAX_LEN) ? pos : STANDARD_FRAME_MAX_LEN;
          this->rx_extract_(frame, len);
          ESP_LOGD(TAG, "Standard frame: %s", format_hex_pretty(frame, len).c_str());
          this->handle_standard_frame_(frame, len);
          return;
        }
        if (pos >= STANDARD_FRAME_MAX_LEN)
        {
          this->rx_drop_(1); // give up resync on this header
          continue;
        }
        return; // wait for more data
      }
    }

    // ---------------------------------------------------------------------------
    // Frame handlers
    // ---------------------------------------------------------------------------
    void LD2410SComponent::handle_minimal_frame_(const uint8_t *buf, int len)
    {
      if (len != (int)MINIMAL_FRAME_LEN || buf[0] != FRAME_HDR_MINIMAL || buf[4] != FRAME_FTR_MINIMAL)
        return;

      bool detected = (buf[MIN_OFF_STATE] == TARGET_STATE_MOVING || buf[MIN_OFF_STATE] == TARGET_STATE_STATIC);
      this->publish_presence_(detected);

      int dist = two_byte_to_int_(buf[MIN_OFF_DIST_LO], buf[MIN_OFF_DIST_HI]);
      if (this->distance_ != nullptr && (int)this->distance_->get_state() != dist)
        this->distance_->publish_state(dist);
    }

    void LD2410SComponent::handle_standard_frame_(const uint8_t *buf, int len)
    {
      if (len < (int)STANDARD_FRAME_MIN_LEN)
        return;
      if (buf[0] != FRAME_HDR_STD0 || buf[1] != FRAME_HDR_STD1 || buf[2] != FRAME_HDR_STD2 || buf[3] != FRAME_HDR_STD3)
        return;
      if (buf[STD_OFF_FRAME_TYPE] != STD_FRAME_TYPE_TARGET)
        return;
      if (buf[len - 4] != STD_FTR0 || buf[len - 3] != STD_FTR1 || buf[len - 2] != STD_FTR2 || buf[len - 1] != STD_FTR3)
        return;

      bool detected = (buf[STD_OFF_STATE] == TARGET_STATE_MOVING || buf[STD_OFF_STATE] == TARGET_STATE_STATIC);
      this->publish_presence_(detected);

      // keepalive "ping" frames carry an all-zero payload (distance + every gate
      // energy == 0); skip them so they don't show up as spikes on the chart
      uint8_t payload_or = 0;
      for (int i = STD_OFF_DIST_LO; i < len - 4; i++)
        payload_or |= buf[i];
      if (payload_or == 0)
        return;

      int dist = two_byte_to_int_(buf[STD_OFF_DIST_LO], buf[STD_OFF_DIST_HI]);
      if (this->distance_ != nullptr && (int)this->distance_->get_state() != dist)
        this->distance_->publish_state(dist);

      // gate energy: 4 bytes per gate (lo, hi, lo, hi) starting at STD_OFF_GATE_ENERGY_START
      // standard frame layout: 16 gates × STD_GATE_ENERGY_STRIDE bytes
      // each gate value = buf[off] | (buf[off + 1] << 8)  (16-bit, hi bytes unused)
      for (uint8_t g = 0; g < NUM_GATES; g++)
      {
        if (this->gate_energy_[g] == nullptr)
          continue;
        int off = STD_OFF_GATE_ENERGY_START + g * STD_GATE_ENERGY_STRIDE;
        uint16_t energy = buf[off] | (buf[off + 1] << 8);
        if ((int)this->gate_energy_[g]->get_state() != energy)
          this->gate_energy_[g]->publish_state(energy);
      }
    }

    void LD2410SComponent::handle_ack_frame_(const uint8_t *buf, int len)
    {
      if (len < (int)ACK_FRAME_MIN_LEN)
        return;
      if (buf[0] != FRAME_HDR_ACK0 || buf[1] != FRAME_HDR_ACK1 || buf[2] != FRAME_HDR_ACK2 || buf[3] != FRAME_HDR_ACK3)
        return;
      if (buf[ACK_OFF_FRAME_TYPE] != ACK_FRAME_TYPE_REPLY)
        return;

      uint16_t ack = static_cast<uint16_t>((buf[ACK_OFF_STATUS_HI] << 8) | buf[ACK_OFF_STATUS_LO]);
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
      if (buf[ACK_OFF_CMD] == CMD_QUERY_PARAMS && len >= (int)ACK_QUERY_PARAMS_LEN)
      {
        uint32_t max_gate = buf[ACK_OFF_MAX_GATE] | (buf[ACK_OFF_MAX_GATE + 1] << 8) |
                            (buf[ACK_OFF_MAX_GATE + 2] << 16) | (buf[ACK_OFF_MAX_GATE + 3] << 24);
        uint32_t min_gate = buf[ACK_OFF_MIN_GATE] | (buf[ACK_OFF_MIN_GATE + 1] << 8) |
                            (buf[ACK_OFF_MIN_GATE + 2] << 16) | (buf[ACK_OFF_MIN_GATE + 3] << 24);
        uint32_t none_delay = buf[ACK_OFF_NONE_DELAY] | (buf[ACK_OFF_NONE_DELAY + 1] << 8) |
                              (buf[ACK_OFF_NONE_DELAY + 2] << 16) | (buf[ACK_OFF_NONE_DELAY + 3] << 24);
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
