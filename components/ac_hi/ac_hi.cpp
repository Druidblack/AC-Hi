#include "ac_hi.h"
#include "esphome/core/log.h"
#include <algorithm>
#include <cmath>

namespace esphome {
namespace ac_hi {

static const char *const TAG = "ac_hi";

// ----- Helpers for mode nibble (byte[18] >> 4) -----
climate::ClimateMode ACHIClimate::decode_mode_from_nibble_(uint8_t nib) {
  switch (nib & 0x0F) {
    case 0x00: return climate::CLIMATE_MODE_FAN_ONLY;
    case 0x01: return climate::CLIMATE_MODE_HEAT;
    case 0x02: return climate::CLIMATE_MODE_COOL;
    case 0x03: return climate::CLIMATE_MODE_DRY;
    default:   return climate::CLIMATE_MODE_FAN_ONLY;
  }
}
uint8_t ACHIClimate::encode_mode_to_nibble_(climate::ClimateMode m) {
  switch (m) {
    case climate::CLIMATE_MODE_FAN_ONLY: return 0x00;
    case climate::CLIMATE_MODE_HEAT: return 0x01;
    case climate::CLIMATE_MODE_COOL: return 0x02;
    case climate::CLIMATE_MODE_DRY: return 0x03;
    default: return 0x02; // default to COOL
  }
}

uint8_t ACHIClimate::pack_power_mode_() const {
  uint8_t mode_nib = encode_mode_to_nibble_(this->mode_) & 0x0F;
  uint8_t power_low = this->power_on_ ? 0x0C : 0x04; // matches legacy: ON=0x0C, OFF=0x04
  return static_cast<uint8_t>((mode_nib << 4) | power_low);
}

void ACHIClimate::setup() {
  // Prepare a default TX state frame template (length 0x29 as in legacy)
  // We only set fields we care about; the rest are zeros.
  this->tx_bytes_.assign({
    0xF4, 0xF5, 0x00, 0x40, 0x29, 0x00, 0x00, 0x01, 0x01,
    0xFE, 0x01, 0x00, 0x00, 0x6A, // 0x6A = write state command in many firmwares
    // payload starts at index 15 .. (len-4)
  });
  // pad payload to required size (0x29 total incl. header+crc)
  while (this->tx_bytes_.size() < 0x29 - 2) this->tx_bytes_.push_back(0x00);
  // Append dummy CRC and footer (will be fixed on send)
  this->tx_bytes_.push_back(0x00);
  this->tx_bytes_.push_back(0x00);
  this->tx_bytes_.push_back(0xF4);
  this->tx_bytes_.push_back(0xFB);

  this->update();  // kick an initial query
}

void ACHIClimate::update() {
  // poll status
  this->send_query_();
}

void ACHIClimate::loop() {
  this->handle_incoming_();

  // If we queued a write and link is free, send
  if (this->pending_write_ && !this->writing_lock_) {
    this->pending_write_ = false;
    this->send_state_frame_();
  }
}

climate::ClimateTraits ACHIClimate::traits() {
  climate::ClimateTraits t;
  t.set_supported_modes({
      climate::CLIMATE_MODE_OFF,
      climate::CLIMATE_MODE_COOL,
      climate::CLIMATE_MODE_HEAT,
      climate::CLIMATE_MODE_DRY,
      climate::CLIMATE_MODE_FAN_ONLY,
  });
  t.set_supported_fan_modes({
      climate::CLIMATE_FAN_AUTO,
      climate::CLIMATE_FAN_LOW,
      climate::CLIMATE_FAN_MEDIUM,
      climate::CLIMATE_FAN_HIGH,
  });
  t.set_supported_custom_presets({});
  t.set_supported_swing_modes({
      climate::CLIMATE_SWING_OFF,
      climate::CLIMATE_SWING_VERTICAL,
      climate::CLIMATE_SWING_HORIZONTAL,
      climate::CLIMATE_SWING_BOTH,
  });
  t.set_visual_min_temperature(16);
  t.set_visual_max_temperature(32);
  t.set_visual_temperature_step(1.0f);
  return t;
}

// === POWER BEHAVIOR ===
// Card has no separate power; any mode != OFF should power on.
// OFF turns power off. Changing ONLY temperature must not change power.
void ACHIClimate::control(const climate::ClimateCall &call) {
  bool need_write = false;

  if (call.get_mode().has_value()) {
    auto m = *call.get_mode();
    if (m == climate::CLIMATE_MODE_OFF) {
      this->power_on_ = false;
      this->mode_ = climate::CLIMATE_MODE_COOL; // keep a sane default for next ON
    } else {
      this->mode_ = m;
      this->power_on_ = true;  // << required behavior
    }
    need_write = true;
  }

  if (call.get_target_temperature().has_value()) {
    auto t = *call.get_target_temperature();
    if (!std::isnan(t)) {
      uint8_t c = static_cast<uint8_t>(std::round(t));
      c = std::max<uint8_t>(16, std::min<uint8_t>(32, c));
      this->target_c_ = c;
      need_write = true;
    }
  }

  if (call.get_fan_mode().has_value()) {
    this->fan_ = *call.get_fan_mode();
    need_write = true;
  }

  if (call.get_swing_mode().has_value()) {
    this->swing_ = *call.get_swing_mode();
    need_write = true;
  }

  if (need_write) {
    this->pending_write_ = true;
    this->writing_lock_ = false;  // allow immediate send
    this->publish_state();
  }
}

void ACHIClimate::send_query_() {
  // Simple write of the short status request
  this->write_array(this->query_.data(), this->query_.size());
  this->last_query_ms_ = millis();
}

uint16_t ACHIClimate::crc16_sum_(const std::vector<uint8_t> &buf, size_t start, size_t end) {
  uint32_t sum = 0;
  for (size_t i = start; i < end; i++) sum += buf[i];
  return static_cast<uint16_t>(sum & 0xFFFF);
}

void ACHIClimate::send_state_frame_() {
  // Update dynamic fields before sending:

  // Byte 18: power + mode
  if (this->tx_bytes_.size() > 18) this->tx_bytes_[18] = this->pack_power_mode_();

  // Byte 19: target temperature with REQUIRED ENCODING (C<<1 | 1)
  if (this->tx_bytes_.size() > 19) this->tx_bytes_[19] = encode_temp_(this->target_c_);

  // Fan speed (byte 16) and swings (byte 32) are left as-is to avoid changing unrelated logic.

  // Recompute CRC over payload between index 2 and len-4 (exclusive of CRC+tail)
  if (this->tx_bytes_.size() >= 8) {
    size_t end = this->tx_bytes_.size() - 4;
    uint16_t crc = crc16_sum_(this->tx_bytes_, 2, end);
    this->tx_bytes_[end + 0] = static_cast<uint8_t>((crc >> 8) & 0xFF);
    this->tx_bytes_[end + 1] = static_cast<uint8_t>(crc & 0xFF);
  }

  this->write_array(this->tx_bytes_.data(), this->tx_bytes_.size());
  this->writing_lock_ = true;
}

void ACHIClimate::handle_incoming_() {
  while (this->available()) {
    uint8_t b;
    if (!this->read_byte(&b)) break;
    this->rx_bytes_.push_back(b);

    // Very naive frame reassembly: look for trailer 0xF4 0xFB and a header 0xF4 0xF5
    if (this->rx_bytes_.size() >= 2 &&
        this->rx_bytes_[this->rx_bytes_.size() - 2] == 0xF4 &&
        this->rx_bytes_[this->rx_bytes_.size() - 1] == 0xFB) {
      // find last header
      size_t start = 0;
      for (size_t i = 0; i + 1 < this->rx_bytes_.size(); i++) {
        if (this->rx_bytes_[i] == 0xF4 && this->rx_bytes_[i+1] == 0xF5) { start = i; break; }
      }
      std::vector<uint8_t> frame(this->rx_bytes_.begin() + start, this->rx_bytes_.end());
      this->rx_bytes_.clear();
      this->parse_frame_(frame);
    }
  }
}

void ACHIClimate::parse_frame_(const std::vector<uint8_t> &frame) {
  if (frame.size() < 10) return;
  if (!(frame[0] == 0xF4 && frame[1] == 0xF5)) return;

  uint8_t cmd = 0;
  if (frame.size() > 14) cmd = frame[14];

  switch (cmd) {
    case 0x66: // short status
    case 0x6B: // long status variants sometimes report as 0x6B with code 0x102
      this->parse_status_102_(frame);
      break;
    case 0x65: // ACK to write (observed as 0x65/0x101)
    case 0x6A:
    case 0x101:
    case 0x00: // some firmwares do weird things; unlock writes on any ack-ish
      this->handle_ack_101_();
      break;
    default:
      // ignore
      break;
  }
}

void ACHIClimate::parse_status_102_(const std::vector<uint8_t> &frame) {
  // Defensive: verify length
  if (frame.size() < 40) return;

  // Byte 18: power + mode
  uint8_t b18 = frame[18];
  uint8_t mode_nib = (b18 >> 4) & 0x0F;
  uint8_t power_low = b18 & 0x0F;
  this->power_on_ = (power_low == 0x0C); // 0x0C => ON, 0x04 => OFF
  this->mode_ = decode_mode_from_nibble_(mode_nib);

  // Byte 19: target temperature (SMART DECODE)
  uint8_t raw_t = frame[19];
  uint8_t dec = decode_temp_(raw_t);
  this->target_c_ = dec;
  this->target_temperature = dec;

  // Publish minimal state; keep other aspects unchanged
  if (!this->power_on_) {
    // HA expects OFF mode when powered down
    this->mode = climate::CLIMATE_MODE_OFF;
  } else {
    this->mode = this->mode_;
  }
  this->current_temperature = NAN; // not provided here
  this->publish_state();
}

void ACHIClimate::handle_ack_101_() {
  this->writing_lock_ = false;
  // do not auto-query here; update() will handle periodic polling
}

}  // namespace ac_hi
}  // namespace esphome
