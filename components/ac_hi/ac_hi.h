#pragma once

#include "esphome/components/climate/climate.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#ifdef USE_SENSOR
  #include "esphome/components/sensor/sensor.h"
#endif

#include <vector>
#include <cstdint>
#include <cstddef>

namespace esphome {
namespace ac_hi {

class ACHIClimate : public climate::Climate, public PollingComponent, public uart::UARTDevice {
 public:
  ACHIClimate() = default;

  void setup() override {}
  void loop() override;
  void update() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  climate::ClimateTraits traits() override;
  void control(const climate::ClimateCall &call) override;

#ifdef USE_SENSOR
  void set_pipe_sensor(sensor::Sensor *s) { this->pipe_sensor_ = s; }
#endif
  void set_enable_presets(bool v) { this->enable_presets_ = v; }

 protected:
  // Protocol constants
  static constexpr uint8_t HI_HDR0  = 0xF4;
  static constexpr uint8_t HI_HDR1  = 0xF5;
  static constexpr uint8_t HI_TAIL0 = 0xF4;
  static constexpr uint8_t HI_TAIL1 = 0xFB;

  static constexpr uint8_t CMD_WRITE  = 0x65;  // 101
  static constexpr uint8_t CMD_STATUS = 0x66;  // 102
  static constexpr uint8_t CMD_NAK    = 0xFD;  // наблюдается в логах как "unknown", трактуем как NAK

  // Field indices in long write/status frames
  static constexpr int IDX_FAN          = 16;
  static constexpr int IDX_SLEEP        = 17;
  static constexpr int IDX_MODE_POWER   = 18;
  static constexpr int IDX_TARGET_TEMP  = 19;
  static constexpr int IDX_AIR_TEMP     = 20;
  static constexpr int IDX_PIPE_TEMP    = 21;
  static constexpr int IDX_SWING        = 32;
  static constexpr int IDX_FLAGS        = 33;
  static constexpr int IDX_FLAGS2       = 35;
  static constexpr int IDX_LED          = 36;

  // Desired/Reported state
  bool power_on_{false};
  climate::ClimateMode mode_{climate::CLIMATE_MODE_COOL};
  uint8_t target_c_{24};
  climate::ClimateFanMode fan_{climate::CLIMATE_FAN_AUTO};
  climate::ClimateSwingMode swing_{climate::CLIMATE_SWING_OFF};
  bool turbo_{false};
  bool eco_{false};
  bool quiet_{false};
  bool led_{true};
  uint8_t sleep_stage_{0};
  bool enable_presets_{true};

#ifdef USE_SENSOR
  sensor::Sensor *pipe_sensor_{nullptr};
#endif

  // Long WRITE template, total 41 bytes; длина (байт[4]) варьируется (см. build_variant_)
  std::vector<uint8_t> tx_bytes_ = {
      0xF4,0xF5,0x00,0x40,0x20, // [0..4] header + length placeholder
      0x00,0x00,0x01,0x01,      // [5..8]
      0xFE,0x01,0x00,0x00,      // [9..12]
      CMD_WRITE,0x00,0x00,      // [13..15]
      0x23,                      // [16] fan
      0x45,                      // [17] sleep
      0x00,                      // [18] mode|power
      0x00,                      // [19] target temp
      0x00,                      // [20] air temp (readback)
      0x00,                      // [21] pipe temp (readback)
      0x00,0x00,0x00,0x00,0x00,0x00, // [22..27]
      0x00,0x00,0x00,0x00,0x00, // [28..32]
      0x00,                      // [33] flags (turbo/eco)
      0x00,                      // [34]
      0x00,                      // [35] quiet/swing report
      0x00,                      // [36] LED
      0x00,                      // [37] CRC_LO (или 0 при CRC8)
      0x00,                      // [38] CRC_HI (или CRC8 при 1-байтовой)
      0xF4,0xFB                  // [39..40] tail
  };

  // Short STATUS query (рабочий)
  const std::vector<uint8_t> query_ = {
      0xF4,0xF5,0x00,0x40,0x0C,0x00,0x00,0x01,0x01,
      0xFE,0x01,0x00,0x00, CMD_STATUS, 0x00,0x00,0x00,0x01,
      0xB3, 0xF4,0xFB
  };

  // RX buffering
  std::vector<uint8_t> rx_;
  size_t rx_start_{0};

  // Write scheduling / reliability
  bool writing_lock_{false};
  bool dirty_{false};
  uint32_t last_tx_ms_{0};
  uint32_t ack_deadline_ms_{0};
  uint32_t last_status_ms_{0};
  uint32_t force_poll_at_ms_{0};

  // multi-variant write attempts
  uint8_t write_attempt_{0};       // 0..N-1
  static constexpr uint8_t kWriteAttemptsMax = 8;

  static constexpr uint32_t kMinGapMs         = 120;
  static constexpr uint32_t kAckTimeoutMs     = 1000;
  static constexpr uint32_t kForcePollDelayMs = 150;

  // Helpers
  void send_query_status_();
  void send_now_();
  void send_write_frame_(const std::vector<uint8_t> &frame);

  // CRC helpers
  void calc_and_patch_crc1_(std::vector<uint8_t> &buf, bool len_is_total) const;         // 1-byte sum
  void calc_and_patch_crc16_sum_(std::vector<uint8_t> &buf, bool len_is_total) const;    // 16-bit sum
  void calc_and_patch_crc16_modbus_(std::vector<uint8_t> &buf, bool len_is_total) const; // CRC16/IBM (Modbus)
  void calc_and_patch_crc16_ccitt_(std::vector<uint8_t> &buf, bool len_is_total) const;  // CRC16/CCITT-FALSE

  // Build N-th variant (длина и CRC-схема)
  void build_variant_(uint8_t attempt, std::vector<uint8_t> &frame);

  bool extract_next_frame_(std::vector<uint8_t> &frame);
  void handle_frame_(const std::vector<uint8_t> &frame);
  void handle_ack_101_();
  void handle_nak_fd_();
  void parse_status_102_(const std::vector<uint8_t> &frame);

  // Encoding helpers
  static uint8_t clamp16_30_(int v) { return v < 16 ? 16 : (v > 30 ? 30 : (uint8_t) v); }

  // Legacy-encoding (как в YAML из обсуждения)
  static uint8_t encode_mode_hi_write_legacy_(climate::ClimateMode m) {
    uint8_t code = 2; // default cool
    switch (m) {
      case climate::CLIMATE_MODE_FAN_ONLY: code = 0; break;
      case climate::CLIMATE_MODE_HEAT:     code = 1; break;
      case climate::CLIMATE_MODE_COOL:     code = 2; break;
      case climate::CLIMATE_MODE_DRY:      code = 3; break;
      default:                             code = 2; break;
    }
    uint8_t v = static_cast<uint8_t>((code << 1) | 0x01);
    return static_cast<uint8_t>(v << 4);
  }

  static uint8_t encode_target_temp_write_legacy_(uint8_t c) {
    c = clamp16_30_(c);
    return static_cast<uint8_t>((c << 1) | 0x01);
  }

  static uint8_t encode_power_lo_write_(bool on) { return on ? 0x0C : 0x04; } // write lo-nibble
  static uint8_t encode_fan_byte_(climate::ClimateFanMode f);
  static uint8_t encode_sleep_byte_(uint8_t stage);
  static uint8_t encode_swing_ud_(bool on);
  static uint8_t encode_swing_lr_(bool on);
};

} // namespace ac_hi
} // namespace esphome
