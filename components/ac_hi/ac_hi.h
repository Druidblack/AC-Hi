#pragma once

#include "esphome/components/climate/climate.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"

#ifdef USE_SENSOR
  #include "esphome/components/sensor/sensor.h"
#endif

#include <vector>
#include <cstdint>

namespace esphome {
namespace ac_hi {

/**
 * Hisense RS-485 AC (HiSense/AEG/Hitachi OEM) climate component.
 * This header exposes only what we need for the fixes:
 * - temperature encode/decode
 * - power/mode packing into byte[18]
 */
class ACHIClimate : public climate::Climate, public PollingComponent, public uart::UARTDevice {
 public:
  ACHIClimate() = default;

  // Component
  void setup() override;
  void update() override;
  void loop() override;

  // Climate
  climate::ClimateTraits traits() override;
  void control(const climate::ClimateCall &call) override;

  // Config
  void set_enable_presets(bool v) { this->enable_presets_ = v; }
#ifdef USE_SENSOR
  void set_pipe_sensor(sensor::Sensor *s) { this->pipe_sensor_ = s; }
#endif

 protected:
  // ---- Protocol helpers (publicly visible to cpp) ----
  // Short status request (cmd 0x66). CRC is precalculated for the template below.
  const std::vector<uint8_t> query_ = {
      0xF4, 0xF5, 0x00, 0x40, 0x0C, 0x00, 0x00, 0x01, 0x01,
      0xFE, 0x01, 0x00, 0x00, 0x66, 0x00, 0x00, 0x00, 0x01,
      0xB3, 0xF4, 0xFB
  };

  // Encode power+mode into byte[18].
  uint8_t pack_power_mode_() const;
  // Extract mode from mode nibble (byte18 >> 4).
  static climate::ClimateMode decode_mode_from_nibble_(uint8_t nib);
  static uint8_t encode_mode_to_nibble_(climate::ClimateMode m);

  // === TEMPERATURE ENCODING FIX ===
  // Encode target Celsius to protocol byte: (C<<1) | 1 in 16..32 range.
  static inline uint8_t encode_temp_(uint8_t c) {
    if (c < 16) c = 16;
    if (c > 32) c = 32;
    return static_cast<uint8_t>((c << 1) | 1);
  }
  // Smart decode: if LSB=1 assume encoded, else use raw.
  static inline uint8_t decode_temp_(uint8_t raw) {
    uint8_t c = (raw & 0x01) ? static_cast<uint8_t>(raw >> 1) : raw;
    if (c < 16 || c > 32) return 24; // fallback sane default
    return c;
  }

  // Frame IO
  void send_query_();
  void send_state_frame_();
  void handle_incoming_();
  void parse_frame_(const std::vector<uint8_t> &frame);
  void parse_status_102_(const std::vector<uint8_t> &frame);
  void handle_ack_101_();

  // CRC for long frames (big-endian 16-bit sum of payload bytes)
  static uint16_t crc16_sum_(const std::vector<uint8_t> &buf, size_t start, size_t end);

  // ---- Reflected state ----
  bool power_on_{false};
  uint8_t target_c_{24}; // 16..32 (human)
  climate::ClimateMode mode_{climate::CLIMATE_MODE_OFF};
  climate::ClimateFanMode fan_{climate::CLIMATE_FAN_AUTO};
  climate::ClimateSwingMode swing_{climate::CLIMATE_SWING_OFF};
  bool eco_{false};
  bool turbo_{false};
  uint8_t sleep_stage_{0};

  // Work buffers
  std::vector<uint8_t> tx_bytes_;
  std::vector<uint8_t> rx_bytes_;

  // Timers/flags
  bool writing_lock_{false};
  bool pending_write_{false};
  uint32_t last_query_ms_{0};

#ifdef USE_SENSOR
  sensor::Sensor *pipe_sensor_{nullptr};
#else
  void *pipe_sensor_{nullptr};
#endif

  bool enable_presets_{true};
};

}  // namespace ac_hi
}  // namespace esphome
