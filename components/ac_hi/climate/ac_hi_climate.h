// SPDX-License-Identifier: MIT
#pragma once

#include "esphome/core/component.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include <vector>
#include <string>
#include <cmath>
#include <cstring>  // memcpy

namespace esphome {
namespace ac_hi {

/**
 * ACHiClimate — ESPHome climate для кондиционеров Ballu/Hisense по RS-485.
 * Кадры: F4 F5 ... F4 FB
 * Команды: запись 0x65, статус 0x66, ACK 0x101
 * Поля: [16]=fan code, [18]=power/mode, [19]=Tset, [20]=Tcur, свинги/флаги вокруг [32..37].
 */
class ACHiClimate : public climate::Climate, public Component, public uart::UARTDevice {
 public:
  void set_update_interval(uint32_t ms) { update_interval_ms_ = ms; }

  // опциональные сенсоры
  void set_tset_sensor(sensor::Sensor *s)     { tset_s_ = s; }
  void set_tcur_sensor(sensor::Sensor *s)     { tcur_s_ = s; }
  void set_tout_sensor(sensor::Sensor *s)     { tout_s_ = s; }
  void set_tpipe_sensor(sensor::Sensor *s)    { tpipe_s_ = s; }
  void set_compfreq_sensor(sensor::Sensor *s) { compfreq_s_ = s; }

  void setup() override;
  void loop() override;
  void dump_config() override;
  climate::ClimateTraits traits() override;

 protected:
  void control(const climate::ClimateCall &call) override;

  // ===== I/O helpers =====
  void build_base_long_frame_();
  void apply_intent_to_frame_();
  void compute_crc_(std::vector<uint8_t> &buf);
  void send_status_request_short_();       // короткий статус (0x66)
  void send_status_request_long_clean_();  // длинный «чистый» 0x66
  void send_write_frame_();                // запись 0x65 + пост-опрос
  void learn_header_(const std::vector<uint8_t> &bytes);
  void handle_status_(const std::vector<uint8_t> &bytes);
  void log_hex_dump_(const char *prefix, const std::vector<uint8_t> &data);

  // ===== runtime =====
  uint32_t update_interval_ms_{2000};
  uint32_t last_poll_{0};
  uint32_t last_rx_ms_{0};
  uint32_t last_long_status_ms_{0};

  // анти-откат (простое окно)
  uint32_t suppress_until_ms_{0};

  // строгая защита до совпадения ожидаемого режима
  bool     guard_mode_until_match_{false};
  uint8_t  expected_mode_byte_{0};
  uint32_t guard_deadline_ms_{0};

  // входной буфер
  std::vector<uint8_t> rx_buf_;

  // «обученная» шапка [2..12]
  uint8_t header_[11]{0x00};
  bool header_learned_{false};

  // исходящий длинный кадр (~50 байт)
  std::vector<uint8_t> out_;

  // текущее намерение/состояние
  bool   power_{false};
  float  target_temp_{24};
  uint8_t temp_byte_{(24U << 1) | 1};
  uint8_t wind_code_{1}; // status: auto=1, low~12, med~14, high~16
  bool swing_ud_{false};
  bool swing_lr_{false};
  uint8_t power_bin_{0}; // 0b00001100=ON, 0b00000100=OFF
  uint8_t mode_bin_{0};  // старшая тетрада: 0=FAN,1=HEAT,2=COOL,3=DRY,4=AUTO
  uint8_t turbo_bin_{0};
  uint8_t eco_bin_{0};
  uint8_t quiet_bin_{0};

  // сенсоры (опционально)
  sensor::Sensor *tset_s_{nullptr};
  sensor::Sensor *tcur_s_{nullptr};
  sensor::Sensor *tout_s_{nullptr};
  sensor::Sensor *tpipe_s_{nullptr};
  sensor::Sensor *compfreq_s_{nullptr};
};

}  // namespace ac_hi
}  // namespace esphome
