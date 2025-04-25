#pragma once

#include "esphome/components/climate_ir/climate_ir.h"
#include "esphome/components/remote_base/remote_base.h"
#include <cinttypes>
#include <cstring>
#include <set>

namespace esphome {
namespace mhi_multi_ir {

// ======== Константы и тайминги IR ========

static const uint16_t HDR_MARK    = 3200;
static const uint16_t HDR_SPACE   = 1600;
static const uint16_t BIT_MARK    = 400;
static const uint16_t ONE_SPACE   = 1200;
static const uint16_t ZERO_SPACE  = 400;
static const uint16_t GAP_MIN     = 17500;
static const uint32_t CARRIER_HZ  = 38000;

// ======== Сигнатуры ========

static const uint8_t SIG_152_LEN = 5;
static const uint8_t SIG_152[5]  = { 0xAD, 0x51, 0x3C, 0xE5, 0x1A };  // ZM/ZMP
static const uint8_t SIG_88[5]   = { 0xAD, 0x51, 0x3C, 0xD9, 0x26 };  // ZJ/ZEA

// ======== Общие диапазоны ========

static const uint8_t MIN_TEMP = 17;
static const uint8_t MAX_TEMP = 31;

// ======== Протокол 152-bit (ZM/ZMP) ========

static const uint16_t LEN_152 = 19;

union Protocol152 {
  uint8_t raw[LEN_152];
  struct {
    uint8_t Sig[5];
    uint8_t Mode   :3;
    uint8_t Power  :1;
    uint8_t        :1;
    uint8_t Clean  :1;
    uint8_t Filter :1;
    uint8_t        :1;
    uint8_t        :8;
    uint8_t Temp   :4;
    uint8_t        :4;
    uint8_t        :8;
    uint8_t Fan    :4;
    uint8_t        :4;
    uint8_t        :8;
    uint8_t        :1;
    uint8_t Three  :1;
    uint8_t        :2;
    uint8_t D      :1;
    uint8_t SwingV :3;
    uint8_t        :8;
    uint8_t SwingH :4;
    uint8_t        :4;
    uint8_t        :8;
    uint8_t        :6;
    uint8_t Night  :1;
    uint8_t Silent :1;
  };
};

// ======== Протокол 88-bit (ZJ/ZEA) ========

static const uint16_t LEN_88 = 11;

union Protocol88 {
  uint8_t raw[LEN_88];
  struct {
    uint8_t Sig[5];
    uint8_t        :1;
    uint8_t SwingV5 :1;
    uint8_t SwingH1 :2;
    uint8_t        :1;
    uint8_t Clean   :1;
    uint8_t SwingH2 :2;
    uint8_t        :8;
    uint8_t        :3;
    uint8_t SwingV7 :2;
    uint8_t Fan     :3;
    uint8_t        :8;
    uint8_t Mode   :3;
    uint8_t Power  :1;
    uint8_t Temp   :4;
  };
};

// ======== Константы протоколов ========

// 152-bit modes
static const uint8_t P152_AUTO = 0;
static const uint8_t P152_COOL = 1;
static const uint8_t P152_DRY  = 2;
static const uint8_t P152_FAN  = 3;
static const uint8_t P152_HEAT = 4;

// 88-bit modes
static const uint8_t P88_AUTO = 0;
static const uint8_t P88_COOL = 1;
static const uint8_t P88_DRY  = 2;
static const uint8_t P88_FAN  = 3;
static const uint8_t P88_HEAT = 4;

// 152-bit fan speeds
static const uint8_t F152_AUTO  = 0;
static const uint8_t F152_LOW   = 1;
static const uint8_t F152_MED   = 2;
static const uint8_t F152_HIGH  = 3;
static const uint8_t F152_MAX   = 4;
static const uint8_t F152_ECONO = 6;
static const uint8_t F152_TURBO = 8;

// 88-bit fan speeds
static const uint8_t F88_AUTO  = 0;
static const uint8_t F88_LOW   = 2;
static const uint8_t F88_MED   = 3;
static const uint8_t F88_HIGH  = 4;
static const uint8_t F88_TURBO = 6;
static const uint8_t F88_ECONO = 7;

// Swing values — пример, для 152
static const uint8_t SV152_AUTO    = 0;
static const uint8_t SV152_HIGHEST = 1;
static const uint8_t SV152_HIGH    = 2;
static const uint8_t SV152_MID     = 3;
static const uint8_t SV152_LOW     = 4;
static const uint8_t SV152_OFF     = 6;

// Swing H 152
static const uint8_t SH152_AUTO      = 0;
static const uint8_t SH152_LEFTMAX   = 1;
static const uint8_t SH152_LEFT      = 2;
static const uint8_t SH152_MID       = 3;
static const uint8_t SH152_RIGHT     = 4;
static const uint8_t SH152_RIGHTMAX  = 5;
static const uint8_t SH152_OFF       = 8;

// Swing values — 88
static const uint8_t SV88_AUTO  = 0;
static const uint8_t SV88_HIGHEST = 6;  // 0b110
static const uint8_t SV88_MID     = 3;  // 0b011
static const uint8_t SV88_OFF     = 0;

// Swing H 88
static const uint8_t SH88_AUTO  = 8;    // 0b1000
static const uint8_t SH88_3D    = 14;   // 0b1110
static const uint8_t SH88_OFF   = 0;

// ======== Помощники ========

inline void invert_byte_pairs(uint8_t *data, size_t len) {
  for (size_t i = 0; i + 1 < len; i += 2)
    data[i + 1] = ~data[i];
}

// Преобразования в common-формат ESPHome
inline climate::ClimateMode convert_mode152(uint8_t m) {
  switch (m) {
    case P152_COOL: return climate::CLIMATE_MODE_COOL;
    case P152_HEAT: return climate::CLIMATE_MODE_HEAT;
    case P152_DRY:  return climate::CLIMATE_MODE_DRY;
    case P152_FAN:  return climate::CLIMATE_MODE_FAN_ONLY;
    default:        return climate::CLIMATE_MODE_HEAT_COOL;
  }
}
inline climate::ClimateMode convert_mode88(uint8_t m) {
  switch (m) {
    case P88_COOL: return climate::CLIMATE_MODE_COOL;
    case P88_HEAT: return climate::CLIMATE_MODE_HEAT;
    case P88_DRY:  return climate::CLIMATE_MODE_DRY;
    case P88_FAN:  return climate::CLIMATE_MODE_FAN_ONLY;
    default:       return climate::CLIMATE_MODE_HEAT_COOL;
  }
}

inline climate::ClimateFanMode convert_fan152(uint8_t f) {
  switch (f) {
    case F152_LOW:   return climate::CLIMATE_FAN_LOW;
    case F152_MED:   return climate::CLIMATE_FAN_MEDIUM;
    case F152_HIGH:  return climate::CLIMATE_FAN_HIGH;
    case F152_ECONO: return climate::CLIMATE_FAN_DIFFUSE;
    case F152_TURBO: return climate::CLIMATE_FAN_FOCUS;
    default:         return climate::CLIMATE_FAN_AUTO;
  }
}
inline climate::ClimateFanMode convert_fan88(uint8_t f) {
  switch (f) {
    case F88_LOW:   return climate::CLIMATE_FAN_LOW;
    case F88_MED:   return climate::CLIMATE_FAN_MEDIUM;
    case F88_HIGH:  return climate::CLIMATE_FAN_HIGH;
    case F88_ECONO: return climate::CLIMATE_FAN_DIFFUSE;
    case F88_TURBO: return climate::CLIMATE_FAN_FOCUS;
    default:        return climate::CLIMATE_FAN_AUTO;
  }
}

inline climate::ClimateSwingMode convert_swing(uint8_t sv, uint8_t sh) {
  const bool vs = (sv != SV152_OFF && sv != SV88_OFF);
  const bool hs = (sh != SH152_OFF && sh != SH88_OFF);
  if (vs && hs) return climate::CLIMATE_SWING_BOTH;
  if (vs)       return climate::CLIMATE_SWING_VERTICAL;
  if (hs)       return climate::CLIMATE_SWING_HORIZONTAL;
  return climate::CLIMATE_SWING_OFF;
}

// ======== ESPHome-класс ========

enum Model : uint8_t { ZJ = 0, ZEA = 1, ZM = 2, ZMP = 3 };

enum SetFanLevels : uint8_t { FAN_LEVELS_3 = 3, FAN_LEVELS_4 = 4 };

class MhiClimate : public climate_ir::ClimateIR {
 public:
  MhiClimate()
      : climate_ir::ClimateIR(
          MIN_TEMP, MAX_TEMP, 1.0f, true, true,
          {climate::CLIMATE_FAN_AUTO, climate::CLIMATE_FAN_LOW, climate::CLIMATE_FAN_MIDDLE,
           climate::CLIMATE_FAN_MEDIUM, climate::CLIMATE_FAN_HIGH, climate::CLIMATE_FAN_FOCUS,
           climate::CLIMATE_FAN_DIFFUSE},
          {climate::CLIMATE_SWING_OFF, climate::CLIMATE_SWING_VERTICAL,
           climate::CLIMATE_SWING_HORIZONTAL, climate::CLIMATE_SWING_BOTH},
          {climate::CLIMATE_PRESET_NONE, climate::CLIMATE_PRESET_ECO,
           climate::CLIMATE_PRESET_BOOST, climate::CLIMATE_PRESET_ACTIVITY}) 
      , model_(ZJ), fan_levels_(FAN_LEVELS_3) {}

  void set_model(Model m)           { model_ = m; }
  void set_fan_levels(SetFanLevels l){ fan_levels_ = l; }

 protected:
  climate::ClimateTraits traits() override {
    auto t = climate_ir::ClimateIR::traits();
    std::set<climate::ClimateFanMode> modes = {
      climate::CLIMATE_FAN_AUTO,
      climate::CLIMATE_FAN_LOW
    };
    if (fan_levels_ == FAN_LEVELS_4) modes.insert(climate::CLIMATE_FAN_MIDDLE);
    modes.insert(climate::CLIMATE_FAN_MEDIUM);
    modes.insert(climate::CLIMATE_FAN_HIGH);
    modes.insert(climate::CLIMATE_FAN_FOCUS);
    modes.insert(climate::CLIMATE_FAN_DIFFUSE);
    t.set_supported_fan_modes(std::move(modes));
    return t;
  }

  bool on_receive(remote_base::RemoteReceiveData data) override;
  void transmit_state() override;

 private:
  Model model_;
  SetFanLevels fan_levels_;
};

}  // namespace mhi_multi_ir
}  // namespace esphome
