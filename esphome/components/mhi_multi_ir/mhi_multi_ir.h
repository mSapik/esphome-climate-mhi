#pragma once

#include "esphome/components/climate_ir/climate_ir.h"
#include "esphome/components/remote_base/remote_base.h"
#include <cinttypes>
#include <cstring>
#include <set>
#include <vector>

namespace esphome {
namespace mhi_multi_ir {

// IR-тайминги
static const uint16_t HDR_MARK   = 3200;
static const uint16_t HDR_SPACE  = 1600;
static const uint16_t BIT_MARK   = 400;
static const uint16_t ONE_SPACE  = 1200;
static const uint16_t ZERO_SPACE = 400;
static const uint32_t CARRIER_HZ = 38000;

// Сигнатуры
static const uint8_t SIG_LEN   = 5;
static const uint8_t SIG_ZM[5] = {0xAD, 0x51, 0x3C, 0xE5, 0x1A};  // ZM/ZMP
static const uint8_t SIG_ZJ[5] = {0xAD, 0x51, 0x3C, 0xD9, 0x26};  // ZJ/ZEA

// Диапазон температуры
static const uint8_t MIN_TEMP = 17;
static const uint8_t MAX_TEMP = 31;

// 152-битный протокол (ZM/ZMP)
static const uint16_t LEN_152    = 19;
static const uint8_t  SV152_AUTO = 0, SH152_AUTO = 0, SV152_OFF = 6, SH152_OFF = 8;

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

// 88-битный протокол (ZJ/ZEA)
static const uint16_t LEN_88    = 11;
static const uint8_t  SV88_AUTO = 0, SH88_3D = 14, SV88_OFF = 0, SH88_OFF = 0;

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

// Коды режимов
static const uint8_t
  P152_AUTO = 0, P152_COOL = 1, P152_DRY = 2, P152_FAN = 3, P152_HEAT = 4,
  P88_AUTO  = 0, P88_COOL  = 1, P88_DRY  = 2, P88_FAN  = 3, P88_HEAT  = 4;

// Коды вентиляторов
static const uint8_t
  F152_AUTO = 0, F152_LOW = 1, F152_MED = 2, F152_HIGH = 3,
  F88_AUTO  = 0, F88_LOW  = 2, F88_MED  = 3, F88_HIGH  = 4;

// Инверсия байтов-пар
inline void invert_byte_pairs(uint8_t *data, size_t len) {
  for (size_t i = 0; i + 1 < len; i += 2)
    data[i + 1] = ~data[i];
}

// Конвертеры
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
    case F152_LOW:  return climate::CLIMATE_FAN_LOW;
    case F152_MED:  return climate::CLIMATE_FAN_MEDIUM;
    case F152_HIGH: return climate::CLIMATE_FAN_HIGH;
    default:        return climate::CLIMATE_FAN_AUTO;
  }
}
inline climate::ClimateFanMode convert_fan88(uint8_t f) {
  switch (f) {
    case F88_LOW:  return climate::CLIMATE_FAN_LOW;
    case F88_MED:  return climate::CLIMATE_FAN_MEDIUM;
    case F88_HIGH: return climate::CLIMATE_FAN_HIGH;
    default:       return climate::CLIMATE_FAN_AUTO;
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

// Компонент

enum Model : uint8_t { ZJ = 0, ZEA = 1, ZM = 2, ZMP = 3 };
enum SetFanLevels : uint8_t { FAN_LEVELS_3 = 3, FAN_LEVELS_4 = 4 };

class MhiClimate : public climate_ir::ClimateIR {
 public:
  MhiClimate()
      : climate_ir::ClimateIR(
          /*min_temp=*/MIN_TEMP,
          /*max_temp=*/MAX_TEMP,
          /*step=*/1.0f,
          /*has_current_temperature=*/true,
          /*has_target_temperature=*/true,
          /*supported_fan_modes=*/
            std::set<climate::ClimateFanMode>{
              climate::CLIMATE_FAN_AUTO,
              climate::CLIMATE_FAN_LOW,
              climate::CLIMATE_FAN_MIDDLE,
              climate::CLIMATE_FAN_MEDIUM,
              climate::CLIMATE_FAN_HIGH
            },
          /*supported_swing_modes=*/
            std::set<climate::ClimateSwingMode>{
              climate::CLIMATE_SWING_OFF,
              climate::CLIMATE_SWING_VERTICAL,
              climate::CLIMATE_SWING_HORIZONTAL,
              climate::CLIMATE_SWING_BOTH
            },
          /*supported_presets=*/
            std::set<climate::ClimatePreset>{
              climate::CLIMATE_PRESET_NONE,
              climate::CLIMATE_PRESET_ECO,
              climate::CLIMATE_PRESET_BOOST,
              climate::CLIMATE_PRESET_ACTIVITY
            }
        ),
        model_(ZJ), fan_levels_(FAN_LEVELS_3) {}

  void set_model(Model m)             { model_ = m; }
  void set_fan_levels(SetFanLevels l) { fan_levels_ = l; }

 protected:
  climate::ClimateTraits traits() override {
    auto t = climate_ir::ClimateIR::traits();
    // добавляем OFF в список режимов
    t.set_supported_modes({
      climate::CLIMATE_MODE_OFF,
      climate::CLIMATE_MODE_HEAT_COOL,
      climate::CLIMATE_MODE_COOL,
      climate::CLIMATE_MODE_HEAT,
      climate::CLIMATE_MODE_DRY,
      climate::CLIMATE_MODE_FAN_ONLY
    });
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
