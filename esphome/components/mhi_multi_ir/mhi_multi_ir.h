// mhi_multi_ir.h
#pragma once

#include "esphome/components/climate_ir/climate_ir.h"
#include <cinttypes>
#include <set>

namespace esphome {
namespace mhi_multi_ir {

// Модели Mitsubishi Heavy
enum Model : uint8_t {
  ZJ  = 0,
  ZM  = 1,
  ZMP = 2,
  ZEA = 3,
};

// Тайминги IR
#define MHI_HDR_MARK    3200
#define MHI_HDR_SPACE   1600
#define MHI_BIT_MARK    400
#define MHI_ONE_SPACE   1200
#define MHI_ZERO_SPACE  400
#define MHI_MIN_GAP     17500

// Префикс байтов
#define MHI_P0            0x52
#define MHI_P1            0xAE
#define MHI_P2            0xC3
#define MHI_P3_ZJZEPM     0x26
#define MHI_P3_ZM         0x1A
#define MHI_P4_ZJZEPM     0xD9
#define MHI_P4_ZM         0xE5

// Питание
#define MHI_OFF           0x08
#define MHI_ON            0x00

// Режимы работы
#define MHI_AUTO          0x07
#define MHI_HEAT          0x03
#define MHI_COOL          0x06
#define MHI_DRY           0x05
#define MHI_FAN           0x04
#define MHI_ZMP_MODE_FAN   0xD4
#define MHI_ZMP_MODE_MAINT 0x06

// Вертикальный swing
#define MHI_VS_SWING      0x0A
#define MHI_VS_UP         0x02
#define MHI_VS_MUP        0x18
#define MHI_VS_MIDDLE     0x10
#define MHI_VS_MDOWN      0x08
#define MHI_VS_DOWN       0x00
#define MHI_VS_STOP       0x1A

// Горизонтальный swing
#define MHI_HS_SWING      0x4C
#define MHI_HS_MIDDLE     0x48
#define MHI_HS_LEFT       0xC8
#define MHI_HS_MLEFT      0x88
#define MHI_HS_MRIGHT     0x08
#define MHI_HS_RIGHT      0xC4
#define MHI_HS_STOP       0xCC
#define MHI_HS_LEFTRIGHT  0x84
#define MHI_HS_RIGHTLEFT  0x44
#define MHI_HS_3DAUTO     0x04

// 3D auto / silent
#define MHI_3DAUTO_ON     0x00
#define MHI_3DAUTO_OFF    0x12
#define MHI_SILENT_ON     0x00
#define MHI_SILENT_OFF    0x80

// Eco / Clean
#define MHI_ECO_ON        0x00
#define MHI_ECO_OFF       0x10
#define MHI_CLEAN_ON      0x00
#define MHI_ZJ_CLEAN_OFF   0x20
#define MHI_ZEA_CLEAN_OFF  0x08
#define MHI_ZM_CLEAN_OFF   0x60
#define MHI_ZMP_CLEAN_OFF  0x20
#define MHI_ZMP_CLEAN_ON   0xDF

// Скорости вентилятора — ZJ
#define MHI_ZJ_FAN_AUTO   0xE0
#define MHI_ZJ_FAN1       0xA0
#define MHI_ZJ_FAN2       0x80
#define MHI_ZJ_FAN3       0x60
#define MHI_ZJ_HIPOWER    0x40
#define MHI_ZJ_ECONO      0x00

// Скорости вентилятора — ZEA
#define MHI_ZEA_FAN_AUTO  0xE0
#define MHI_ZEA_FAN1      0xC2
#define MHI_ZEA_FAN2      0xA4
#define MHI_ZEA_FAN3      0x86
#define MHI_ZEA_FAN4      0x68
#define MHI_ZEA_HIPOWER   0x2C
#define MHI_ZEA_ECONO     0x0E

// Скорости вентилятора — ZM
#define MHI_ZM_FAN_AUTO   0x0F
#define MHI_ZM_FAN1       0x0E
#define MHI_ZM_FAN2       0x0D
#define MHI_ZM_FAN3       0x0C
#define MHI_ZM_FAN4       0x0B
#define MHI_ZM_HIPOWER    0x07
#define MHI_ZM_ECONO      0x09

// Скорости вентилятора — ZMP
#define MHI_ZMP_FAN_AUTO  0xE0
#define MHI_ZMP_FAN1      0xA0
#define MHI_ZMP_FAN2      0x80
#define MHI_ZMP_FAN3      0x60
#define MHI_ZMP_HIPOWER   0x20
#define MHI_ZMP_ECONO     0x00

/** Количество уровней вентилятора, поддерживаемое блоком */
enum SetFanLevels : uint8_t {
  FAN_LEVELS_3 = 3,
  FAN_LEVELS_4 = 4,
};

class MhiClimate : public climate_ir::ClimateIR {
 public:
  MhiClimate()
      : climate_ir::ClimateIR(
            18, 30, 1.0f, true, true,
            std::set<climate::ClimateFanMode>{
                climate::CLIMATE_FAN_AUTO,
                climate::CLIMATE_FAN_LOW,
                climate::CLIMATE_FAN_MIDDLE,
                climate::CLIMATE_FAN_MEDIUM,
                climate::CLIMATE_FAN_HIGH,
                climate::CLIMATE_FAN_FOCUS,
                climate::CLIMATE_FAN_DIFFUSE},
            std::set<climate::ClimateSwingMode>{
                climate::CLIMATE_SWING_OFF,
                climate::CLIMATE_SWING_VERTICAL,
                climate::CLIMATE_SWING_HORIZONTAL,
                climate::CLIMATE_SWING_BOTH},
            std::set<climate::ClimatePreset>{
                climate::CLIMATE_PRESET_NONE,
                climate::CLIMATE_PRESET_ECO,
                climate::CLIMATE_PRESET_BOOST,
                climate::CLIMATE_PRESET_ACTIVITY}),
        model_(ZJ),
        fan_levels_(FAN_LEVELS_3) {}

  /** Установить модель блока */
  void set_model(Model m) { model_ = m; }

  /** Установить, сколько скоростей вентилятора */
  void set_fan_levels(SetFanLevels levels) { fan_levels_ = levels; }

 protected:
  climate::ClimateTraits traits() override {
    auto traits = climate_ir::ClimateIR::traits();
    // Формируем set на основе количества скоростей
    std::set<climate::ClimateFanMode> modes;
    modes.insert(climate::CLIMATE_FAN_AUTO);
    modes.insert(climate::CLIMATE_FAN_LOW);
    if (fan_levels_ == FAN_LEVELS_4)
      modes.insert(climate::CLIMATE_FAN_MIDDLE);
    modes.insert(climate::CLIMATE_FAN_MEDIUM);
    modes.insert(climate::CLIMATE_FAN_HIGH);
    // Всегда доступны FOCUS и DIFFUSE
    modes.insert(climate::CLIMATE_FAN_FOCUS);
    modes.insert(climate::CLIMATE_FAN_DIFFUSE);
    traits.set_supported_fan_modes(std::move(modes));
    return traits;
  }

  void transmit_state() override;
  bool on_receive(remote_base::RemoteReceiveData data) override;

 private:
  Model model_;
  SetFanLevels fan_levels_;

  static void send_bytes(remote_base::RemoteTransmitData *data,
                         const uint8_t *tpl, size_t len);
};

}  // namespace mhi_multi_ir
}  // namespace esphome
