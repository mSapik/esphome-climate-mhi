// mhi_multi_ir.cpp
#include "mhi_multi_ir.h"
#include "esphome/core/log.h"
#include <cstring>

namespace esphome {
namespace mhi_multi_ir {

static const char *TAG = "mhi_multi_ir.climate";

// Универсальный ридер пакета: заголовок + N байт + проверка static + инверсий
static bool read_bytes(remote_base::RemoteReceiveData &data,
                       const uint8_t *prefix, size_t prefix_len,
                       uint8_t *out, size_t len) {
  if (!data.expect_item(MHI_HDR_MARK, MHI_HDR_SPACE))
    return false;
  for (size_t i = 0; i < len; i++) {
    uint8_t v = 0;
    for (int b = 0; b < 8; b++) {
      if (data.expect_item(MHI_BIT_MARK, MHI_ONE_SPACE))
        v |= 1 << b;
      else if (!data.expect_item(MHI_BIT_MARK, MHI_ZERO_SPACE))
        return false;
    }
    out[i] = v;
  }
  // static prefix
  for (size_t i = 0; i < prefix_len; i++)
    if (out[i] != prefix[i])
      return false;
  // inversions: for each byte in second half vs first half
  size_t half = len / 2;
  for (size_t i = 0; i < half - prefix_len; i++)
    if (out[prefix_len + i] != uint8_t(~out[prefix_len + half + i]))
      return false;
  return true;
}

bool MhiClimate::on_receive(remote_base::RemoteReceiveData data) {
  ESP_LOGD(TAG, "on_receive for model %u", uint(model_));

  // Выбираем длину и префикс по модели
  uint8_t buf[19];
  const uint8_t *prefix;
  size_t prefix_len = 5, total_len;

  if (model_ == ZM) {
    // ZM: 19 байт, prefix 52 AE C3 1A E5
    static const uint8_t p_zm[5] = {0x52, 0xAE, 0xC3, 0x1A, 0xE5};
    prefix = p_zm;
    total_len = 19;
  } else {
    // ZJ, ZEA, ZMP: 11 байт, prefix 52 AE C3 26 D9
    static const uint8_t p_other[5] = {0x52, 0xAE, 0xC3, 0x26, 0xD9};
    prefix = p_other;
    total_len = 11;
  }

  if (!read_bytes(data, prefix, prefix_len, buf, total_len))
    return false;

  // Разбор общих полей
  if (model_ == ZM) {
    // ZM: buf[5]=flags, buf[7]=~temp, buf[9]=fan+mode, buf[11]=swingV, buf[13]=swingH
    auto pwr = buf[5] & 0x08;
    auto md  = buf[5] & 0x07;
    auto tmp = ((~buf[7] & 0x0F)) + 17;
    auto fan = buf[9] & 0x0F;
    auto sv  = buf[11] & 0xE0;
    auto sh  = buf[13] & 0x0F;
    this->target_temperature = tmp;
    this->mode = (pwr == MHI_ON
        ? (md == MHI_HEAT   ? climate::CLIMATE_MODE_HEAT
         : md == MHI_COOL   ? climate::CLIMATE_MODE_COOL
         : md == MHI_DRY    ? climate::CLIMATE_MODE_DRY
         : md == MHI_FAN    ? climate::CLIMATE_MODE_FAN_ONLY
         :                     climate::CLIMATE_MODE_HEAT_COOL)
        : climate::CLIMATE_MODE_OFF);
    // swing
    if (sv == MHI_VS_SWING && sh == MHI_HS_SWING)
      this->swing_mode = climate::CLIMATE_SWING_BOTH;
    else if (sv == MHI_VS_SWING)
      this->swing_mode = climate::CLIMATE_SWING_VERTICAL;
    else if (sh == MHI_HS_SWING)
      this->swing_mode = climate::CLIMATE_SWING_HORIZONTAL;
    else
      this->swing_mode = climate::CLIMATE_SWING_OFF;
    // fan
    switch (fan) {
      case 1:  this->fan_mode = climate::CLIMATE_FAN_LOW;    break;
      case 2:  this->fan_mode = (fan_levels_ == FAN_LEVELS_3 ? climate::CLIMATE_FAN_MEDIUM
                                                             : climate::CLIMATE_FAN_MIDDLE);
               break;
      case 3:  this->fan_mode = (fan_levels_ == FAN_LEVELS_3 ? climate::CLIMATE_FAN_HIGH
                                                             : climate::CLIMATE_FAN_MEDIUM);
               break;
      case 4:  this->fan_mode = climate::CLIMATE_FAN_HIGH;   break;
      default: this->fan_mode = climate::CLIMATE_FAN_AUTO;   break;
    }
  } else {
    // ZJ, ZEA, ZMP: buf[9]=mode+temp, buf[7]=fan+sv bits, buf[5]=hs+sv bits
    auto pwr = buf[9] & 0x08;
    auto md  = buf[9] & 0x07;
    auto tmp = ((~buf[9] & 0xF0) >> 4) + 17;
    auto fan = buf[7] & 0xE0;
    auto sv  = (buf[5] & 0x02) | (buf[7] & 0x18);
    auto sh  = buf[5] & 0xCC;
    this->target_temperature = tmp;
    this->mode = (pwr == MHI_ON
        ? (md == MHI_HEAT   ? climate::CLIMATE_MODE_HEAT
         : md == MHI_COOL   ? climate::CLIMATE_MODE_COOL
         : md == MHI_DRY    ? climate::CLIMATE_MODE_DRY
         : md == MHI_FAN    ? climate::CLIMATE_MODE_FAN_ONLY
         :                     climate::CLIMATE_MODE_AUTO)
        : climate::CLIMATE_MODE_OFF);
    // swing
    if (sv == MHI_VS_SWING && sh == MHI_HS_SWING)
      this->swing_mode = climate::CLIMATE_SWING_BOTH;
    else if (sv == MHI_VS_SWING)
      this->swing_mode = climate::CLIMATE_SWING_VERTICAL;
    else if (sh == MHI_HS_SWING)
      this->swing_mode = climate::CLIMATE_SWING_HORIZONTAL;
    else
      this->swing_mode = climate::CLIMATE_SWING_OFF;
    // fan
    switch (fan) {
      case MHI_ZJ_FAN1: case MHI_ZEA_FAN1: case MHI_ZMP_FAN1:
        this->fan_mode = climate::CLIMATE_FAN_LOW;    break;
      case MHI_ZJ_FAN2: case MHI_ZEA_FAN2: case MHI_ZMP_FAN2:
        this->fan_mode = climate::CLIMATE_FAN_MEDIUM; break;
      case MHI_ZJ_FAN3: case MHI_ZEA_FAN3: case MHI_ZMP_FAN3:
        this->fan_mode = climate::CLIMATE_FAN_HIGH;   break;
      default:
        this->fan_mode = climate::CLIMATE_FAN_AUTO;
        if (sh == MHI_HS_MIDDLE)    this->fan_mode = climate::CLIMATE_FAN_MIDDLE;
        if (sh == MHI_HS_RIGHTLEFT) this->fan_mode = climate::CLIMATE_FAN_FOCUS;
        if (sh == MHI_HS_LEFTRIGHT) this->fan_mode = climate::CLIMATE_FAN_DIFFUSE;
    }
  }

  ESP_LOGD(TAG, "on_receive done: mode=%u temp=%.1f fan=%u swing=%u",
           this->mode, this->target_temperature, this->fan_mode.value(), this->swing_mode);
  this->publish_state();
  return true;
}

void MhiClimate::transmit_state() {
  auto tx = this->transmitter_->transmit();
  auto *data = tx.get_data();
  data->set_carrier_frequency(38000);
  data->mark(MHI_HDR_MARK);
  data->space(MHI_HDR_SPACE);

  switch (model_) {
    case ZJ: {
      const uint8_t tpl[11] = {
        MHI_P0, MHI_P1, MHI_P2, MHI_P3_ZJZEPM, MHI_P4_ZJZEPM,
        0x11, 0x00, 0x07, 0x00, 0x00, 0x00
      };
      uint8_t buf[11]; std::memcpy(buf, tpl, 11);

      // Пресеты
      switch (this->preset.value()) {
        case climate::CLIMATE_PRESET_ECO:
          buf[7] = (buf[7] & ~0xE0) | MHI_ZJ_ECONO;
          break;
        case climate::CLIMATE_PRESET_BOOST:
          buf[7] = (buf[7] & ~0xE0) | MHI_ZJ_HIPOWER;
          break;
        case climate::CLIMATE_PRESET_ACTIVITY:
          buf[5] |= MHI_HS_3DAUTO;
          buf[7] = (buf[7] & ~0xE0) | MHI_ZJ_FAN_AUTO;
          buf[7] |= MHI_VS_SWING;
          break;
        default:
          break;
      }

      uint8_t pm = (this->mode == climate::CLIMATE_MODE_OFF ? MHI_OFF : MHI_ON);
      uint8_t om = (this->mode == climate::CLIMATE_MODE_HEAT   ? MHI_HEAT   :
                    this->mode == climate::CLIMATE_MODE_COOL  ? MHI_COOL   :
                    this->mode == climate::CLIMATE_MODE_DRY   ? MHI_DRY    :
                    this->mode == climate::CLIMATE_MODE_FAN_ONLY ? MHI_FAN :
                    MHI_AUTO);
      uint8_t t  = uint8_t((this->target_temperature > 17 && this->target_temperature < 31)
                   ? (~((uint8_t)(this->target_temperature - 17) << 4) & 0xF0)
                   : 0);

      // FAN-режимы, включая MIDDLE/FOCUS/DIFFUSE
      uint8_t fs = buf[7] & 0xE0;
      switch (this->fan_mode.value()) {
        case climate::CLIMATE_FAN_LOW:    fs = MHI_ZJ_FAN1; break;
        case climate::CLIMATE_FAN_MEDIUM: fs = MHI_ZJ_FAN2; break;
        case climate::CLIMATE_FAN_HIGH:   fs = MHI_ZJ_FAN3; break;
        case climate::CLIMATE_FAN_MIDDLE:
          fs = MHI_ZJ_FAN_AUTO; buf[5] |= MHI_HS_MIDDLE; break;
        case climate::CLIMATE_FAN_FOCUS:
          fs = MHI_ZJ_HIPOWER; buf[5] |= MHI_HS_RIGHTLEFT; break;
        case climate::CLIMATE_FAN_DIFFUSE:
          fs = MHI_ZJ_HIPOWER; buf[5] |= MHI_HS_LEFTRIGHT; break;
        default:
          break;
      }

      // Сборка пакета
      uint8_t sv = (buf[5] & 0x02) | (buf[7] & 0x18);
      uint8_t sh = buf[5] & 0xCC;
      buf[5] |= sh | MHI_ZJ_CLEAN_OFF;
      buf[7] = (buf[7] & ~0xE0) | fs | (sv & 0x18);
      buf[9] |= om | pm | t;
      buf[6]  = ~buf[5];
      buf[8]  = ~buf[7];
      buf[10] = ~buf[9];

      send_bytes(data, buf, 11);
      break;
    }

    case ZEA: {
      const uint8_t tpl[11] = {
        MHI_P0, MHI_P1, MHI_P2, MHI_P3_ZJZEPM, MHI_P4_ZJZEPM,
        0xDF, 0x20, 0x07, 0x00, 0x00, 0x00
      };
      uint8_t buf[11]; std::memcpy(buf, tpl, 11);

      // Пресеты
      switch (this->preset.value()) {
        case climate::CLIMATE_PRESET_ECO:
          buf[7] = (buf[7] & ~0xE0) | MHI_ZEA_ECONO;
          break;
        case climate::CLIMATE_PRESET_BOOST:
          buf[7] = (buf[7] & ~0xE0) | MHI_ZEA_HIPOWER;
          break;
        case climate::CLIMATE_PRESET_ACTIVITY:
          buf[5] |= MHI_HS_3DAUTO;
          buf[7] = (buf[7] & ~0xE0) | MHI_ZEA_FAN_AUTO;
          buf[7] |= MHI_VS_SWING;
          break;
        default:
          break;
      }

      uint8_t pm = (this->mode == climate::CLIMATE_MODE_OFF ? MHI_OFF : MHI_ON);
      uint8_t om = (this->mode == climate::CLIMATE_MODE_HEAT   ? MHI_HEAT   :
                    this->mode == climate::CLIMATE_MODE_COOL  ? MHI_COOL   :
                    this->mode == climate::CLIMATE_MODE_DRY   ? MHI_DRY    :
                    this->mode == climate::CLIMATE_MODE_FAN_ONLY ? MHI_FAN :
                    MHI_AUTO);
      uint8_t t  = uint8_t((this->target_temperature > 17 && this->target_temperature < 31)
                   ? (~((uint8_t)(this->target_temperature - 17) << 4) & 0xF0)
                   : 0);

      uint8_t fs = buf[7] & 0xE0;
      switch (this->fan_mode.value()) {
        case climate::CLIMATE_FAN_LOW:    fs = MHI_ZEA_FAN1; break;
        case climate::CLIMATE_FAN_MEDIUM: fs = MHI_ZEA_FAN2; break;
        case climate::CLIMATE_FAN_HIGH:   fs = MHI_ZEA_FAN3; break;
        case climate::CLIMATE_FAN_MIDDLE: fs = MHI_ZEA_FAN4; buf[5] |= MHI_HS_MIDDLE; break;
        case climate::CLIMATE_FAN_FOCUS:  fs = MHI_ZEA_HIPOWER; buf[5] |= MHI_HS_RIGHTLEFT; break;
        case climate::CLIMATE_FAN_DIFFUSE:fs = MHI_ZEA_ECONO; buf[5] |= MHI_HS_LEFTRIGHT; break;
        default:                           break;
      }

      uint8_t sv = (buf[5] & 0x01) | ((buf[7] & 0x18));
      buf[5] |= ((buf[5] & 0xF0) & 0xF0) | MHI_ZEA_CLEAN_OFF;
      buf[6]  = ~buf[5];
      buf[7]  = (buf[7] & ~0xE0) | fs | ((sv << 1) & 0x18);
      buf[8]  = ~buf[7];
      buf[9] |= om | pm | t;
      buf[10] = ~buf[9];

      send_bytes(data, buf, 11);
      break;
    }

    case ZM: {
      const uint8_t tpl[19] = {
        MHI_P0, MHI_P1, MHI_P2, MHI_P3_ZM, MHI_P4_ZM,
        0x90,0x00,0xF0,0x00,0xF0,0x00,0x0D,0x00,0x10,0x00,0xFF,0x00,0x7B,0x00
      };
      uint8_t buf[19]; std::memcpy(buf, tpl, 19);

      // Пресеты
      switch (this->preset.value()) {
        case climate::CLIMATE_PRESET_ECO:
          buf[9] = (buf[9] & ~0x0F) | MHI_ZM_ECONO;
          break;
        case climate::CLIMATE_PRESET_BOOST:
          buf[9] = (buf[9] & ~0x0F) | MHI_ZM_HIPOWER;
          break;
        case climate::CLIMATE_PRESET_ACTIVITY:
          buf[11] |= MHI_3DAUTO_ON;
          buf[11] |= MHI_VS_SWING;
          break;
        default:
          break;
      }

      uint8_t pm = (this->mode == climate::CLIMATE_MODE_OFF ? MHI_OFF : MHI_ON);
      uint8_t om = (this->mode == climate::CLIMATE_MODE_HEAT   ? MHI_HEAT   :
                    this->mode == climate::CLIMATE_MODE_COOL  ? MHI_COOL   :
                    this->mode == climate::CLIMATE_MODE_DRY   ? MHI_DRY    :
                    this->mode == climate::CLIMATE_MODE_FAN_ONLY ? MHI_FAN :
                    MHI_AUTO);
      uint8_t t  = uint8_t((this->target_temperature > 17 && this->target_temperature < 31)
                   ? (~((uint8_t)(this->target_temperature - 17) << 4) & 0xF0)
                   : 0);

      uint8_t fs = buf[9] & 0x0F;
      switch (this->fan_mode.value()) {
        case climate::CLIMATE_FAN_LOW:    fs = MHI_ZM_FAN1; break;
        case climate::CLIMATE_FAN_MEDIUM: fs = MHI_ZM_FAN2; break;
        case climate::CLIMATE_FAN_HIGH:   fs = MHI_ZM_FAN3; break;
        case climate::CLIMATE_FAN_MIDDLE: fs = MHI_ZM_FAN4; buf[13] |= MHI_HS_MIDDLE; break;
        case climate::CLIMATE_FAN_FOCUS:  fs = MHI_ZM_HIPOWER; buf[13] |= MHI_HS_RIGHTLEFT; break;
        case climate::CLIMATE_FAN_DIFFUSE:fs = MHI_ZM_ECONO; buf[13] |= MHI_HS_LEFTRIGHT; break;
        default:                           break;
      }

      uint8_t sv = buf[11] & 0xE0;
      uint8_t hd = buf[13] & 0x0F;
      buf[5] |= om | pm | MHI_ZM_CLEAN_OFF;
      buf[6]  = ~buf[5];
      buf[7] |= t;
      buf[8]  = ~buf[7];
      buf[9]  = (buf[9] & ~0x0F) | fs;
      buf[10] = ~buf[9];
      buf[11] = (buf[11] & ~0xE0) | sv;
      buf[12] = ~buf[11];
      buf[13] = (buf[13] & ~0x0F) | hd;
      buf[14] = ~buf[13];
      buf[15] |= MHI_SILENT_OFF;
      buf[16] = ~buf[15];
      buf[17] = 0;
      buf[18] = ~buf[17];

      send_bytes(data, buf, 19);
      break;
    }

    case ZMP: {
      const uint8_t tpl[11] = {
        MHI_P0, MHI_P1, MHI_P2, MHI_P3_ZJZEPM, MHI_P4_ZJZEPM,
        0x11,0x00,0x07,0x00,0x00,0x00
      };
      uint8_t buf[11]; std::memcpy(buf, tpl, 11);

      // Пресеты
      switch (this->preset.value()) {
        case climate::CLIMATE_PRESET_ECO:
          buf[7] = (buf[7] & ~0xE0) | MHI_ZMP_ECONO;
          break;
        case climate::CLIMATE_PRESET_BOOST:
          buf[7] = (buf[7] & ~0xE0) | MHI_ZMP_HIPOWER;
          break;
        case climate::CLIMATE_PRESET_ACTIVITY:
          buf[7] |= MHI_VS_SWING;
          break;
        default:
          break;
      }

      uint8_t pm = (this->mode == climate::CLIMATE_MODE_OFF ? MHI_OFF : MHI_ON);
      uint8_t om = (this->mode == climate::CLIMATE_MODE_HEAT   ? MHI_HEAT   :
                    this->mode == climate::CLIMATE_MODE_COOL  ? MHI_COOL   :
                    this->mode == climate::CLIMATE_MODE_DRY   ? MHI_DRY    :
                    this->mode == climate::CLIMATE_MODE_FAN_ONLY ? MHI_FAN :
                    MHI_AUTO);
      uint8_t t  = uint8_t((this->target_temperature > 17 && this->target_temperature < 31)
                   ? (~((uint8_t)(this->target_temperature - 17) << 4) & 0xF0)
                   : 0);

      uint8_t fs = buf[7] & 0xE0;
      switch (this->fan_mode.value()) {
        case climate::CLIMATE_FAN_LOW:    fs = MHI_ZMP_FAN1; break;
        case climate::CLIMATE_FAN_MEDIUM: fs = MHI_ZMP_FAN2; break;
        case climate::CLIMATE_FAN_HIGH:   fs = MHI_ZMP_FAN3; break;
        default:                           break;
      }

      uint8_t sv = (buf[5] & 0x02) | (buf[7] & 0x18);
      uint8_t hd = buf[5] & 0xCC;
      buf[5] |= hd | MHI_ZMP_CLEAN_OFF;
      buf[6]  = ~buf[5];
      buf[7]  = (buf[7] & ~0xE0) | fs | (sv & 0x18);
      buf[8]  = ~buf[7];
      buf[9] |= om | pm | t;
      buf[10] = ~buf[9];

      send_bytes(data, buf, 11);
      break;
    }
  }

  data->mark(MHI_BIT_MARK);
  data->space(0);
  tx.perform();
}

}  // namespace mhi_multi_ir
}  // namespace esphome
