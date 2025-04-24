// mhi_multi_ir.cpp
#include "mhi_multi_ir.h"
#include "esphome/core/log.h"
#include <cstring>

namespace esphome {
namespace mhi_multi_ir {

static const char *TAG = "mhi_multi_ir.climate";

void MhiClimate::send_bytes(remote_base::RemoteTransmitData *data, const uint8_t *tpl, size_t len) {
  for (size_t i = 0; i < len; i++) {
    for (int b = 0; b < 8; b++) {
      data->mark(MHI_BIT_MARK);
      data->space((tpl[i] & (1 << b)) ? MHI_ONE_SPACE : MHI_ZERO_SPACE);
    }
  }
}

bool MhiClimate::on_receive(remote_base::RemoteReceiveData data) {
  ESP_LOGD(TAG, "on_receive start");
  if (!data.expect_item(MHI_HDR_MARK, MHI_HDR_SPACE)) return false;

  uint8_t bytes[19] = {};
  for (int i = 0; i < 19; i++) {
    uint8_t v = 0;
    for (int b = 0; b < 8; b++) {
      if (data.expect_item(MHI_BIT_MARK, MHI_ONE_SPACE))
        v |= (1 << b);
      else if (!data.expect_item(MHI_BIT_MARK, MHI_ZERO_SPACE))
        return false;
    }
    bytes[i] = v;
  }

  if (bytes[0] != MHI_P0 || bytes[1] != MHI_P1 || bytes[2] != MHI_P2 ||
      (bytes[3] != MHI_P3_ZJZEPM && bytes[3] != MHI_P3_ZM) ||
      (bytes[4] != MHI_P4_ZJZEPM && bytes[4] != MHI_P4_ZM))
    return false;
  if (bytes[5] != uint8_t(~bytes[6]) ||
      bytes[7] != uint8_t(~bytes[8]) ||
      bytes[9] != uint8_t(~bytes[10]))
    return false;

  uint8_t pwr = bytes[9] & 0x08;
  uint8_t md  = bytes[9] & 0x07;
  uint8_t tmp = ((~bytes[9] & 0xF0) >> 4) + 17;
  uint8_t fan = bytes[7] & 0xE0;
  uint8_t sv  = (bytes[5] & 0x02) | (bytes[7] & 0x18);
  uint8_t sh  = bytes[5] & 0xCC;

  if (pwr == MHI_ON) {
    switch (md) {
      case MHI_HEAT: this->mode = climate::CLIMATE_MODE_HEAT; break;
      case MHI_COOL: this->mode = climate::CLIMATE_MODE_COOL; break;
      case MHI_DRY:  this->mode = climate::CLIMATE_MODE_DRY; break;
      case MHI_FAN:  this->mode = climate::CLIMATE_MODE_FAN_ONLY; break;
      default:       this->mode = climate::CLIMATE_MODE_AUTO; break;
    }
  } else {
    this->mode = climate::CLIMATE_MODE_OFF;
  }
  this->target_temperature = tmp;

  if (sv == MHI_VS_SWING && sh == MHI_HS_SWING)
    this->swing_mode = climate::CLIMATE_SWING_BOTH;
  else if (sv == MHI_VS_SWING)
    this->swing_mode = climate::CLIMATE_SWING_VERTICAL;
  else if (sh == MHI_HS_SWING)
    this->swing_mode = climate::CLIMATE_SWING_HORIZONTAL;
  else
    this->swing_mode = climate::CLIMATE_SWING_OFF;

  if      (fan == MHI_ZJ_FAN1  || fan == MHI_ZEA_FAN1  || fan == MHI_ZM_FAN1  || fan == MHI_ZMP_FAN1) this->fan_mode = climate::CLIMATE_FAN_LOW;
  else if (fan == MHI_ZJ_FAN2  || fan == MHI_ZEA_FAN2  || fan == MHI_ZM_FAN2  || fan == MHI_ZMP_FAN2) this->fan_mode = climate::CLIMATE_FAN_MEDIUM;
  else if (fan == MHI_ZJ_FAN3  || fan == MHI_ZEA_FAN3  || fan == MHI_ZM_FAN3  || fan == MHI_ZMP_FAN3) this->fan_mode = climate::CLIMATE_FAN_HIGH;
  else                                                                                          this->fan_mode = climate::CLIMATE_FAN_AUTO;

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
