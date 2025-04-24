// mhi_multi_ir.cpp
#include "mhi_multi_ir.h"
#include "esphome/core/log.h"

namespace esphome {
namespace mhi_multi_ir {

static const char *TAG = "mhi_multi_ir.climate";

// ----------------------------------------------------------------------------
// send_bytes
// ----------------------------------------------------------------------------
template <typename TransmitCall>
void MhiClimate::send_bytes(TransmitCall &tx, const uint8_t buf[], size_t len) {
  auto *data = tx.get_data();
  data->set_carrier_frequency(38000);
  data->mark(MHI_HDR_MARK);
  data->space(MHI_HDR_SPACE);
  for (size_t i = 0; i < len; i++) {
    for (uint8_t b = 0; b < 8; b++) {
      data->mark(MHI_BIT_MARK);
      bool one = buf[i] & (1 << b);
      data->space(one ? MHI_ONE_SPACE : MHI_ZERO_SPACE);
    }
  }
  data->mark(MHI_BIT_MARK);
  data->space(0);
  tx.perform();
}

// ----------------------------------------------------------------------------
// on_receive
// ----------------------------------------------------------------------------
bool MhiClimate::on_receive(remote_base::RemoteReceiveData data) {
  ESP_LOGD(TAG, "on_receive model=%u", unsigned(model_));
  // Выбираем длину пакета
  const size_t LEN = (model_ == ZM ? 19 : 11);
  uint8_t buf[19];

  // Шаблон префикса
  uint8_t prefix[5] = {
    MHI_P0, MHI_P1, MHI_P2,
    (model_ == ZM ? MHI_P3_ZM : MHI_P3_ZJZEPM),
    (model_ == ZM ? MHI_P4_ZM : MHI_P4_ZJZEPM)
  };

  if (!data.expect_item(MHI_HDR_MARK, MHI_HDR_SPACE))
    return false;

  // Чтение байт
  for (size_t i = 0; i < LEN; i++) {
    uint8_t v = 0;
    for (int b = 0; b < 8; b++) {
      if      (data.expect_item(MHI_BIT_MARK, MHI_ONE_SPACE))  v |= 1 << b;
      else if (!data.expect_item(MHI_BIT_MARK, MHI_ZERO_SPACE)) return false;
    }
    buf[i] = v;
  }

  // Проверяем префикс
  for (int i = 0; i < 5; i++)
    if (buf[i] != prefix[i]) return false;

  // Проверяем инверсию
  size_t half = LEN / 2;
  for (size_t i = 0; i < half - 5; i++)
    if (buf[5 + i] != uint8_t(~buf[5 + half + i])) return false;

  // Теперь распаковываем статус
  uint8_t power_mode, op_mode, fan_spd, swing_v, swing_h;
  int temperature;

  if (model_ == ZM) {
    power_mode = buf[5] & 0x08;
    op_mode    = buf[5] & 0x07;
    temperature= ((~buf[7] & 0x0F) + 17);
    fan_spd     = buf[9] & 0x0F;
    swing_v     = buf[11] & 0xE0;
    swing_h     = buf[13] & 0x0F;
  } else {
    power_mode = buf[9] & 0x08;
    op_mode    = buf[9] & 0x07;
    temperature= (((~buf[9] & 0xF0) >> 4) + 17);
    fan_spd     = buf[7] & 0xE0;
    swing_v     = ((buf[5] & 0x02) | (buf[7] & 0x18));
    swing_h     = (buf[5] & 0xCC);
  }

  // Power & Mode
  if (power_mode == MHI_ON) {
    switch (op_mode) {
      case MHI_COOL:      mode = climate::CLIMATE_MODE_COOL;      break;
      case MHI_HEAT:      mode = climate::CLIMATE_MODE_HEAT;      break;
      case MHI_FAN:       mode = climate::CLIMATE_MODE_FAN_ONLY;  break;
      case MHI_DRY:       mode = climate::CLIMATE_MODE_DRY;       break;
      default:            mode = (model_==ZM? climate::CLIMATE_MODE_HEAT_COOL : climate::CLIMATE_MODE_AUTO); break;
    }
  } else {
    mode = climate::CLIMATE_MODE_OFF;
  }

  // Temperature
  target_temperature = temperature;

  // Swing
  if (swing_v == MHI_VS_SWING && swing_h == MHI_HS_SWING)
    swing_mode = climate::CLIMATE_SWING_BOTH;
  else if (swing_v == MHI_VS_SWING)
    swing_mode = climate::CLIMATE_SWING_VERTICAL;
  else if (swing_h == MHI_HS_SWING)
    swing_mode = climate::CLIMATE_SWING_HORIZONTAL;
  else
    swing_mode = climate::CLIMATE_SWING_OFF;

  // Fan speed
  switch (fan_spd) {
    case MHI_ZJ_FAN1: case MHI_ZEA_FAN1: case MHI_ZMP_FAN1: this->fan_mode = climate::CLIMATE_FAN_LOW;    break;
    case MHI_ZJ_FAN2: case MHI_ZEA_FAN2: case MHI_ZMP_FAN2: this->fan_mode = climate::CLIMATE_FAN_MEDIUM; break;
    case MHI_ZJ_FAN3: case MHI_ZEA_FAN3: case MHI_ZMP_FAN3: this->fan_mode = climate::CLIMATE_FAN_HIGH;   break;
    default:
      this->fan_mode = climate::CLIMATE_FAN_AUTO;
      break;
  }

  publish_state();
  return true;
}

// ----------------------------------------------------------------------------
// transmit_state
// ----------------------------------------------------------------------------
void MhiClimate::transmit_state() {
  ESP_LOGD(TAG, "transmit_state model=%u", unsigned(model_));

  if (model_ == ZJ) {
    uint8_t buf[11] = {
      MHI_P0, MHI_P1, MHI_P2, MHI_P3_ZJZEPM, MHI_P4_ZJZEPM,
      0x11, 0x00, 0x07, 0x00, 0x00, 0x00
    };

    // Mode
    uint8_t op = MHI_AUTO, pm = MHI_ON;
    switch (mode) {
      case climate::CLIMATE_MODE_COOL:      op = MHI_COOL; break;
      case climate::CLIMATE_MODE_HEAT:      op = MHI_HEAT; break;
      case climate::CLIMATE_MODE_FAN_ONLY:  op = MHI_FAN;  break;
      case climate::CLIMATE_MODE_DRY:       op = MHI_DRY;  break;
      case climate::CLIMATE_MODE_OFF:       pm = MHI_OFF;  break;
      case climate::CLIMATE_MODE_AUTO:      op = MHI_AUTO; break;
      default: break;
    }
    buf[9] |= op | pm | ((~(uint8_t((target_temperature>17 && target_temperature<31? target_temperature:22)-17) << 4)) & 0xF0);

    // Fan
    uint8_t fs = MHI_ZJ_FAN_AUTO;
    switch (fan_mode.value()) {
      case climate::CLIMATE_FAN_LOW:    fs = MHI_ZJ_FAN1;  break;
      case climate::CLIMATE_FAN_MEDIUM: fs = MHI_ZJ_FAN2;  break;
      case climate::CLIMATE_FAN_HIGH:   fs = MHI_ZJ_FAN3;  break;
      default:                          fs = MHI_ZJ_FAN_AUTO; break;
    }
    buf[7] |= fs;

    // Swing
    if (swing_mode == climate::CLIMATE_SWING_BOTH) {
      buf[5] |= MHI_HS_SWING | (MHI_VS_SWING & 0x02);
      buf[7] |= (MHI_VS_SWING & 0x18);
    } else if (swing_mode == climate::CLIMATE_SWING_VERTICAL) {
      buf[5] |= (MHI_VS_SWING & 0x02);
      buf[7] |= (MHI_VS_SWING & 0x18);
    } else if (swing_mode == climate::CLIMATE_SWING_HORIZONTAL) {
      buf[5] |= MHI_HS_SWING;
    }

    // Inversions
    buf[6]  = ~buf[5];
    buf[8]  = ~buf[7];
    buf[10] = ~buf[9];

    // Send
    auto tx = transmitter_->transmit();
    send_bytes(tx, buf, 11);
    return;
  }

  if (model_ == ZEA) {
    uint8_t buf[11] = {
      MHI_P0, MHI_P1, MHI_P2, MHI_P3_ZJZEPM, MHI_P4_ZJZEPM,
      0xDF, 0x20, 0x07, 0x00, 0x00, 0x00
    };

    // Mode
    uint8_t op = MHI_AUTO, pm = MHI_ON;
    switch (mode) {
      case climate::CLIMATE_MODE_COOL:      op = MHI_COOL; break;
      case climate::CLIMATE_MODE_HEAT:      op = MHI_HEAT; break;
      case climate::CLIMATE_MODE_FAN_ONLY:  op = MHI_FAN;  break;
      case climate::CLIMATE_MODE_DRY:       op = MHI_DRY;  break;
      case climate::CLIMATE_MODE_OFF:       pm = MHI_OFF;  break;
      default: break;
    }
    buf[9] |= op | pm | ((~(uint8_t((target_temperature>17 && target_temperature<31? target_temperature:22)-17) << 4)) & 0xF0);

    // Fan
    uint8_t fs = MHI_ZEA_FAN_AUTO;
    switch (fan_mode.value()) {
      case climate::CLIMATE_FAN_LOW:    fs = MHI_ZEA_FAN1;  break;
      case climate::CLIMATE_FAN_MEDIUM: fs = MHI_ZEA_FAN2;  break;
      case climate::CLIMATE_FAN_HIGH:   fs = MHI_ZEA_FAN3;  break;
      case climate::CLIMATE_FAN_MIDDLE: fs = MHI_ZEA_FAN4;  break;
      default:                          fs = MHI_ZEA_FAN_AUTO; break;
    }
    buf[7] |= fs;

    // Swing same as ZJ
    if (swing_mode == climate::CLIMATE_SWING_BOTH) {
      buf[5] |= MHI_HS_SWING | (MHI_VS_SWING & 0x02);
      buf[7] |= (MHI_VS_SWING & 0x18);
    } else if (swing_mode == climate::CLIMATE_SWING_VERTICAL) {
      buf[5] |= (MHI_VS_SWING & 0x02);
      buf[7] |= (MHI_VS_SWING & 0x18);
    } else if (swing_mode == climate::CLIMATE_SWING_HORIZONTAL) {
      buf[5] |= MHI_HS_SWING;
    }

    buf[6]  = ~buf[5];
    buf[8]  = ~buf[7];
    buf[10] = ~buf[9];
    auto tx = transmitter_->transmit();
    send_bytes(tx, buf, 11);
    return;
  }

  if (model_ == ZMP) {
    uint8_t buf[11] = {
      MHI_P0, MHI_P1, MHI_P2, MHI_P3_ZJZEPM, MHI_P4_ZJZEPM,
      0x11, 0x00, 0x07, 0x00, 0x00, 0x00
    };
    // Логика как в ZJ, но для ZMP чистка через MODE_MAINT
    uint8_t op = MHI_AUTO, pm = MHI_ON;
    if (mode == climate::CLIMATE_MODE_OFF) pm = MHI_OFF;
    buf[9] |= op | pm | ((~(uint8_t((target_temperature>17 && target_temperature<31? target_temperature:22)-17) << 4)) & 0xF0);

    uint8_t fs = MHI_ZMP_FAN_AUTO;
    switch (fan_mode.value()) {
      case climate::CLIMATE_FAN_LOW:    fs = MHI_ZMP_FAN1;  break;
      case climate::CLIMATE_FAN_MEDIUM: fs = MHI_ZMP_FAN2;  break;
      case climate::CLIMATE_FAN_HIGH:   fs = MHI_ZMP_FAN3;  break;
      default:                          fs = MHI_ZMP_FAN_AUTO; break;
    }
    buf[7] |= fs;

    if (swing_mode == climate::CLIMATE_SWING_BOTH) {
      buf[5] |= MHI_HS_SWING | (MHI_VS_SWING & 0x02);
      buf[7] |= (MHI_VS_SWING & 0x18);
    } else if (swing_mode == climate::CLIMATE_SWING_VERTICAL) {
      buf[5] |= (MHI_VS_SWING & 0x02);
      buf[7] |= (MHI_VS_SWING & 0x18);
    } else if (swing_mode == climate::CLIMATE_SWING_HORIZONTAL) {
      buf[5] |= MHI_HS_SWING;
    }

    buf[6] = ~buf[5];
    buf[8] = ~buf[7];
    buf[10] = ~buf[9];
    auto tx = transmitter_->transmit();
    send_bytes(tx, buf, 11);
    return;
  }

  // модель ZM (19 байт)
  {
    uint8_t buf19[19] = {
      MHI_P0, MHI_P1, MHI_P2, MHI_P3_ZM, MHI_P4_ZM,
      0x90, 0x00, 0xF0, 0x00, 0xE0,
      0x00, 0x0D, 0x00, 0x10, 0x00,
      0x3F, 0x00, 0x7F, 0x00
    };
    // Power & Mode
    uint8_t op = MHI_AUTO, pm = MHI_ON;
    switch (mode) {
      case climate::CLIMATE_MODE_COOL:      op = MHI_COOL; break;
      case climate::CLIMATE_MODE_HEAT:      op = MHI_HEAT; break;
      case climate::CLIMATE_MODE_FAN_ONLY:  op = MHI_FAN;  break;
      case climate::CLIMATE_MODE_DRY:       op = MHI_DRY;  break;
      case climate::CLIMATE_MODE_OFF:       pm = MHI_OFF;  break;
      case climate::CLIMATE_MODE_HEAT_COOL: op = MHI_AUTO;break;
      default: break;
    }
    buf19[5] |= pm | op;

    // Temperature
    buf19[7] = uint8_t(roundf(clamp<float>(target_temperature, 18, 30)) - 18);

    // Fan & Vane
    uint8_t fs = MHI_ZM_FAN_AUTO;
    switch (fan_mode.value()) {
      case climate::CLIMATE_FAN_LOW:    fs = MHI_ZM_FAN1; break;
      case climate::CLIMATE_FAN_MEDIUM: fs = MHI_ZM_FAN3; break; // 3л: MED=3
      case climate::CLIMATE_FAN_HIGH:   fs = MHI_ZM_FAN4; break;
      default:                          fs = MHI_ZM_FAN_AUTO; break;
    }
    buf19[9] |= fs;
    if (swing_mode == climate::CLIMATE_SWING_BOTH)
      buf19[11] |= MHI_VS_SWING | MHI_3DAUTO_ON;
    else if (swing_mode == climate::CLIMATE_SWING_VERTICAL)
      buf19[11] |= MHI_VS_SWING;
    else if (swing_mode == climate::CLIMATE_SWING_HORIZONTAL)
      buf19[13] |= MHI_HS_SWING;

    // Clean/Silent/3D
    if (preset == climate::CLIMATE_PRESET_ECO)
      buf19[15] |= MHI_ECO_ON;
    if (preset == climate::CLIMATE_PRESET_BOOST)
      buf19[15] |= MHI_ZM_HIPOWER;
    if (preset == climate::CLIMATE_PRESET_ACTIVITY)
      buf19[11] |= MHI_3DAUTO_ON;

    // Инверсия
    buf19[6]  = ~buf19[5];
    buf19[8]  = ~buf19[7];
    buf19[10] = ~buf19[9];
    buf19[12] = ~buf19[11];
    buf19[14] = ~buf19[13];
    buf19[16] = ~buf19[15];
    buf19[18] = ~buf19[17];

    auto tx = transmitter_->transmit();
    for (int i = 0; i < 2; i++) {
      send_bytes(tx, buf19, 19);
      if (i == 0) {
        auto *d = tx.get_data();
        d->mark(MHI_BIT_MARK);
        d->space(MHI_MIN_GAP);
      }
    }
  }
}

}  // namespace mhi_multi_ir
}  // namespace esphome
