// mhi_multi_ir.cpp

#include "mhi_multi_ir.h"
#include "esphome/core/log.h"

namespace esphome {
namespace mhi_multi_ir {

static const char *TAG = "mhi_multi_ir.climate";

// ----------------------------------------------------------------------------
// Универсальная функция отправки
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
// Приём данных с пульта
// ----------------------------------------------------------------------------
bool MhiClimate::on_receive(remote_base::RemoteReceiveData data) {
  ESP_LOGD(TAG, "on_receive model=%u", unsigned(model_));

  const size_t LEN = (model_ == ZM ? 19 : 11);
  uint8_t buf[19];

  const uint8_t prefix[5] = {
    MHI_P0, MHI_P1, MHI_P2,
    static_cast<uint8_t>((model_ == ZM ? MHI_P3_ZM : MHI_P3_ZJZEPM)),
    static_cast<uint8_t>((model_ == ZM ? MHI_P4_ZM : MHI_P4_ZJZEPM))
  };

  if (!data.expect_item(MHI_HDR_MARK, MHI_HDR_SPACE))
    return false;

  for (size_t i = 0; i < LEN; i++) {
    uint8_t v = 0;
    for (int b = 0; b < 8; b++) {
      if (data.expect_item(MHI_BIT_MARK, MHI_ONE_SPACE)) {
        v |= 1 << b;
      } else if (!data.expect_item(MHI_BIT_MARK, MHI_ZERO_SPACE)) {
        return false;
      }
    }
    buf[i] = v;
  }

  for (int i = 0; i < 5; i++)
    if (buf[i] != prefix[i])
      return false;

  size_t half = LEN / 2;
  for (size_t i = 0; i < half - 5; i++)
    if (buf[5 + i] != static_cast<uint8_t>(~buf[5 + half + i]))
      return false;

  uint8_t power_mode, op_mode, fan_spd, swing_v, swing_h;
  int temperature;

  if (model_ == ZM) {
    power_mode = buf[5] & 0x08;
    op_mode = buf[5] & 0x07;
    temperature = (static_cast<uint8_t>(~buf[7]) & 0x0F) + 17;
    fan_spd = buf[9] & 0x0F;
    swing_v = buf[11] & 0xE0;
    swing_h = buf[13] & 0x0F;
  } else {
    power_mode = buf[9] & 0x08;
    op_mode = buf[9] & 0x07;
    temperature = ((static_cast<uint8_t>(~buf[9]) & 0xF0) >> 4) + 17;
    fan_spd = buf[7] & 0xE0;
    swing_v = (buf[5] & 0x02) | (buf[7] & 0x18);
    swing_h = buf[5] & 0xCC;
  }

  if (power_mode == MHI_ON) {
    switch (op_mode) {
      case MHI_COOL: this->mode = climate::CLIMATE_MODE_COOL; break;
      case MHI_HEAT: this->mode = climate::CLIMATE_MODE_HEAT; break;
      case MHI_DRY:  this->mode = climate::CLIMATE_MODE_DRY; break;
      case MHI_FAN:  this->mode = climate::CLIMATE_MODE_FAN_ONLY; break;
      default:       this->mode = climate::CLIMATE_MODE_AUTO; break;
    }
  } else {
    this->mode = climate::CLIMATE_MODE_OFF;
  }

  this->target_temperature = temperature;

  if (swing_v == MHI_VS_SWING && swing_h == MHI_HS_SWING)
    this->swing_mode = climate::CLIMATE_SWING_BOTH;
  else if (swing_v == MHI_VS_SWING)
    this->swing_mode = climate::CLIMATE_SWING_VERTICAL;
  else if (swing_h == MHI_HS_SWING)
    this->swing_mode = climate::CLIMATE_SWING_HORIZONTAL;
  else
    this->swing_mode = climate::CLIMATE_SWING_OFF;

  if (fan_spd == MHI_ZJ_FAN1 || fan_spd == MHI_ZEA_FAN1 || fan_spd == MHI_ZMP_FAN1)
    this->fan_mode = climate::CLIMATE_FAN_LOW;
  else if (fan_spd == MHI_ZJ_FAN2 || fan_spd == MHI_ZEA_FAN2 || fan_spd == MHI_ZMP_FAN2)
    this->fan_mode = climate::CLIMATE_FAN_MEDIUM;
  else if (fan_spd == MHI_ZJ_FAN3 || fan_spd == MHI_ZEA_FAN3 || fan_spd == MHI_ZMP_FAN3)
    this->fan_mode = climate::CLIMATE_FAN_HIGH;
  else
    this->fan_mode = climate::CLIMATE_FAN_AUTO;

  this->publish_state();
  return true;
}

// ----------------------------------------------------------------------------
// Передача состояния
// ----------------------------------------------------------------------------
void MhiClimate::transmit_state() {
  ESP_LOGD(TAG, "transmit_state model=%u", unsigned(this->model_));

  uint8_t power_mode = (mode == climate::CLIMATE_MODE_OFF) ? MHI_OFF : MHI_ON;
  uint8_t operation_mode = MHI_AUTO;

  switch (mode) {
    case climate::CLIMATE_MODE_COOL:     operation_mode = MHI_COOL; break;
    case climate::CLIMATE_MODE_HEAT:     operation_mode = MHI_HEAT; break;
    case climate::CLIMATE_MODE_DRY:      operation_mode = MHI_DRY; break;
    case climate::CLIMATE_MODE_FAN_ONLY: operation_mode = MHI_FAN; break;
    case climate::CLIMATE_MODE_AUTO:     operation_mode = MHI_AUTO; break;
    default:                             operation_mode = MHI_AUTO; break;
  }

  uint8_t temp = static_cast<uint8_t>(clamp<int>(roundf(this->target_temperature), 17, 30) - 17);
  uint8_t swing_v = 0, swing_h = 0;

  if (swing_mode == climate::CLIMATE_SWING_BOTH) {
    swing_v = MHI_VS_SWING;
    swing_h = MHI_HS_SWING;
  } else if (swing_mode == climate::CLIMATE_SWING_VERTICAL) {
    swing_v = MHI_VS_SWING;
  } else if (swing_mode == climate::CLIMATE_SWING_HORIZONTAL) {
    swing_h = MHI_HS_SWING;
  }

  if (model_ == ZJ || model_ == ZEA || model_ == ZMP) {
    uint8_t buf[11] = {
      MHI_P0, MHI_P1, MHI_P2, MHI_P3_ZJZEPM, MHI_P4_ZJZEPM,
      0x11, 0x00, 0x07, 0x00, 0x00, 0x00
    };

    // FAN
    uint8_t fan_speed = MHI_ZJ_FAN_AUTO;
    if (model_ == ZJ) {
      if (fan_mode == climate::CLIMATE_FAN_LOW) fan_speed = MHI_ZJ_FAN1;
      else if (fan_mode == climate::CLIMATE_FAN_MEDIUM) fan_speed = MHI_ZJ_FAN2;
      else if (fan_mode == climate::CLIMATE_FAN_HIGH) fan_speed = MHI_ZJ_FAN3;
      else fan_speed = MHI_ZJ_FAN_AUTO;
    } else if (model_ == ZEA) {
      if (fan_mode == climate::CLIMATE_FAN_LOW) fan_speed = MHI_ZEA_FAN1;
      else if (fan_mode == climate::CLIMATE_FAN_MEDIUM) fan_speed = MHI_ZEA_FAN2;
      else if (fan_mode == climate::CLIMATE_FAN_HIGH) fan_speed = MHI_ZEA_FAN3;
      else if (fan_mode == climate::CLIMATE_FAN_MIDDLE) fan_speed = MHI_ZEA_FAN4;
      else fan_speed = MHI_ZEA_FAN_AUTO;
    } else if (model_ == ZMP) {
      if (fan_mode == climate::CLIMATE_FAN_LOW) fan_speed = MHI_ZMP_FAN1;
      else if (fan_mode == climate::CLIMATE_FAN_MEDIUM) fan_speed = MHI_ZMP_FAN2;
      else if (fan_mode == climate::CLIMATE_FAN_HIGH) fan_speed = MHI_ZMP_FAN3;
      else fan_speed = MHI_ZMP_FAN_AUTO;
    }

    buf[7] = fan_speed;
    buf[5] |= swing_h;
    buf[7] |= swing_v;
    buf[9] |= power_mode | operation_mode | ((~(temp << 4)) & 0xF0);
    buf[6]  = ~buf[5];
    buf[8]  = ~buf[7];
    buf[10] = ~buf[9];

    auto tx = this->transmitter_->transmit();
    send_bytes(tx, buf, 11);
    return;
  }

  if (model_ == ZM) {
    uint8_t buf[19] = {
      MHI_P0, MHI_P1, MHI_P2, MHI_P3_ZM, MHI_P4_ZM,
      0x90, 0x00, 0xF0, 0x00, 0xE0,
      0x00, 0x0D, 0x00, 0x10, 0x00,
      0x3F, 0x00, 0x7F, 0x00
    };

    buf[5] |= power_mode | operation_mode;
    buf[7] = ~temp & 0x0F;
    uint8_t fan_speed = MHI_ZM_FAN_AUTO;
    if (fan_mode == climate::CLIMATE_FAN_LOW) fan_speed = MHI_ZM_FAN1;
    else if (fan_mode == climate::CLIMATE_FAN_MEDIUM) fan_speed = MHI_ZM_FAN3;
    else if (fan_mode == climate::CLIMATE_FAN_HIGH) fan_speed = MHI_ZM_FAN4;

    buf[9] |= fan_speed;
    buf[11] |= swing_v;
    buf[13] |= swing_h;

    buf[6]  = ~buf[5];
    buf[8]  = ~buf[7];
    buf[10] = ~buf[9];
    buf[12] = ~buf[11];
    buf[14] = ~buf[13];
    buf[16] = ~buf[15];
    buf[18] = ~buf[17];

    auto tx = this->transmitter_->transmit();
    for (int r = 0; r < 2; r++) {
      send_bytes(tx, buf, 19);
      if (r == 0) {
        auto *d = tx.get_data();
        d->mark(MHI_BIT_MARK);
        d->space(MHI_MIN_GAP);
      }
    }
  }
}

}  // namespace mhi_multi_ir
}  // namespace esphome
