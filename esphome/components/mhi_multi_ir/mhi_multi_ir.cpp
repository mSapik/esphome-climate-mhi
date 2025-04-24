#include "mhi_multi_ir.h"
#include "esphome/core/log.h"

namespace esphome {
namespace mhi_multi_ir {

static const char *TAG = "mhi_multi_ir.climate";

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

bool MhiClimate::on_receive(remote_base::RemoteReceiveData data) {
  ESP_LOGD(TAG, "on_receive model=%u", unsigned(model_));

  const size_t LEN = (model_ == ZM ? 19 : 11);
  uint8_t buf[19] = {};
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
    temperature = (~buf[7] & 0x0F) + 17;
    fan_spd = buf[9] & 0x0F;
    swing_v = buf[11] & 0xE0;
    swing_h = buf[13] & 0x0F;
  } else {
    power_mode = buf[9] & 0x08;
    op_mode = buf[9] & 0x07;
    temperature = ((~buf[9] & 0xF0) >> 4) + 17;
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
      case MHI_AUTO: this->mode = climate::CLIMATE_MODE_HEAT_COOL; break;
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

  if (model_ == ZJ) {
    if (fan_spd == MHI_ZJ_FAN1) this->fan_mode = climate::CLIMATE_FAN_LOW;
    else if (fan_spd == MHI_ZJ_FAN2) this->fan_mode = climate::CLIMATE_FAN_MEDIUM;
    else if (fan_spd == MHI_ZJ_FAN3) this->fan_mode = climate::CLIMATE_FAN_HIGH;
    else if (fan_spd == MHI_ZJ_FAN_AUTO) {
      switch (swing_h) {
        case MHI_HS_MIDDLE: this->fan_mode = climate::CLIMATE_FAN_MIDDLE; break;
        case MHI_HS_RIGHTLEFT: this->fan_mode = climate::CLIMATE_FAN_FOCUS; break;
        case MHI_HS_LEFTRIGHT: this->fan_mode = climate::CLIMATE_FAN_DIFFUSE; break;
        default: this->fan_mode = climate::CLIMATE_FAN_AUTO; break;
      }
    } else this->fan_mode = climate::CLIMATE_FAN_AUTO;
  } else if (model_ == ZM) {
    if (fan_spd == MHI_ZM_FAN1) this->fan_mode = climate::CLIMATE_FAN_LOW;
    else if (fan_spd == MHI_ZM_FAN3) this->fan_mode = climate::CLIMATE_FAN_MEDIUM;
    else if (fan_spd == MHI_ZM_FAN4) this->fan_mode = climate::CLIMATE_FAN_HIGH;
    else if (fan_spd == MHI_ZM_HIPOWER) this->preset = climate::CLIMATE_PRESET_BOOST;
    else this->fan_mode = climate::CLIMATE_FAN_AUTO;
  }

  this->publish_state();
  return true;
}

void MhiClimate::transmit_state() {
  ESP_LOGD(TAG, "transmit_state model=%u", unsigned(this->model_));

  uint8_t power_mode = (mode == climate::CLIMATE_MODE_OFF) ? MHI_OFF : MHI_ON;
  uint8_t operation_mode = MHI_AUTO;

  switch (mode) {
    case climate::CLIMATE_MODE_COOL:     operation_mode = MHI_COOL; break;
    case climate::CLIMATE_MODE_HEAT:     operation_mode = MHI_HEAT; break;
    case climate::CLIMATE_MODE_DRY:      operation_mode = MHI_DRY; break;
    case climate::CLIMATE_MODE_FAN_ONLY: operation_mode = MHI_FAN; break;
    case climate::CLIMATE_MODE_HEAT_COOL: operation_mode = MHI_AUTO; break;
    default: break;
  }

  uint8_t temp = static_cast<uint8_t>(clamp<int>(roundf(this->target_temperature), 17, 30) - 17);
  uint8_t swing_v = MHI_VS_STOP, swing_h = MHI_HS_STOP;
  if (swing_mode == climate::CLIMATE_SWING_BOTH) {
    swing_v = MHI_VS_SWING;
    swing_h = MHI_HS_SWING;
  } else if (swing_mode == climate::CLIMATE_SWING_VERTICAL) {
    swing_v = MHI_VS_SWING;
  } else if (swing_mode == climate::CLIMATE_SWING_HORIZONTAL) {
    swing_h = MHI_HS_SWING;
  }

  uint8_t fan_speed = 0x00;
  uint8_t eco_mode = MHI_ECO_OFF;
  uint8_t clean_mode = (model_ == ZM ? MHI_ZM_CLEAN_OFF : MHI_ZJ_CLEAN_OFF);
  uint8_t preset_mode = this->preset.value_or(climate::CLIMATE_PRESET_NONE);
  uint8_t _3d_auto = MHI_3DAUTO_OFF;
  uint8_t silent_mode = MHI_SILENT_OFF;

  if (preset_mode == climate::CLIMATE_PRESET_ECO) {
    eco_mode = MHI_ECO_ON;
    fan_speed = (model_ == ZM ? MHI_ZM_FAN2 : MHI_ZJ_FAN2);
  } else if (preset_mode == climate::CLIMATE_PRESET_BOOST) {
    fan_speed = (model_ == ZM ? MHI_ZM_HIPOWER : MHI_ZJ_HIPOWER);
  } else if (preset_mode == climate::CLIMATE_PRESET_ACTIVITY) {
    _3d_auto = MHI_3DAUTO_ON;
  }

  if (model_ == ZM) {
    uint8_t buf[19] = {
      MHI_P0, MHI_P1, MHI_P2, MHI_P3_ZM, MHI_P4_ZM,
      0x90 | power_mode | operation_mode | clean_mode,
      0x00,
      static_cast<uint8_t>(~temp & 0x0F),
      0x00,
      fan_speed | eco_mode,
      0x00,
      swing_v | _3d_auto,
      0x00,
      swing_v | swing_h,
      0x00,
      silent_mode,
      0x00,
      0x7F,
      0x00
    };

    buf[6] = ~buf[5];
    buf[8] = ~buf[7];
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
  } else {
    uint8_t buf[11] = {
      MHI_P0, MHI_P1, MHI_P2, MHI_P3_ZJZEPM, MHI_P4_ZJZEPM,
      static_cast<uint8_t>(swing_h | (swing_v & 0x02) | clean_mode),
      0x00,
      static_cast<uint8_t>(fan_speed | (swing_v & 0x18)),
      0x00,
      static_cast<uint8_t>(operation_mode | power_mode | ((~(temp << 4)) & 0xF0)),
      0x00
    };

    buf[6] = ~buf[5];
    buf[8] = ~buf[7];
    buf[10] = ~buf[9];

    auto tx = this->transmitter_->transmit();
    send_bytes(tx, buf, 11);
  }
}

}  // namespace mhi_multi_ir
}  // namespace esphome
