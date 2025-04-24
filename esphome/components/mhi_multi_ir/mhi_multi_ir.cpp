// mhi_multi_ir.cpp

#include "mhi_multi_ir.h"
#include "esphome/core/log.h"
#include "esphome/components/remote_base/remote_base.h"

namespace esphome {
namespace mhi_multi_ir {

static const char *TAG = "mhi_multi_ir.climate";

// Универсальная отправка буфера
template<typename TransmitCall>
static void send_bytes(TransmitCall &tx, const uint8_t buf[], size_t len) {
  auto *data = tx.get_data();
  data->set_carrier_frequency(38000);
  data->mark(MHI_HDR_MARK);
  data->space(MHI_HDR_SPACE);
  for (size_t i = 0; i < len; i++) {
    for (uint8_t bit = 0; bit < 8; bit++) {
      data->mark(MHI_BIT_MARK);
      bool one = buf[i] & (1 << bit);
      data->space(one ? MHI_ONE_SPACE : MHI_ZERO_SPACE);
    }
  }
  data->mark(MHI_BIT_MARK);
  data->space(0);
  tx.perform();
}

bool MhiClimate::on_receive(remote_base::RemoteReceiveData data) {
  ESP_LOGD(TAG, "on_receive model=%u", unsigned(this->model_));
  // Длина кадра и префикс для модели
  const size_t LEN = (this->model_ == ZM ? 19 : 11);
  uint8_t buf[19];
  uint8_t prefix[5] = {
    MHI_P0, MHI_P1, MHI_P2,
    (this->model_ == ZM ? MHI_P3_ZM  : MHI_P3_ZJZEPM),
    (this->model_ == ZM ? MHI_P4_ZM  : MHI_P4_ZJZEPM)
  };
  // Чтение raw
  if (!data.expect_item(MHI_HDR_MARK, MHI_HDR_SPACE)) return false;
  for (size_t i = 0; i < LEN; i++) {
    uint8_t v = 0;
    for (int b = 0; b < 8; b++) {
      if (data.expect_item(MHI_BIT_MARK, MHI_ONE_SPACE)) v |= 1 << b;
      else if (!data.expect_item(MHI_BIT_MARK, MHI_ZERO_SPACE)) return false;
    }
    buf[i] = v;
  }
  // Проверка префикса
  for (int i = 0; i < 5; i++) if (buf[i] != prefix[i]) return false;
  // Инверсии (вторая половина)
  size_t half = LEN / 2;
  for (size_t i = 0; i < half - 5; i++)
    if (buf[5 + i] != uint8_t(~buf[5 + half + i])) return false;

  // Теперь распакуем в зависимости от модели:
  switch (this->model_) {
    case ZJ: {
      bool on = (buf[9] & 0x08) == MHI_ON;
      uint8_t md       = buf[9] & 0x07;
      uint8_t rawtemp  = uint8_t((~buf[9] & 0xF0) >> 4);
      uint8_t fansraw  = buf[7] & 0xE0;
      uint8_t swingv   = (buf[5] & 0x02) | (buf[7] & 0x18);
      uint8_t swingh   = buf[5] & 0xCC;
      this->mode = !on ? climate::CLIMATE_MODE_OFF
        : (md == MHI_COOL ? climate::CLIMATE_MODE_COOL
        : md == MHI_HEAT ? climate::CLIMATE_MODE_HEAT
        : md == MHI_DRY  ? climate::CLIMATE_MODE_DRY
        : md == MHI_FAN  ? climate::CLIMATE_MODE_FAN_ONLY
                        : climate::CLIMATE_MODE_AUTO);
      this->target_temperature = rawtemp + 17;
      if      (swingv == MHI_VS_SWING && swingh == MHI_HS_SWING) this->swing_mode = climate::CLIMATE_SWING_BOTH;
      else if (swingv == MHI_VS_SWING)                          this->swing_mode = climate::CLIMATE_SWING_VERTICAL;
      else if (swingh == MHI_HS_SWING)                          this->swing_mode = climate::CLIMATE_SWING_HORIZONTAL;
      else                                                      this->swing_mode = climate::CLIMATE_SWING_OFF;
      if      (fansraw == MHI_ZJ_FAN1) this->fan_mode = climate::CLIMATE_FAN_LOW;
      else if (fansraw == MHI_ZJ_FAN2) this->fan_mode = climate::CLIMATE_FAN_MEDIUM;
      else if (fansraw == MHI_ZJ_FAN3) this->fan_mode = climate::CLIMATE_FAN_HIGH;
      else {
        this->fan_mode = climate::CLIMATE_FAN_AUTO;
        if      (swingh == MHI_HS_MIDDLE)    this->fan_mode = climate::CLIMATE_FAN_MIDDLE;
        else if (swingh == MHI_HS_RIGHTLEFT) this->fan_mode = climate::CLIMATE_FAN_FOCUS;
        else if (swingh == MHI_HS_LEFTRIGHT) this->fan_mode = climate::CLIMATE_FAN_DIFFUSE;
      }
      break;
    }
    case ZEA: {
      bool on = (buf[9] & 0x08) == MHI_ON;
      uint8_t md      = buf[9] & 0x07;
      uint8_t rawtemp = uint8_t((~buf[9] & 0xF0) >> 4);
      uint8_t fansraw = buf[7] & 0xE0;
      uint8_t swingv  = (buf[5] & 0x02) | (buf[7] & 0x18);
      uint8_t swingh  = buf[5] & 0xCC;
      this->mode = !on ? climate::CLIMATE_MODE_OFF
        : (md == MHI_COOL ? climate::CLIMATE_MODE_COOL
        : md == MHI_HEAT ? climate::CLIMATE_MODE_HEAT
        : md == MHI_DRY  ? climate::CLIMATE_MODE_DRY
        : md == MHI_FAN  ? climate::CLIMATE_MODE_FAN_ONLY
                        : climate::CLIMATE_MODE_AUTO);
      this->target_temperature = rawtemp + 17;
      if      (swingv == MHI_VS_SWING && swingh == MHI_HS_SWING) this->swing_mode = climate::CLIMATE_SWING_BOTH;
      else if (swingv == MHI_VS_SWING)                          this->swing_mode = climate::CLIMATE_SWING_VERTICAL;
      else if (swingh == MHI_HS_SWING)                          this->swing_mode = climate::CLIMATE_SWING_HORIZONTAL;
      else                                                      this->swing_mode = climate::CLIMATE_SWING_OFF;
      if      (fansraw == MHI_ZEA_FAN1) this->fan_mode = climate::CLIMATE_FAN_LOW;
      else if (fansraw == MHI_ZEA_FAN2) this->fan_mode = climate::CLIMATE_FAN_MEDIUM;
      else if (fansraw == MHI_ZEA_FAN3) this->fan_mode = climate::CLIMATE_FAN_HIGH;
      else if (fansraw == MHI_ZEA_FAN4) this->fan_mode = climate::CLIMATE_FAN_MIDDLE;
      else                             this->fan_mode = climate::CLIMATE_FAN_AUTO;
      break;
    }
    case ZMP: {
      bool on = (buf[9] & 0x08) == MHI_ON;
      uint8_t md      = buf[9] & 0x07;
      uint8_t rawtemp = uint8_t((~buf[9] & 0xF0) >> 4);
      uint8_t fansraw = buf[7] & 0xE0;
      uint8_t swingv  = (buf[5] & 0x02) | (buf[7] & 0x18);
      this->mode = !on ? climate::CLIMATE_MODE_OFF
        : (md == MHI_COOL ? climate::CLIMATE_MODE_COOL
        : md == MHI_HEAT ? climate::CLIMATE_MODE_HEAT
        : md == MHI_DRY  ? climate::CLIMATE_MODE_DRY
        : md == MHI_FAN  ? climate::CLIMATE_MODE_FAN_ONLY
                        : climate::CLIMATE_MODE_AUTO);
      this->target_temperature = rawtemp + 17;
      this->swing_mode = (swingv == MHI_VS_SWING
                         ? climate::CLIMATE_SWING_VERTICAL
                         : climate::CLIMATE_SWING_OFF);
      if      (fansraw == MHI_ZMP_FAN1) this->fan_mode = climate::CLIMATE_FAN_LOW;
      else if (fansraw == MHI_ZMP_FAN2) this->fan_mode = climate::CLIMATE_FAN_MEDIUM;
      else if (fansraw == MHI_ZMP_FAN3) this->fan_mode = climate::CLIMATE_FAN_HIGH;
      else                              this->fan_mode = climate::CLIMATE_FAN_AUTO;
      break;
    }
    case ZM: {
      bool on = (buf[5] & 0x08) == MHI_ON;
      uint8_t md       = buf[5] & 0x07;
      uint8_t rawtemp  = uint8_t(~buf[7] & 0x0F);
      uint8_t fansraw  = buf[9] & 0x0F;
      uint8_t swingv   = buf[11] & 0xE0;
      uint8_t swingh   = buf[13] & 0x0F;
      this->mode = !on ? climate::CLIMATE_MODE_OFF
        : (md == MHI_COOL ? climate::CLIMATE_MODE_COOL
        : md == MHI_HEAT ? climate::CLIMATE_MODE_HEAT
        : md == MHI_DRY  ? climate::CLIMATE_MODE_DRY
        : md == MHI_FAN  ? climate::CLIMATE_MODE_FAN_ONLY
                        : climate::CLIMATE_MODE_HEAT_COOL);
      this->target_temperature = rawtemp + 17;
      if      (swingv == MHI_VS_SWING && swingh == MHI_HS_SWING) this->swing_mode = climate::CLIMATE_SWING_BOTH;
      else if (swingv == MHI_VS_SWING)                          this->swing_mode = climate::CLIMATE_SWING_VERTICAL;
      else if (swingh == MHI_HS_SWING)                          this->swing_mode = climate::CLIMATE_SWING_HORIZONTAL;
      else                                                      this->swing_mode = climate::CLIMATE_SWING_OFF;
      switch (fansraw) {
        case 1:
          this->fan_mode = climate::CLIMATE_FAN_LOW; break;
        case 2:
          this->fan_mode = (this->fan_levels_ == FAN_LEVELS_3
                            ? climate::CLIMATE_FAN_MEDIUM
                            : climate::CLIMATE_FAN_MIDDLE);
          break;
        case 3:
          this->fan_mode = (this->fan_levels_ == FAN_LEVELS_3
                            ? climate::CLIMATE_FAN_HIGH
                            : climate::CLIMATE_FAN_MEDIUM);
          break;
        case 4:
          this->fan_mode = climate::CLIMATE_FAN_HIGH; break;
        default:
          this->fan_mode = climate::CLIMATE_FAN_AUTO; break;
      }
      break;
    }
  }

  ESP_LOGD(TAG, "Received state: mode=%u temp=%.1f fan=%u swing=%u",
           this->mode, this->target_temperature,
           this->fan_mode.value(), this->swing_mode);
  this->publish_state();
  return true;
}

void MhiClimate::transmit_state() {
  ESP_LOGD(TAG, "transmit_state model=%u", unsigned(this->model_));
  // Каждый блок почти дословно скопирован из оригинальных реализаций,
  // только поправлены сдвиги и приведения float->uint8_t.

  if (this->model_ == ZJ) {
    uint8_t buf[11] = {
      MHI_P0, MHI_P1, MHI_P2,
      MHI_P3_ZJZEPM, MHI_P4_ZJZEPM,
      0x11, 0x00, 0x07, 0x00, 0x00, 0x00
    };
    // Power+mode
    switch (this->mode) {
      case climate::CLIMATE_MODE_COOL:
        buf[9] = MHI_COOL | MHI_ON; buf[7] |= MHI_VS_UP; break;
      case climate::CLIMATE_MODE_HEAT:
        buf[9] = MHI_HEAT | MHI_ON; buf[7] |= MHI_VS_DOWN; break;
      case climate::CLIMATE_MODE_DRY:
        buf[9] = MHI_DRY  | MHI_ON; buf[7] |= MHI_VS_MIDDLE; break;
      case climate::CLIMATE_MODE_FAN_ONLY:
        buf[9] = MHI_FAN  | MHI_ON; buf[7] |= MHI_VS_MIDDLE; break;
      case climate::CLIMATE_MODE_AUTO:
        buf[9] = MHI_AUTO | MHI_ON;
        buf[7] |= MHI_VS_MIDDLE;
        buf[5] |= MHI_HS_3DAUTO;
        break;
      default:
        buf[9] = MHI_OFF;
    }
    // Температура
    uint8_t t = uint8_t(this->target_temperature);
    if (t > 17 && t < 31)
      buf[9] |= ((~uint8_t((t - 17) << 4)) & 0xF0);
    // Fan & swing
    switch (this->fan_mode.value()) {
      case climate::CLIMATE_FAN_LOW:
        buf[7] |= MHI_ZJ_FAN1; break;
      case climate::CLIMATE_FAN_MEDIUM:
        buf[7] |= MHI_ZJ_FAN2; break;
      case climate::CLIMATE_FAN_HIGH:
        buf[7] |= MHI_ZJ_FAN3; break;
      case climate::CLIMATE_FAN_MIDDLE:
        buf[7] |= MHI_ZJ_FAN_AUTO;
        buf[5] |= MHI_HS_MIDDLE;
        buf[7] |= MHI_VS_SWING;
        break;
      case climate::CLIMATE_FAN_FOCUS:
        buf[7] |= MHI_ZJ_HIPOWER;
        buf[5] |= MHI_HS_RIGHTLEFT;
        buf[7] |= MHI_VS_SWING;
        break;
      case climate::CLIMATE_FAN_DIFFUSE:
        buf[7] |= MHI_ZJ_HIPOWER;
        buf[5] |= MHI_HS_LEFTRIGHT;
        buf[7] |= MHI_VS_SWING;
        break;
      default:
        buf[7] |= MHI_ZJ_FAN_AUTO;
    }
    // Инверсии
    buf[6]  = ~buf[5];
    buf[8]  = ~buf[7];
    buf[10] = ~buf[9];
    auto tx = this->transmitter_->transmit();
    send_bytes(tx, buf, 11);
  }
  else if (this->model_ == ZEA) {
    uint8_t buf[11] = {
      MHI_P0, MHI_P1, MHI_P2,
      MHI_P3_ZJZEPM, MHI_P4_ZJZEPM,
      0xDF, 0x20, 0x07, 0x00, 0x00, 0x00
    };
    switch (this->mode) {
      case climate::CLIMATE_MODE_COOL: buf[9] = MHI_COOL | MHI_ON; break;
      case climate::CLIMATE_MODE_HEAT: buf[9] = MHI_HEAT | MHI_ON; break;
      case climate::CLIMATE_MODE_DRY:  buf[9] = MHI_DRY  | MHI_ON; break;
      case climate::CLIMATE_MODE_FAN_ONLY: buf[9] = MHI_FAN  | MHI_ON; break;
      case climate::CLIMATE_MODE_AUTO:    buf[9] = MHI_AUTO | MHI_ON; break;
      default: buf[9] = MHI_OFF;
    }
    uint8_t t = uint8_t(this->target_temperature);
    if (t > 17 && t < 31)
      buf[9] |= ((~uint8_t((t - 17) << 4)) & 0xF0);
    switch (this->fan_mode.value()) {
      case climate::CLIMATE_FAN_LOW:    buf[7] |= MHI_ZEA_FAN1; break;
      case climate::CLIMATE_FAN_MEDIUM: buf[7] |= MHI_ZEA_FAN2; break;
      case climate::CLIMATE_FAN_HIGH:   buf[7] |= MHI_ZEA_FAN3; break;
      case climate::CLIMATE_FAN_MIDDLE: buf[7] |= MHI_ZEA_FAN4; break;
      default:                          buf[7] |= MHI_ZEA_FAN_AUTO;
    }
    if (this->swing_mode == climate::CLIMATE_SWING_VERTICAL ||
        this->swing_mode == climate::CLIMATE_SWING_BOTH)
      buf[5] |= MHI_VS_SWING;
    if (this->swing_mode == climate::CLIMATE_SWING_HORIZONTAL ||
        this->swing_mode == climate::CLIMATE_SWING_BOTH)
      buf[5] |= MHI_HS_SWING;
    buf[6]  = ~buf[5];
    buf[8]  = ~buf[7];
    buf[10] = ~buf[9];
    auto tx = this->transmitter_->transmit();
    send_bytes(tx, buf, 11);
  }
  else if (this->model_ == ZMP) {
    uint8_t buf[11] = {
      MHI_P0, MHI_P1, MHI_P2,
      MHI_P3_ZJZEPM, MHI_P4_ZJZEPM,
      0x11, 0x00, 0x07, 0x00, 0x00, 0x00
    };
    switch (this->mode) {
      case climate::CLIMATE_MODE_COOL: buf[9] = MHI_COOL | MHI_ON; break;
      case climate::CLIMATE_MODE_HEAT: buf[9] = MHI_HEAT | MHI_ON; break;
      case climate::CLIMATE_MODE_DRY:  buf[9] = MHI_DRY  | MHI_ON; break;
      case climate::CLIMATE_MODE_FAN_ONLY: buf[9] = MHI_FAN  | MHI_ON; break;
      case climate::CLIMATE_MODE_AUTO:    buf[9] = MHI_AUTO | MHI_ON; break;
      default: buf[9] = MHI_OFF;
    }
    uint8_t t = uint8_t(this->target_temperature);
    if (t > 17 && t < 31)
      buf[9] |= ((~uint8_t((t - 17) << 4)) & 0xF0);
    switch (this->fan_mode.value()) {
      case climate::CLIMATE_FAN_LOW:    buf[7] |= MHI_ZMP_FAN1; break;
      case climate::CLIMATE_FAN_MEDIUM: buf[7] |= MHI_ZMP_FAN2; break;
      case climate::CLIMATE_FAN_HIGH:   buf[7] |= MHI_ZMP_FAN3; break;
      default:                          buf[7] |= MHI_ZMP_FAN_AUTO;
    }
    if (this->swing_mode == climate::CLIMATE_SWING_VERTICAL ||
        this->swing_mode == climate::CLIMATE_SWING_BOTH)
      buf[5] |= MHI_VS_SWING;
    buf[6]  = ~buf[5];
    buf[8]  = ~buf[7];
    buf[10] = ~buf[9];
    auto tx = this->transmitter_->transmit();
    send_bytes(tx, buf, 11);
  }
  else {  // ZM
    uint8_t buf[19] = {
      MHI_P0, MHI_P1, MHI_P2, MHI_P3_ZM, MHI_P4_ZM,
      0x90, 0x00, 0xF0, 0x00, 0xE0,
      0x00, 0x0D, 0x00, 0x10, 0x00,
      0x3F, 0x00, 0x7F, 0x00
    };
    switch (this->mode) {
      case climate::CLIMATE_MODE_HEAT:
        buf[6] = MHI_HEAT | MHI_ON; buf[8] = MHI_ZMP_MODE_MAINT; break;
      case climate::CLIMATE_MODE_COOL:
        buf[6] = MHI_COOL | MHI_ON; buf[8] = MHI_ZMP_MODE_MAINT; break;
      case climate::CLIMATE_MODE_DRY:
        buf[6] = MHI_DRY  | MHI_ON; buf[8] = MHI_ZMP_MODE_MAINT; break;
      case climate::CLIMATE_MODE_FAN_ONLY:
        buf[6] = MHI_ZMP_MODE_FAN | MHI_ON; buf[8] = MHI_ZMP_MODE_MAINT; buf[7]=0; break;
      case climate::CLIMATE_MODE_HEAT_COOL:
        buf[6] = MHI_AUTO | MHI_ON; buf[8] = MHI_ZMP_MODE_MAINT; break;
      default:
        buf[5] = MHI_OFF;
    }
    uint8_t t = uint8_t(this->target_temperature);
    if (t > 17 && t < 31)
      buf[7] = (t - 17) & 0x0F;
    if (this->swing_mode == climate::CLIMATE_SWING_VERTICAL ||
        this->swing_mode == climate::CLIMATE_SWING_BOTH)
      buf[11] |= MHI_VS_SWING;
    switch (this->fan_mode.value()) {
      case climate::CLIMATE_FAN_LOW:    buf[9] = MHI_ZM_FAN1; break;
      case climate::CLIMATE_FAN_MEDIUM: buf[9] = MHI_ZM_FAN2; break;
      case climate::CLIMATE_FAN_HIGH:   buf[9] = MHI_ZM_FAN3; break;
      case climate::CLIMATE_FAN_MIDDLE: buf[9] = MHI_ZM_FAN4; break;
      default:                          buf[9] = MHI_ZM_FAN_AUTO;
    }
    // инверсии
    buf[6]  = ~buf[5]; buf[8]  = ~buf[7]; buf[10] = ~buf[9];
    buf[12] = ~buf[11]; buf[14] = ~buf[13]; buf[16] = ~buf[15]; buf[18] = ~buf[17];
    // checksum в buf[17]
    for (int i = 0; i < 17; i++) buf[17] += buf[i];
    auto tx = this->transmitter_->transmit();
    auto *d = tx.get_data();
    d->set_carrier_frequency(38000);
    for (int r = 0; r < 2; r++) {
      d->mark(MHI_HDR_MARK); d->space(MHI_HDR_SPACE);
      for (uint8_t b : buf) {
        for (int bit = 0; bit < 8; bit++) {
          d->mark(MHI_BIT_MARK);
          bool one = b & (1 << bit);
          d->space(one ? MHI_ONE_SPACE : MHI_ZERO_SPACE);
        }
      }
      if (r == 0) { d->mark(MHI_BIT_MARK); d->space(MHI_MIN_GAP); }
    }
    d->mark(MHI_BIT_MARK);
    tx.perform();
  }
}

}  // namespace mhi_multi_ir
}  // namespace esphome
