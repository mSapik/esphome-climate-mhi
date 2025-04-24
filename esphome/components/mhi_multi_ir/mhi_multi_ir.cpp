// mhi_multi_ir.cpp

#include "mhi_multi_ir.h"
#include "esphome/core/log.h"

namespace esphome {
namespace mhi_multi_ir {

static const char *TAG = "mhi_multi_ir.climate";

// Вспомогательная функция чтения кадра IR
static bool read_bytes(remote_base::RemoteReceiveData &data,
                       const uint8_t prefix[], size_t prefix_len,
                       uint8_t out[], size_t len) {
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
  // Проверка статического префикса
  for (size_t i = 0; i < prefix_len; i++)
    if (out[i] != prefix[i])
      return false;
  // Проверка инверсий: вторая половина = побитовое НЕ первой
  size_t half = len / 2;
  for (size_t i = 0; i < half - prefix_len; i++)
    if (out[prefix_len + i] != uint8_t(~out[prefix_len + half + i]))
      return false;
  return true;
}

// Отправка буфера по IR
void MhiClimate::send_bytes(remote_base::RemoteTransmitData *txd,
                            const uint8_t buf[], size_t len) {
  auto *data = txd->get_data();
  data->set_carrier_frequency(38000);
  // Заголовок
  data->mark(MHI_HDR_MARK);
  data->space(MHI_HDR_SPACE);
  // Данные
  for (size_t i = 0; i < len; i++) {
    for (uint8_t bit = 0; bit < 8; bit++) {
      data->mark(MHI_BIT_MARK);
      bool one = buf[i] & (1 << bit);
      data->space(one ? MHI_ONE_SPACE : MHI_ZERO_SPACE);
    }
  }
  // Финальный бит
  data->mark(MHI_BIT_MARK);
  data->space(0);
  // Выполнить
  txd->perform();
}

bool MhiClimate::on_receive(remote_base::RemoteReceiveData data) {
  ESP_LOGD(TAG, "on_receive model=%u", unsigned(this->model_));
  // Длина кадра
  const size_t len = (this->model_ == ZM ? 19 : 11);
  uint8_t buf[19];
  // Собираем префикс
  uint8_t prefix[5] = {
    MHI_P0, MHI_P1, MHI_P2,
    (this->model_ == ZM ? MHI_P3_ZM : MHI_P3_ZJZEPM),
    (this->model_ == ZM ? MHI_P4_ZM : MHI_P4_ZJZEPM)
  };
  if (!read_bytes(data, prefix, 5, buf, len))
    return false;

  // Распаковываем по модели
  switch (this->model_) {

    case ZJ: {
      auto pwr = buf[9] & 0x08;
      auto md  = buf[9] & 0x07;
      float tmp = ((~buf[9] & 0xF0) >> 4) + 17;
      uint8_t fan = buf[7] & 0xE0;
      uint8_t sv  = (buf[5] & 0x02) | (buf[7] & 0x18);
      uint8_t sh  = buf[5] & 0xCC;

      // Mode
      if (pwr != MHI_ON) this->mode = climate::CLIMATE_MODE_OFF;
      else switch (md) {
        case MHI_COOL: this->mode = climate::CLIMATE_MODE_COOL; break;
        case MHI_HEAT: this->mode = climate::CLIMATE_MODE_HEAT; break;
        case MHI_DRY:  this->mode = climate::CLIMATE_MODE_DRY; break;
        case MHI_FAN:  this->mode = climate::CLIMATE_MODE_FAN_ONLY; break;
        default:       this->mode = climate::CLIMATE_MODE_AUTO; break;
      }
      this->target_temperature = tmp;

      // Swing
      if      (sv == MHI_VS_SWING && sh == MHI_HS_SWING) this->swing_mode = climate::CLIMATE_SWING_BOTH;
      else if (sv == MHI_VS_SWING)                      this->swing_mode = climate::CLIMATE_SWING_VERTICAL;
      else if (sh == MHI_HS_SWING)                      this->swing_mode = climate::CLIMATE_SWING_HORIZONTAL;
      else                                              this->swing_mode = climate::CLIMATE_SWING_OFF;

      // Fan
      if      (fan == MHI_ZJ_FAN1) this->fan_mode = climate::CLIMATE_FAN_LOW;
      else if (fan == MHI_ZJ_FAN2) this->fan_mode = climate::CLIMATE_FAN_MEDIUM;
      else if (fan == MHI_ZJ_FAN3) this->fan_mode = climate::CLIMATE_FAN_HIGH;
      else {
        this->fan_mode = climate::CLIMATE_FAN_AUTO;
        if      (sh == MHI_HS_MIDDLE)    this->fan_mode = climate::CLIMATE_FAN_MIDDLE;
        else if (sh == MHI_HS_RIGHTLEFT) this->fan_mode = climate::CLIMATE_FAN_FOCUS;
        else if (sh == MHI_HS_LEFTRIGHT) this->fan_mode = climate::CLIMATE_FAN_DIFFUSE;
      }
      break;
    }

    case ZEA: {
      auto pwr = buf[9] & 0x08;
      auto md  = buf[9] & 0x07;
      float tmp = ((~buf[9] & 0xF0) >> 4) + 17;
      uint8_t fan = buf[7] & 0xE0;
      uint8_t sv  = (buf[5] & 0x02) | (buf[7] & 0x18);
      uint8_t sh  = buf[5] & 0xCC;

      if (pwr != MHI_ON) this->mode = climate::CLIMATE_MODE_OFF;
      else switch (md) {
        case MHI_COOL: this->mode = climate::CLIMATE_MODE_COOL; break;
        case MHI_HEAT: this->mode = climate::CLIMATE_MODE_HEAT; break;
        case MHI_DRY:  this->mode = climate::CLIMATE_MODE_DRY; break;
        case MHI_FAN:  this->mode = climate::CLIMATE_MODE_FAN_ONLY; break;
        default:       this->mode = climate::CLIMATE_MODE_AUTO; break;
      }
      this->target_temperature = tmp;

      if      (sv == MHI_VS_SWING && sh == MHI_HS_SWING) this->swing_mode = climate::CLIMATE_SWING_BOTH;
      else if (sv == MHI_VS_SWING)                      this->swing_mode = climate::CLIMATE_SWING_VERTICAL;
      else if (sh == MHI_HS_SWING)                      this->swing_mode = climate::CLIMATE_SWING_HORIZONTAL;
      else                                              this->swing_mode = climate::CLIMATE_SWING_OFF;

      if      (fan == MHI_ZEA_FAN1) this->fan_mode = climate::CLIMATE_FAN_LOW;
      else if (fan == MHI_ZEA_FAN2) this->fan_mode = climate::CLIMATE_FAN_MEDIUM;
      else if (fan == MHI_ZEA_FAN3) this->fan_mode = climate::CLIMATE_FAN_HIGH;
      else if (fan == MHI_ZEA_FAN4) this->fan_mode = climate::CLIMATE_FAN_MIDDLE;
      else                          this->fan_mode = climate::CLIMATE_FAN_AUTO;
      break;
    }

    case ZMP: {
      auto pwr = buf[9] & 0x08;
      auto md  = buf[9] & 0x07;
      float tmp = ((~buf[9] & 0xF0) >> 4) + 17;
      uint8_t fan = buf[7] & 0xE0;
      uint8_t sv  = (buf[5] & 0x02) | (buf[7] & 0x18);

      if (pwr != MHI_ON) this->mode = climate::CLIMATE_MODE_OFF;
      else switch (md) {
        case MHI_COOL: this->mode = climate::CLIMATE_MODE_COOL; break;
        case MHI_HEAT: this->mode = climate::CLIMATE_MODE_HEAT; break;
        case MHI_DRY:  this->mode = climate::CLIMATE_MODE_DRY; break;
        case MHI_FAN:  this->mode = climate::CLIMATE_MODE_FAN_ONLY; break;
        default:       this->mode = climate::CLIMATE_MODE_AUTO; break;
      }
      this->target_temperature = tmp;

      this->swing_mode = (sv == MHI_VS_SWING
                         ? climate::CLIMATE_SWING_VERTICAL
                         : climate::CLIMATE_SWING_OFF);

      if      (fan == MHI_ZMP_FAN1) this->fan_mode = climate::CLIMATE_FAN_LOW;
      else if (fan == MHI_ZMP_FAN2) this->fan_mode = climate::CLIMATE_FAN_MEDIUM;
      else if (fan == MHI_ZMP_FAN3) this->fan_mode = climate::CLIMATE_FAN_HIGH;
      else                          this->fan_mode = climate::CLIMATE_FAN_AUTO;
      break;
    }

    case ZM: {
      auto pwr = buf[5] & 0x08;
      auto md  = buf[5] & 0x07;
      float tmp = ((~buf[7] & 0x0F)) + 17;
      uint8_t fan = buf[9] & 0x0F;
      uint8_t sv  = buf[11] & 0xE0;
      uint8_t sh  = buf[13] & 0x0F;

      if (pwr != MHI_ON) this->mode = climate::CLIMATE_MODE_OFF;
      else switch (md) {
        case MHI_COOL: this->mode = climate::CLIMATE_MODE_COOL; break;
        case MHI_HEAT: this->mode = climate::CLIMATE_MODE_HEAT; break;
        case MHI_DRY:  this->mode = climate::CLIMATE_MODE_DRY; break;
        case MHI_FAN:  this->mode = climate::CLIMATE_MODE_FAN_ONLY; break;
        default:       this->mode = climate::CLIMATE_MODE_HEAT_COOL; break;
      }
      this->target_temperature = tmp;

      if      (sv == MHI_VS_SWING && sh == MHI_HS_SWING) this->swing_mode = climate::CLIMATE_SWING_BOTH;
      else if (sv == MHI_VS_SWING)                      this->swing_mode = climate::CLIMATE_SWING_VERTICAL;
      else if (sh == MHI_HS_SWING)                      this->swing_mode = climate::CLIMATE_SWING_HORIZONTAL;
      else                                              this->swing_mode = climate::CLIMATE_SWING_OFF;

      // Распределяем скорости по количеству уровней
      switch (fan) {
        case 1:
          this->fan_mode = climate::CLIMATE_FAN_LOW;
          break;
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
          this->fan_mode = climate::CLIMATE_FAN_HIGH;
          break;
        default:
          this->fan_mode = climate::CLIMATE_FAN_AUTO;
          break;
      }
      break;
    }
  }

  ESP_LOGD(TAG, "received state: mode=%u temp=%.1f fan=%u swing=%u",
           this->mode, this->target_temperature,
           this->fan_mode.value(), this->swing_mode);
  this->publish_state();
  return true;
}

void MhiClimate::transmit_state() {
  // ZJ: 11 bytes
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
    // Temperature
    if (this->target_temperature > 17 && this->target_temperature < 31)
      buf[9] |= ((~(uint8_t((this->target_temperature - 17) << 4))) & 0xF0);
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
    send_bytes(&tx, buf, 11);
  }

  // ZEA: 11 bytes
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
    if (this->target_temperature > 17 && this->target_temperature < 31)
      buf[9] |= ((~(uint8_t((this->target_temperature - 17) << 4))) & 0xF0);
    switch (this->fan_mode.value()) {
      case climate::CLIMATE_FAN_LOW:    buf[7] |= MHI_ZEA_FAN1; break;
      case climate::CLIMATE_FAN_MEDIUM: buf[7] |= MHI_ZEA_FAN2; break;
      case climate::CLIMATE_FAN_HIGH:   buf[7] |= MHI_ZEA_FAN3; break;
      case climate::CLIMATE_FAN_MIDDLE: buf[7] |= MHI_ZEA_FAN4; break;
      default:                          buf[7] |= MHI_ZEA_FAN_AUTO;
    }
    if (this->swing_mode == climate::CLIMATE_SWING_VERTICAL ||
        this->swing_mode == climate::CLIMATE_SWING_BOTH)
      buf[5] |= MHI_ZEA_VS_SWING;
    if (this->swing_mode == climate::CLIMATE_SWING_HORIZONTAL ||
        this->swing_mode == climate::CLIMATE_SWING_BOTH)
      buf[5] |= MHI_ZEA_HS_SWING;
    buf[6]  = ~buf[5];
    buf[8]  = ~buf[7];
    buf[10] = ~buf[9];
    auto tx = this->transmitter_->transmit();
    send_bytes(&tx, buf, 11);
  }

  // ZMP: 11 bytes
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
    if (this->target_temperature > 17 && this->target_temperature < 31)
      buf[9] |= ((~(uint8_t((this->target_temperature - 17) << 4))) & 0xF0);
    switch (this->fan_mode.value()) {
      case climate::CLIMATE_FAN_LOW:    buf[7] |= MHI_ZMP_FAN1; break;
      case climate::CLIMATE_FAN_MEDIUM: buf[7] |= MHI_ZMP_FAN2; break;
      case climate::CLIMATE_FAN_HIGH:   buf[7] |= MHI_ZMP_FAN3; break;
      default:                          buf[7] |= MHI_ZMP_FAN_AUTO;
    }
    if (this->swing_mode == climate::CLIMATE_SWING_VERTICAL ||
        this->swing_mode == climate::CLIMATE_SWING_BOTH)
      buf[5] |= MHI_ZMP_VS_SWING;
    buf[6]  = ~buf[5];
    buf[8]  = ~buf[7];
    buf[10] = ~buf[9];
    auto tx = this->transmitter_->transmit();
    send_bytes(&tx, buf, 11);
  }

  // ZM: 19 bytes
  else {  // model_ == ZM
    uint8_t buf[19] = {
      MHI_P0, MHI_P1, MHI_P2, MHI_P3_ZM, MHI_P4_ZM,
      0x90, 0x00, 0xF0, 0x00, 0xE0,
      0x00, 0x0D, 0x00, 0x10, 0x00,
      0x3F, 0x00, 0x7F, 0x00
    };
    // Mode bytes 6 и 8
    switch (this->mode) {
      case climate::CLIMATE_MODE_HEAT:
        buf[6] = MHI_HEAT | MHI_ON;
        buf[8] = MHI_ZMP_MODE_MAINT;
        break;
      case climate::CLIMATE_MODE_COOL:
        buf[6] = MHI_COOL | MHI_ON;
        buf[8] = MHI_ZMP_MODE_MAINT;
        break;
      case climate::CLIMATE_MODE_DRY:
        buf[6] = MHI_DRY  | MHI_ON;
        buf[8] = MHI_ZMP_MODE_MAINT;
        break;
      case climate::CLIMATE_MODE_FAN_ONLY:
        buf[6] = MHI_ZMP_MODE_FAN | MHI_ON;
        buf[8] = MHI_ZMP_MODE_MAINT;
        buf[7] = 0;
        break;
      case climate::CLIMATE_MODE_HEAT_COOL:
        buf[6] = MHI_AUTO | MHI_ON;
        buf[8] = MHI_ZMP_MODE_MAINT;
        break;
      default:
        buf[5] = MHI_OFF;
    }
    // Temperature
    if (this->target_temperature > 17 && this->target_temperature < 31)
      buf[7] = uint8_t((this->target_temperature - 17) & 0x0F);
    // Swing horizontal отсутствует
    // Vertical swing
    if (this->swing_mode == climate::CLIMATE_SWING_VERTICAL ||
        this->swing_mode == climate::CLIMATE_SWING_BOTH)
      buf[11] |= MHI_VS_SWING;
    // Fan
    switch (this->fan_mode.value()) {
      case climate::CLIMATE_FAN_LOW:    buf[9] = MHI_ZM_FAN1; break;
      case climate::CLIMATE_FAN_MEDIUM: buf[9] = MHI_ZM_FAN2; break;
      case climate::CLIMATE_FAN_HIGH:   buf[9] = MHI_ZM_FAN3; break;
      case climate::CLIMATE_FAN_MIDDLE: buf[9] = MHI_ZM_FAN4; break;
      default:                          buf[9] = MHI_ZM_FAN_AUTO;
    }
    // Инверсии всех нужных байт
    buf[6]  = ~buf[5];
    buf[8]  = ~buf[7];
    buf[10] = ~buf[9];
    buf[12] = ~buf[11];
    buf[14] = ~buf[13];
    buf[16] = ~buf[15];
    buf[18] = ~buf[17];
    // Служебные CRC
    for (int i = 0; i < 17; i++)
      buf[17] += buf[i];
    // Отправить дважды
    auto tx = this->transmitter_->transmit();
    auto *d = tx.get_data();
    d->set_carrier_frequency(38000);
    for (int rep = 0; rep < 2; rep++) {
      d->mark(MHI_HDR_MARK);
      d->space(MHI_HDR_SPACE);
      for (auto b : buf) {
        for (int bit = 0; bit < 8; bit++) {
          d->mark(MHI_BIT_MARK);
          bool one = b & (1 << bit);
          d->space(one ? MHI_ONE_SPACE : MHI_ZERO_SPACE);
        }
      }
      if (rep == 0) {
        d->mark(MHI_BIT_MARK);
        d->space(MHI_MIN_GAP);
      }
    }
    d->mark(MHI_BIT_MARK);
    tx.perform();
  }
}

}  // namespace mhi_multi_ir
}  // namespace esphome
