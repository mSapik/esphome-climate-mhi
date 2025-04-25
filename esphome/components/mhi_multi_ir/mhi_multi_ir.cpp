#include "mhi_multi_ir.h"
#include "esphome/core/log.h"

namespace esphome {
namespace mhi_multi_ir {

static const char *TAG = "mhi_multi_ir.climate";

// Универсальная отправка битов
template<typename Tx>
static void send_bytes(Tx &tx, const uint8_t *buf, size_t len) {
  auto *d = tx.get_data();
  d->set_carrier_frequency(CARRIER_HZ);
  d->mark(HDR_MARK); d->space(HDR_SPACE);
  for (size_t i = 0; i < len; i++) {
    for (uint8_t b = 0; b < 8; b++) {
      d->mark(BIT_MARK);
      d->space((buf[i] & (1<<b)) ? ONE_SPACE : ZERO_SPACE);
    }
  }
  d->mark(BIT_MARK); d->space(0);
  tx.perform();
}

bool MhiClimate::on_receive(remote_base::RemoteReceiveData data) {
  ESP_LOGD(TAG, "on_receive model=%u", model_);

  // Выбираем union и длину
  if (model_ == ZM || model_ == ZMP) {
    Protocol152 msg; size_t len = LEN_152;
    if (!data.expect_item(HDR_MARK, HDR_SPACE)) return false;
    for (size_t i = 0; i < len; i++) {
      uint8_t v = 0;
      for (int b = 0; b < 8; b++) {
        if (data.expect_item(BIT_MARK, ONE_SPACE))      v |= 1<<b;
        else if (!data.expect_item(BIT_MARK, ZERO_SPACE)) return false;
      }
      msg.raw[i] = v;
      if (i >= SIG_152_LEN && i % 2 == SIG_152_LEN%2) {
        // проверка инверсии пар
        if (msg.raw[i] != static_cast<uint8_t>(~msg.raw[i^1])) return false;
      }
    }
    // Проверяем сигнатуру
    if (std::memcmp(msg.Sig, SIG_152, SIG_152_LEN) != 0) return false;

    // Преобразуем
    this->mode                = convert_mode152(msg.Mode);
    this->target_temperature  = msg.Temp + MIN_TEMP;
    this->fan_mode            = convert_fan152(msg.Fan);
    this->swing_mode          = convert_swing(msg.SwingV, msg.SwingH);
    if (msg.Econ o) this->preset = climate::CLIMATE_PRESET_ECO;
    if (msg.Three)  this->preset = climate::CLIMATE_PRESET_ACTIVITY;

  } else {
    Protocol88 msg; size_t len = LEN_88;
    if (!data.expect_item(HDR_MARK, HDR_SPACE)) return false;
    for (size_t i = 0; i < len; i++) {
      uint8_t v = 0;
      for (int b = 0; b < 8; b++) {
        if (data.expect_item(BIT_MARK, ONE_SPACE))      v |= 1<<b;
        else if (!data.expect_item(BIT_MARK, ZERO_SPACE)) return false;
      }
      msg.raw[i] = v;
      if (i >= SIG_152_LEN && i % 2 == SIG_152_LEN%2) {
        if (msg.raw[i] != static_cast<uint8_t>(~msg.raw[i^1])) return false;
      }
    }
    if (std::memcmp(msg.Sig, SIG_88, SIG_152_LEN) != 0) return false;

    this->mode               = convert_mode88(msg.Mode);
    this->target_temperature = msg.Temp + MIN_TEMP;
    this->fan_mode           = convert_fan88(msg.Fan);
    this->swing_mode         = convert_swing(msg.SwingV5 | (msg.SwingV7<<1), msg.SwingH1 | (msg.SwingH2<<2));
    if (msg.Clean) this->preset = climate::CLIMATE_PRESET_ECO;
    if (msg.SwingH2 == 1) this->preset = climate::CLIMATE_PRESET_ACTIVITY;
  }

  this->publish_state();
  return true;
}

void MhiClimate::transmit_state() {
  ESP_LOGD(TAG, "transmit_state model=%u", model_);

  // Выбираем union
  if (model_ == ZM || model_ == ZMP) {
    Protocol152 msg; std::memset(&msg, 0, sizeof(msg));
    std::memcpy(msg.Sig, SIG_152, SIG_152_LEN);
    msg.Power = (mode!=climate::CLIMATE_MODE_OFF);
    // Режим
    switch (mode) {
      case climate::CLIMATE_MODE_COOL: msg.Mode = P152_COOL; break;
      case climate::CLIMATE_MODE_HEAT: msg.Mode = P152_HEAT; break;
      case climate::CLIMATE_MODE_DRY:  msg.Mode = P152_DRY;  break;
      case climate::CLIMATE_MODE_FAN_ONLY: msg.Mode = P152_FAN; break;
      default: msg.Mode = P152_AUTO;
    }
    msg.Temp  = static_cast<uint8_t>(target_temperature - MIN_TEMP);
    // Fan
    switch (fan_mode) {
      case climate::CLIMATE_FAN_LOW:    msg.Fan = F152_LOW;   break;
      case climate::CLIMATE_FAN_MEDIUM: msg.Fan = F152_MED;   break;
      case climate::CLIMATE_FAN_HIGH:   msg.Fan = F152_HIGH;  break;
      default: msg.Fan = F152_AUTO;
    }
    // Swing
    switch (swing_mode) {
      case climate::CLIMATE_SWING_BOTH:    msg.SwingV=SV152_AUTO, msg.SwingH=SH152_AUTO; break;
      case climate::CLIMATE_SWING_VERTICAL:   msg.SwingV=SV152_AUTO; break;
      case climate::CLIMATE_SWING_HORIZONTAL: msg.SwingH=SH152_AUTO; break;
      default: break;
    }
    // Предсеты
    msg.Clean  = (preset==climate::CLIMATE_PRESET_ECO);
    msg.Three  = (preset==climate::CLIMATE_PRESET_ACTIVITY);
    msg.D      = msg.Three;

    // Инверсия пар
    invert_byte_pairs(msg.raw+SIG_152_LEN, LEN_152-SIG_152_LEN);

    // Отправка
    auto tx = this->transmitter_->transmit();
    send_bytes(tx, msg.raw, LEN_152);
    // Для ZM дублируем
    send_bytes(tx, msg.raw, LEN_152);

  } else {
    Protocol88 msg; std::memset(&msg, 0, sizeof(msg));
    std::memcpy(msg.Sig, SIG_88, SIG_152_LEN);
    msg.Power = (mode!=climate::CLIMATE_MODE_OFF);
    switch (mode) {
      case climate::CLIMATE_MODE_COOL: msg.Mode = P88_COOL; break;
      case climate::CLIMATE_MODE_HEAT: msg.Mode = P88_HEAT; break;
      case climate::CLIMATE_MODE_DRY:  msg.Mode = P88_DRY;  break;
      case climate::CLIMATE_MODE_FAN_ONLY: msg.Mode = P88_FAN; break;
      default: msg.Mode = P88_AUTO;
    }
    msg.Temp = static_cast<uint8_t>(target_temperature - MIN_TEMP);
    switch (fan_mode) {
      case climate::CLIMATE_FAN_LOW:    msg.Fan = F88_LOW;   break;
      case climate::CLIMATE_FAN_MEDIUM: msg.Fan = F88_MED;   break;
      case climate::CLIMATE_FAN_HIGH:   msg.Fan = F88_HIGH;  break;
      default: msg.Fan = F88_AUTO;
    }
    // Swing: просто 3D как горизонтальный
    if (swing_mode == climate::CLIMATE_SWING_BOTH) {
      msg.SwingH1 = SH88_3D>>0; msg.SwingH2 = SH88_3D>>2;
    }
    msg.Clean = (preset==climate::CLIMATE_PRESET_ECO);

    invert_byte_pairs(msg.raw+SIG_152_LEN, LEN_88-SIG_152_LEN);

    auto tx = this->transmitter_->transmit();
    send_bytes(tx, msg.raw, LEN_88);
  }
}

}  // namespace mhi_multi_ir
}  // namespace esphome
