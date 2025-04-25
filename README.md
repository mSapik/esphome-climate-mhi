# ESPHome Mitsubishi Heavy IR Climate Component

`mhi_multi_ir` is a universal ESPHome custom component that lets you control Mitsubishi Heavy Industries (MHI) air conditioners via infrared. It supports four hardware series (ZJ, ZEA, ZM, ZMP) and their specific protocols: packet length, swing bits, fan speeds, and presets (ECO, BOOST, ACTIVITY).

---

## 📦 Supported Models

| YAML Value |
| `ZJ`       |
| `ZEA`      |
| `ZM`       |
| `ZMP`      |

---

## 🌬️ Fan Speed Options

The `set_fan_levels` option lets you choose between 3-speed mode (LOW, MEDIUM, HIGH) or 4-speed mode (adds MIDDLE).

---

## ⚙️ Installation

1. Clone/download this repository into `/config/esphome/esphome-climate-mhi` 
2. Add to your esphome configuration:

```yaml
custom_components:
  - mhi_multi_ir

esphome:
  name: ac_controller
  platform: ESP8266
  board: nodemcuv2

remote_transmitter:
  pin: GPIO4
  carrier_duty_percent: 50%

remote_receiver:
  pin:
    number: GPIO5
    inverted: True
  dump: all
  id: rcvr

climate:
  - platform: mhi_multi_ir
    name: "MHI AC"
    id: mhi_ac
    model: ZJ           # or ZEA, ZM, ZMP
    set_fan_levels: 4   # 3 or 4 fan speeds
    reciever_id: rcvr
```