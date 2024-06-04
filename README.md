# esphome-climate-mhi
Custom component to support Mitsubishi Heavy Industries as climate IR in ESPhome
Mitsubishi Heavy SRKxxZJ-S (remote control P/N RKX502A001C)

## Using this component
Clone/download this repository into '/config/esphome/esphome-climate-mhi' and add to your esphome configuration:
```
external_components:
  # use all components from a local folder
  - source:
      type: local
      path: esphome-climate-mhi
```
Then, add the climate config:

```
remote_transmitter:
  pin: GPIO14
  carrier_duty_percent: 50%
  
climate:
  - platform: mhi
    name: "AC Livingroom"
```
