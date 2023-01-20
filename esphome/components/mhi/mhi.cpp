#include "mhi.h"
#include "esphome/core/log.h"

namespace esphome {
    namespace mhi {
        static const char *TAG = "mhi.climate";

        // Power
        const uint32_t MHI_OFF = 0x08;
        const uint32_t MHI_ON = 0x00;

        // Operating mode
        const uint8_t MHI_AUTO = 0x07;
        const uint8_t MHI_HEAT = 0x03;
        const uint8_t MHI_COOL = 0x06;
        const uint8_t MHI_DRY = 0x05;
        const uint8_t MHI_FAN = 0x04;

        // Fan speed
        const uint8_t MHI_FAN_AUTO = 0xE0;
        const uint8_t MHI_FAN1 = 0xA0;
        const uint8_t MHI_FAN2 = 0x80;
        const uint8_t MHI_FAN3 = 0x60;
        const uint8_t MHI_HIPOWER = 0x40;
        const uint8_t MHI_ECONO = 0x00;

        // Vertical swing
        const uint8_t MHI_VS_SWING = 0x0A;
        const uint8_t MHI_VS_UP = 0x02;
        const uint8_t MHI_VS_MUP = 0x18;
        const uint8_t MHI_VS_MIDDLE = 0x10;
        const uint8_t MHI_VS_MDOWN = 0x08;
        const uint8_t MHI_VS_DOWN = 0x00;
        const uint8_t MHI_VS_STOP = 0x1A;

        // Horizontal swing
        const uint8_t MHI_HS_SWING = 0x4C;
        const uint8_t MHI_HS_MIDDLE = 0x48;
        const uint8_t MHI_HS_LEFT = 0xC8;
        const uint8_t MHI_HS_MLEFT = 0x88;
        const uint8_t MHI_HS_MRIGHT = 0x08;
        const uint8_t MHI_HS_RIGHT = 0xC4;
        const uint8_t MHI_HS_STOP = 0xCC;
        const uint8_t MHI_HS_LEFTRIGHT = 0x84;
        const uint8_t MHI_HS_RIGHTLEFT = 0x44;
        const uint8_t MHI_HS_3DAUTO = 0x04;

        // Only available in Auto, Cool and Heat mode
        // const uint8_t MHI_3DAUTO_ON = 0x00;
        // const uint8_t MHI_3DAUTO_OFF = 0x12;

        // NOT available in Fan or Dry mode
        const uint8_t MHI_SILENT_ON = 0x00;
        const uint8_t MHI_SILENT_OFF = 0x80;

        // Pulse parameters in usec
        const uint16_t MHI_BIT_MARK = 400;
        const uint16_t MHI_ONE_SPACE = 1200;
        const uint16_t MHI_ZERO_SPACE = 400;
        const uint16_t MHI_HEADER_MARK = 3200;
        const uint16_t MHI_HEADER_SPACE = 1600;
        const uint16_t MHI_MIN_GAP = 17500;

        void MhiClimate::transmit_state() {
            uint8_t remote_state[] = {
                0x52, 0xAE, 0xC3, 0x26,
                0xD9, 0x11, 0x00, 0x07,
                0x00, 0x00, 0x00
            };

            // ----------------------
            // Initial values
            // ----------------------

            auto operatingMode = MHI_AUTO;
            auto powerMode = MHI_ON;
            auto cleanMode = 0x20; // always off

            auto temperature = 22;
            auto fanSpeed = MHI_FAN_AUTO;
            auto swingV = MHI_VS_STOP;
            // auto swingH = MHI_HS_RIGHT;  // custom preferred value for this mode, should be MHI_HS_STOP
            auto swingH = MHI_HS_STOP;
            auto silentMode = MHI_SILENT_OFF;

            // ----------------------
            // Assign the values
            // ----------------------

            // Power and operating mode
            switch (this->mode) {
                case climate::CLIMATE_MODE_COOL:
                    operatingMode = MHI_COOL;
                    swingV = MHI_VS_UP; // custom preferred value for this mode
                    break;
                case climate::CLIMATE_MODE_HEAT:
                    operatingMode = MHI_HEAT;
                    swingV = MHI_VS_DOWN; // custom preferred value for this mode
                    break;
                case climate::CLIMATE_MODE_AUTO:
                    operatingMode = MHI_AUTO;
                    swingV = MHI_VS_MIDDLE; // custom preferred value for this mode
                    break;
                case climate::CLIMATE_MODE_FAN_ONLY:
                    operatingMode = MHI_FAN;
                    swingV = MHI_VS_MIDDLE; // custom preferred value for this mode
                    break;
                case climate::CLIMATE_MODE_DRY:
                    operatingMode = MHI_DRY;
                    swingV = MHI_VS_MIDDLE; // custom preferred value for this mode
                    break;
                case climate::CLIMATE_MODE_OFF:
                default:
                    powerMode = MHI_OFF;
                    break;
            }

            // Temperature
            if (this->target_temperature > 17 && this->target_temperature < 31)
                temperature = this->target_temperature;

            // Horizontal and vertical swing
            switch (this->swing_mode) {
                case climate::CLIMATE_SWING_BOTH:
                    swingV = MHI_VS_SWING;
                    swingH = MHI_HS_SWING;
                    break;
                case climate::CLIMATE_SWING_HORIZONTAL:
                    swingH = MHI_HS_SWING;
                    break;
                case climate::CLIMATE_SWING_VERTICAL:
                    swingV = MHI_VS_SWING;
                    break;
                case climate::CLIMATE_SWING_OFF:
                default:
                    // Already on STOP
                    break;
            }

            // Fan speed
            switch (this->fan_mode.value()) {
                case climate::CLIMATE_FAN_LOW:
                    fanSpeed = MHI_FAN1;
                    break;
                case climate::CLIMATE_FAN_MEDIUM:
                    fanSpeed = MHI_FAN2;
                    break;
                case climate::CLIMATE_FAN_HIGH:
                    fanSpeed = MHI_FAN3;
                    break;
                case climate::CLIMATE_FAN_MIDDLE:
                    fanSpeed = MHI_FAN_AUTO;
                    swingH = MHI_HS_MIDDLE;
                    break;
                case climate::CLIMATE_FAN_FOCUS:
                    fanSpeed = MHI_FAN_AUTO;
                    swingH = MHI_HS_RIGHTLEFT;
                    break;
                case climate::CLIMATE_FAN_DIFFUSE:
                    fanSpeed = MHI_FAN_AUTO;
                    swingH = MHI_HS_LEFTRIGHT;
                    break;
                case climate::CLIMATE_FAN_AUTO:
                default:
                    fanSpeed = MHI_FAN_AUTO;
                    break;
            }

            // ----------------------
            // Assign the bytes
            // ----------------------

            // Power state + operating mode
            remote_state[5] |= swingH | (swingV & 0b00000010) | cleanMode;

            // Temperature
            remote_state[7] |= fanSpeed | (swingV & 0b00011000);

            // Fan speed
            remote_state[9] |= operatingMode | powerMode | (~((uint8_t)temperature - 17) & 0x0F);

            // There is no real checksum, but some bytes are inverted
            remote_state[6] = ~remote_state[5];
            remote_state[8] = ~remote_state[7];
            remote_state[10] = ~remote_state[9];

            // ESP_LOGD(TAG, "Sending MHI target temp: %.1f state: %02X mode: %02X temp: %02X", this->target_temperature, remote_state[5], remote_state[6], remote_state[7]);

            auto bytes = remote_state;
            ESP_LOGD(TAG, 
                "Sent bytes 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X",
                bytes[0], bytes[1], bytes[2], bytes[3],
                bytes[4], bytes[5], bytes[6], bytes[7],
                bytes[8], bytes[9], bytes[10]
            );

            auto transmit = this->transmitter_->transmit();
            auto data = transmit.get_data();

            data->set_carrier_frequency(38000);

            // Header
            data->mark(MHI_HEADER_MARK);
            data->space(MHI_HEADER_SPACE);

            // Data
            for (uint8_t i : remote_state)
                for (uint8_t j = 0; j < 8; j++) {
                    data->mark(MHI_BIT_MARK);
                    bool bit = i & (1 << j);
                    data->space(bit ? MHI_ONE_SPACE : MHI_ZERO_SPACE);
                }
            data->mark(MHI_BIT_MARK);
            data->space(0);

            transmit.perform();
        }
    } // namespace mhi
} // namespace esphome