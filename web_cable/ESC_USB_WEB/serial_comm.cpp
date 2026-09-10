#include <Arduino.h>

#include "Global.h"
#include "serial_comm.h"
#include "MSP.h"
#include "4Way.h"

uint16_t serial_rx_counter = 0;
uint16_t serial_tx_counter = 0;
uint16_t serial_buffer_len = 0;

bool serial_command = false;

namespace {

uint32_t lastSerialByteMs = 0;

void resetSerialParser()
{
    serial_command = false;
    serial_rx_counter = 0;
    serial_tx_counter = 0;
    serial_buffer_len = 0;
}

bool validMspFrame()
{
    if (serial_buffer_len < 6 ||
        serial_rx_counter != serial_buffer_len) {
        return false;
    }

    if (serial_rx[0] != 0x24 ||
        serial_rx[1] != 0x4D ||
        serial_rx[2] != 0x3C) {
        return false;
    }

    uint8_t checksum = 0;

    // XOR: size, command và payload.
    for (uint16_t i = 3;
         i < serial_buffer_len - 1;
         i++) {
        checksum ^= serial_rx[i];
    }

    return checksum ==
           serial_rx[serial_buffer_len - 1];
}

bool validFourWayFrame()
{
    if (serial_buffer_len < 8 ||
        serial_rx_counter != serial_buffer_len ||
        serial_rx[0] != cmd_Local_Escape) {
        return false;
    }

    uint16_t crc = 0;

    for (uint16_t i = 0;
         i < serial_buffer_len - 2;
         i++) {
        crc = _crc_xmodem_update(
            crc,
            serial_rx[i]);
    }

    const uint16_t received =
        (static_cast<uint16_t>(
            serial_rx[serial_buffer_len - 2]) << 8) |
        serial_rx[serial_buffer_len - 1];

    return crc == received;
}

} // namespace

void process_serial(void)
{
    if (Serial.available()) {
        // Giữ cách gom frame của bản USB gốc.
        delay(10);
        serial_command = true;

        while (Serial.available()) {
            if (serial_rx_counter >= 300) {
                resetSerialParser();
                return;
            }

            serial_rx[serial_rx_counter] =
                static_cast<uint8_t>(Serial.read());

            lastSerialByteMs = millis();

            if (serial_rx_counter == 4 &&
                serial_rx[0] == cmd_Local_Escape) {

                // 4-Way: size 0 nghĩa là 256 byte, cộng 7 byte overhead.
                serial_buffer_len =
                    serial_rx[4] == 0
                    ? 256 + 7
                    : serial_rx[4] + 7;
            }

            if (serial_rx_counter == 3 &&
                serial_rx[0] == 0x24) {

                // MSP v1: payload + 6 byte overhead.
                serial_buffer_len =
                    serial_rx[3] + 6;
            }

            serial_rx_counter++;

            if (serial_buffer_len > 0 &&
                serial_rx_counter >= serial_buffer_len) {
                break;
            }
        }
    }

    if (!serial_command) {
        return;
    }

    /*
     * Frame bị chia nhỏ quá lâu hoặc byte rác:
     * xóa parser để lần kết nối kế tiếp vẫn hoạt động.
     */
    if (serial_buffer_len == 0) {
        if (serial_rx_counter > 0 &&
            millis() - lastSerialByteMs > 50) {
            resetSerialParser();
        }
        return;
    }

    if (serial_rx_counter < serial_buffer_len) {
        if (millis() - lastSerialByteMs > 80) {
            resetSerialParser();
        }
        return;
    }

    if (serial_rx[0] == cmd_Local_Escape) {
        if (!validFourWayFrame()) {
            resetSerialParser();
            return;
        }

        /*
         * Chỉ frame 4-Way hợp lệ mới được chọn APP.
         * stopWebRadio() chạy trước Check_4Way().
         */
        serial_tx_counter =
            Check_4Way(serial_rx);

        for (uint16_t b = 0;
             b < serial_tx_counter;
             b++) {
            Serial.write(serial_rx[b]);
        }

        Serial.flush();
        resetSerialParser();
        return;
    }

    if (serial_rx[0] == 0x24 &&
        serial_rx[1] == 0x4D &&
        serial_rx[2] == 0x3C) {

        if (!validMspFrame()) {
            resetSerialParser();
            return;
        }

        /*
         * Một frame MSP hoàn chỉnh và đúng checksum chọn APP.
         * Wi-Fi tắt trước khi App bắt đầu phiên 4-Way.
         */
        serial_tx_counter =
            MSP_Check(
                serial_rx,
                static_cast<uint8_t>(
                    serial_rx_counter));

        for (uint16_t b = 0;
             b < serial_tx_counter;
             b++) {
            Serial.write(serial_rx[b]);
        }

        Serial.flush();
        resetSerialParser();
        return;
    }

    resetSerialParser();
}
