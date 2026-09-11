# VPTEK AM32 Web Serial Configurator

Bản này giữ giao diện VPTEK cũ nhưng thay backend Wi-Fi/4Way bằng Web Serial. Trình duyệt đóng vai trò ConfigTool, còn ESP32-C3 chỉ là cầu USB CDC <-> GPIO21 <-> bootloader AM32.

## 1. Nạp firmware bridge vào ESP32-C3

Mở:

`ESP32C3_AM32_USB_Bridge/ESP32C3_AM32_USB_Bridge.ino`

Arduino IDE:

- Board: ESP32C3 Dev Module
- USB CDC On Boot: Enabled
- USB Mode: Hardware CDC and JTAG / USB Serial-JTAG (`ARDUINO_USB_MODE=1`)
- Core Debug Level: None
- Library: EspSoftwareSerial 8.2.0

Điểm bắt buộc của bản này:

```cpp
escSerial.enableIntTx(true);
```

Không đổi lại `false`, vì block firmware dài sẽ làm USB CDC/echo bị trễ và ConfigTool/Web bị fail.

## 2. Đấu dây

```text
ESP32-C3 GPIO21 ---- ESC Signal / PB4
ESP32-C3 GND    ---- ESC GND
```

Giữ mạch 3.3 V hiện tại. Nếu đang dùng điện trở nối tiếp ở card cũ thì giữ nguyên. Không đưa 5 V trực tiếp vào PB4.

## 3. Mở web

Không mở `index.html` trực tiếp bằng `file://`.

Trên Windows, chạy:

`start_web.bat`

Sau đó Chrome/Edge sẽ mở:

`http://localhost:8765/index.html`

Web Serial cần localhost/HTTPS và trình duyệt desktop Chrome hoặc Edge.

## 4. Kết nối ESC

1. Cắm ESP32-C3 vào PC trước.
2. Mở web.
3. Bấm `PC Connect`, sau đó `Connect Device`.
4. Chọn COM `USB Serial/JTAG` của ESP32-C3.
5. Nếu ESC chưa vào bootloader: giữ ESP đang cắm, tắt/bật lại nguồn ESC rồi bấm `Connect Device` lại.
6. Khi đúng, web sẽ đọc DeviceInfo và 48 byte EEPROM rồi đưa lên giao diện.

## 5. Firmware

- Chọn file `.hex` tại tab Firmware.
- Bấm `Install Selected File`.
- Browser parse Intel HEX, gửi từng block 128 byte.
- Luồng mỗi block giống ConfigTool Direct:

```text
SET_ADDRESS -> ACK
SET_BUFFER  -> không có ACK riêng
PAYLOAD+CRC -> ACK
PROGRAM_FLASH -> ACK
```

- Thanh tiến trình lấy số byte đã ghi thật, không dùng phần trăm giả theo thời gian.
- Sau khi ghi xong, web khôi phục EEPROM/boot byte và gửi lệnh RUN.

Nút `Recommended VPTEK Firmware` vẫn giữ trong giao diện nhưng bản này không nhúng `builtin_firmware.h`. Hãy dùng `Custom Firmware` với file `.hex`.

## 6. Lưu Settings

Web đọc lại EEPROM trước khi ghi, sửa đúng các field 0..47 theo giao diện, ghi 48 byte theo đường Direct và đọc lại để verify. Byte boot được ép về `0x01` khi Save Settings.

## 7. Lưu ý

- Không mở ConfigTool và web cùng lúc trên cùng COM.
- Không rút USB/nguồn ESC khi đang flash.
- Nếu flash báo lỗi, power-cycle ESC trước khi thử lại.
- Bản này dùng `HOST_ECHO=true`; JavaScript tự bỏ và kiểm tra đúng echo trước khi đọc ACK/data thật của ESC.
