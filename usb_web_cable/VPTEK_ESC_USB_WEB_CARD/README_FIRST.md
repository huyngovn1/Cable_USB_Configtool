# VPTEK ESC - USB / Web / Card

The firmware provides three ways to configure the same AM32 ESC:

1. USB Config Tool
2. Wi-Fi Web interface
3. Local ST7567 LCD and three buttons

USB, Web and Card remain available after startup. ESC transactions are
serialized so two interfaces cannot drive GPIO21 at the same time.

## Card controls

- OK: read, open setting, confirm and save
- Down: next setting or decrease value
- Up: previous setting or increase value
- Hold OK for 1.2 seconds: read settings again

Card mode reads, writes and verifies the existing 48-byte AM32 EEPROM block.
Firmware installation is not included on the Card screen yet. Web firmware
installation remains available.

## Build

- Board: ESP32-C3 Dev Module
- ESP32 Arduino core used for the verified build: 3.3.10
- Required libraries: EspSoftwareSerial, ghostl and U8g2 2.36.19
- Open `VPTEK_ESC_USB_WEB_CARD/VPTEK_ESC_USB_WEB_CARD.ino`

## Ready-to-flash BIN

For a simple full flash, select:

`BIN/VPTEK_ESC_USB_WEB_CARD_MERGED.bin`

Write it at address `0x00000000`. The merged image includes the bootloader,
partition table and application.
