# VPTEK ESC USB + Web + Card

This project keeps the existing AM32 USB and Web functions and adds a local
ST7567 LCD/button interface.

## Card hardware

- LCD CS: GPIO7
- LCD RESET: GPIO2
- LCD DC: GPIO3
- LCD CLOCK: GPIO4
- LCD MOSI: GPIO6
- LCD backlight: GPIO10
- OK button: GPIO5
- Down/minus button: GPIO8
- Up/plus button: GPIO9
- ESC one-wire signal: GPIO21 (unchanged)

Buttons use `INPUT_PULLUP` and must pull the GPIO to GND when pressed.

## Card operation

1. Power the ESC and the ESP32-C3.
2. Press OK on `OK: READ ESC`.
3. Use Up/Down to select a setting.
4. Press OK to edit.
5. Use Up/Down to change the value.
6. Press OK to write and verify the complete 48-byte EEPROM block.
7. Hold OK for 1.2 seconds to read the ESC again.

Firmware installation is intentionally not included in the Card menu. The
existing Web firmware page remains unchanged.

## Communication ownership

USB, Web and Card remain available in the main loop. ESC transactions are
serialized: when Web/Card/firmware work is active, another interface waits.
This prevents two interfaces from driving GPIO21 at the same time.

## Required library

Install the Arduino `U8g2` library before building.
