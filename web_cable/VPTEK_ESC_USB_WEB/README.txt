VPTEK ESC USB + WEB

This project has two interfaces only:
- USB: compatible with the AM32 Config Tool through the ESP32-C3 USB COM port.
- Web: connect to the VPTEK-ESC Wi-Fi AP, then open http://192.168.4.1.

The Card/LCD interface is not included in this project.

USB and Web can stay available together. Do not start an ESC operation from
both interfaces at the same time.

USB reliability changes:
- GPIO21 half-duplex serial is initialized for USB App Tool requests.
- ESC bootloader discovery safely retries up to three times before reporting
  a connection error.
- No firmware writes, erases, or EEPROM writes are automatically retried.

The included built-in firmware image is restricted to AT32F421 / flash code
0x1F. Do not remove ESC power while firmware is being written.
