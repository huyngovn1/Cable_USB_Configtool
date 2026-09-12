# Cable USB ConfigTool

USB/Web communication cable for AM32 ESC configuration and firmware update.

This project uses an ESP32-C3 as a communication bridge between the PC configuration software and the ESC.

## Features

- USB communication with AM32 ConfigTool.
- Compatible with AM32 ConfigTool v1.93.
- Web-based ESC configuration.
- Firmware update through USB or Web interface.
- USB/Web operating mode selection.
- Optimized communication bandwidth.
- Improved data transfer stability.
- Half-duplex communication with ESC.
- ESP32-C3 USB CDC interface.
- Designed for AM32 ESC development and debugging.

## Communication Structure

```text
PC / AM32 ConfigTool
        |
      USB
        |
    ESP32-C3
        |
   ESC Signal Line
        |
      AM32 ESC
