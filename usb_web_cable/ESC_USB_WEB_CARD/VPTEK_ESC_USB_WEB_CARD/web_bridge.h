#pragma once

#include <Arduino.h>

constexpr size_t ESC_EEPROM_SIZE = 48;

void setupWebBridge();
void webBridgeLoop();

bool webBridgeLocksEsc();

// Shared EEPROM access for the local LCD/Card interface.
bool cardReadEscSettings(uint8_t *data, size_t length, String &message);
bool cardWriteEscSettings(const uint8_t *data, size_t length, String &message);

enum class CardFirmwareStage : uint8_t {
    Idle,
    Preparing,
    Safety,
    Writing,
    Finalizing,
    Resetting,
    Done,
    Error
};

struct CardFirmwareStatus {
    CardFirmwareStage stage;
    uint32_t written;
    uint32_t total;
    uint8_t percent;
    String message;
};

// Install the built-in VPTEK Intel HEX image directly from the local Card UI.
bool cardStartBuiltInFirmware(String &message);
CardFirmwareStatus cardGetFirmwareStatus();

void startWebRadio();
void stopWebRadio();
bool webRadioIsRunning();
