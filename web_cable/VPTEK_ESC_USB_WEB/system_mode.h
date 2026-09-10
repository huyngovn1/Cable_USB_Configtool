#pragma once

#include <Arduino.h>

enum SystemMode : uint8_t {
    MODE_WAIT = 0,
    MODE_APP  = 1,
    MODE_WEB  = 2,
    MODE_SHARED = 3
};

extern SystemMode systemMode;

bool selectAppMode();
bool selectWebMode();

bool isWaitMode();
bool isAppMode();
bool isWebMode();

const char *systemModeName();
