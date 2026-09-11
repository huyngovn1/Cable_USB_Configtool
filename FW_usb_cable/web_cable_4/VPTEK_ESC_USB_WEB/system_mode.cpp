#include <Arduino.h>
#include "system_mode.h"

SystemMode systemMode = MODE_WAIT;


bool selectAppMode()
{
    // Web đã được chọn trước -> USB không được quyền.
    if (systemMode == MODE_WEB) {
        return false;
    }

    // USB được chọn lần đầu.
    if (systemMode == MODE_WAIT) {
        systemMode = MODE_APP;
    }

    return systemMode == MODE_APP;
}


bool selectWebMode()
{
    // USB đã được chọn trước -> Web không được quyền.
    if (systemMode == MODE_APP) {
        return false;
    }

    // Web được chọn lần đầu.
    if (systemMode == MODE_WAIT) {
        systemMode = MODE_WEB;
    }

    return systemMode == MODE_WEB;
}


bool isWaitMode()
{
    return systemMode == MODE_WAIT;
}


bool isAppMode()
{
    return systemMode == MODE_APP;
}


bool isWebMode()
{
    return systemMode == MODE_WEB;
}


const char *systemModeName()
{
    switch (systemMode) {

        case MODE_APP:
            return "USB";

        case MODE_WEB:
            return "WEB";

        default:
            return "WAIT";
    }
}