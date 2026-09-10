#include <Arduino.h>

#include "system_mode.h"

SystemMode systemMode = MODE_WAIT;

bool selectAppMode()
{
    // Wi-Fi đã giữ quyền thì không cho USB chen vào.
    if (systemMode == MODE_WEB) {
        return true;
    }

    // USB gửi frame hợp lệ đầu tiên: chọn USB và tắt Wi-Fi.
    if (systemMode != MODE_APP) {
        systemMode = MODE_SHARED;
    }

    return true;
}

bool selectWebMode()
{
    // USB đã giữ quyền thì không cho Web chen vào.
    if (systemMode == MODE_APP) {
        return false;
    }

    systemMode = MODE_WEB;
    return true;
}

bool isWaitMode()
{
    return systemMode == MODE_WAIT;
}

bool isAppMode()
{
    return false;
}

bool isWebMode()
{
    return systemMode == MODE_WEB;
}

const char *systemModeName()
{
    return "USB / Wi-Fi";
}
