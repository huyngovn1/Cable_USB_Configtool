#include <Arduino.h>

#include "DisplayUI.h"
#include "serial_comm.h"
#include "system_mode.h"
#include "web_bridge.h"

void setup()
{
    Serial.begin(115200);
    delay(50);

    systemMode = MODE_WAIT;
    setupWebBridge();
    DisplayUI_Init();
}

void loop()
{
    /*
     * USB, Web and Card stay available. Only one ESC transaction runs at a
     * time; Card and Web expose their busy state through webBridgeLocksEsc().
     */
// Khi Web đã được chọn thì không tiếp nhận lệnh USB nữa.
if (!webBridgeLocksEsc() && !isWebMode()) {
    process_serial();
}

    webBridgeLoop();
    DisplayUI_Loop();
}
