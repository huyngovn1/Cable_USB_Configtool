#include <Arduino.h>

#include "serial_comm.h"
#include "web_bridge.h"

void setup()
{
    Serial.begin(115200);
    delay(50);

    setupWebBridge();
}

void loop()
{
    // Keep the Wi-Fi AP available while the USB App Tool is connected.
    // USB is only paused during an active Web ESC operation or flash.
    if (!webBridgeLocksEsc()) {
        process_serial();
    }

    webBridgeLoop();
}
