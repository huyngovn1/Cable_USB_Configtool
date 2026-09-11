#include <Arduino.h>

#include "serial_comm.h"
#include "web_bridge.h"
#include "system_mode.h"

void setup()
{
    Serial.begin(115200);
    delay(50);

    setupWebBridge();
}

void loop()
{
    // =====================================================
    // USB đã khóa
    // =====================================================
    if (isAppMode()) {

        // Wi-Fi đã bị tắt.
        process_serial();

        return;
    }


    // =====================================================
    // WEB đã khóa
    // =====================================================
    if (isWebMode()) {

        // QUAN TRỌNG:
        // Phải chạy webBridgeLoop()
        // thì server.handleClient() mới hoạt động.
        webBridgeLoop();

        return;
    }


    // =====================================================
    // WAIT
    // Cả Web và USB đều đang chờ.
    // =====================================================

    process_serial();

    // Nếu USB vừa lấy quyền thì thoát ngay.
    if (isAppMode()) {
        return;
    }

    // QUAN TRỌNG:
    // Khi chưa có USB thì vẫn phải chạy Web.
    webBridgeLoop();
}