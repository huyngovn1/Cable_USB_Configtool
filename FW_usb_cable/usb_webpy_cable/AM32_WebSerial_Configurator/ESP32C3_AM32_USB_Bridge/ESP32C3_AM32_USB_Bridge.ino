/*

 * ESP32-C3 -> AM32 raw bootloader, USB Direct only.

 *

 * GIU NGUYEN CHAN CUA BAN GOC:

 *   GPIO21 <-> ESC signal (TX/RX chung mot day, 19200 baud, 8N1).

 *   GND    <-> ESC GND. Dung lai mach giao tiep 3.3 V hien tai.

 *   PC     <-> cong USB native cua ESP32-C3 (USB Serial/JTAG).

 *

 * Arduino IDE:

 *   Board: ESP32C3 Dev Module; USB CDC On Boot: Enabled.

 *   Core Debug Level: None. Library: EspSoftwareSerial 8.2.0 (+ ghostl).

 *   Mo file nay trong thu muc rieng ESP32C3_AM32_Direct.

 *   Khong ghep voi cac file .cpp/.ino cua project cu.

 *

 * ConfigTool: chon COM USB cua ESP, bat "USB / Arduino Connect"

 * (Direct, 19200 baud), Connect, sau do chon M1.

 * Cap nguon ESP truoc, roi cap nguon ESC de vao bootloader.

 *

 * Khong dung 4Way.h, MSP.h, Wi-Fi, web server hay firmware ESC nhung san.

 * PC tao BootInit, lenh raw, payload va CRC; ESP chuyen nguyen cac byte.

 * ACK va CRC phan hoi deu den tu ESC, khong tao ACK thanh cong gia.

 *

 * Gom du frame truoc khi phat: bootloader AT32F421 co timeout khoang

 * 250 us giua cac byte. Khong dua truc tiep cac manh USB chua du xuong ESC.

 * Chi nhan tap lenh raw duoi day, KHONG phai cap UART da nang.

 *

 * Echo mac dinh bat de hop ConfigTool da cung cap (getMusic bo 4 byte dau).

 * App raw rieng khong xu ly echo: doi HOST_ECHO thanh false.

 * Echo chi la ban sao byte da gui, khong phai ACK tu ESC.

 *

 * Khong tu sua CRC, dia chi hay loi SET_BUFFER(256) cua app cu.

 * App cu nen giu block ghi 128 byte. Voi bootloader da cung cap,

 * SET_BUFFER 256 dung FE 00 01 00 + CRC, khong phai FE 00 00 00.

 * Viec bao toan ca page Flash va read-back van thuoc ve app.

 * Sau loi/mat ket noi giua luc ghi: cap nguon lai ESC va ket noi lai;

 * khong tu dong gui lai payload hoac lenh ghi.

 *

 * Library reference: https://github.com/plerup/espsoftwareserial

 */



#include <Arduino.h>

#include <SoftwareSerial.h>

#include <string.h>



#if !defined(CONFIG_IDF_TARGET_ESP32C3) || !CONFIG_IDF_TARGET_ESP32C3

#error "Select an ESP32-C3 board. This sketch preserves GPIO21."

#endif

#if !defined(ARDUINO_USB_CDC_ON_BOOT) || !ARDUINO_USB_CDC_ON_BOOT

#error "Enable USB CDC On Boot. UART0 TX uses GPIO21 and conflicts with the ESC."

#endif

#if !defined(ARDUINO_USB_MODE) || ARDUINO_USB_MODE != 1

#error "Use ESP32-C3 hardware USB Serial/JTAG mode (ARDUINO_USB_MODE=1)."

#endif



namespace {



constexpr int SERVO_OUT = 21;

constexpr uint32_t ESC_BAUD = 19200;

constexpr bool HOST_ECHO = true;

constexpr size_t MAX_RAW_FRAME = 258;  // 256 payload bytes + raw CRC.

constexpr size_t USB_QUEUE_SIZE = 2048;

constexpr uint32_t HOST_FRAME_TIMEOUT_MS = 250;

constexpr uint32_t PAYLOAD_WAIT_TIMEOUT_MS = 2000;

constexpr uint32_t FRAME_GAP_US = 520;



EspSoftwareSerial::UART escSerial;

uint8_t hostFrame[MAX_RAW_FRAME];

size_t hostUsed = 0;

size_t payloadFrameLength = 0;

bool waitingPayloadAck = false;

uint32_t lastHostByteMs = 0;

uint32_t payloadWaitStartedMs = 0;

uint32_t lastWireActivityUs = 0;

bool discardHostBurst = false;

bool usbWasConnected = false;



// Non-blocking output queue. Preserve partial USB writes and byte order.

uint8_t usbQueue[USB_QUEUE_SIZE];

size_t usbHead = 0;

size_t usbTail = 0;

size_t usbUsed = 0;



void queueUsbByte(uint8_t byte)

{

    // Every caller reserves/checks capacity before reading or transmitting.

    usbQueue[usbHead] = byte;

    usbHead = (usbHead + 1) % USB_QUEUE_SIZE;

    ++usbUsed;

}



void pumpUsbOutput()

{

    if (usbUsed == 0) return;

    const int available = Serial.availableForWrite();

    if (available <= 0) return;

    size_t count = usbUsed;

    if (count > USB_QUEUE_SIZE - usbTail) count = USB_QUEUE_SIZE - usbTail;

    if (count > static_cast<size_t>(available)) count = available;

    if (count > 64) count = 64;

    const size_t sent = Serial.write(usbQueue + usbTail, count);

    usbTail = (usbTail + sent) % USB_QUEUE_SIZE;

    usbUsed -= sent;

}



void receiveEsc()

{

    while (usbUsed < USB_QUEUE_SIZE && escSerial.available() > 0) {

        const int byte = escSerial.read();

        if (byte < 0) break;

        queueUsbByte(static_cast<uint8_t>(byte));

        lastWireActivityUs = micros();

        if (waitingPayloadAck && (byte == 0x30 || byte == 0xC2)) {

            waitingPayloadAck = false;

            if (byte == 0x30) payloadFrameLength = 0;

            // This bootloader remains in payload mode after C2. Keep the

            // length so a host-requested payload retry is still framed right.

            payloadWaitStartedMs = millis();

        } else if (!waitingPayloadAck && payloadFrameLength != 0 &&

                   hostUsed == 0 && byte == 0xC2) {

            // SET_BUFFER was rejected on the ESC side (e.g. wire corruption).

            payloadFrameLength = 0;

        }

    }

    // Do not flush/clear RX here: a response may start immediately after TX.

}



uint16_t rawCrc(const uint8_t *data, size_t count)

{

    uint16_t crc = 0;

    while (count--) {

        crc ^= *data++;

        for (uint8_t bit = 0; bit < 8; ++bit) {

            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;

        }

    }

    return crc;

}



// >0: total frame length; 0: incomplete BootInit prefix; -1: unsupported.

int expectedFrameLength()

{

    if (hostUsed == 0) return 0;

    // A payload is opaque. It can begin with any byte, even 0x2F or 0x24.

    if (payloadFrameLength != 0) return payloadFrameLength;

    switch (hostFrame[0]) {

    case 0xFF:  // SET_ADDRESS

    case 0xFE:  // SET_BUFFER

        return 6;

    case 0x01:  // PROGRAM_FLASH

    case 0x02:  // ERASE request

    case 0x03:  // READ_FLASH

    case 0xFD:  // KEEP_ALIVE

        return 4;

    case 0x00:

        break; // BootInit, or four zero bytes for RUN.

    default:

        return -1;

    }



    size_t zeros = 0;

    while (zeros < hostUsed && hostFrame[zeros] == 0) ++zeros;

    if (zeros > 32) return -1;

    if (zeros == hostUsed) {

        // RUN and a fragmented BootInit start identically. Wait for idle

        // before forwarding four zeros, so USB fragmentation cannot normally

        // trigger an accidental RUN. Send BootInit in one host write.

        if (zeros == 4 && millis() - lastHostByteMs >= HOST_FRAME_TIMEOUT_MS) {

            return 4;

        }

        return 0;

    }

    if (zeros != 8 && zeros != 12 && zeros != 32) return -1;

    static const uint8_t marker[] = {0x0D, 'B', 'L', 'H', 'e', 'l', 'i', 0xF4, 0x7D};

    const size_t suffix = hostUsed - zeros;

    if (suffix > sizeof(marker) || memcmp(hostFrame + zeros, marker, suffix) != 0) {

        return -1;

    }

    return static_cast<int>(zeros + sizeof(marker));

}



void resetParser()

{

    hostUsed = 0;

    payloadFrameLength = 0;

    waitingPayloadAck = false;

    discardHostBurst = false;

}



void rejectHostBurst()

{

    resetParser();

    discardHostBurst = true;

    // Stay silent: an interface-side parsing error is not an ESC ACK.

}



void collectHostFrame()

{

    if (waitingPayloadAck) return;

    while (Serial.available() > 0) {

        if (!discardHostBurst) {

            const int expected = expectedFrameLength();

            if (expected > 0 && hostUsed == static_cast<size_t>(expected)) return;

            if (expected < 0 || hostUsed == sizeof(hostFrame)) rejectHostBurst();

        }

        const int byte = Serial.read();

        if (byte < 0) break;

        lastHostByteMs = millis();

        if (!discardHostBurst) hostFrame[hostUsed++] = static_cast<uint8_t>(byte);

    }

}



void transmitHostFrame()

{

    const int expected = expectedFrameLength();

    if (discardHostBurst || expected <= 0 || hostUsed != static_cast<size_t>(expected)) return;

    if (micros() - lastWireActivityUs < FRAME_GAP_US) return;

    // Reserve room for echo AND the largest raw read response (259 bytes).

    const size_t reserve = (HOST_ECHO ? hostUsed : 0) + 259;

    if (USB_QUEUE_SIZE - usbUsed < reserve) return;



    const bool isPayload = payloadFrameLength != 0;

    size_t nextPayloadLength = 0;

    if (!isPayload && hostFrame[0] == 0xFE && hostUsed == 6) {

        const uint16_t crc = rawCrc(hostFrame, 4);

        if (hostFrame[4] == (crc & 0xFF) && hostFrame[5] == (crc >> 8)) {

            // Match the supplied bootloader, including zero-length requests.

            nextPayloadLength = (hostFrame[2] == 1 ? 256 : hostFrame[3]) + 2;

        }

    }



    escSerial.enableTx(true);

    // One contiguous write: no USB calls, delay(), or yield() between bytes.

    // The library completes the final stop bit before write() returns.

    const size_t sent = escSerial.write(hostFrame, hostUsed);
    escSerial.enableTx(false); // Release GPIO21 immediately and enable RX.

    lastWireActivityUs = micros();



    if (HOST_ECHO) {

        for (size_t i = 0; i < sent && i < hostUsed; ++i) queueUsbByte(hostFrame[i]);

    }

    if (sent != hostUsed) {

        rejectHostBurst(); // Do not automatically repeat a partial write.

        return;

    }

    hostUsed = 0;

    waitingPayloadAck = isPayload;

    if (!isPayload) payloadFrameLength = nextPayloadLength;

    payloadWaitStartedMs = millis();

}



void handleTimeouts()

{

    if ((hostUsed > 0 || discardHostBurst) &&

        millis() - lastHostByteMs >= HOST_FRAME_TIMEOUT_MS) {

        const int expected = expectedFrameLength();

        // A complete queued frame may be waiting for USB capacity or bus idle.

        if (discardHostBurst || expected <= 0 || hostUsed != static_cast<size_t>(expected)) {

            resetParser();

        }

    }

    if (payloadFrameLength != 0 && hostUsed == 0 &&

        millis() - payloadWaitStartedMs >= PAYLOAD_WAIT_TIMEOUT_MS) {

        resetParser(); // Host must re-establish the ESC session after a timeout.

    }

}



} // namespace



void setup()

{

    // Serial must be native USB. Never initialize UART0 on the ESC's GPIO21.

    Serial.setRxBufferSize(2048);

    Serial.setTxBufferSize(2048);

    Serial.setTxTimeoutMs(0);

    Serial.begin(ESC_BAUD); // USB line coding only; ESC speed is fixed below.

    Serial.setDebugOutput(false);



    escSerial.begin(ESC_BAUD, SWSERIAL_8N1, SERVO_OUT, SERVO_OUT, false, 2048);

    if (!escSerial) {

        pinMode(SERVO_OUT, INPUT_PULLUP);

        while (true) delay(1000); // Fail quietly; do not corrupt the binary COM.

    }

    escSerial.enableIntTx(true);  // IMPORTANT: keep USB CDC responsive during long firmware payloads.

    escSerial.enableTx(false);    // Receive/idle state is input with pull-up.

    lastWireActivityUs = micros();

}



void loop()

{

    // Incoming bytes also prove a host is present while CDC IN is starting.

    const bool connected = static_cast<bool>(Serial) || Serial.available() > 0;

    if (!connected) {

        if (usbWasConnected) {

            resetParser();

            usbHead = usbTail = usbUsed = 0;

        }

        usbWasConnected = false;

        // Only discard when there is no host session, never during a request.

        while (Serial.available() > 0) Serial.read();

        while (escSerial.available() > 0) escSerial.read();

        delay(1);

        return;

    }

    usbWasConnected = true;

    receiveEsc();

    pumpUsbOutput();

    collectHostFrame();

    transmitHostFrame();

    receiveEsc();

    pumpUsbOutput();

    handleTimeouts();

    yield();// RX edges are interrupt-buffered; never sleep inside ESC TX.

}