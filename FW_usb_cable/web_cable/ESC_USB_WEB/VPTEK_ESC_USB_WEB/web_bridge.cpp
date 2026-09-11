#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>

#include "Global.h"
#include "4Way.h"
#include "ESC_Serial.h"
#include "web_bridge.h"
#include "system_mode.h"
#include "builtin_firmware.h"
//HELLO
namespace {

constexpr char AP_SSID[] = "ESC_control-center";
constexpr char AP_PASSWORD[] = "";
constexpr uint8_t DNS_PORT = 53;
const IPAddress AP_IP(192, 168, 4, 1);

constexpr uint16_t EEPROM_SIZE_BYTES = 48;
constexpr uint16_t FRAME_CAPACITY = 300;
constexpr uint32_t APP_PHYSICAL_START = 0x1000;
constexpr uint16_t FLASH_CHUNK_SIZE = 128;
constexpr uint8_t FLASH_MAX_RETRIES = 8;
constexpr uint32_t FLASH_CHUNK_GAP_MS = 55;
constexpr size_t HEX_LINE_MAX = 600;
constexpr uint8_t DEFAULT_ESC_REQUIRED_FLASH_CODE = 0x1F;

WebServer server(80);
DNSServer dnsServer;

bool webRadioRunning = false;
bool webRoutesConfigured = false;

bool webFlashPending = false;
bool webFlashResumePending = false;
uint32_t webFlashStartAt = 0;
uint32_t webFlashResumeAt = 0;

uint8_t eepromData[EEPROM_SIZE_BYTES] = {0};
uint8_t eepromBeforeFlash[EEPROM_SIZE_BYTES] = {0};

uint16_t eepromAddress = 0;
uint8_t flashSizeCode = 0;
uint8_t signalPinCode = 0;

bool escConnected = false;
bool eepromLoaded = false;
bool firmwareMissing = false;
bool eepromRawAvailable = false;
bool requestBusy = false;
String lastMessage = "Chưa kết nối ESC";

struct FourWayResult {
    bool transportOk = false;
    bool crcOk = false;
    uint8_t ack = ACK_D_GENERAL_ERROR;
    uint8_t command = 0;
    uint16_t address = 0;
    uint16_t payloadLength = 0;
    uint8_t payload[256] = {0};
};

enum class HexAddressMode : uint8_t {
    Unknown,
    ZeroBased,
    AppAddress
};

enum class FlashState : uint8_t {
    Idle,
    Ready,
    Pending,
    SafetyOff,
    Writing,
    SafetyOn,
    ResetEsc,
    Done,
    Error
};

uint8_t *firmwareImage = nullptr;
uint32_t firmwareCapacity = 0;
uint32_t firmwareSize = 0;
uint32_t firmwareWritten = 0;

bool uploadInProgress = false;
bool uploadValid = false;
bool hexSawEof = false;
uint32_t hexAddressBase = 0;
HexAddressMode hexAddressMode = HexAddressMode::Unknown;
String hexLine;
String uploadError;

FlashState flashState = FlashState::Idle;
uint8_t flashRetry = 0;
uint32_t nextFlashActionMs = 0;
String flashError;

class RequestGuard {
public:
    RequestGuard() { requestBusy = true; }
    ~RequestGuard() { requestBusy = false; }
};

uint8_t clampByte(long value)
{
    if (value < 0) return 0;
    if (value > 255) return 255;
    return static_cast<uint8_t>(value);
}

String jsonEscape(const String &input)
{
    String output;
    output.reserve(input.length() + 8);

    for (size_t i = 0; i < input.length(); i++) {
        const char c = input[i];

        switch (c) {
            case '\\': output += F("\\\\"); break;
            case '"':  output += F("\\\""); break;
            case '\n': output += F("\\n"); break;
            case '\r': output += F("\\r"); break;
            case '\t': output += F("\\t"); break;
            default:
                if (static_cast<uint8_t>(c) >= 0x20) {
                    output += c;
                }
                break;
        }
    }

    return output;
}

void sendJson(int statusCode, const String &json)
{
    server.sendHeader(F("Cache-Control"), F("no-store"));
    server.send(statusCode, F("application/json; charset=utf-8"), json);
}

void sendError(int statusCode, const String &message)
{
    String json = F("{\"ok\":false,\"message\":\"");
    json += jsonEscape(message);
    json += F("\"}");
    sendJson(statusCode, json);
}

size_t makeFourWayFrame( uint8_t command, uint16_t address, const uint8_t *parameters, uint16_t parameterLength, uint8_t *frame, size_t capacity)
{
    // Check_4Way() của code gốc luôn cần ít nhất 1 byte parameter.
    if (parameterLength == 0 || parameterLength > 256) {
        return 0;
    }

    const size_t frameLength = 5U + parameterLength + 2U;
    if (frameLength > capacity) {
        return 0;
    }

    frame[0] = cmd_Local_Escape; // 0X2F
    frame[1] = command;
    frame[2] = static_cast<uint8_t>(address >> 8);
    frame[3] = static_cast<uint8_t>(address & 0xFF);
    frame[4] = parameterLength == 256 ? 0 : static_cast<uint8_t>(parameterLength);
    memcpy(&frame[5], parameters, parameterLength);
    uint16_t crc = 0;
    for (size_t i = 0; i < 5U + parameterLength; i++) {
        crc = _crc_xmodem_update(crc, frame[i]);
    }

    frame[5U + parameterLength] = static_cast<uint8_t>(crc >> 8);
    frame[6U + parameterLength] = static_cast<uint8_t>(crc & 0xFF);

    return frameLength;
}

FourWayResult runFourWay(
    uint8_t command,
    uint16_t address,
    const uint8_t *parameters,
    uint16_t parameterLength)
{
    FourWayResult result;
    result.command = command;
    result.address = address;

    uint8_t frame[FRAME_CAPACITY] = {0};

    const size_t requestLength = makeFourWayFrame(
        command,
        address,
        parameters,
        parameterLength,
        frame,
        sizeof(frame));

    if (requestLength == 0) {
        return result;
    }

    // Đây là đúng cùng hàm mà App Tool USB đang sử dụng.
    const uint16_t responseLength = Check_4Way(frame);

    if (responseLength < 8 || responseLength > sizeof(frame)) {
        return result;
    }

    result.transportOk = frame[0] == cmd_Remote_Escape && frame[1] == command;
    if (!result.transportOk) {
        return result;
    }

    const uint16_t payloadLength = frame[4] == 0 ? 256 : frame[4];

    if (payloadLength > 256 || payloadLength + 8U != responseLength) {
        return result;
    }

    result.payloadLength = payloadLength;
    memcpy(result.payload, &frame[5], payloadLength);
    result.ack = frame[5U + payloadLength];

    uint16_t crc = 0;
    for (uint16_t i = 0; i < payloadLength + 6U; i++) {
        crc = _crc_xmodem_update(crc, frame[i]);
    }

    const uint16_t receivedCrc =
        (static_cast<uint16_t>(frame[payloadLength + 6U]) << 8) | frame[payloadLength + 7U];

    result.crcOk = crc == receivedCrc;
    return result;
}

bool fourWayOk(const FourWayResult &result)
{
    return result.transportOk && result.crcOk && result.ack == ACK_OK;
}

uint16_t eepromAddressFromFlashCode(uint8_t code)
{
    switch (code) {
        case 0x1F: return 0x7C00; // 32 KB
        case 0x35: return 0xF800; // 64 KB
        case 0x2B: return 0x7E00; // 128 KB, protocol address = physical / 4
        default: return 0;
    }
}

uint32_t physicalEepromAddressFromFlashCode(uint8_t code)
{
    switch (code) {
        case 0x1F: return 0x07C00UL;
        case 0x35: return 0x0F800UL;
        case 0x2B: return 0x1F800UL;
        default: return 0;
    }
}

uint16_t protocolAddressFromPhysical(uint32_t physicalAddress)
{
    if (flashSizeCode == 0x2B) {
        physicalAddress >>= 2;
    }

    return static_cast<uint16_t>(physicalAddress & 0xFFFF);
}

uint32_t firmwareCapacityForEsc()
{
    const uint32_t eepromPhysical = physicalEepromAddressFromFlashCode(flashSizeCode);

    if (eepromPhysical <= APP_PHYSICAL_START) {
        return 0;
    }

    return eepromPhysical - APP_PHYSICAL_START;
}

void ensureEscSerial()
{
    if (!Enable4Way) {
        InitSerialOutput();
        delay(20);
    }
}

bool connectEscInternal()
{
    ensureEscSerial();

    const uint8_t mode = imARM_BLB;
    runFourWay(cmd_InterfaceSetMode, 0, &mode, 1);

    const uint8_t motor = 0;
    FourWayResult init;

    for (uint8_t attempt = 0; attempt < 3; attempt++) {
        init = runFourWay(cmd_DeviceInitFlash, 0, &motor, 1);

        if (fourWayOk(init) && init.payloadLength >= 4) {
            break;
        }

        delay(80);
    }

    if (!fourWayOk(init) || init.payloadLength < 4) {
        escConnected = false;
        eepromLoaded = false;
        lastMessage = "Không nhận được bootloader ESC";
        return false;
    }

    // Payload do 4Way.cpp gốc trả:
    // [device type, flash-size code, pin code, ARM mode]
    flashSizeCode = init.payload[1];
    signalPinCode = init.payload[2];
    eepromAddress = eepromAddressFromFlashCode(flashSizeCode);

    if (eepromAddress == 0) {
        escConnected = false;
        eepromLoaded = false;
        lastMessage = "Flash code ESC chưa được hỗ trợ";
        return false;
    }

    escConnected = true;
    lastMessage = "Đã kết nối bootloader ESC";
    delay(400);

    return true;
}

bool readMemory( uint16_t address, uint16_t length, uint8_t *destination)
{
    if (length == 0 || length > 256 || destination == nullptr) {
        return false;
    }

    const uint8_t readLength = length == 256 ? 0 : static_cast<uint8_t>(length);

    for (uint8_t attempt = 0; attempt < 2; ++attempt) {
        const FourWayResult result = runFourWay(
            cmd_DeviceRead,
            address,
            &readLength,
            1);

        if (fourWayOk(result) && result.payloadLength == length) {
            memcpy(destination, result.payload, length);
            return true;
        }

        delay(80);
    }

    return false;
}

bool writeMemory( uint16_t address, const uint8_t *data, uint16_t length)
{
    if (length == 0 || length > 256 ||
        data == nullptr) {
        return false;
    }
    const FourWayResult result = runFourWay( cmd_DeviceWrite, address, data, length);
    return fourWayOk(result);
}

bool validEepromHeader( const uint8_t *data, uint16_t length)
{
    if (data == nullptr || length < 5) {
        return false;
    }

    // 1 = firmware hợp lệ; 0 = lần nạp trước chưa hoàn tất.
    // Vẫn cho đọc byte 0 để người dùng có thể nạp lại firmware recovery.
    if (data[0] != 0 && data[0] != 1) {
        return false;
    }

    if (data[3] == 0xFF || data[4] == 0xFF) {
        return false;
    }

    if (data[3] == 0 && data[4] == 0) {
        return false;
    }

    return true;
}

bool readEepromInternal()
{
    if (!escConnected && !connectEscInternal()) {
        return false;
    }

    uint8_t first[EEPROM_SIZE_BYTES] = {0};
    uint8_t second[EEPROM_SIZE_BYTES] = {0};

    firmwareMissing = false;
    eepromRawAvailable = false;

    for (uint8_t attempt = 0; attempt < 3; attempt++) {
        const bool firstOk = readMemory(
            eepromAddress,
            EEPROM_SIZE_BYTES,
            first);

        delay(220);

        const bool secondOk = readMemory(
            eepromAddress,
            EEPROM_SIZE_BYTES,
            second);

        if (firstOk && secondOk && memcmp(first, second, EEPROM_SIZE_BYTES) == 0) {

            // Luôn giữ lại block EEPROM gốc. Khi firmware chưa tồn tại,
            // block này vẫn được dùng để khóa/mở safety byte lúc recovery.
            memcpy(eepromData, second, EEPROM_SIZE_BYTES);
            eepromRawAvailable = true;

            if (validEepromHeader(second, EEPROM_SIZE_BYTES)) {
                eepromLoaded = true;
                firmwareMissing = false;

                if (second[0] == 0) {
                    lastMessage = "ESC is in recovery mode (safety byte = 0). " "Please reflash the firmware.";
                } else {
                    lastMessage ="Đã đọc EEPROM 48 byte tại 0x" + String(eepromAddress, HEX);
                }

                return true;
            }

            // Bootloader kết nối và EEPROM đọc ổn định, nhưng không có
            // firmware/version hợp lệ. Cho phép chuyển thẳng sang Flash.
            eepromLoaded = false;
            firmwareMissing = true;
            lastMessage = "No firmware detected. Please upload firmware first.";
            return false;
        }

        delay(260);
    }

    eepromLoaded = false;
    firmwareMissing = false;
    eepromRawAvailable = false;
    lastMessage = "Không đọc ổn định được EEPROM từ bootloader";
    return false;
}

bool argToByte(const char *name, uint8_t &target)
{
    if (!server.hasArg(name)) {
        return false;
    }

    target = clampByte(server.arg(name).toInt());
    return true;
}

void applyKnownArguments()
{
    argToByte("max_ramp", eepromData[5]);
    argToByte("minimum_duty_cycle", eepromData[6]);
    argToByte("disable_stick_calibration", eepromData[7]);
    argToByte("absolute_voltage_cutoff", eepromData[8]);
    argToByte("current_P", eepromData[9]);
    argToByte("current_I", eepromData[10]);
    argToByte("current_D", eepromData[11]);
    argToByte("active_brake_power", eepromData[12]);

    argToByte("dir_reversed", eepromData[17]);
    argToByte("bi_direction", eepromData[18]);
    eepromData[19] =1 ;
    eepromData[20] =1 ;
    argToByte("variable_pwm", eepromData[21]);
    argToByte("stuck_rotor_protection", eepromData[22]);
    argToByte("advance_level", eepromData[23]);
    argToByte("pwm_frequency", eepromData[24]);
    argToByte("startup_power", eepromData[25]);
    argToByte("motor_kv", eepromData[26]);
    argToByte("motor_poles", eepromData[27]);
    argToByte("brake_on_stop", eepromData[28]);
    argToByte("stall_protection", eepromData[29]);
    argToByte("beep_volume", eepromData[30]);
    argToByte("telemetry_on_interval", eepromData[31]);

    argToByte("servo_low_threshold", eepromData[32]);
    argToByte("servo_high_threshold", eepromData[33]);
    argToByte("servo_neutral", eepromData[34]);
    argToByte("servo_dead_band", eepromData[35]);
    argToByte("low_voltage_cut_off", eepromData[36]);
    argToByte("low_cell_volt_cutoff", eepromData[37]);
    argToByte("rc_car_reverse", eepromData[38]);
    argToByte("use_hall_sensors", eepromData[39]);
    argToByte("sine_changeover", eepromData[40]);
    argToByte("drag_brake_strength", eepromData[41]);
    argToByte("driving_brake_strength", eepromData[42]);
    argToByte("temperature_limit", eepromData[43]);
    argToByte("current_limit", eepromData[44]);
    argToByte("sine_mode_power", eepromData[45]);
    argToByte("input_type", eepromData[46]);
    argToByte("auto_advance", eepromData[47]);

    eepromData[0] = 1;
}

String flashStateName()
{
    switch (flashState) {
        case FlashState::Idle:      return "idle";
        case FlashState::Ready:     return "ready";
        case FlashState::Pending:   return "pending";
        case FlashState::SafetyOff: return "safety";
        case FlashState::Writing:   return "writing";
        case FlashState::SafetyOn:  return "finishing";
        case FlashState::ResetEsc:  return "reset";
        case FlashState::Done:      return "done";
        case FlashState::Error:     return "error";
    }

    return "unknown";
}

bool flashIsActive()
{
    return flashState == FlashState::Pending ||
           flashState == FlashState::SafetyOff ||
           flashState == FlashState::Writing ||
           flashState == FlashState::SafetyOn ||
           flashState == FlashState::ResetEsc;
}

String statusJson(bool includeSettings)
{
    String json;
    json.reserve(includeSettings ? 2800 : 700);

    json += F("{\"ok\":true,\"connected\":");
    json += escConnected ? F("true") : F("false");

    json += F(",\"loaded\":");
    json += eepromLoaded ? F("true") : F("false");

    json += F(",\"firmwareMissing\":");
    json += firmwareMissing ? F("true") : F("false");

    json += F(",\"recovery\":");
    json += (eepromLoaded && eepromData[0] == 0)
        ? F("true")
        : F("false");

    json += F(",\"busy\":");
    json += webBridgeLocksEsc() ? F("true") : F("false");

    json += F(",\"message\":\"");
    json += jsonEscape(lastMessage);
    json += F("\"");

    json += F(",\"mode\":\"");
    json += systemModeName();
    json += F("\"");

    json += F(",\"wifiRunning\":");
    json += webRadioRunning ? F("true") : F("false");

    json += F(",\"gpio\":");
    json += SERVO_OUT;

    json += F(",\"flashCode\":");
    json += flashSizeCode;

    json += F(",\"pinCode\":");
    json += signalPinCode;

    json += F(",\"eepromAddress\":");
    json += eepromAddress;

    json += F(",\"eepromLength\":");
    json += eepromLoaded ? EEPROM_SIZE_BYTES : 0;

    json += F(",\"firmwareUploaded\":");
    json += uploadValid ? F("true") : F("false");

    json += F(",\"firmwareSize\":");
    json += firmwareSize;

    json += F(",\"flashState\":\"");
    json += flashStateName();
    json += F("\"");

    json += F(",\"flashWritten\":");
    json += firmwareWritten;

    json += F(",\"flashError\":\"");
    json += jsonEscape(flashError);
    json += F("\"");

    if (eepromLoaded) {
        json += F(",\"firmwareMajor\":");
        json += eepromData[3];

        json += F(",\"firmwareMinor\":");
        json += eepromData[4];

        json += F(",\"eepromVersion\":");
        json += eepromData[1];
    }

    if (includeSettings && eepromLoaded) {
        json += F(",\"raw\":{");

        const struct {
            const char *name;
            uint8_t offset;
        } fields[] = {
            {"max_ramp",5},
            {"minimum_duty_cycle",6},
            {"disable_stick_calibration",7},
            {"absolute_voltage_cutoff",8},
            {"current_P",9},
            {"current_I",10},
            {"current_D",11},
            {"active_brake_power",12},
            {"dir_reversed",17},
            {"bi_direction",18},
            {"use_sine_start",19},
            {"comp_pwm",20},
            {"variable_pwm",21},
            {"stuck_rotor_protection",22},
            {"advance_level",23},
            {"pwm_frequency",24},
            {"startup_power",25},
            {"motor_kv",26},
            {"motor_poles",27},
            {"brake_on_stop",28},
            {"stall_protection",29},
            {"beep_volume",30},
            {"telemetry_on_interval",31},
            {"servo_low_threshold",32},
            {"servo_high_threshold",33},
            {"servo_neutral",34},
            {"servo_dead_band",35},
            {"low_voltage_cut_off",36},
            {"low_cell_volt_cutoff",37},
            {"rc_car_reverse",38},
            {"use_hall_sensors",39},
            {"sine_changeover",40},
            {"drag_brake_strength",41},
            {"driving_brake_strength",42},
            {"temperature_limit",43},
            {"current_limit",44},
            {"sine_mode_power",45},
            {"input_type",46},
            {"auto_advance",47}
        };

        for (size_t i = 0;
             i < sizeof(fields) / sizeof(fields[0]);
             i++) {

            if (i) {
                json += ',';
            }

            json += '"';
            json += fields[i].name;
            json += F("\":");
            json += eepromData[fields[i].offset];
        }

        json += '}';
    }

    json += '}';
    return json;
}

void clearFirmwareImage()
{
    if (firmwareImage != nullptr) {
        free(firmwareImage);
        firmwareImage = nullptr;
    }

    firmwareCapacity = 0;
    firmwareSize = 0;
    firmwareWritten = 0;
    uploadValid = false;
    hexSawEof = false;
    hexAddressBase = 0;
    hexAddressMode = HexAddressMode::Unknown;
    hexLine = "";
    uploadError = "";
}

bool consumeHexData(const uint8_t *data, size_t length);

bool prepareDefaultFirmwareImage()
{
    clearFirmwareImage();

    if (flashSizeCode != DEFAULT_ESC_REQUIRED_FLASH_CODE) {
        uploadError =
            "The built-in firmware is only compatible with AT32F421 ESCs "
            "(flash code 0x1F)";
        return false;
    }

    firmwareCapacity = firmwareCapacityForEsc();

    if (firmwareCapacity == 0) {
        uploadError = "The ESC firmware area is not valid";
        return false;
    }

    firmwareImage =
        static_cast<uint8_t *>(malloc(firmwareCapacity));

    if (firmwareImage == nullptr) {
        firmwareCapacity = 0;
        uploadError = "Not enough memory to prepare the built-in firmware";
        return false;
    }

    memset(firmwareImage, 0, firmwareCapacity);
    hexLine.reserve(HEX_LINE_MAX);

    uint8_t chunk[128];
    size_t sourceOffset = 0;
    while (sourceOffset < VPTEK_BUILTIN_FIRMWARE_HEX_LENGTH) {
        const size_t count = min(
            sizeof(chunk),
            VPTEK_BUILTIN_FIRMWARE_HEX_LENGTH - sourceOffset);

        for (size_t i = 0; i < count; ++i) {
            chunk[i] = pgm_read_byte(
                VPTEK_BUILTIN_FIRMWARE_HEX + sourceOffset + i);
        }

        if (!consumeHexData(chunk, count)) {
            const String error = uploadError;
            clearFirmwareImage();
            uploadError = error;
            return false;
        }
        sourceOffset += count;
    }

    if (!hexSawEof || firmwareSize == 0) {
        clearFirmwareImage();
        uploadError = "The built-in Intel HEX data is incomplete";
        return false;
    }

    firmwareWritten = 0;
    uploadValid = true;
    flashState = FlashState::Ready;
    flashError = "";
    lastMessage =
        "Built-in firmware ready: " +
        String(firmwareSize) + " bytes";
    return true;
}

int hexNibble(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

bool decodeHexByte(
    const String &line,
    size_t index,
    uint8_t &value)
{
    if (index + 1 >= line.length()) {
        return false;
    }

    const int high = hexNibble(line[index]);
    const int low = hexNibble(line[index + 1]);

    if (high < 0 || low < 0) {
        return false;
    }

    value = static_cast<uint8_t>((high << 4) | low);
    return true;
}

bool parseIntelHexLine(const String &line)
{
    if (line.length() == 0) {
        return true;
    }

    if (line[0] != ':') {
        uploadError = "Dòng HEX không bắt đầu bằng ':'";
        return false;
    }

    if (((line.length() - 1U) & 1U) != 0U) {
        uploadError = "Dòng HEX có số ký tự không hợp lệ";
        return false;
    }

    const size_t byteTotal = (line.length() - 1U) / 2U;

    if (byteTotal < 5 || byteTotal > 260) {
        uploadError = "Độ dài dòng HEX không hợp lệ";
        return false;
    }

    uint8_t decoded[260] = {0};

    for (size_t i = 0; i < byteTotal; i++) {
        if (!decodeHexByte(line, 1U + i * 2U, decoded[i])) {
            uploadError = "File HEX chứa ký tự sai";
            return false;
        }
    }

    const uint8_t dataLength = decoded[0];

    if (byteTotal != static_cast<size_t>(dataLength) + 5U) {
        uploadError = "Độ dài record HEX không khớp";
        return false;
    }

    uint8_t checksum = 0;
    for (size_t i = 0; i < byteTotal; i++) {
        checksum = static_cast<uint8_t>(checksum + decoded[i]);
    }

    if (checksum != 0) {
        uploadError = "Checksum Intel HEX bị sai";
        return false;
    }

    const uint16_t recordAddress =
        (static_cast<uint16_t>(decoded[1]) << 8) |
        decoded[2];

    const uint8_t recordType = decoded[3];

    if (recordType == 0x00) {
        uint32_t fullAddress =
            hexAddressBase + recordAddress;

        // Chuyển địa chỉ ARM tuyệt đối 0x080xxxxx về offset Flash.
        if (fullAddress >= 0x08000000UL &&
            fullAddress < 0x09000000UL) {
            fullAddress -= 0x08000000UL;
        }

        if (hexAddressMode == HexAddressMode::Unknown) {
            hexAddressMode =
                fullAddress >= APP_PHYSICAL_START
                ? HexAddressMode::AppAddress
                : HexAddressMode::ZeroBased;
        }

        uint32_t outputOffset = 0;

        if (hexAddressMode == HexAddressMode::AppAddress) {
            if (fullAddress < APP_PHYSICAL_START) {
                uploadError = "HEX trộn địa chỉ 0x0000 và 0x1000";
                return false;
            }

            outputOffset =
                fullAddress - APP_PHYSICAL_START;
        } else {
            outputOffset = fullAddress;
        }

        if (outputOffset + dataLength >
            firmwareCapacity) {
            uploadError =
                "Firmware vượt quá vùng Flash trước EEPROM";
            return false;
        }

        memcpy(
            firmwareImage + outputOffset,
            &decoded[4],
            dataLength);

        const uint32_t endOffset =
            outputOffset + dataLength;

        if (endOffset > firmwareSize) {
            firmwareSize = endOffset;
        }

        return true;
    }

    if (recordType == 0x01) {
        hexSawEof = true;
        return true;
    }

    if (recordType == 0x02) {
        if (dataLength != 2) {
            uploadError = "Record segment HEX không hợp lệ";
            return false;
        }

        hexAddressBase =
            (static_cast<uint32_t>(decoded[4]) << 12) |
            (static_cast<uint32_t>(decoded[5]) << 4);

        return true;
    }

    if (recordType == 0x04) {
        if (dataLength != 2) {
            uploadError = "Record linear HEX không hợp lệ";
            return false;
        }

        hexAddressBase =
            (static_cast<uint32_t>(decoded[4]) << 24) |
            (static_cast<uint32_t>(decoded[5]) << 16);

        return true;
    }

    // Start Segment Address và Start Linear Address không chứa firmware data.
    if (recordType == 0x03 ||
        recordType == 0x05) {
        return true;
    }

    uploadError = "Intel HEX record type chưa hỗ trợ";
    return false;
}

bool consumeHexData(
    const uint8_t *data,
    size_t length)
{
    for (size_t i = 0; i < length; i++) {
        const char c = static_cast<char>(data[i]);

        if (c == '\r') {
            continue;
        }

        if (c == '\n') {
            if (!parseIntelHexLine(hexLine)) {
                return false;
            }

            hexLine = "";
            continue;
        }

        if (hexLine.length() >= HEX_LINE_MAX) {
            uploadError = "Dòng Intel HEX quá dài";
            return false;
        }

        hexLine += c;
    }

    return true;
}

void handleFirmwareUploadData()
{
    HTTPUpload &upload = server.upload();

    if (upload.status == UPLOAD_FILE_START) {
        uploadInProgress = true;
        clearFirmwareImage();

        if (!escConnected) {
            uploadError =
                "ESC bootloader is not connected";
            return;
        }

        if (!eepromLoaded &&
            (!firmwareMissing || !eepromRawAvailable)) {
            uploadError =
                "EEPROM could not be read reliably. Reconnect the ESC.";
            return;
        }

        firmwareCapacity = firmwareCapacityForEsc();

        if (firmwareCapacity == 0) {
            uploadError = "Không xác định được dung lượng Flash ESC";
            return;
        }

        firmwareImage =
            static_cast<uint8_t *>(malloc(firmwareCapacity));

        if (firmwareImage == nullptr) {
            uploadError =
                "Thiết bị không đủ RAM để chứa firmware";
            return;
        }

        // Config TooL lấp khoảng trống HEX bằng 0x00.
        memset(firmwareImage, 0x00, firmwareCapacity);

        firmwareSize = 0;
        firmwareWritten = 0;
        hexAddressBase = 0;
        hexAddressMode = HexAddressMode::Unknown;
        hexSawEof = false;
        hexLine.reserve(HEX_LINE_MAX);
        flashState = FlashState::Idle;
        flashError = "";
    }

    if (upload.status == UPLOAD_FILE_WRITE) {
        if (uploadError.length() != 0 || firmwareImage == nullptr) {
            return;
        }

        if (!consumeHexData( upload.buf, upload.currentSize)) {
            return;
        }
    }

    if (upload.status == UPLOAD_FILE_END) {
        uploadInProgress = false;

        if (uploadError.length() == 0 && hexLine.length() != 0) {
            parseIntelHexLine(hexLine);
            hexLine = "";
        }

        if (uploadError.length() == 0 && !hexSawEof) {
            uploadError = "File Intel HEX không có record EOF";
        }

        if (uploadError.length() == 0 && firmwareSize == 0) {
            uploadError = "File HEX không có firmware data";
        }

        if (uploadError.length() == 0) {
            uploadValid = true;
            flashState = FlashState::Ready;
            lastMessage = "Đã tải HEX: " + String(firmwareSize) + " byte";
        } else {
            uploadValid = false;
            flashState = FlashState::Error;
            flashError = uploadError;
            lastMessage = uploadError;
        }
    }

    if (upload.status == UPLOAD_FILE_ABORTED) {
        uploadInProgress = false;
        uploadValid = false;
        uploadError = "Upload firmware bị hủy";
        flashState = FlashState::Error;
        flashError = uploadError;
        lastMessage = uploadError;
    }
}

void handleFirmwareUploadFinished()
{
    if (uploadError.length() != 0) {
        sendError(400, uploadError);
        return;
    }

    if (!uploadValid) {
        sendError(400, "Firmware chưa được tải hợp lệ");
        return;
    }

    sendJson(200, statusJson(false));
}

void setFlashFailure(const String &message)
{
    flashState = FlashState::Error;
    flashError = message;
    lastMessage = message;
}

void beginFlash()
{
    if (!eepromRawAvailable) {
        setFlashFailure(
            "EEPROM is unavailable. Reconnect the ESC before flashing.");
        return;
    }

    memcpy(
        eepromBeforeFlash,
        eepromData,
        EEPROM_SIZE_BYTES);

    firmwareWritten = 0;
    flashRetry = 0;
    nextFlashActionMs = millis();
    flashError = "";
    flashState = FlashState::SafetyOff;
    lastMessage = "Đang khóa safety byte";
}

void processFlashStep()
{
    if (!flashIsActive()) {
        return;
    }

    if (static_cast<int32_t>(
            millis() - nextFlashActionMs) < 0) {
        return;
    }

    if (flashState == FlashState::SafetyOff) {
        uint8_t safety[EEPROM_SIZE_BYTES] = {0};

        memcpy(
            safety,
            eepromBeforeFlash,
            EEPROM_SIZE_BYTES);

        safety[0] = 0;

        if (writeMemory(
                eepromAddress,
                safety,
                EEPROM_SIZE_BYTES)) {

            // Phản ánh đúng trạng thái vật lý để nếu nạp lỗi,
            // Web lập tức hiện chế độ recovery sau khi Wi-Fi bật lại.
            eepromData[0] = 0;

            flashRetry = 0;
            flashState = FlashState::Writing;
            lastMessage = "Đang nạp firmware";
            nextFlashActionMs =
                millis() + FLASH_CHUNK_GAP_MS;
            return;
        }

        flashRetry++;

        if (flashRetry > FLASH_MAX_RETRIES) {
            setFlashFailure("Không ghi được safety byte EEPROM");
            return;
        }

        nextFlashActionMs = millis() + 180;
        return;
    }

    if (flashState == FlashState::Writing) {
        if (firmwareWritten >= firmwareSize) {
            flashRetry = 0;
            flashState = FlashState::SafetyOn;
            lastMessage = "Đang hoàn tất firmware";
            nextFlashActionMs = millis() + 120;
            return;
        }

        const uint32_t remaining =
            firmwareSize - firmwareWritten;

        const uint16_t chunkLength =
            static_cast<uint16_t>( remaining > FLASH_CHUNK_SIZE ? FLASH_CHUNK_SIZE : remaining);

        const uint32_t physicalAddress = APP_PHYSICAL_START + firmwareWritten;
        const uint16_t protocolAddress = protocolAddressFromPhysical( physicalAddress);

        if (writeMemory( protocolAddress,firmwareImage + firmwareWritten,chunkLength)) {

            firmwareWritten += chunkLength;
            flashRetry = 0;

            const uint32_t percent = firmwareSize == 0 ? 0 : (firmwareWritten * 100UL) / firmwareSize;
            lastMessage = "Đang nạp firmware " + String(percent) + "%";

            nextFlashActionMs =
                millis() + FLASH_CHUNK_GAP_MS;

            return;
        }

        flashRetry++;

        if (flashRetry > FLASH_MAX_RETRIES) {
            setFlashFailure( "ESC không ACK tại offset " + String(firmwareWritten));
            return;
        }

        nextFlashActionMs = millis() + 160;
        return;
    }

    if (flashState == FlashState::SafetyOn) {
        uint8_t restored[EEPROM_SIZE_BYTES] = {0};

        memcpy( restored, eepromBeforeFlash, EEPROM_SIZE_BYTES);

        restored[0] = 1;

        if (writeMemory( eepromAddress, restored, EEPROM_SIZE_BYTES)) {
            memcpy( eepromData, restored, EEPROM_SIZE_BYTES);

            flashRetry = 0;
            flashState = FlashState::ResetEsc;
            lastMessage = "Firmware đã ghi xong";
            nextFlashActionMs = millis() + 250;
            return;
        }

        flashRetry++;

        if (flashRetry > FLASH_MAX_RETRIES) {
            setFlashFailure("Firmware đã ghi nhưng chưa mở được safety byte");
            return;
        }

        nextFlashActionMs = millis() + 200;
        return;
    }

    if (flashState == FlashState::ResetEsc) {
        const uint8_t motor = 0;
        runFourWay( cmd_DeviceReset, 0, &motor, 1);

        // EEPROM trong RAM thuộc firmware trước khi nạp.
        // Buộc đọc lại sau tiếng bíp để không hiển thị phiên bản/thông số cũ.
        escConnected = false;
        eepromLoaded = false;
        firmwareMissing = false;
        eepromRawAvailable = false;

        flashState = FlashState::Done;
        flashRetry = 0;
        lastMessage =
            "Nạp firmware thành công. Kết nối lại và đọc Settings";
        return;
    }
}

void handleConnect()
{
    if (isAppMode()) {
        sendError(409, "App Tool đang giữ quyền đến khi reset thiết bị");
        return;
    }

    if (isWaitMode() && !selectWebMode()) {
        sendError(409, "Không thể chọn chế độ Web");
        return;
    }

    if (webBridgeLocksEsc()) {
        sendError(409, "Đang có thao tác khác");
        return;
    }

    RequestGuard guard;

    escConnected = false;
    eepromLoaded = false;
    firmwareMissing = false;
    eepromRawAvailable = false;

    if (!connectEscInternal()) {
        sendJson(503, statusJson(false));
        return;
    }

    const bool settingsOk = readEepromInternal();

    if (settingsOk) {
        sendJson(200, statusJson(true));
        return;
    }

    if (firmwareMissing && eepromRawAvailable) {
        // Bootloader đã kết nối. Không coi thiếu firmware là lỗi kết nối.
        escConnected = true;
        sendJson(200, statusJson(false));
        return;
    }

    sendJson(503, statusJson(false));
}

void handleSave()
{
    if (isAppMode()) {
        sendError(409, "App Tool đang giữ quyền đến khi reset thiết bị");
        return;
    }

    if (isWaitMode() && !selectWebMode()) {
        sendError(409, "Không thể chọn chế độ Web");
        return;
    }

    if (webBridgeLocksEsc()) {
        sendError(409, "Đang có thao tác khác");
        return;
    }

    if (!escConnected || !eepromLoaded) {
        sendError( 409, "Phải bấm Đọc & Config trước khi lưu");
        return;
    }

    RequestGuard guard;

    uint8_t first[EEPROM_SIZE_BYTES] = {0};
    uint8_t second[EEPROM_SIZE_BYTES] = {0};

    bool stableBaseline = false;
    for (uint8_t attempt = 0; attempt < 3; ++attempt) {
        if (readMemory(eepromAddress, EEPROM_SIZE_BYTES, first)) {
            delay(180);
            if (readMemory(eepromAddress, EEPROM_SIZE_BYTES, second) &&
                memcmp(first, second, EEPROM_SIZE_BYTES) == 0 &&
                validEepromHeader(second, EEPROM_SIZE_BYTES)) {
                stableBaseline = true;
                break;
            }
        }
        delay(180);
    }

    if (!stableBaseline) {
        sendError( 409, "EEPROM nền chưa ổn định");
        return;
    }

    if (second[0] != 1) {
        memcpy( eepromData, second, EEPROM_SIZE_BYTES);

        sendError( 409, "ESC đang ở recovery (safety byte = 0). " "Hãy nạp firmware trước khi lưu Settings");
        return;
    }

    memcpy( eepromData, second, EEPROM_SIZE_BYTES);
    applyKnownArguments();

    bool writeOk = false;

    for (uint8_t attempt = 0;
         attempt < 3;
         attempt++) {

        if (writeMemory( eepromAddress, eepromData, EEPROM_SIZE_BYTES)) {
            writeOk = true;
            break;
        }

        delay(260);
    }

    if (!writeOk) {
        sendError( 503, "Ghi EEPROM thất bại");
        return;
    }

    delay(350);

    uint8_t verify[EEPROM_SIZE_BYTES] = {0};

    if (!readMemory( eepromAddress, EEPROM_SIZE_BYTES, verify)) {

        sendError( 503, "Đã ghi nhưng không đọc lại được");
        return;
    }

    for (uint16_t i = 0;
         i < EEPROM_SIZE_BYTES; i++) {

        // Byte 2 có thể do bootloader cập nhật.
        if (i != 2 && verify[i] != eepromData[i]) {
            sendError( 500, "Verify EEPROM sai tại offset " + String(i));
            return;
        }
    }

    memcpy( eepromData, verify, EEPROM_SIZE_BYTES);

    lastMessage = "Lưu EEPROM thành công";
    sendJson(200, statusJson(true));
}

void handleReset()
{
    if (isAppMode()) {
        sendError(409, "App Tool đang giữ quyền đến khi reset thiết bị");
        return;
    }

    if (isWaitMode() && !selectWebMode()) {
        sendError(409, "Không thể chọn chế độ Web");
        return;
    }

    if (webBridgeLocksEsc()) {
        sendError(409, "Đang có thao tác khác");
        return;
    }

    if (!escConnected) {
        sendError(409, "ESC chưa kết nối");
        return;
    }

    RequestGuard guard;

    const uint8_t motor = 0;
    const FourWayResult result = runFourWay( cmd_DeviceReset,0,&motor,1);

    escConnected = false;
    eepromLoaded = false;

    lastMessage = fourWayOk(result) ? "Đã reset ESC" : "Đã gửi lệnh reset ESC";

    sendJson(200, statusJson(false));
}

void handleFlashStart()
{
    if (isAppMode()) {
        sendError(409, "App Tool đang giữ quyền đến khi reset thiết bị");
        return;
    }

    if (isWaitMode() && !selectWebMode()) {
        sendError(409, "Không thể chọn chế độ Web");
        return;
    }

    if (webBridgeLocksEsc()) {
        sendError(409, "Đang có thao tác khác");
        return;
    }

    if (!escConnected) {
        sendError( 409,"ESC bootloader is not connected");
        return;
    }

    if (!eepromLoaded && (!firmwareMissing || !eepromRawAvailable)) {
        sendError( 409, "EEPROM could not be read reliably. Reconnect the ESC.");
        return;
    }

    if (!uploadValid || firmwareImage == nullptr || firmwareSize == 0) {
        sendError( 409, "Chưa tải file Intel HEX hợp lệ");
        return;
    }

    /*
     * Trả HTTP trước. Sau một khoảng ngắn mới tắt Wi-Fi
     * và bắt đầu ghi firmware từ RAM xuống ESC.
     */
    webFlashPending = true;
    webFlashStartAt = millis() + 800;
    flashState = FlashState::Pending;
    flashError = "";
    lastMessage =
        "Đã nhận firmware, chuẩn bị tạm ngắt Wi-Fi";

    sendJson(202, statusJson(false));
}

void handleDefaultFirmwareStart()
{
    if (isAppMode()) {
        sendError(409, "App Tool mode is active until the device is reset");
        return;
    }

    if (isWaitMode() && !selectWebMode()) {
        sendError(409, "Web mode could not be selected");
        return;
    }

    if (webBridgeLocksEsc()) {
        sendError(409, "Another operation is in progress");
        return;
    }

    if (!escConnected) {
        sendError(409, "ESC bootloader is not connected");
        return;
    }

    if (!eepromLoaded && (!firmwareMissing || !eepromRawAvailable)) {
        sendError(
            409,
            "EEPROM could not be read reliably. Reconnect the ESC.");
        return;
    }

    if (!prepareDefaultFirmwareImage()) {
        flashState = FlashState::Error;
        flashError = uploadError;
        lastMessage = uploadError;
        sendError(409, uploadError);
        return;
    }

    handleFlashStart();
}

void handleFlashStatus()
{
    sendJson(200, statusJson(false));
}

void handleFlashClear()
{
    if (flashIsActive()) {
        sendError( 409, "Không thể xóa file khi đang nạp");
        return;
    }

    clearFirmwareImage();
    flashState = FlashState::Idle;
    flashError = "";
    lastMessage = "Đã xóa firmware tạm";
    sendJson(200, statusJson(false));
}

void redirectToPortal()
{
    server.sendHeader( F("Location"), F("http://192.168.4.1/"), true);

    server.send(302, F("text/plain"), "");
}

const char INDEX_HTML[] PROGMEM = R"VPTEKWEB(
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title> ESC Control Center</title>
<style>
*{box-sizing:border-box}
:root{--border:#111;--soft:#d8d8d8;--bg:#fff;--muted:#666}
body{margin:0;background:var(--bg);color:#000;font:14px Arial,sans-serif}
.wrap{max-width:1180px;margin:auto;padding:14px}
.top{display:flex;align-items:center;justify-content:space-between;gap:10px;flex-wrap:wrap}
h1{font-size:22px;margin:0}
.status{display:flex;align-items:center;gap:8px;border:1px solid var(--border);padding:8px 10px}
.dot{width:10px;height:10px;border:1px solid #000;border-radius:50%;background:#fff}
.dot.ok{background:#000}
.actions{display:flex;gap:8px;flex-wrap:wrap;margin:14px 0}
button,.fileBtn{border:1px solid #000;background:#fff;color:#000;padding:9px 13px;font-weight:700;cursor:pointer}
button:hover,.fileBtn:hover{background:#eee}
button:disabled{opacity:.45;cursor:not-allowed}
.meta{display:grid;grid-template-columns:repeat(auto-fit,minmax(145px,1fr));gap:8px;margin-bottom:12px}
.meta div{border:1px solid #000;padding:8px}
.meta b{display:block;font-size:11px}
.meta span{font-size:15px}
.tabs{display:flex;overflow:auto;border-bottom:1px solid #000}
.tab{border:0;border-bottom:3px solid transparent;padding:10px 14px;white-space:nowrap}
.tab.active{border-bottom-color:#000}
.page{display:none;padding-top:12px}
.page.active{display:block}
.grid{display:grid;grid-template-columns:repeat(3,minmax(260px,1fr));gap:10px}
.grid.two{grid-template-columns:repeat(2,minmax(280px,1fr))}
.grid.one{grid-template-columns:minmax(0,1fr)}
.card{border:1px solid #000;padding:12px}
.card h3{font-size:14px;margin:0 0 10px}
.row{display:grid;grid-template-columns:1fr 128px;gap:10px;align-items:center;padding:7px 0;border-bottom:1px solid var(--soft)}
.row:last-child{border-bottom:0}
.row.wide{grid-template-columns:1fr}
.inline{display:grid;grid-template-columns:22px 1fr;gap:8px;align-items:center}
input[type=number],select,input[type=file]{width:100%;border:1px solid #000;background:#fff;padding:7px}
input[type=range]{width:100%}
input[type=checkbox]{width:18px;height:18px}
.checkboxRow{display:flex;align-items:center;gap:8px;padding:7px 0;border-bottom:1px solid var(--soft)}
.checkboxRow:last-child{border-bottom:0}
.valueWithUnit{display:grid;grid-template-columns:1fr auto;gap:6px;align-items:center}
.unit{min-width:28px;color:#333}
progress{width:100%;height:26px}
.flashLine{display:flex;gap:8px;align-items:center;flex-wrap:wrap;margin-top:10px}
.defaultFirmware{
  display:grid;
  grid-template-columns:1fr auto;
  gap:14px;
  align-items:center;
  margin:0 0 16px;
  padding:14px;
  border:1px solid #bfdbfe;
  border-radius:12px;
  background:#eff6ff;
}
.defaultFirmware strong{display:block;margin-bottom:3px;color:#1e3a8a}
.defaultFirmware span{display:block;color:#475467;font-size:12px}
.defaultFirmware button{white-space:nowrap}
.configActions{
  display:grid;
  grid-template-columns:repeat(3,minmax(0,1fr));
  gap:8px;
}

.configActions button,
.configActions .fileBtn{
  width:100%;
  min-width:0;
  margin:0;
  display:flex;
  align-items:center;
  justify-content:center;
  text-align:center;
  white-space:nowrap;
}

.card{
  min-width:0;
}
.flashLine span{font-weight:700}
.flashNote{border:2px solid #000;padding:12px;margin-top:12px;font-weight:700;line-height:1.55}
.recoveryBanner{display:none;border:3px solid #000;padding:12px;margin:12px 0;font-weight:700;line-height:1.5}
.mobileHint{border:1px dashed #000;padding:10px;margin-top:10px;line-height:1.45}
.motorPanel{opacity:.62}
.motorReadout{display:grid;grid-template-columns:85px 1fr 65px;gap:10px;align-items:center;margin:12px 0}
.small{font-size:12px;color:var(--muted);line-height:1.45}
.toast{position:fixed;right:14px;bottom:14px;max-width:460px;background:#fff;border:2px solid #000;padding:11px 14px;display:none;z-index:20}
.toast.err{border-style:dashed}
.busy:after{content:'Processing...';position:fixed;inset:0;background:rgba(255,255,255,.92);display:grid;place-items:center;font-size:22px;font-weight:700;z-index:10}

.flashSpinner{
  position:relative;
  width:96px;
  height:96px;
  margin:18px auto;
  display:none;
}

.flashSpinner.active{
  display:block;
}

.flashSpinner span{
  position:absolute;
  left:50%;
  top:50%;
  width:15px;
  height:29px;
  margin-left:-7.5px;
  margin-top:-14.5px;
  border-radius:7px;
  background:#000;
  transform:rotate(calc(var(--i) * 30deg)) translateY(-32px);
  animation:flashSpinnerFade 1.2s linear infinite;
  animation-delay:calc(var(--i) * -0.1s);
}

@keyframes flashSpinnerFade{
  0%{opacity:1;background:#000}
  35%{opacity:.72;background:#555}
  70%{opacity:.32;background:#aaa}
  100%{opacity:.12;background:#fff}
}

@media(max-width:920px){.grid{grid-template-columns:1fr 1fr}}
@media(max-width:650px){
  .grid,
  .grid.two{
    grid-template-columns:1fr;
  }

  .row{
    grid-template-columns:1fr 112px;
  }

  .actions button,
  .actions .fileBtn{
    flex:1;
  }

  .configActions{
    grid-template-columns:1fr;
  }

  .configActions button,
  .configActions .fileBtn{
    min-height:46px;
  }

  .defaultFirmware{
    grid-template-columns:1fr;
  }

  .defaultFirmware button{
    width:100%;
  }
}
/* Commercial UI layer — local assets only. */
:root{
  --ink:#101828;
  --muted:#667085;
  --line:#e4e7ec;
  --surface:#f4f7fb;
  --panel:#ffffff;
  --primary:#2563eb;
  --primary-dark:#1d4ed8;
  --success:#12b76a;
  --danger:#d92d20;
  --navy:#111827;
  --radius:14px;
  --shadow:0 12px 32px rgba(16,24,40,.08);
}
body{
  min-height:100vh;
  background:linear-gradient(180deg,#eef4ff 0,#f7f9fc 240px,#f7f9fc 100%);
  color:var(--ink);
  font:14px/1.45 Inter,ui-sans-serif,-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;
}
.wrap{max-width:1180px;padding:24px 18px 40px}
.top{
  min-height:108px;
  padding:22px 24px;
  border:1px solid rgba(255,255,255,.08);
  border-radius:18px;
  background:linear-gradient(135deg,#111827,#172554);
  color:#fff;
  box-shadow:0 18px 45px rgba(15,23,42,.18);
}
.brand{display:flex;align-items:center;gap:14px}
.brandMark{
  width:48px;height:48px;display:grid;place-items:center;
  border-radius:13px;background:linear-gradient(135deg,#60a5fa,#2563eb);
  box-shadow:inset 0 0 0 1px rgba(255,255,255,.24),0 8px 18px rgba(37,99,235,.3);
  font-size:16px;font-weight:800;letter-spacing:-.04em;
}
.eyebrow{margin-bottom:3px;color:#93c5fd;font-size:10px;font-weight:800;letter-spacing:.14em;text-transform:uppercase}
h1{font-size:24px;letter-spacing:-.03em}
.subtitle{margin:3px 0 0;color:#cbd5e1;font-size:12px}
.status{
  min-height:40px;max-width:430px;padding:9px 13px;
  border:1px solid rgba(255,255,255,.16);border-radius:999px;
  background:rgba(255,255,255,.08);color:#f8fafc;
}
.statusLabel{color:#93c5fd;font-size:9px;font-weight:800;letter-spacing:.12em}
.dot{width:9px;height:9px;border:0;background:#94a3b8;box-shadow:0 0 0 4px rgba(148,163,184,.15)}
.dot.ok{background:#4ade80;box-shadow:0 0 0 4px rgba(74,222,128,.15)}
.recoveryBanner{
  margin:14px 0 0;padding:11px 14px;border:1px solid #fdb022;border-radius:10px;
  background:#fffaeb;color:#93370d;font-size:13px;font-weight:700;
}
.actions{
  margin:14px 0;padding:10px;gap:8px;
  border:1px solid var(--line);border-radius:var(--radius);background:rgba(255,255,255,.95);
  box-shadow:0 4px 14px rgba(16,24,40,.04);
}
button,.fileBtn{
  min-height:40px;padding:9px 14px;border:1px solid #d0d5dd;border-radius:9px;
  background:#fff;color:#344054;font:inherit;font-size:13px;font-weight:700;line-height:1.2;
  transition:background .15s,border-color .15s,color .15s,box-shadow .15s;
}
button:hover,.fileBtn:hover{border-color:#98a2b3;background:#f9fafb}
button.primary{border-color:var(--primary);background:var(--primary);color:#fff;box-shadow:0 4px 10px rgba(37,99,235,.18)}
button.primary:hover{border-color:var(--primary-dark);background:var(--primary-dark)}
button.danger{color:#b42318}
button:focus-visible,.fileBtn:focus-within,input:focus-visible,select:focus-visible{outline:3px solid rgba(37,99,235,.18);outline-offset:1px}
button:disabled{opacity:.45;box-shadow:none}
.meta{gap:10px;margin-bottom:14px}
.meta div{
  min-height:72px;padding:12px 13px;border:1px solid var(--line);border-radius:12px;
  background:var(--panel);box-shadow:0 3px 10px rgba(16,24,40,.035);
}
.meta b{margin-bottom:6px;color:#667085;font-size:9px;letter-spacing:.1em;text-transform:uppercase}
.meta span{color:#101828;font-size:16px;font-weight:750}
.tabs{gap:5px;padding:5px;border:1px solid var(--line);border-radius:12px;background:#e9eef6}
.tab{min-height:37px;padding:9px 15px;border:0;border-radius:8px;background:transparent;color:#475467}
.tab:hover{border:0;background:rgba(255,255,255,.55)}
.tab.active{border:0;background:#fff;color:var(--primary);box-shadow:0 1px 4px rgba(16,24,40,.12)}
.page{padding-top:13px}
.grid{gap:12px}
.card{
  padding:16px;border:1px solid var(--line);border-radius:var(--radius);
  background:var(--panel);box-shadow:0 4px 14px rgba(16,24,40,.045);
}
.card h3{margin:0 0 12px;color:#101828;font-size:15px;letter-spacing:-.01em}
.cardKicker{margin:-7px 0 14px;color:var(--muted);font-size:11px}
.row,.checkboxRow{border-bottom:1px solid #f0f2f5}
.row{grid-template-columns:minmax(0,1fr) minmax(176px,184px);padding:9px 0}
.checkboxRow{padding:9px 0}
input[type=number],select,input[type=file]{
  min-height:38px;padding:8px 9px;border:1px solid #d0d5dd;border-radius:8px;
  background:#fff;color:#101828;
}
.numberStepper{
  display:grid;
  grid-template-columns:38px minmax(48px,1fr) 38px;
  width:100%;
  min-width:0;
}
.numberStepper input[type=number]{
  min-width:0;
  padding-left:4px;
  padding-right:4px;
  border-radius:0;
  text-align:center;
  -moz-appearance:textfield;
}
.numberStepper input[type=number]::-webkit-outer-spin-button,
.numberStepper input[type=number]::-webkit-inner-spin-button{
  margin:0;
  -webkit-appearance:none;
}
button.numberStepButton{
  min-width:0;
  min-height:38px;
  padding:0;
  border-color:#d0d5dd;
  border-radius:0;
  background:#f8fafc;
  color:#344054;
  font-size:20px;
  font-weight:700;
  line-height:1;
  touch-action:manipulation;
  user-select:none;
  -webkit-user-select:none;
}
button.numberStepButton:first-child{
  border-radius:8px 0 0 8px;
  border-right:0;
}
button.numberStepButton:last-child{
  border-radius:0 8px 8px 0;
  border-left:0;
}
button.numberStepButton:hover{
  border-color:#98a2b3;
  background:#eef2f6;
}
.row.controlDisabled{
  opacity:.5;
}
.numberStepper.isDisabled{
  pointer-events:none;
}
.numberStepper.isDisabled input,
.numberStepper.isDisabled button{
  background:#f2f4f7;
  color:#98a2b3;
  cursor:not-allowed;
}
.valueWithUnit{grid-template-columns:minmax(0,1fr) auto}
input[type=checkbox]{accent-color:var(--primary)}
.unit{color:#667085}
.unit.limitDisabled{
  min-width:58px;
  color:#b42318;
  font-weight:700;
}
.flashLine{margin-top:12px}
.progressPanel{margin-top:14px;padding:13px;border:1px solid var(--line);border-radius:10px;background:#f8fafc}
progress{display:block!important;height:10px;margin-top:10px;border:0;border-radius:999px;overflow:hidden;accent-color:var(--primary)}
progress[hidden]{display:none!important}
progress::-webkit-progress-bar{background:#e4e7ec;border-radius:999px}
progress::-webkit-progress-value{background:linear-gradient(90deg,#2563eb,#60a5fa);border-radius:999px}
.flashStage{
  display:flex;align-items:center;gap:12px;min-height:58px;margin-top:2px;padding:12px 13px;
  border:1px solid #bfdbfe;border-radius:9px;background:#eff6ff;color:#1d4ed8;
}
.flashStage[hidden]{display:none!important}
.flashStageIcon{
  width:22px;height:22px;flex:0 0 22px;border:3px solid #bfdbfe;
  border-top-color:var(--primary);border-radius:50%;animation:flashStageSpin .72s linear infinite;
}
.flashStageCopy{display:flex;flex-direction:column;gap:2px}
.flashStageCopy strong{color:#1e3a8a;font-size:13px}
.flashStageCopy span{color:#475467;font-size:11px}
.flashStage.success{border-color:#a6f4c5;background:#ecfdf3;color:#027a48}
.flashStage.success .flashStageIcon{
  display:grid;place-items:center;border:0;background:#12b76a;color:#fff;animation:none;
}
.flashStage.success .flashStageIcon:after{content:'✓';font-size:14px;font-weight:900}
.flashStage.success .flashStageCopy strong{color:#027a48}
@keyframes flashStageSpin{to{transform:rotate(360deg)}}
.flashNote,.mobileHint,.small,.flashSpinner{display:none!important}
.motorPanel{opacity:1}
.motorPanel input:disabled{opacity:.55}
.disabledTag{display:inline-flex;margin-bottom:12px;padding:4px 8px;border-radius:999px;background:#f2f4f7;color:#667085;font-size:10px;font-weight:800;letter-spacing:.08em;text-transform:uppercase}
.toast{
  right:18px;bottom:18px;max-width:380px;padding:12px 15px;
  border:1px solid #344054;border-radius:10px;background:#101828;color:#fff;
  box-shadow:0 16px 36px rgba(16,24,40,.24);font-weight:650;
}
.toast.err{border:1px solid #fda29b;background:#b42318}
.busy:after{
  content:'Working…';background:rgba(244,247,251,.88);color:#101828;
  font-size:15px;backdrop-filter:blur(2px);
}
.footer{padding:18px 0 0;color:#98a2b3;text-align:center;font-size:11px}
@media(max-width:650px){
  .wrap{padding:10px 10px 28px}
  .top{padding:17px;min-height:auto}
  .brandMark{width:42px;height:42px}
  h1{font-size:20px}
  .status{width:100%;max-width:none;margin-top:4px}
  .actions{padding:8px}
  .actions button,.actions .fileBtn{flex:1 1 145px}
  .meta{grid-template-columns:repeat(2,minmax(0,1fr))}
  .row{grid-template-columns:minmax(0,1fr) minmax(176px,196px)}
  .numberStepper{grid-template-columns:44px minmax(48px,1fr) 44px}
  button.numberStepButton{min-height:44px}
  .numberStepper input[type=number]{min-height:44px}
}
@media(max-width:390px){
  .row{grid-template-columns:1fr}
  .numberStepper{max-width:none}
}
.connectionChooser{
  position:fixed;
  inset:0;
  z-index:1000;
  display:grid;
  place-items:center;
  padding:20px;
  background:linear-gradient(145deg,#eaf2ff,#f8fafc 55%,#eef4ff);
}
.connectionChooser[hidden]{display:none!important}
.connectionChooserPanel{
  width:min(720px,100%);
  padding:28px;
  border:1px solid #d7e0ee;
  border-radius:22px;
  background:#fff;
  box-shadow:0 24px 70px rgba(15,23,42,.16);
}
.connectionChooserBrand{
  display:flex;
  align-items:center;
  gap:12px;
  margin-bottom:22px;
}
.connectionChooserLogo{
  width:46px;
  height:46px;
  display:grid;
  place-items:center;
  border-radius:13px;
  background:linear-gradient(135deg,#60a5fa,#2563eb);
  color:#fff;
  font-weight:800;
  box-shadow:0 9px 22px rgba(37,99,235,.25);
}
.connectionChooserTitle{
  display:block;
  color:#101828;
  font-size:20px;
  font-weight:800;
}
.connectionChooserSubtitle{
  display:block;
  margin-top:3px;
  color:#667085;
  font-size:12px;
}
.connectionChooserGrid{
  display:grid;
  grid-template-columns:repeat(2,minmax(0,1fr));
  gap:14px;
}
button.connectionChoice{
  min-height:154px;
  padding:20px;
  border:1px solid #d0d5dd;
  border-radius:15px;
  background:#fff;
  text-align:left;
  transition:border-color .16s ease,box-shadow .16s ease,transform .16s ease;
}
button.connectionChoice:hover{
  border-color:#60a5fa;
  background:#f8fbff;
  box-shadow:0 12px 30px rgba(37,99,235,.12);
  transform:translateY(-2px);
}
.connectionChoiceIcon{
  display:inline-flex;
  align-items:center;
  justify-content:center;
  width:auto;
  min-width:42px;
  height:42px;
  padding:0 12px;
  margin-bottom:16px;
  border-radius:11px;
  background:#eff6ff;
  color:#2563eb;
  font-size:21px;
  white-space:nowrap;
}
.connectionChoiceTitle{
  display:block;
  margin-bottom:5px;
  color:#101828;
  font-size:16px;
  font-weight:800;
}
.connectionChoiceText{
  display:block;
  color:#667085;
  font-size:12px;
  font-weight:500;
  line-height:1.5;
}
.mobileBrowserFallback{
  margin-top:16px;
  padding:13px 14px;
  border:1px solid #fdb022;
  border-radius:10px;
  background:#fffaeb;
  color:#7a2e0e;
  font-size:12px;
  line-height:1.5;
}
.mobileBrowserFallback[hidden]{display:none!important}
.mobileBrowserAddress{
  display:block;
  margin-top:5px;
  color:#101828;
  font-size:14px;
  font-weight:800;
}
@media(max-width:600px){
  .connectionChooserPanel{padding:20px}
  .connectionChooserGrid{grid-template-columns:1fr}
  button.connectionChoice{min-height:128px}
}
</style>
</head>
<body>
<div class="wrap">
<div class="top">
  <div class="brand">
    <div class="brandMark">EC</div>
    <div>
      <div class="eyebrow">ESC</div>
      <h1> ESC Control Center</h1>
      <p class="subtitle">ESC configuration and firmware management</p>
    </div>
  </div>
  <div class="status"><i id="dot" class="dot"></i><span class="statusLabel">DEVICE</span><span id="statusText">Not connected</span></div>
</div>

<div id="recoveryBanner" class="recoveryBanner">
  Recovery mode detected. Reflash firmware to continue.
</div>

<div class="actions">
  <button class="primary" id="connectButton" onclick="connectRead()">Connect Device</button>
  <button class="primary" id="saveButton" onclick="saveSettings()">Save Settings</button>
  <button class="danger" onclick="resetEsc()">Reset ESC</button>
  <button onclick="downloadConfig()">Export Config</button>
  <label class="fileBtn">Import Config
    <input id="configFile" type="file" accept="*/*" hidden onchange="loadConfigFile(this)">
  </label>
  <button onclick="loadDefaults()">Restore Defaults</button>
</div>

<div class="meta">
  <div><b>Firmware</b><span id="fw">—</span></div>
  <div><b>EEPROM</b><span id="ee">—</span></div>
  <div><b>EEPROM Address</b><span id="addr">—</span></div>
  <div><b>Flash Code</b><span id="flashCode">—</span></div>
  <div><b>Connection Mode</b><span id="modeText">WAIT</span></div>
</div>

<div class="tabs">
  <button class="tab active" data-page="settings">Settings</button>
  <button class="tab" data-page="firmware">Firmware</button>
  <button class="tab" data-page="input">Input</button>
</div>

<form id="settingsForm">
<section id="settings" class="page active">
<div class="grid">

<div class="card">
<h3>ESC Features</h3>
<div class="checkboxRow"><input data-field="comp_pwm" type="checkbox"><label>Complementary PWM</label></div>
<div class="checkboxRow"><input data-field="stuck_rotor_protection" type="checkbox"><label>Stuck Rotor Protection</label></div>
<div class="checkboxRow"><input data-field="stall_protection" type="checkbox"><label>Stall Protection</label></div>
</div>

<div class="card">
<h3>Motor Settings</h3>
<div class="row"><label>Timing Advance</label><div class="valueWithUnit"><input data-field="advance_level" data-conv="timingAdvance" type="number" min="0" max="30" step="0.1"><span class="unit">°</span></div></div>
<div class="row"><label>Motor KV</label><input data-field="motor_kv" data-conv="kv" type="number" min="20" max="10220" step="40"></div>
<div class="row"><label>Motor Poles</label><input data-field="motor_poles" type="number" min="2" max="32" step="2"></div>
<div class="row"><label>Startup Power</label><input data-field="startup_power" type="number" min="50" max="150"></div>
<div class="row"><label>PWM Frequency</label><div class="valueWithUnit"><input data-field="pwm_frequency" type="number" min="8" max="144"><span class="unit">kHz</span></div></div>
<div class="row"><label>Beep Volume</label><input data-field="beep_volume" type="number" min="0" max="11"></div>
<div class="row"><label>Brake On Stop Level</label><input data-field="drag_brake_strength" data-conv="brakeLevel" type="number" min="1" max="10"></div>
<div class="row"><label>Sine Startup Range</label><input data-field="sine_changeover" data-conv="sineChangeover" type="number" min="5" max="25"></div>
<div class="row"><label>Sine Mode Power</label><input data-field="sine_mode_power" data-conv="sinePower" type="number" min="1" max="10"></div>
<div class="row"><label>Running Brake Level</label><input data-field="driving_brake_strength" data-conv="brakeLevel" type="number" min="1" max="10"></div>
</div>

<div class="card">
<h3>Ramp / PWM / Brake</h3>
<div class="row"><label>Throttle Rate of Change</label><div class="valueWithUnit"><input data-field="max_ramp" data-conv="ramp" type="number" min="0.1" max="25" step="0.1"><span class="unit">%/ms</span></div></div>
<div class="row"><label>Minimum Duty Cycle</label><div class="valueWithUnit"><input data-field="minimum_duty_cycle" data-conv="minimumDuty" type="number" min="0" max="25" step="0.5"><span class="unit">%</span></div></div>
<div class="checkboxRow"><input data-field="auto_advance" type="checkbox"><label>Auto-Timing</label></div>
<div class="checkboxRow"><input data-field="variable_pwm" type="checkbox"><label>Variable PWM</label></div>
<div class="checkboxRow"><input data-field="brake_on_stop" type="checkbox"><label>Brake On Stop</label></div>
<div class="checkboxRow"><input id="activeBrakeEnabled" type="checkbox"><label>Active Brake On Stop</label></div>
<div class="row"><label>Active Brake Power</label><div class="valueWithUnit"><input data-field="active_brake_power" type="number" min="0" max="5"><span class="unit">%</span></div></div>
</div>

</div>
</section>

<section id="input" class="page">
<div class="grid two">

<div class="card">
<h3>Servo Settings</h3>
<div class="row"><label>Low Threshold</label><div class="valueWithUnit"><input data-field="servo_low_threshold" data-conv="servoLow" type="number" min="750" max="1260" step="2"><span class="unit">us</span></div></div>
<div class="row"><label>High Threshold</label><div class="valueWithUnit"><input data-field="servo_high_threshold" data-conv="servoHigh" type="number" min="1750" max="2260" step="2"><span class="unit">us</span></div></div>
<div class="row"><label>Servo Neutral</label><div class="valueWithUnit"><input data-field="servo_neutral" data-conv="servoNeutral" type="number" min="1374" max="1629"><span class="unit">us</span></div></div>
<div class="row"><label>Servo Dead Band</label><input data-field="servo_dead_band" type="number" min="0" max="255"></div>

<div class="checkboxRow"><input id="absoluteVoltageEnabled" type="checkbox"><label>Absolute Voltage Cut Off</label></div>
<div class="row"><label>Absolute Cutoff</label><div class="valueWithUnit"><input data-field="absolute_voltage_cutoff" data-conv="halfVolt" type="number" min="0.5" max="100" step="0.5"><span class="unit">V</span></div></div>

<div class="checkboxRow"><input id="cellVoltageEnabled" data-field="low_voltage_cut_off" type="checkbox"><label>Low Voltage Cut Off Cell</label></div>
<div class="row"><label>Low Cell Voltage</label><div class="valueWithUnit"><input data-field="low_cell_volt_cutoff" data-conv="cellV" type="number" min="2.5" max="3.5" step="0.01"><span class="unit">V</span></div></div>

<div class="row"><label>Temperature Limit</label><div class="valueWithUnit"><input data-field="temperature_limit" data-conv="temperature" type="number" min="70" max="141" step="1"><span id="temperatureLimitUnit" class="unit">°C</span></div></div>

<div class="row"><label>Current Limit</label><div class="valueWithUnit"><input data-field="current_limit" data-conv="currentA" type="number" min="2" max="202" step="2"><span id="currentLimitUnit" class="unit">A</span></div></div>

</div>

<div class="card">
<h3>Current Limit PID Control</h3>
<div class="row"><label>Current Limit P</label><input data-field="current_P" data-conv="currentPID" type="number" min="0" max="510" step="2"></div>
<div class="row"><label>Current Limit I</label><input data-field="current_I" type="number" min="0" max="255"></div>
<div class="row"><label>Current Limit D</label><input data-field="current_D" data-conv="currentPID" type="number" min="0" max="510" step="2"></div>
</div>

</div>
</section>
</form>

<section id="firmware" class="page">
<div class="grid one">
<div class="card">
<h3>Firmware Update</h3>
<div class="defaultFirmware">
  <div>
    <strong>Recommended ESC Firmware</strong><span>Firmware supported by manufacturer</span>
  </div>
  <button
    class="primary"
    id="defaultFlashButton"
    onclick="flashDefaultFirmware()"
  >Install Firmware</button>
</div>
<div class="cardKicker">Custom firmware recovery (.hex)</div>
<input
  id="hexFile"
  type="file"
  accept="*/*"
  onchange="firmwareFileSelected(this)"
>
<div class="flashLine">
  <button id="flashButton" onclick="uploadAndFlash()">Install Selected File</button>
  <button onclick="clearFirmware()">Clear File</button>
  <button onclick="resetWeb()">Refresh</button>
</div>
<div class="progressPanel">
  <span id="flashText">No firmware selected</span>
  <progress id="flashProgress" value="0" max="100"></progress>
  <div id="flashStage" class="flashStage" hidden>
    <i class="flashStageIcon" aria-hidden="true"></i>
    <div class="flashStageCopy">
      <strong id="flashStageTitle">Finalizing settings</strong>
      <span id="flashStageDetail">Completing the final setup</span>
    </div>
  </div>
</div>
</div>
</div>
</section>

</div>

<div class="footer"> ESC · Local Device Interface</div>

<div id="toast" class="toast"></div>

<script>
let state=null;
let importedRawSettings=null;
let flashTimer=null;

let localFlashTimer=null;
let localFlashActive=false;
let localFlashStartMs=0;
let localFlashExpectedMs=0;
let localFlashPercent=0;
let localFinalizeTimer=null;
let localFlashVisualDone=false;
const LOCAL_FINALIZE_DELAY_MS=5000;

const $=selector=>document.querySelector(selector);
const $$=selector=>[...document.querySelectorAll(selector)];

$$('.tab').forEach(button=>{
  button.onclick=()=>{
    $$('.tab').forEach(item=>item.classList.remove('active'));
    $$('.page').forEach(item=>item.classList.remove('active'));
    button.classList.add('active');
    $('#'+button.dataset.page).classList.add('active');
  };
});

function clampNumberInput(input,fillEmpty=false){
  const raw=String(input.value||'').trim();
  const minimum=input.min===''?null:Number(input.min);
  const maximum=input.max===''?null:Number(input.max);

  if(raw===''){
    if(fillEmpty){
      input.value=minimum!==null?minimum:0;
    }
    return;
  }

  let value=Number(raw);

  if(!Number.isFinite(value)){
    if(fillEmpty){
      input.value=minimum!==null?minimum:0;
    }
    return;
  }

  if(minimum!==null&&value<minimum)value=minimum;
  if(maximum!==null&&value>maximum)value=maximum;

  if(Number(input.value)!==value){
    input.value=value;
  }
}

function stepNumberInput(input,direction){
  const raw=String(input.value||'').trim();

  if(raw===''||!Number.isFinite(Number(raw))){
    const minimum=input.min===''?null:Number(input.min);
    const maximum=input.max===''?null:Number(input.max);
    input.value=direction>0
      ?(minimum!==null?minimum:0)
      :(maximum!==null?maximum:0);
  }else{
    try{
      if(direction>0){
        input.stepUp();
      }else{
        input.stepDown();
      }
    }catch(error){
      const step=input.step===''||input.step==='any'
        ?1
        :Number(input.step);
      input.value=Number(raw)+(direction*(Number.isFinite(step)?step:1));
    }
  }

  clampNumberInput(input,true);
  input.dispatchEvent(new Event('input',{bubbles:true}));
  input.dispatchEvent(new Event('change',{bubbles:true}));
}

function enhanceNumberInputs(){
  $$('input[type=number]').forEach(input=>{
    if(input.closest('.numberStepper'))return;

    const row=input.closest('.row');
    const label=row?row.querySelector('label'):null;
    const fieldName=label?label.textContent.trim():'value';
    const wrapper=document.createElement('div');
    const decrease=document.createElement('button');
    const increase=document.createElement('button');

    wrapper.className='numberStepper';

    decrease.type='button';
    decrease.className='numberStepButton';
    decrease.textContent='−';
    decrease.setAttribute('aria-label',`Decrease ${fieldName}`);

    increase.type='button';
    increase.className='numberStepButton';
    increase.textContent='+';
    increase.setAttribute('aria-label',`Increase ${fieldName}`);

    input.parentNode.insertBefore(wrapper,input);
    wrapper.append(decrease,input,increase);

    decrease.addEventListener('click',()=>stepNumberInput(input,-1));
    increase.addEventListener('click',()=>stepNumberInput(input,1));

    /*
     * Do not clamp on every input event. A temporary value such as "1"
     * must remain editable so the user can continue typing "11" or "14".
     */
    input.addEventListener('change',()=>clampNumberInput(input,true));
    input.addEventListener('blur',()=>clampNumberInput(input,true));
    input.addEventListener('keydown',event=>{
      if(event.key==='Enter'){
        event.preventDefault();
        clampNumberInput(input,true);
        input.blur();
      }
    });
  });
}

enhanceNumberInputs();

function setNumberControlEnabled(input,enabled){
  if(!input)return;

  input.disabled=!enabled;

  const stepper=input.closest('.numberStepper');
  if(stepper){
    stepper.classList.toggle('isDisabled',!enabled);
    stepper.querySelectorAll('button').forEach(button=>{
      button.disabled=!enabled;
    });
  }

  const row=input.closest('.row');
  if(row)row.classList.toggle('controlDisabled',!enabled);
}

function syncRelatedControls(changedControl=null){
  const variablePwm=$('[data-field="variable_pwm"]');
  const pwmFrequency=$('[data-field="pwm_frequency"]');
  const brakeOnStop=$('[data-field="brake_on_stop"]');
  const activeBrake=$('#activeBrakeEnabled');
  const brakeOnStopLevel=$('[data-field="drag_brake_strength"]');
  const activeBrakePower=$('[data-field="active_brake_power"]');

  /*
   * brake_on_stop uses one EEPROM field:
   * 0 = off, 1 = brake on stop, 2 = active brake on stop.
   */
  if(changedControl===brakeOnStop&&brakeOnStop.checked){
    activeBrake.checked=false;
  }else if(changedControl===activeBrake&&activeBrake.checked){
    brakeOnStop.checked=false;
  }

  setNumberControlEnabled(pwmFrequency,!variablePwm.checked);
  setNumberControlEnabled(
    brakeOnStopLevel,
    brakeOnStop.checked&&!activeBrake.checked
  );
  setNumberControlEnabled(
    activeBrakePower,
    activeBrake.checked&&!brakeOnStop.checked
  );
}

function updateLimitStatus(){
  const temperatureInput=$('[data-field="temperature_limit"]');
  const currentInput=$('[data-field="current_limit"]');
  const temperatureUnit=$('#temperatureLimitUnit');
  const currentUnit=$('#currentLimitUnit');
  const temperatureDisabled=Number(temperatureInput.value)>=141;
  const currentDisabled=Number(currentInput.value)>=202;

  temperatureUnit.textContent=temperatureDisabled?'Disabled':'°C';
  temperatureUnit.classList.toggle('limitDisabled',temperatureDisabled);
  currentUnit.textContent=currentDisabled?'Disabled':'A';
  currentUnit.classList.toggle('limitDisabled',currentDisabled);
}

const variablePwmCheckbox=$('[data-field="variable_pwm"]');
const brakeOnStopCheckbox=$('[data-field="brake_on_stop"]');
const activeBrakeCheckbox=$('#activeBrakeEnabled');
const temperatureLimitInput=$('[data-field="temperature_limit"]');
const currentLimitInput=$('[data-field="current_limit"]');

variablePwmCheckbox.addEventListener(
  'change',
  ()=>syncRelatedControls(variablePwmCheckbox)
);
brakeOnStopCheckbox.addEventListener(
  'change',
  ()=>syncRelatedControls(brakeOnStopCheckbox)
);
activeBrakeCheckbox.addEventListener(
  'change',
  ()=>syncRelatedControls(activeBrakeCheckbox)
);
[temperatureLimitInput,currentLimitInput].forEach(input=>{
  input.addEventListener('input',updateLimitStatus);
  input.addEventListener('change',updateLimitStatus);
});
syncRelatedControls();
updateLimitStatus();

function toast(message,error=false){
  const box=$('#toast');
  box.textContent=message;
  box.className='toast'+(error?' err':'');
  box.style.display='block';
  setTimeout(()=>box.style.display='none',4800);
}

function setBusy(value){
  document.body.classList.toggle('busy',value);
}

function rawToDisplay(raw,conversion){
  raw=Number(raw);

  switch(conversion){
    case'ramp':return +(raw/10).toFixed(1);
    case'minimumDuty':return +(raw/2).toFixed(1);
    case'timingAdvance':
      if(raw<4)return +(raw*7.5).toFixed(1);
      if(raw>=10&&raw<=42){
        return +((raw-10)*0.9375).toFixed(1);
      }
      return 15;
    case'brakeLevel':return raw>=1&&raw<=10?raw:10;
    case'sineChangeover':return raw>=5&&raw<=25?raw:5;
    case'sinePower':return raw>=1&&raw<=10?raw:5;
    case'kv':return raw*40+20;
    case'currentPID':return raw*2;
    case'servoLow':return raw*2+750;
    case'servoHigh':return raw*2+1750;
    case'servoNeutral':return raw+1374;
    case'cellV':return +((raw+250)/100).toFixed(2);
    case'currentA':return raw===0||raw>100?202:raw*2;
    case'halfVolt':return raw===0?10:raw*.5;
    case'temperature':return raw<70||raw>140?141:raw;
    default:return raw;
  }
}

function displayToRaw(value,conversion){
  value=Number(value);

  switch(conversion){
    case'ramp':return Math.round(value*10);
    case'minimumDuty':return Math.round(value*2);
    case'timingAdvance':
      return Math.max(
        10,
        Math.min(42,Math.round(value/0.9375)+10)
      );
    case'brakeLevel':
    case'sineChangeover':
    case'sinePower':return Math.round(value);
    case'kv':return Math.round((value-20)/40);
    case'currentPID':return Math.round(value/2);
    case'servoLow':return Math.round((value-750)/2);
    case'servoHigh':return Math.round((value-1750)/2);
    case'servoNeutral':return Math.round(value-1374);
    case'cellV':return Math.round(value*100-250);
    case'currentA':return value>=202?101:Math.round(value/2);
    case'halfVolt':return Math.round(value*2);
    case'temperature':return value>=141?255:Math.round(value);
    default:return Math.round(value);
  }
}

function formatTime(totalSeconds){
  totalSeconds=Math.max(0,Math.ceil(totalSeconds));
  const minutes=Math.floor(totalSeconds/60);
  const seconds=totalSeconds%60;
  return minutes>0
    ?`${minutes} min ${seconds} sec`
    :`${seconds} sec`;
}

function showFlashStage(mode='none'){
  const stage=$('#flashStage');
  const progress=$('#flashProgress');
  const text=$('#flashText');
  const visible=mode==='finalizing'||mode==='success';

  stage.hidden=!visible;
  progress.hidden=visible;
  text.hidden=visible;
  stage.classList.toggle('success',mode==='success');

  if(mode==='finalizing'){
    $('#flashStageTitle').textContent='Finalizing settings';
    $('#flashStageDetail').textContent='Completing the final setup';
  }else if(mode==='success'){
    $('#flashStageTitle').textContent='Firmware installed successfully';
    $('#flashStageDetail').textContent='Installation complete';
  }
}

function resetLocalFinalization(){
  if(localFinalizeTimer){
    clearTimeout(localFinalizeTimer);
    localFinalizeTimer=null;
  }

  localFlashVisualDone=false;
}

function beginLocalFinalization(){
  if(localFlashVisualDone){
    showFlashStage('success');
    return;
  }

  showFlashStage('finalizing');
  stopLocalFlashProgress();

  if(localFinalizeTimer)return;

  localFinalizeTimer=setTimeout(()=>{
    localFinalizeTimer=null;
    localFlashVisualDone=true;
    $('#flashProgress').value=100;
    showFlashStage('success');
    $('#flashButton').disabled=false;

    /* The success display is local and does not wait for Wi-Fi to return. */
    clearTimeout(flashTimer);
    flashTimer=null;
  },LOCAL_FINALIZE_DELAY_MS);
}

function stopLocalFlashProgress(){
  localFlashActive=false;

  if(localFlashTimer){
    clearInterval(localFlashTimer);
    localFlashTimer=null;
  }
}

function renderLocalFlashProgress(){
  if(!localFlashActive)return;

  const elapsedMs=Date.now()-localFlashStartMs;

  /* Local progress estimate for 128-byte blocks. At 99% the page runs a
     five-second local finalization display, independent of Wi-Fi. */
  const calculatedPercent=Math.min(
    99,
    Math.floor(elapsedMs*99/localFlashExpectedMs)
  );

  if(calculatedPercent>localFlashPercent){
    localFlashPercent=calculatedPercent;
  }

  const progress=$('#flashProgress');
  if(localFlashPercent>Number(progress.value||0)){
    progress.value=localFlashPercent;
  }

  const remainingMs=localFlashExpectedMs-elapsedMs;

  if(localFlashPercent>=99||remainingMs<=0){
    beginLocalFinalization();
  }else{
    showFlashStage('none');
    $('#flashText').textContent=
      `Installing ${localFlashPercent}% · ${formatTime(remainingMs/1000)} remaining`;
  }
}

function startLocalFlashProgress(totalBytes){
  resetLocalFinalization();
  stopLocalFlashProgress();

  const blockSize=128;
  const blockCount=Math.max(1,Math.ceil(totalBytes/blockSize));

  // Estimate preparation and block time, then finalize locally for 5 seconds.
  localFlashExpectedMs=6000+(blockCount*200);
  localFlashStartMs=Date.now();
  localFlashPercent=0;
  localFlashActive=true;

  showFlashStage('none');
  $('#flashProgress').value=0;
  renderLocalFlashProgress();

  localFlashTimer=setInterval(
    renderLocalFlashProgress,
    200
  );
}

function uiMessage(message){
  const text=String(message||'');
  const exact={
    'Chưa kết nối ESC':'Not connected',
    'Không nhận được bootloader ESC':'Try again connect',
    'Flash code ESC chưa được hỗ trợ':'Unsupported ESC flash code',
    'Đã kết nối bootloader ESC':'ESC bootloader connected',
    'Không đọc ổn định được EEPROM từ bootloader':'Unable to read stable EEPROM data',
    "Dòng HEX không bắt đầu bằng ':'":"HEX line must begin with ':'",
    'Dòng HEX có số ký tự không hợp lệ':'Invalid HEX line length',
    'Độ dài dòng HEX không hợp lệ':'Invalid HEX record length',
    'File HEX chứa ký tự sai':'HEX file contains invalid characters',
    'Độ dài record HEX không khớp':'HEX record length mismatch',
    'Checksum Intel HEX bị sai':'Intel HEX checksum failed',
    'HEX trộn địa chỉ 0x0000 và 0x1000':'HEX file mixes 0x0000 and 0x1000 addressing',
    'Firmware vượt quá vùng Flash trước EEPROM':'Firmware exceeds the available flash area',
    'Record segment HEX không hợp lệ':'Invalid HEX segment record',
    'Record linear HEX không hợp lệ':'Invalid HEX linear address record',
    'Intel HEX record type chưa hỗ trợ':'Unsupported Intel HEX record type',
    'Dòng Intel HEX quá dài':'Intel HEX line is too long',
    'Không xác định được dung lượng Flash ESC':'Unable to determine ESC flash capacity',
    'Thiết bị không đủ RAM để chứa firmware':'Insufficient memory for this firmware',
    'File Intel HEX không có record EOF':'Intel HEX file has no EOF record',
    'File HEX không có firmware data':'HEX file contains no firmware data',
    'Upload firmware bị hủy':'Firmware upload cancelled',
    'Firmware chưa được tải hợp lệ':'No valid firmware uploaded',
    'Đang khóa safety byte':'Preparing device',
    'Đang nạp firmware':'Installing firmware',
    'Không ghi được safety byte EEPROM':'Unable to prepare EEPROM',
    'Đang hoàn tất firmware':'Finalizing firmware',
    'Firmware đã ghi xong':'Firmware written',
    'Firmware đã ghi nhưng chưa mở được safety byte':'Firmware written, but EEPROM could not be restored',
    'Nạp firmware thành công. Kết nối lại và đọc Settings':'Firmware installed successfully',
    'App Tool đang giữ quyền đến khi reset thiết bị':'USB App Tool is active until the device is reset',
    'Không thể chọn chế độ Web':'Unable to select Web mode',
    'Đang có thao tác khác':'Another operation is in progress',
    'Phải bấm Đọc & Config trước khi lưu':'Connect to the ESC before saving settings',
    'Không đọc được EEPROM trước khi lưu':'Unable to read EEPROM before saving',
    'EEPROM nền chưa ổn định':'EEPROM data is not stable',
    'ESC đang ở recovery (safety byte = 0). Hãy nạp firmware trước khi lưu Settings':'Recovery mode is active. Install firmware before saving settings.',
    'Ghi EEPROM thất bại':'EEPROM write failed',
    'Đã ghi nhưng không đọc lại được':'EEPROM written, but readback failed',
    'Lưu EEPROM thành công':'Settings saved',
    'ESC chưa kết nối':'ESC is not connected',
    'Đã reset ESC':'ESC reset complete',
    'Đã gửi lệnh reset ESC':'ESC reset requested',
    'Chưa tải file Intel HEX hợp lệ':'No valid Intel HEX file uploaded',
    'Đã nhận firmware, chuẩn bị tạm ngắt Wi-Fi':'Firmware ready',
    'Không thể xóa file khi đang nạp':'Cannot clear the file during installation',
    'Đã xóa firmware tạm':'Firmware file cleared',
    'Không tìm thấy API':'API endpoint not found',
    'No firmware detected. Please upload firmware first.':'Firmware required'
  };
  if(exact[text])return exact[text];
  if(text.startsWith('Đã đọc EEPROM 48 byte tại 0x'))return text.replace('Đã đọc EEPROM 48 byte tại 0x','EEPROM read at 0x');
  if(text.startsWith('Đã tải HEX: '))return text.replace('Đã tải HEX: ','Firmware ready: ');
  if(text.startsWith('Đang nạp firmware '))return text.replace('Đang nạp firmware ','Installing firmware ');
  if(text.startsWith('ESC không ACK tại offset '))return text.replace('ESC không ACK tại offset ','No ESC acknowledgement at offset ');
  if(text.startsWith('Verify EEPROM sai tại offset '))return text.replace('Verify EEPROM sai tại offset ','EEPROM verification failed at offset ');
  if(text.startsWith('ESC is in recovery mode'))return 'Recovery mode';
  return text;
}

function updateUI(data){
  state=data;

  const firmwareMissing=!!data.firmwareMissing;

  if(firmwareMissing){
    const flashTab=document.querySelector(
      '.tab[data-page="firmware"]'
    );

    if(flashTab&&!flashTab.classList.contains('active')){
      flashTab.click();
    }
  }

  $('#dot').classList.toggle('ok',!!data.connected);
  $('#statusText').textContent=uiMessage(data.message)||'—';
  $('#modeText').textContent=data.mode||'WAIT';

  const recovery=!!data.recovery;
  const flashBusy=[
    'pending','safety','writing','finishing','reset'
  ].includes(data.flashState);

  $('#recoveryBanner').style.display=recovery?'block':'none';
  $('#saveButton').disabled=
    !data.loaded||recovery||firmwareMissing||flashBusy;

  $('#fw').textContent=data.loaded
    ?`${data.firmwareMajor}.${data.firmwareMinor}`
    :'—';

  $('#ee').textContent=data.loaded
    ?`v${data.eepromVersion} · ${data.eepromLength} byte`
    :'—';

  $('#addr').textContent=data.eepromAddress
    ?'0x'+Number(data.eepromAddress).toString(16).toUpperCase().padStart(4,'0')
    :'—';

  $('#flashCode').textContent=data.flashCode
    ?'0x'+Number(data.flashCode).toString(16).toUpperCase().padStart(2,'0')
    :'—';

  if(data.raw){
    $$('[data-field]').forEach(element=>{
      const raw=data.raw[element.dataset.field];
      if(raw===undefined)return;

      if(element.type==='checkbox'){
        element.checked=!!raw;
      }else{
        element.value=rawToDisplay(raw,element.dataset.conv);
      }
    });

    const brakeMode=Number(data.raw.brake_on_stop||0);
    const voltageMode=Number(data.raw.low_voltage_cut_off||0);

    $('[data-field="brake_on_stop"]').checked=brakeMode===1;
    $('#activeBrakeEnabled').checked=brakeMode===2;
    $('#absoluteVoltageEnabled').checked=voltageMode===2;
    $('#cellVoltageEnabled').checked=voltageMode===1;
  }

  syncRelatedControls();
  updateLimitStatus();
  updateFlashUI(data);
}

function updateFlashUI(data){
  const total=Number(data.firmwareSize||0);
  const written=Number(data.flashWritten||0);
  const realPercent=total?Math.round(written*100/total):0;
  const progress=$('#flashProgress');

  if(!localFlashActive||realPercent>Number(progress.value||0)){
    progress.value=realPercent;
  }

  if(data.flashState==='error'){
    resetLocalFinalization();
    stopLocalFlashProgress();
    showFlashStage('none');
    $('#flashText').textContent=
      uiMessage(data.flashError)||'Firmware installation failed';
    $('#flashButton').disabled=false;
    $('#defaultFlashButton').disabled=false;
    return;
  }

  if(localFlashVisualDone){
    progress.value=100;
    showFlashStage('success');
    $('#flashButton').disabled=false;
    $('#defaultFlashButton').disabled=false;
    return;
  }

  if(localFinalizeTimer){
    showFlashStage('finalizing');
    $('#flashButton').disabled=true;
    $('#defaultFlashButton').disabled=true;
    return;
  }

  let text=data.firmwareMissing
    ?'Firmware required'
    :'No firmware selected';

  if(data.firmwareUploaded){
    text=`Firmware ready · ${total} bytes`;
  }

  if(data.flashState==='pending'){
    showFlashStage('none');
    text='Preparing firmware';
  }else if(data.flashState==='safety'){
    showFlashStage('none');
    text='Preparing device';
  }else if(data.flashState==='writing'){
    if(realPercent>=99||(localFlashActive&&localFlashPercent>=99)){
      beginLocalFinalization();
    }else{
      showFlashStage('none');
    }
    if(!localFlashActive){
      text=`Installing ${realPercent}%`;
    }else{
      return;
    }
  }else if(data.flashState==='finishing'){
    beginLocalFinalization();
    text='Finalizing settings';
  }else if(data.flashState==='reset'){
    beginLocalFinalization();
    text='Finalizing settings';
  }else if(data.flashState==='done'){
    beginLocalFinalization();
    text='Finalizing settings';
  }else{
    showFlashStage('none');
  }

  $('#flashText').textContent=text;

  const flashBusy=[
    'pending','safety','writing','finishing','reset'
  ].includes(data.flashState);
  $('#flashButton').disabled=flashBusy;
  $('#defaultFlashButton').disabled=flashBusy;
}

async function api(url,options={}){
  const response=await fetch(url,options);
  const data=await response.json();

  if(!response.ok||data.ok===false){
    throw new Error(uiMessage(data.message)||`HTTP ${response.status}`);
  }

  return data;
}

async function connectRead(){
  setBusy(true);
  try{
    const data=await api('/api/connect',{method:'POST'});
    importedRawSettings=null;
    updateUI(data);

    if(data.firmwareMissing){
      toast('Bootloader connected');
    }else{
      toast('Device connected');
    }
  }catch(error){
    toast(error.message,true);
  }finally{
    setBusy(false);
  }
}

function normalizeSettings(){
  const numberInputs=$$('input[type=number][data-field]');

  for(const input of numberInputs){
    let value=Number(input.value);
    const minimum=input.min===''?null:Number(input.min);
    const maximum=input.max===''?null:Number(input.max);

    if(!Number.isFinite(value)){
      value=minimum!==null?minimum:0;
    }

    if(minimum!==null&&value<minimum)value=minimum;
    if(maximum!==null&&value>maximum)value=maximum;

    input.value=value;
  }

  const temperatureInput=$('[data-field="temperature_limit"]');
  let temperature=Number(temperatureInput.value);
  if(temperature>=141)temperature=141;
  else temperature=Math.max(70,Math.min(140,temperature));
  temperatureInput.value=temperature;

  const currentInput=$('[data-field="current_limit"]');
  let current=Number(currentInput.value);
  if(current>0&&current<2){
    currentInput.value=2;
  }

  const polesInput=$('[data-field="motor_poles"]');
  let motorPoles=Math.round(Number(polesInput.value));
  motorPoles=Math.max(2,Math.min(32,motorPoles));
  if(motorPoles%2!==0){
    motorPoles=motorPoles<32?motorPoles+1:motorPoles-1;
  }
  polesInput.value=motorPoles;

  const lowInput=$('[data-field="servo_low_threshold"]');
  const neutralInput=$('[data-field="servo_neutral"]');
  const highInput=$('[data-field="servo_high_threshold"]');

  const servoLow=Number(lowInput.value);
  const servoNeutral=Number(neutralInput.value);
  const servoHigh=Number(highInput.value);

  if(!(servoLow<servoNeutral&&servoNeutral<servoHigh)){
    lowInput.value=1100;
    neutralInput.value=1500;
    highInput.value=1900;
  }
}

function collectSettings(){
  const params=new URLSearchParams();

  // Preserve every raw field from an imported profile, including fields that
  // are intentionally hidden from the customer UI. Visible controls below
  // then overwrite only the values the customer can edit.
  if(importedRawSettings){
    Object.entries(importedRawSettings).forEach(([name,raw])=>{
      const value=Number(raw);
      if(Number.isFinite(value)){
        params.set(name,Math.max(0,Math.min(255,Math.round(value))));
      }
    });
  }

  $$('[data-field]').forEach(element=>{
    let value=element.type==='checkbox'
      ?(element.checked?1:0)
      :displayToRaw(element.value,element.dataset.conv);

    value=Math.max(0,Math.min(255,value));
    params.set(element.dataset.field,value);
  });

  if($('#activeBrakeEnabled').checked){
    params.set('brake_on_stop','2');

    if(Number(params.get('active_brake_power'))===0){
      params.set('active_brake_power','1');
    }
  }else{
    params.set(
      'brake_on_stop',
      $('[data-field="brake_on_stop"]').checked?'1':'0'
    );
    params.set('active_brake_power','0');
  }

  if($('#absoluteVoltageEnabled').checked){
    params.set('low_voltage_cut_off','2');
  }else if($('#cellVoltageEnabled').checked){
    params.set('low_voltage_cut_off','1');
    params.set('absolute_voltage_cutoff','0');
  }else{
    params.set('low_voltage_cut_off','0');
    params.set('absolute_voltage_cutoff','0');
  }

  return params;
}

async function saveSettings(){
  normalizeSettings();

  if(!confirm('Write all displayed Settings to ESC EEPROM?'))return;

  setBusy(true);
  try{
    const data=await api('/api/save',{
      method:'POST',
      headers:{'Content-Type':'application/x-www-form-urlencoded'},
      body:collectSettings()
    });
    importedRawSettings=null;
    updateUI(data);
    toast('Settings saved');
  }catch(error){
    toast(error.message,true);
  }finally{
    setBusy(false);
  }
}

async function resetEsc(){
  if(!confirm('Reset ESC?'))return;

  setBusy(true);
  try{
    updateUI(await api('/api/reset',{method:'POST'}));
    toast('Reset requested');
  }catch(error){
    toast(error.message,true);
  }finally{
    setBusy(false);
  }
}

function resetWeb(){
  stopLocalFlashProgress();
  window.location.replace('/?reload='+Date.now());
}

function downloadConfig(){
  if(!state||!state.loaded||!state.raw){
    return toast('No settings to export',true);
  }

  const exportRaw=Object.assign({},state.raw);
  delete exportRaw.use_hall_sensors;
  delete exportRaw.disable_stick_calibration;

  const exportData={
      format:'ESC-Config',
    firmwareMajor:state.firmwareMajor,
    firmwareMinor:state.firmwareMinor,
    eepromVersion:state.eepromVersion,
    raw:exportRaw
  };

  const blob=new Blob(
    [JSON.stringify(exportData,null,2)],
    {type:'application/json'}
  );

  const link=document.createElement('a');
  link.href=URL.createObjectURL(blob);
  link.download=`ESC_FW_${state.firmwareMajor}_${state.firmwareMinor}_Config.json`;
  link.click();
  URL.revokeObjectURL(link.href);
}

function applyRawToPage(raw,message){
  if(!raw||typeof raw!=='object'||Array.isArray(raw)){
    throw new Error('The configuration has an invalid raw Settings section');
  }

  const filteredRaw=Object.assign({},raw);
  delete filteredRaw.use_hall_sensors;
  delete filteredRaw.disable_stick_calibration;

  importedRawSettings=filteredRaw;
  const next=Object.assign({},state||{});
  next.connected=!!(state&&state.connected);
  next.loaded=!!(state&&state.loaded);
  next.message=message;
  next.raw=filteredRaw;
  updateUI(next);
}

function loadConfigFile(input){
  const file=input.files[0];
  if(!file)return;

  const reader=new FileReader();

  reader.onload=()=>{
    try{
      const data=JSON.parse(reader.result);
      if(!data.raw)throw new Error('The file has no raw Settings section');
      applyRawToPage(data.raw,'Configuration imported');
      toast('Configuration imported');
    }catch(error){
      toast(error.message,true);
    }
  };

  reader.readAsText(file);
}

function loadDefaults(){
  const defaults={
    max_ramp:60,
    minimum_duty_cycle:10,
    disable_stick_calibration:0,
    absolute_voltage_cutoff:0,
    current_P:100,
    current_I:0,
    current_D:50,
    active_brake_power:2,
    dir_reversed:0,
    bi_direction:0,
    use_sine_start:1,
    comp_pwm:1,
    variable_pwm:1,
    stuck_rotor_protection:1,
    advance_level:26,
    pwm_frequency:24,
    startup_power:150,
    motor_kv:55,
    motor_poles:14,
    brake_on_stop:0,
    stall_protection:1,
    beep_volume:11,
    telemetry_on_interval:0,
    servo_low_threshold:125,
    servo_high_threshold:125,
    servo_neutral:126,
    servo_dead_band:50,
    low_voltage_cut_off:0,
    low_cell_volt_cutoff:0,
    rc_car_reverse:1,
    use_hall_sensors:0,
    sine_changeover:6,
    drag_brake_strength:10,
    driving_brake_strength:10,
    temperature_limit:0,
    current_limit:0,
    sine_mode_power:4,
    input_type:2,
    auto_advance:0
  };

  applyRawToPage(defaults,'Defaults loaded');
  $('#activeBrakeEnabled').checked=false;
  $('#absoluteVoltageEnabled').checked=false;
  $('#cellVoltageEnabled').checked=false;
  syncRelatedControls();
  toast('Defaults loaded');
}

async function pollFlash(){
  clearTimeout(flashTimer);

  try{
    const data=await api('/api/firmware/status');
    updateUI(data);

    if(localFlashVisualDone)return;

    if([
      'pending','safety','writing','finishing','reset'
    ].includes(data.flashState)){
      flashTimer=setTimeout(pollFlash,700);
      return;
    }

    if(data.flashState==='done'){
      beginLocalFinalization();
      return;
    }

    if(data.flashState==='error'){
      resetLocalFinalization();
      stopLocalFlashProgress();
      showFlashStage('none');
      toast(uiMessage(data.flashError)||'Firmware installation failed',true);
    }
  }catch(error){
    /* Wi-Fi is temporarily unavailable while firmware is written.
       The local timer continues to display estimated progress. */
    if(!localFlashVisualDone){
      flashTimer=setTimeout(pollFlash,1300);
    }
  }
}

function firmwareFileSelected(input){
  const file=(input.files&&input.files.length>0)
    ?input.files[0]
    :null;

  resetLocalFinalization();
  showFlashStage('none');
  $('#flashProgress').value=0;

  if(!file){
    $('#flashText').textContent='No firmware selected';
    return;
  }

  const sizeKB=Math.max(1,Math.ceil(file.size/1024));
  $('#flashText').textContent=file.name+' · '+sizeKB+' KB';
}

async function flashDefaultFirmware(){
  if(!state||!state.connected){
    return toast('Bootloader not connected',true);
  }

  if(!state.loaded&&!state.firmwareMissing){
    return toast('Connect the device first',true);
  }

  if(!confirm('Install the recommended ESC firmware?'))return;

  setBusy(true);
  resetLocalFinalization();
  showFlashStage('none');
  $('#flashProgress').value=0;
  $('#flashText').textContent='Preparing ESC firmware';

  try{
    const started=await api('/api/firmware/default',{
      method:'POST'
    });

    updateUI(started);
    startLocalFlashProgress(Number(started.firmwareSize||0));

    setBusy(false);
    toast('Firmware installation started');
    pollFlash();
  }catch(error){
    setBusy(false);
    resetLocalFinalization();
    stopLocalFlashProgress();
    toast(error.message,true);
  }
}

async function uploadAndFlash(){
  const file=$('#hexFile').files[0];

  if(!file){
    return toast('Select a firmware file',true);
  }

  if(file.size===0){
    return toast('Firmware file is empty',true);
  }

  // Intel HEX text can be significantly larger than the binary image.
  if(file.size>600000){
    return toast('Firmware file is too large',true);
  }

  if(!state||!state.connected){
    return toast('Bootloader not connected',true);
  }

  if(!state.loaded&&!state.firmwareMissing){
    return toast('Connect the device first',true);
  }

  if(!confirm(`Flash ${file.name} to the ESC?`))return;

  setBusy(true);
  resetLocalFinalization();
  showFlashStage('none');

  try{
    const form=new FormData();
    form.append('firmware',file,file.name);

    updateUI(await api('/api/firmware/upload',{
      method:'POST',
      body:form
    }));

    const started=await api('/api/firmware/start',{
      method:'POST'
    });

    updateUI(started);
    startLocalFlashProgress(Number(started.firmwareSize||0));

    setBusy(false);
    toast('Firmware installation started');
    pollFlash();
  }catch(error){
    setBusy(false);
    resetLocalFinalization();
    stopLocalFlashProgress();
    toast(error.message,true);
  }
}

async function clearFirmware(){
  try{
    updateUI(await api('/api/firmware/clear',{method:'POST'}));
    $('#hexFile').value='';
    resetLocalFinalization();
    showFlashStage('none');
    $('#flashProgress').value=0;
    $('#flashText').textContent='No firmware selected';
  }catch(error){
    toast(error.message,true);
  }
}

fetch('/api/status')
  .then(response=>response.json())
  .then(updateUI)
  .catch(()=>{});
</script>
</body>
</html>
)VPTEKWEB";

void setupRoutes()
{
    server.on("/", HTTP_GET, []() {
        server.sendHeader(
            F("Cache-Control"),
            F("no-store, no-cache, must-revalidate, max-age=0"));
        server.sendHeader(F("Pragma"), F("no-cache"));
        server.sendHeader(F("Expires"), F("0"));

        server.send_P(
            200,
            "text/html; charset=utf-8",
            INDEX_HTML);
    });

    server.on("/favicon.ico", HTTP_GET, []() {
        server.send(204, "text/plain", "");
    });

    server.on("/api/status", HTTP_GET, []() {
        sendJson(200, statusJson(true));
    });

    server.on("/api/connect", HTTP_POST, handleConnect);
    server.on("/api/save", HTTP_POST, handleSave);
    server.on("/api/reset", HTTP_POST, handleReset);
    server.on( "/api/firmware/upload", HTTP_POST, handleFirmwareUploadFinished, handleFirmwareUploadData);
    server.on( "/api/firmware/default", HTTP_POST, handleDefaultFirmwareStart);
    server.on( "/api/firmware/start", HTTP_POST, handleFlashStart);
    server.on( "/api/firmware/status", HTTP_GET, handleFlashStatus);
    server.on( "/api/firmware/clear", HTTP_POST, handleFlashClear);
    server.onNotFound([]() {
        sendError(404, "Not found");
    });
}

} // namespace

bool webBridgeLocksEsc()
{
    return requestBusy || uploadInProgress || webFlashPending || flashIsActive();
}

bool webRadioIsRunning()
{
    return webRadioRunning;
}

void stopWebRadio()
{
    if (!webRadioRunning) {
        return;
    }

    dnsServer.stop();
    server.stop();

    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);

    webRadioRunning = false;

    // Cho radio và task Wi-Fi dừng hoàn toàn trước timing GPIO21/PB4.
    delay(40);
}

void startWebRadio()
{
    // USB đang giữ quyền thì tuyệt đối không bật lại Wi-Fi.
    if (isAppMode()) {
        return;
    }

    if (webRadioRunning) {
        return;
    }

    WiFi.mode(WIFI_AP);
    WiFi.setSleep(false);

    WiFi.softAPConfig(
        AP_IP, AP_IP, IPAddress(255, 255, 255, 0));
    WiFi.softAP(
        AP_SSID, AP_PASSWORD);
    // Captive portal is intentionally disabled. Customers open 192.168.4.1
    // in Chrome, Safari, or another full browser.
    if (!webRoutesConfigured) {
        setupRoutes();
        webRoutesConfigured = true;
    }

    server.begin();
    webRadioRunning = true;
}

void setupWebBridge()
{
    WiFi.persistent(false);

    if (!webRoutesConfigured) {
        setupRoutes();
        webRoutesConfigured = true;
    }

    startWebRadio();
}

void webBridgeLoop()
{
    /*
     * Chỉ xử lý HTTP/DNS khi radio đang bật.
     * MODE_WEB vẫn giữ nguyên cả lúc radio tạm tắt để flash.
     */
    if (webRadioRunning) {
        server.handleClient();
    }

    /*
     * /api/firmware/start đã trả response.
     * Bây giờ mới tắt radio và bắt đầu ghi từ RAM.
     */
    if (webFlashPending &&
        static_cast<int32_t>(
        millis() - webFlashStartAt) >= 0) {
        webFlashPending = false;
        stopWebRadio();
        beginFlash();
    }

    processFlashStep();

    /*
     * Sau Done/Error, bật lại AP nhưng vẫn giữ MODE_WEB.
     * App Tool tiếp tục bị khóa đến khi thiết bị reset.
     */
    if (!isAppMode() && !webRadioRunning && !webFlashResumePending && !webFlashPending && (flashState == FlashState::Done || flashState == FlashState::Error)) {

        webFlashResumePending = true;
        webFlashResumeAt = millis() + 500;
    }

    if (webFlashResumePending && static_cast<int32_t>(
            millis() - webFlashResumeAt) >= 0) {

        webFlashResumePending = false;
        startWebRadio();
    }

    delay(1);
}
