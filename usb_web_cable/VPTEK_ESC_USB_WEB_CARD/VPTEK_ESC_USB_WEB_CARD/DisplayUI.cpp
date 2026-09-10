#include "DisplayUI.h"

#include <Arduino.h>
#include <SPI.h>
#include <U8g2lib.h>

#include "web_bridge.h"

namespace {

U8G2_ST7567_ENH_DG128064I_F_4W_HW_SPI display(
    U8G2_R2, LCD_CS_PIN, LCD_DC_PIN, LCD_RST_PIN);

enum class Screen : uint8_t {
    Home,
    Working,
    Menu,
    Edit,
    Result,
    FirmwareMenu,
    FirmwareFlashing,
    FirmwareResult
};

enum class Format : uint8_t {
    Raw,
    Bool,
    Ramp,
    MinimumDuty,
    Timing,
    Kv,
    CurrentPid,
    ServoLow,
    ServoHigh,
    ServoNeutral,
    CellVoltage,
    CurrentLimit,
    HalfVoltage,
    Temperature,
    InputType
};

struct SettingItem {
    const char *name;
    uint8_t offset;
    uint8_t rawMin;
    uint8_t rawMax;
    uint8_t rawStep;
    Format format;
    const char *unit;
};

/*
 * These offsets and conversions are the same ones used by web_bridge.cpp.
 * Card mode edits the existing 48-byte AM32 EEPROM block; it does not use the
 * unrelated AA/CB protocol from the LCD reference project.
 */
constexpr SettingItem items[] = {
    {"Comp PWM", 20, 0, 1, 1, Format::Bool, ""},
    {"Stuck Prot", 22, 0, 1, 1, Format::Bool, ""},
    {"Stall Prot", 29, 0, 1, 1, Format::Bool, ""},
    {"Hall", 39, 0, 1, 1, Format::Bool, ""},
    {"No Calib", 7, 0, 1, 1, Format::Bool, ""},
    {"Sine Start", 19, 0, 1, 1, Format::Bool, ""},

    {"Timing", 23, 10, 42, 1, Format::Timing, "deg"},
    {"Motor KV", 26, 0, 255, 1, Format::Kv, ""},
    {"Poles", 27, 2, 32, 2, Format::Raw, ""},
    {"Start Power", 25, 50, 150, 1, Format::Raw, ""},
    {"PWM Freq", 24, 8, 144, 1, Format::Raw, "kHz"},
    {"Beep Vol", 30, 0, 11, 1, Format::Raw, ""},
    {"Brake Stop", 28, 0, 1, 1, Format::Bool, ""},
    {"Stop Level", 41, 1, 10, 1, Format::Raw, ""},
    {"Sine Range", 40, 5, 25, 1, Format::Raw, ""},
    {"Sine Power", 45, 1, 10, 1, Format::Raw, ""},
    {"Run Brake", 42, 1, 10, 1, Format::Raw, ""},

    {"Throttle", 5, 1, 250, 1, Format::Ramp, "%/ms"},
    {"Min Duty", 6, 0, 50, 1, Format::MinimumDuty, "%"},
    {"Auto Timing", 47, 0, 1, 1, Format::Bool, ""},
    {"Var PWM", 21, 0, 1, 1, Format::Bool, ""},
    {"Act Brake", 12, 0, 5, 1, Format::Raw, "%"},

    {"Servo Low", 32, 0, 255, 1, Format::ServoLow, "us"},
    {"Servo High", 33, 0, 255, 1, Format::ServoHigh, "us"},
    {"Neutral", 34, 0, 255, 1, Format::ServoNeutral, "us"},
    {"Deadband", 35, 0, 255, 1, Format::Raw, ""},
    {"Abs Cutoff", 8, 0, 200, 1, Format::HalfVoltage, "V"},
    {"Cell Cutoff", 36, 0, 1, 1, Format::Bool, ""},
    {"Cell Volt", 37, 0, 100, 1, Format::CellVoltage, "V"},
    {"Temp Limit", 43, 70, 141, 1, Format::Temperature, "C"},
    {"Curr Limit", 44, 1, 101, 1, Format::CurrentLimit, "A"},
    {"Curr P", 9, 0, 255, 1, Format::CurrentPid, ""},
    {"Curr I", 10, 0, 255, 1, Format::Raw, ""},
    {"Curr D", 11, 0, 255, 1, Format::CurrentPid, ""}
};

constexpr uint8_t ITEM_COUNT = sizeof(items) / sizeof(items[0]);

Screen screen = Screen::Home;
uint8_t eeprom[ESC_EEPROM_SIZE] = {0};
uint8_t selected = 0;
uint8_t editOriginal = 0;
String resultTitle;
String resultDetail;
uint32_t resultUntil = 0;
uint32_t okPressedAt = 0;
bool okLongHandled = false;
uint32_t lastMarqueeDraw = 0;
uint32_t lastFirmwareDraw = 0;
uint8_t firmwareMenuSelected = 0;
Screen firmwareReturnScreen = Screen::Home;
bool firmwareSucceeded = false;

constexpr int MENU_LABEL_RIGHT = 68;
constexpr int MENU_VALUE_LEFT = 70;
constexpr int MENU_VALUE_RIGHT = 128;
constexpr int MENU_VALUE_WIDTH = MENU_VALUE_RIGHT - MENU_VALUE_LEFT;
constexpr uint32_t MARQUEE_STEP_MS = 120;
constexpr int MARQUEE_GAP = 16;

struct KeyState {
    uint8_t pin;
    bool previous;
    uint32_t changedAt;
};

KeyState keyOk   = {KEY_OK_PIN, true, 0};
KeyState keyDown = {KEY_DOWN_PIN, true, 0};
KeyState keyUp   = {KEY_UP_PIN, true, 0};

void copyText(char *out, size_t capacity, const String &text)
{
    if (capacity == 0) return;
    text.substring(0, capacity - 1).toCharArray(out, capacity);
}

void centerText(int y, const char *text)
{
    const int width = display.getStrWidth(text);
    display.drawStr(max(0, (128 - width) / 2), y, text);
}

bool itemEditable(uint8_t index)
{
    const SettingItem &item = items[index];

    // Same dependencies used by the Web interface.
    if (item.offset == 24 && eeprom[21] != 0) return false;
    if (item.offset == 41 && eeprom[28] == 0) return false;
    if (item.offset == 12 && eeprom[28] != 0) return false;
    if (item.offset == 37 && eeprom[36] == 0) return false;
    return true;
}

void formatValue(uint8_t index, char *buffer, size_t capacity)
{
    const SettingItem &item = items[index];
    const uint8_t raw = eeprom[item.offset];

    switch (item.format) {
        case Format::Bool:
            snprintf(buffer, capacity, "%s", raw ? "ON" : "OFF");
            return;

        case Format::Ramp:
            snprintf(buffer, capacity, "%.1f%s", raw / 10.0f, item.unit);
            return;

        case Format::MinimumDuty:
            snprintf(buffer, capacity, "%.1f%s", raw / 2.0f, item.unit);
            return;

        case Format::Timing: {
            const float value =
                (raw >= 10 && raw <= 42) ? (raw - 10) * 0.9375f : 15.0f;
            snprintf(buffer, capacity, "%.1f%s", value, item.unit);
            return;
        }

        case Format::Kv:
            snprintf(buffer, capacity, "%u", raw * 40U + 20U);
            return;

        case Format::CurrentPid:
            snprintf(buffer, capacity, "%u", raw * 2U);
            return;

        case Format::ServoLow:
            snprintf(buffer, capacity, "%uus", raw * 2U + 750U);
            return;

        case Format::ServoHigh:
            snprintf(buffer, capacity, "%uus", raw * 2U + 1750U);
            return;

        case Format::ServoNeutral:
            snprintf(buffer, capacity, "%uus", raw + 1374U);
            return;

        case Format::CellVoltage:
            snprintf(buffer, capacity, "%.2fV", (raw + 250) / 100.0f);
            return;

        case Format::CurrentLimit:
            if (raw == 0 || raw > 100) snprintf(buffer, capacity, "Disabled");
            else snprintf(buffer, capacity, "%uA", raw * 2U);
            return;

        case Format::HalfVoltage:
            if (raw == 0) snprintf(buffer, capacity, "Disabled");
            else snprintf(buffer, capacity, "%.1fV", raw * 0.5f);
            return;

        case Format::Temperature:
            if (raw < 70 || raw > 140) snprintf(buffer, capacity, "Disabled");
            else snprintf(buffer, capacity, "%uC", raw);
            return;

        case Format::InputType: {
            static const char *names[] = {
                "AUTO", "DSHOT", "SERVO", "SERIAL", "SAFE ARM"
            };
            snprintf(buffer, capacity, "%s", names[raw <= 4 ? raw : 4]);
            return;
        }

        default:
            snprintf(buffer, capacity, "%u%s", raw, item.unit);
            return;
    }
}

void drawHeader(const char *title)
{
    display.setDrawColor(1);
    display.drawBox(0, 0, 128, 13);
    display.setDrawColor(0);
    display.setFont(u8g2_font_6x10_tf);
    display.drawStr(3, 10, title);
    display.setDrawColor(1);
}

const char *firmwareStageText(CardFirmwareStage stage)
{
    switch (stage) {
        case CardFirmwareStage::Preparing:  return "Preparing ESC";
        case CardFirmwareStage::Safety:     return "Setting safety";
        case CardFirmwareStage::Writing:    return "Writing firmware";
        case CardFirmwareStage::Finalizing: return "Finalizing";
        case CardFirmwareStage::Resetting:  return "Restarting ESC";
        case CardFirmwareStage::Done:       return "Install complete";
        case CardFirmwareStage::Error:      return "Install failed";
        default:                            return "Starting...";
    }
}

void draw()
{
    display.clearBuffer();
    display.setFont(u8g2_font_6x10_tf);

    if (screen == Screen::Home) {
        display.setFont(u8g2_font_ncenB12_tr);
        centerText(20, "VPTEK");
        display.setFont(u8g2_font_6x10_tf);
        centerText(39, "ESC CONTROL CARD");
        display.drawFrame(13, 47, 102, 15);
        centerText(58, "OK: READ ESC");
    } else if (screen == Screen::Working) {
        drawHeader("CARD SETTINGS");
        centerText(34, "Working...");
        centerText(52, "Please wait");
    } else if (screen == Screen::FirmwareMenu) {
        drawHeader("FIRMWARE");
        display.setFont(u8g2_font_6x10_tf);

        const char *menuLines[] = {
            "INSTALL VPTEK FW",
            "EXIT FIRMWARE"
        };

        for (uint8_t row = 0; row < 2; ++row) {
            const int y = 31 + row * 18;
            if (firmwareMenuSelected == row) {
                display.drawBox(3, y - 12, 122, 15);
                display.setDrawColor(0);
            }
            display.drawStr(8, y, menuLines[row]);
            display.setDrawColor(1);
        }

        display.setFont(u8g2_font_5x7_tf);
        centerText(63, "UP/DOWN  OK SELECT");
    } else if (screen == Screen::FirmwareFlashing) {
        const CardFirmwareStatus status = cardGetFirmwareStatus();
        drawHeader("INSTALL FIRMWARE");
        display.setFont(u8g2_font_6x10_tf);
        centerText(27, firmwareStageText(status.stage));

        display.drawFrame(5, 33, 118, 11);
        const int progressWidth =
            static_cast<int>(status.percent) * 116 / 100;
        if (progressWidth > 0) {
            display.drawBox(6, 34, progressWidth, 9);
        }

        char progress[18];
        snprintf(progress, sizeof(progress), "%u%%", status.percent);
        centerText(55, progress);

        display.setFont(u8g2_font_5x7_tf);
        char bytes[28];
        snprintf(
            bytes,
            sizeof(bytes),
            "%lu/%lu bytes",
            static_cast<unsigned long>(status.written),
            static_cast<unsigned long>(status.total));
        centerText(63, bytes);
    } else if (screen == Screen::FirmwareResult) {
        drawHeader(firmwareSucceeded ? "FW SUCCESS" : "FW FAILED");
        display.setFont(u8g2_font_6x10_tf);
        centerText(
            31,
            firmwareSucceeded ? "Install complete" : "Installation error");

        display.setFont(u8g2_font_5x7_tf);
        char line[26];
        copyText(line, sizeof(line), resultDetail);
        centerText(44, line);
        centerText(
            61,
            firmwareSucceeded ? "OK: READ ESC" : "OK: TRY AGAIN");
    } else if (screen == Screen::Result) {
        drawHeader(resultTitle.c_str());
        char line[22];
        copyText(line, sizeof(line), resultDetail);
        centerText(36, line);
        centerText(57, "OK: CONTINUE");
    } else if (screen == Screen::Menu) {
        drawHeader("ESC SETTINGS");
        const uint8_t first =
            selected == 0 ? 0 :
            (selected >= ITEM_COUNT - 1 ? ITEM_COUNT - 3 : selected - 1);

        for (uint8_t row = 0; row < 3; ++row) {
            const uint8_t index = first + row;
            if (index >= ITEM_COUNT) break;

            const int y = 27 + row * 14;
            if (index == selected) {
                display.drawBox(0, y - 10, 128, 13);
                display.setDrawColor(0);
            }

            char name[16];
            char value[18];
            copyText(name, sizeof(name), items[index].name);
            formatValue(index, value, sizeof(value));

            // Keep both columns separated. Long values scroll only inside
            // the right-hand value area and never overwrite the label.
            display.setClipWindow(0, y - 10, MENU_LABEL_RIGHT, y + 3);
            display.drawStr(2, y, name);
            display.setMaxClipWindow();

            const int valueWidth = display.getStrWidth(value);
            display.setClipWindow(
                MENU_VALUE_LEFT, y - 10, MENU_VALUE_RIGHT, y + 3);

            if (valueWidth <= MENU_VALUE_WIDTH - 2) {
                display.drawStr(MENU_VALUE_RIGHT - valueWidth - 2, y, value);
            } else {
                const int trackWidth = valueWidth + MARQUEE_GAP;
                const int offset =
                    (millis() / MARQUEE_STEP_MS) % trackWidth;
                const int firstX = MENU_VALUE_LEFT - offset;

                display.drawStr(firstX, y, value);
                display.drawStr(firstX + trackWidth, y, value);
            }

            display.setMaxClipWindow();
            display.setDrawColor(1);
        }

        if (!itemEditable(selected)) {
            display.setFont(u8g2_font_5x7_tf);
            display.drawStr(2, 63, "LOCKED BY RELATED SETTING");
        }
    } else if (screen == Screen::Edit) {
        drawHeader("EDIT SETTING");
        char name[22];
        char value[24];
        copyText(name, sizeof(name), items[selected].name);
        formatValue(selected, value, sizeof(value));
        centerText(28, name);
        display.setFont(u8g2_font_7x13B_tf);
        centerText(45, value);
        display.setFont(u8g2_font_5x7_tf);
        centerText(61, "DOWN -   UP +   OK SAVE");
    }

    display.sendBuffer();
}

void showResult(const String &title, const String &detail, uint32_t duration = 0)
{
    resultTitle = title;
    resultDetail = detail;
    resultUntil = duration == 0 ? 0 : millis() + duration;
    screen = Screen::Result;
    draw();
}

void readEsc()
{
    screen = Screen::Working;
    draw();

    String message;
    if (cardReadEscSettings(eeprom, sizeof(eeprom), message)) {
        selected = 0;
        showResult("ESC CONNECTED", "Settings loaded", 900);
    } else {
        showResult("READ FAILED", message);
    }
}

void saveSelected()
{
    screen = Screen::Working;
    draw();

    String message;
    if (cardWriteEscSettings(eeprom, sizeof(eeprom), message)) {
        showResult("SAVE COMPLETE", "ESC restarted", 1100);
    } else {
        eeprom[items[selected].offset] = editOriginal;
        showResult("SAVE FAILED", message);
    }
}

void openFirmwareMenu()
{
    if (screen == Screen::Edit) {
        // Discard an unsaved edit before leaving the settings screen.
        eeprom[items[selected].offset] = editOriginal;
        firmwareReturnScreen = Screen::Menu;
    } else if (screen == Screen::Menu) {
        firmwareReturnScreen = Screen::Menu;
    } else {
        firmwareReturnScreen = Screen::Home;
    }

    firmwareMenuSelected = 0;
    screen = Screen::FirmwareMenu;
    draw();
}

void startCardFirmware()
{
    screen = Screen::FirmwareFlashing;
    firmwareSucceeded = false;
    resultDetail = "";
    draw();

    String message;
    if (!cardStartBuiltInFirmware(message)) {
        firmwareSucceeded = false;
        resultDetail = message;
        screen = Screen::FirmwareResult;
        draw();
    }
}

void updateCardFirmwareScreen()
{
    const CardFirmwareStatus status = cardGetFirmwareStatus();

    if (status.stage == CardFirmwareStage::Done) {
        firmwareSucceeded = true;
        resultDetail = "ESC restarted";
        screen = Screen::FirmwareResult;
        draw();
        return;
    }

    if (status.stage == CardFirmwareStage::Error) {
        firmwareSucceeded = false;
        resultDetail = status.message;
        screen = Screen::FirmwareResult;
        draw();
        return;
    }

    draw();
}

void changeValue(int direction)
{
    const SettingItem &item = items[selected];
    uint8_t &raw = eeprom[item.offset];

    if (item.format == Format::Bool) {
        raw = raw ? 0 : 1;
        return;
    }

    if (item.format == Format::Temperature) {
        if (direction > 0) {
            raw = raw > 140 ? 70 : (raw == 140 ? 255 : raw + 1);
        } else {
            raw = raw > 140 ? 140 : (raw <= 70 ? 70 : raw - 1);
        }
        return;
    }

    if (item.format == Format::CurrentLimit) {
        if (direction > 0) {
            raw = raw > 100 ? 1 : (raw == 100 ? 101 : raw + 1);
        } else {
            raw = raw > 100 ? 100 : (raw <= 1 ? 1 : raw - 1);
        }
        return;
    }

    const int next = static_cast<int>(raw) + direction * item.rawStep;
    raw = static_cast<uint8_t>(constrain(next, item.rawMin, item.rawMax));
}

bool pressed(KeyState &key)
{
    const bool now = digitalRead(key.pin) == HIGH;
    const uint32_t current = millis();
    bool event = false;

    if (now != key.previous && current - key.changedAt >= 35) {
        key.changedAt = current;
        key.previous = now;
        event = !now;
    }
    return event;
}

void onOkPressed()
{
    if (screen == Screen::Home) {
        readEsc();
        return;
    }

    if (screen == Screen::FirmwareMenu) {
        if (firmwareMenuSelected == 0) {
            startCardFirmware();
        } else {
            screen = firmwareReturnScreen;
            draw();
        }
        return;
    }

    if (screen == Screen::FirmwareResult) {
        if (firmwareSucceeded) {
            // Firmware completion resets the ESC, so read it again before
            // allowing any settings to be changed.
            screen = Screen::Home;
            readEsc();
        } else {
            firmwareMenuSelected = 0;
            screen = Screen::FirmwareMenu;
            draw();
        }
        return;
    }

    if (screen == Screen::Result) {
        screen = resultTitle == "READ FAILED" ? Screen::Home : Screen::Menu;
        resultUntil = 0;
        draw();
        return;
    }

    if (screen == Screen::Menu) {
        if (!itemEditable(selected)) {
            showResult("SETTING LOCKED", "Check related option", 1000);
            return;
        }
        editOriginal = eeprom[items[selected].offset];
        screen = Screen::Edit;
        draw();
        return;
    }

    if (screen == Screen::Edit) {
        if (eeprom[items[selected].offset] == editOriginal) {
            screen = Screen::Menu;
            draw();
        } else {
            saveSelected();
        }
    }
}

} // namespace

void DisplayUI_Init()
{
    pinMode(KEY_OK_PIN, INPUT_PULLUP);
    pinMode(KEY_DOWN_PIN, INPUT_PULLUP);
    pinMode(KEY_UP_PIN, INPUT_PULLUP);
    pinMode(LCD_VLED_PIN, OUTPUT);
    digitalWrite(LCD_VLED_PIN, HIGH);

    SPI.begin(LCD_CLK_PIN, -1, LCD_MOSI_PIN, LCD_CS_PIN);
    display.begin();
    display.setContrast(127);
    draw();
}

void DisplayUI_Loop()
{
    const uint32_t now = millis();

    if (screen == Screen::Menu &&
        now - lastMarqueeDraw >= MARQUEE_STEP_MS) {
        lastMarqueeDraw = now;
        draw();
    }

    if (screen == Screen::FirmwareFlashing &&
        now - lastFirmwareDraw >= 120) {
        lastFirmwareDraw = now;
        updateCardFirmwareScreen();
    }

    if (screen == Screen::Result &&
        resultUntil != 0 &&
        static_cast<int32_t>(now - resultUntil) >= 0) {
        resultUntil = 0;
        screen = Screen::Menu;
        draw();
    }

    const bool okEvent = pressed(keyOk);
    const bool downEvent = pressed(keyDown);
    const bool upEvent = pressed(keyUp);

    if (digitalRead(KEY_OK_PIN) == LOW) {
        if (okEvent || okPressedAt == 0) {
            okPressedAt = now;
            okLongHandled = false;
        } else if (!okLongHandled &&
                   now - okPressedAt >= 1200 &&
                   (screen == Screen::Home ||
                    screen == Screen::Menu ||
                    screen == Screen::Edit)) {
            okLongHandled = true;
            openFirmwareMenu();
        }
    } else if (okPressedAt != 0) {
        if (!okLongHandled) onOkPressed();
        okPressedAt = 0;
        okLongHandled = false;
    }

    if (screen == Screen::Menu) {
        if (downEvent) {
            selected = (selected + 1) % ITEM_COUNT;
            draw();
        } else if (upEvent) {
            selected = selected == 0 ? ITEM_COUNT - 1 : selected - 1;
            draw();
        }
    } else if (screen == Screen::FirmwareMenu) {
        if (downEvent || upEvent) {
            firmwareMenuSelected = firmwareMenuSelected == 0 ? 1 : 0;
            draw();
        }
    } else if (screen == Screen::Edit) {
        if (downEvent) {
            changeValue(-1);
            draw();
        } else if (upEvent) {
            changeValue(1);
            draw();
        }
    }
}
