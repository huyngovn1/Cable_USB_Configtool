#include <Arduino.h>
#include "Global.h"             // Global variables
#include "ESC_Serial.h"         // ESC Serial Header
#include "SoftwareSerial.h"     // Library ür Softserial

//#define _DEBUG_ 
uint16_t esc_crc = 0;

SoftwareSerial swSer1;

namespace {

// Discard bytes that arrived after the previous command timed out. A stale
// ACK must not be accepted as the response to the next command.
void clearEscReceiveBuffer()
{
  while (swSer1.available()) {
    swSer1.read();
  }
}

} // namespace

void InitSerialOutput() {
  Enable4Way = true;
  swSer1.begin(19200, SWSERIAL_8N1, SERVO_OUT, SERVO_OUT, false, 512);
  swSer1.enableIntTx(false);
#ifdef _DEBUG_
  Serial.println("Init ESC Serial");
#endif
}

void DeinitSerialOutput() {
  swSer1.end();
  Enable4Way = false;
}

uint16_t SendESC(uint8_t tx_buf[], uint16_t buf_size) {
  uint16_t i = 0;
  esc_crc = 0;
  if (buf_size == 0) {
    buf_size = 256;
  }
#ifdef _DEBUG_
  Serial.print("write ESC: ");
#endif
  clearEscReceiveBuffer();
  swSer1.enableTx(true);
  for (i = 0; i < buf_size; i++) {
    swSer1.write(tx_buf[i]);
#ifdef _DEBUG_
    Serial.print(tx_buf[i], HEX);
    Serial.print(" ");
#endif
    esc_crc = ByteCrc(tx_buf[i], esc_crc);
  }
  swSer1.write(esc_crc & 0xff);
  swSer1.write((esc_crc >> 8) & 0xff);
#ifdef _DEBUG_
  Serial.print(esc_crc & 0xff, HEX);
  Serial.print(" ");
  Serial.print((esc_crc >> 8) & 0xff, HEX);
  Serial.print(" ");
#endif
  buf_size = buf_size + 2;
  swSer1.enableTx(false);
#ifdef _DEBUG_
  Serial.println("done");
#endif
  i = 0;
  return i;
}

uint16_t SendESC(uint8_t tx_buf[], uint16_t buf_size, bool CRC) {
  uint16_t i = 0;
  esc_crc = 0;
  if (buf_size == 0) {
    buf_size = 256;
  }
#ifdef _DEBUG_
  Serial.print("write ESC: ");
#endif
  clearEscReceiveBuffer();
  swSer1.enableTx(true);
  for (i = 0; i < buf_size; i++) {
    swSer1.write(tx_buf[i]);
#ifdef _DEBUG_
    Serial.print(tx_buf[i], HEX);
    Serial.print(" ");
#endif
    esc_crc = ByteCrc(tx_buf[i], esc_crc);
  }
  if (CRC) {
    swSer1.write(esc_crc & 0xff);
    swSer1.write((esc_crc >> 8) & 0xff);
#ifdef _DEBUG_
    Serial.print(esc_crc & 0xff, HEX);
    Serial.print(" ");
    Serial.print((esc_crc >> 8) & 0xff, HEX);
    Serial.print(" ");
#endif
    buf_size = buf_size + 2;
  }
  swSer1.enableTx(false);
#ifdef _DEBUG_
  Serial.println("done");
#endif
  i = 0;
  return i;
}

uint16_t GetESC(uint8_t rx_buf[], uint16_t wait_ms, uint16_t capacity) {
  uint16_t i = 0;
  esc_crc = 0;
  const uint32_t startedAt = millis();
  uint32_t lastByteAtUs = micros();
  bool receivedAny = false;
#ifdef _DEBUG_
  Serial.print("ESC Read: ");
#endif
  while (millis() - startedAt < wait_ms) {
    while (swSer1.available() && i < capacity) {
      rx_buf[i] = swSer1.read();
#ifdef _DEBUG_
      Serial.print(rx_buf[i], HEX);
      Serial.print(" ");
#endif
      i++;
      receivedAny = true;
      lastByteAtUs = micros();
    }

    // 19200 baud needs about 0.52 ms per byte. A 2 ms quiet gap reliably
    // collects a complete response even when the bytes arrive gradually.
    if (receivedAny &&
        static_cast<uint32_t>(micros() - lastByteAtUs) >= 2000U) {
      break;
    }

    delayMicroseconds(100);
    yield();
  }
#ifdef _DEBUG_
  if (!receivedAny) Serial.println("Timeout");
#endif
#ifdef _DEBUG_
  Serial.println("done");
#endif
  return i;
}

uint16_t ByteCrc(uint8_t data, uint16_t crc)
{
  uint8_t xb = data;
  for (uint8_t i = 0; i < 8; i++)
  {
    if (((xb & 0x01) ^ (crc & 0x0001)) != 0 ) {
      crc = crc >> 1;
      crc = crc ^ 0xA001;
    } else {
      crc = crc >> 1;
    }
    xb = xb >> 1;
  }
  return crc;
}
