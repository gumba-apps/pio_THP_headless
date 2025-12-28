#include <Arduino.h>

void hexDump(const void* data, uint32_t length) {
  const uint8_t* bytes = (const uint8_t*)data;
  const uint32_t bytesPerLine = 16;

  for (uint32_t i = 0; i < length; i += bytesPerLine) {
    // Print offset
    Serial.printf("%08X  ", i);

    // Print hex bytes
    for (uint32_t j = 0; j < bytesPerLine; j++) {
      if (i + j < length)
        Serial.printf("%02X ", bytes[i + j]);
      else
        Serial.print("   ");
    }

    Serial.print(" ");

    // Print ASCII representation
    for (uint32_t j = 0; j < bytesPerLine; j++) {
      if (i + j < length) {
        char c = bytes[i + j];
        Serial.print((c >= 32 && c <= 126) ? c : '.');
      }
    }

    Serial.println();
  }
}
