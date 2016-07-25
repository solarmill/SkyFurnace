#include "Settings.h"
#include "Arduino.h"
#include <EEPROM.h>

void saveElevation(float angle) {
  Serial.print("Save Elev ");
  Serial.println(angle);
  saveInt(SETTING_ELEVATION_LOC, (angle + 100) * 1000);
  saveInt(SETTING_ELEVATION_LOC+1, 999);
}

float loadElevation() {
  float val = readInt(SETTING_ELEVATION_LOC) / 1000.0 - 100.0;
  Serial.print("Load Elev ");
  Serial.println(val);
  if (readInt(SETTING_ELEVATION_LOC+1) != 999) val = 0;
  Serial.println(val);
  return val;
}

void saveFocus(float distance) {
  Serial.print("Save Focus ");
  Serial.println(distance);
  saveInt(SETTING_FOCUS_LOC, (distance + 100) * 1000);
  saveInt(SETTING_FOCUS_LOC+1, 888);
}

float loadFocus() {
  float val = readInt(SETTING_FOCUS_LOC) / 1000.0 - 100.0;
  Serial.print("Load Focus ");
  Serial.println(val);
  if (readInt(SETTING_FOCUS_LOC+1) != 999) val = 0;
  Serial.println(val);
  return val;
}

void saveTimezoneOffset(int offset) {
  Serial.print("Save timezone offset ");
  Serial.println(offset);
  saveInt(SETTING_TIMEZONE_LOC, offset + 100);
}

int loadTimezoneOffset() {
  int offset = readInt(SETTING_TIMEZONE_LOC) - 100;
  if (offset > 12 || offset < -12) offset = 0;
  Serial.print("Load timezone offset ");
  Serial.println(offset);
  return offset;  
}

void saveTempSource(int source) {
  Serial.print("Save temperature source ");
  Serial.println(source);
  saveInt(SETTING_TEMP_SOURCE_LOC, source);
}

int loadTempSource() {
  int source = readInt(SETTING_TEMP_SOURCE_LOC);
  if (source > 9) source = 0;
  Serial.print("Load temperature source ");
  Serial.println(source);
  return source;  
}

void saveInt(byte position, unsigned long value) {
  byte b;
  b = value % 256;
  Serial.print("SaveInt position "); Serial.print(position); Serial.print(" <= "); Serial.println(value);
  EEPROM.write(position * 4, b);
  Serial.print("SaveInt A: ["); Serial.print(position * 4); Serial.print("] <= "); Serial.println(b);
  value /= 256;
  b = value % 256;
  EEPROM.write(position * 4 + 1, b);
  Serial.print("SaveInt B: ["); Serial.print(position * 4 + 1); Serial.print("] <= "); Serial.println(b);
  value /= 256;
  b = value % 256;
  EEPROM.write(position * 4 + 2, b);
  Serial.print("SaveInt C: ["); Serial.print(position * 4 + 2); Serial.print("] <= "); Serial.println(b);
  value /= 256;
  b = value % 256;  
  EEPROM.write(position * 4 + 3, b);
  Serial.print("SaveInt D: ["); Serial.print(position * 4 + 3); Serial.print("] <= "); Serial.println(b);
}

unsigned long readInt(byte position) {
  byte b;
  b = EEPROM.read(position * 4 + 3);
  Serial.print("ReadInt A: ["); Serial.print(position * 4 + 3); Serial.print("]="); Serial.println(b);
  unsigned long output = b;
  output *= 256;
  b = EEPROM.read(position * 4 + 2);
  Serial.print("ReadInt B: ["); Serial.print(position * 4 + 2); Serial.print("]="); Serial.println(b);
  output += b;
  output *= 256;
  b = EEPROM.read(position * 4 + 1);
  Serial.print("ReadInt C: ["); Serial.print(position * 4 + 1); Serial.print("]="); Serial.println(b);
  output += b;
  output *= 256;
  b = EEPROM.read(position * 4 + 0);
  Serial.print("ReadInt D: ["); Serial.print(position * 4 + 0); Serial.print("]="); Serial.println(b);
  output += b;
  Serial.print("ReadInt Result for position "); Serial.print(position); Serial.print(" = "); Serial.println(output);
  return output;
}
