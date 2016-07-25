#ifndef _SETTINGS_h
#define _SETTINGS_h

#include "Arduino.h"

#define SETTING_ELEVATION_LOC 0
#define SETTING_FOCUS_LOC 2
#define SETTING_TIMEZONE_LOC 4
#define SETTING_TEMP_SOURCE_LOC 6

void saveElevation(float angle);
float loadElevation();
void saveFocus(float distance);
float loadFocus();
void saveInt(byte position, unsigned long value);
unsigned long readInt(byte position);
void saveTimezoneOffset(int offset);
int loadTimezoneOffset();
int loadTempSource();
void saveTempSource(int source);

#endif
