#ifndef _GPS_h
#define _GPS_h

#include "Arduino.h"
#include "Display.h"

#define LINE_STATE_DOLLAR 1
#define LINE_STATE_TITLE 2
#define LINE_STATE_BODY 3

class GPS {

public:
  bool hasFix();
  bool hasTime();
  bool hasHadFix();
  unsigned int getTime();
  float getLat();
  float getLon();
  void init();
  int handleByte(byte byteGPS);
  int getDayOfYear();
  int getMinuteOfDayGMT();
private:
  float lat;
  float lon;
  bool fix = 0, hadFix = 0;
  bool validTime = 0;
  int month, day, year;
  int hours, minutes;

  void processBuffer();
  String title;
  String GPSbuffer;
  int lineState = LINE_STATE_DOLLAR;
  int sectionCounter;
};

void gps_test(GPS gps, Display disp);

#endif

