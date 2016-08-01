#include "GPS.h"
#include "Pins.h"

void gps_test(GPS gps, Display disp) {
  char TestBuffer[20];
  byte b;
  
  Serial.println("GPS Test Mode");

  disp.clear();
  disp.setCursor(0, 0);
  disp.print("GPS Test Mode");
  
  while (!getInputMomentary(PIN_AUTO_ELEV, 0) && !getInputMomentary(PIN_AUTO_FOCUS, 0)) {
    while (Serial2.available()) {
      b = Serial2.read();
      gps.handleByte(b);
    }

    if (gps.hasFix()) {
      Serial.print("Lat: "); Serial.println(gps.getLat());
      Serial.print("Lon: "); Serial.println(gps.getLon());
      Serial.print("Day: "); Serial.println(gps.getDayOfYear());
      Serial.print("Minute: "); Serial.println(gps.getMinuteOfDayGMT());
    }
    else {
      Serial.println("No fix");
    }

    if (gps.hasFix()) {
      disp.setCursor(0, 1);
      disp.print("Lat: ");
      dtostrf(gps.getLat(), 6, 2, TestBuffer);
      disp.print(TestBuffer);
      disp.setCursor(0, 2);
      disp.print("Lon: ");
      dtostrf(gps.getLon(), 6, 2, TestBuffer);
      disp.print(TestBuffer);
      disp.setCursor(0, 3);
      disp.print("Day:");
      disp.print(String(gps.getDayOfYear()));
      disp.print(" Minute:");
      disp.print(String(gps.getMinuteOfDayGMT()));
    }
    else {
      disp.setCursor(0, 3);
      disp.print("        No fix      ");      
    }
  }
}

bool GPS::hasFix() {
  return fix;
}

bool GPS::hasHadFix() {
  return hadFix;
}

bool GPS::hasTime() {
  return validTime;
}

float GPS::getLat() {
  return lat;
}

float GPS::getLon() {
  return lon;
}

void GPS::init() {
  GPSbuffer.reserve(100);
  title.reserve(10);
}

int GPS::getDayOfYear() {
  return day + month * 30.42;
}

int GPS::getMinuteOfDayGMT() {
  return hours * 60 + minutes;
}

int GPS::handleByte(byte byteGPS) {
  switch (lineState) {
  case LINE_STATE_DOLLAR:
    if (byteGPS == '$') {
      lineState = LINE_STATE_TITLE;
      title = "";
    }
    break;
  
  case LINE_STATE_TITLE:
    if (byteGPS == ',') {
      lineState = LINE_STATE_BODY;
      GPSbuffer = "";
      sectionCounter = 0;
    }
    else {
      title += (char)byteGPS;
    }
    break;

  case LINE_STATE_BODY:
    if (byteGPS == '*') {
      lineState = LINE_STATE_DOLLAR;
    }
    else if (byteGPS == ',') {
      sectionCounter++;
      processBuffer();
      GPSbuffer = "";
    }
    else {
      GPSbuffer += (char)byteGPS;
    }
    break;
  }
}

void GPS::processBuffer() {
  int newHours, newMinutes;

  if (title == "GPRMC") {
    if (sectionCounter == 1 /* timestamp */) {
      newHours = GPSbuffer.substring(0, 2).toInt();
      newMinutes = GPSbuffer.substring(2, 4).toInt();
      if (newHours == 0 && newMinutes == 0) {
        validTime = false;
        return;
      }
      else {
        hours = newHours;
        minutes = newMinutes;
        validTime = true;
      }
    }
    else if (sectionCounter == 2 /* GPS ok */) {
      fix = GPSbuffer == "A";
      if (fix) hadFix = true;
    }
    else if (sectionCounter == 3 /* longitude magnitude */) {
      if (fix) lon = GPSbuffer.toFloat() / 100.0;
    }
    else if (sectionCounter == 4 /* longitude sign */) {
      if (fix && GPSbuffer == "S") lon = -lon;
    }
    else if (sectionCounter == 5 /* latitude magnitude */) {
      if (fix) lat = GPSbuffer.toFloat() / 100.0;
    }
    else if (sectionCounter == 6 /* latitude sign */) {
      if (fix && GPSbuffer == "W") lat = -lat;
    }
    else if (sectionCounter == 9 /* date */) {
      day = GPSbuffer.substring(0, 2).toInt();
      month = GPSbuffer.substring(2, 4).toInt();
      year = 2000 + GPSbuffer.substring(4, 6).toInt();
    }
  }
}

