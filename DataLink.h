#ifndef _DATALINK_h
#define _DATALINK_h

#include "Arduino.h"

class DataLink {

public:
  void init(HardwareSerial *_sp);
  void setValue(int i, int value);
  void send(void);

private:
  String encode(int data[]);
  int values[8];
  HardwareSerial *sp;

};

#endif

