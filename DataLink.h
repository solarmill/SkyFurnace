#ifndef _DATALINK_h
#define _DATALINK_h

#include "Arduino.h"

#define DATALINK_NUM_VALUES 12
class DataLink {

public:
  void init(HardwareSerial *_sp);
  void setValue(int i, int value);
  void send(void);

private:
  String encode(int data[]);
  int values[DATALINK_NUM_VALUES];
  HardwareSerial *sp;

};

#endif

