#include "DataLink.h"

void DataLink::init(HardwareSerial *_sp) {
   sp = _sp;
   sp->begin(4800);
}

void DataLink::setValue(int i, int value) {
  values[i] = value;  
}

void DataLink::send() {
  sp->print(encode(values));
  //Serial.print(encode(values)); // for debugging
}

String DataLink::encode(int data[]) {
  String output = "[";
  for (int i=0; i<DATALINK_NUM_VALUES; i++) {
    output += data[i];
    if (i < (DATALINK_NUM_VALUES - 1)) output += ",";
  }
  output += "]\n";
  return output;
}


