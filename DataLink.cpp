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
}

String DataLink::encode(int data[]) {
  String output = "[";
  for (int i=0; i<8; i++) {
    output += data[i];
    if (i < 7) output += ",";
  }
  output += "]\n";
  return output;
}


