#ifndef _MELEXIS_h
#define _MELEXIS_h


class MelexisTempProbe {

public:
  bool sensorExists();
  void start();
  float getAmbientTemp();
  float getTargetTemp();
  
private:
  bool sensorExistsCached;
  bool sensorExistsChecked = false;
};

#endif

