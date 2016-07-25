#include <Wire.h>
#include <Arduino.h>
#include <MLX90614.h>
#include "Melexis.h"

MLX90614 mlx = MLX90614(MLX90614_BROADCASTADDR); 

bool MelexisTempProbe::sensorExists()
{
  if (!sensorExistsChecked) {
    Wire.begin();
    Wire.setClock(100000);
    Wire.beginTransmission (90);
    sensorExistsCached = (Wire.endTransmission () == 0);
    sensorExistsChecked = true;
  }
  return (sensorExistsCached);
}

void MelexisTempProbe::start()
{
  mlx.begin();
}

float MelexisTempProbe::getAmbientTemp()
{
  float kelvin = mlx.readTemp(MLX90614::MLX90614_SRCA, MLX90614::MLX90614_TK);
  float celsius = mlx.convKtoC(kelvin);
  float fahrenheit = mlx.convCtoF(celsius);
  return fahrenheit;
}

float MelexisTempProbe::getTargetTemp()
{
  float kelvin = mlx.readTemp(MLX90614::MLX90614_SRCO1, MLX90614::MLX90614_TK);
  float celsius = mlx.convKtoC(kelvin);
  float fahrenheit = mlx.convCtoF(celsius);
  return fahrenheit;
}

