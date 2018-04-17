#include "BMP280x.h"
BMP280x bmp280;

#define BME280_ADDRESS 0x76   // The Address need to find by i2c_scanner.ino

void setup()
{
  bmp280.init(BME280_ADDRESS);
  Serial.begin(115200);
}

void loop()
{
  bmp280.check();
  delay(750);
  Serial.print("TEMP : ");
  Serial.print(bmp280.Temp_C);
  Serial.print(" DegC  PRESS : ");
  Serial.print(bmp280.Press_Pa);
  Serial.print(" Pa  OR  ");
  Serial.print(bmp280.Press_mmHg);
  Serial.println(" mmHg");
}

