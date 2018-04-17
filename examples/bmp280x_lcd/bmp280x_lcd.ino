#define SKETCHVERSION __FILE__ " " __DATE__ " " __TIME__
#define BME280_ADDRESS 0x76     // адрес датчика BMP280, определяется скетчем i2c_scanner.ino
#define I2C_LCDADDRESS 0x3F

#include <LiquidCrystal_I2C.h>      // https://github.com/marcoschwartz/LiquidCrystal_I2C
LiquidCrystal_I2C lcd(I2C_LCDADDRESS, 16, 2);  // LCD 16x2 connected by I2C

#include "BMP280x.h"
BMP280x bmp280;
char degC = 223;

void setup()
{
  Serial.begin(115200);
  bmp280.init(BME280_ADDRESS);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  delay(300);
  Serial.println(SKETCHVERSION);
}

void loop()
{
  bmp280.check();
  Serial.print("TEMP : ");
  Serial.print(bmp280.Temp_C);
  Serial.print(" DegC  PRESS : ");
  Serial.print(bmp280.Press_Pa);
  Serial.print(" Pa | ");
  Serial.print(bmp280.Press_mmHg);
  Serial.println(" mmHg");

  lcd.setCursor(0, 0);
  lcd.print("Temp  ");
  lcd.print(bmp280.Temp_C);
  lcd.print(degC);

  lcd.setCursor(0, 1);
  lcd.print("Press ");
  lcd.print(bmp280.Press_mmHg);
  lcd.print(" mmHg");

  delay(1000);
}

