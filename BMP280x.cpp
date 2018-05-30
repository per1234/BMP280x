/*
	BMP280.cpp
	
	Bosch BMP280 pressure sensor library for the Arduino microcontroller.
	This library uses I2C connection.
	
	Uses floating-point equations from BMP280 datasheet.
	
	modified by mhafuzul islam
	
	version 1.01		 16/9/2014 initial version
	
	Our example code uses the "pizza-eating" license. You can do anything
	you like with this code. No really, anything. If you find it useful,
	buy me italian pizza someday.
*/

#include "BMP280x.h"
#include <Wire.h>
#include <stdio.h>
#include <math.h>

BMP280x::BMP280x(int bmp280_addr)
{
	_bmp280_addr = bmp280_addr;
}


void BMP280x::init()
{
	//_bmp280_addr = bmp280_addr;
	uint8_t osrs_t = 1;             //Temperature oversampling x 1
	uint8_t osrs_p = 1;             //Pressure oversampling x 1
	//  uint8_t osrs_h = 1;             //Humidity oversampling x 1
	uint8_t mode = 3;               //Normal mode
	uint8_t t_sb = 5;               //Tstandby 1000ms
	uint8_t filter = 0;             //Filter off
	uint8_t spi3w_en = 0;           //3-wire SPI Disable
	
	//  uint8_t ctrl_hum_reg  = osrs_h;
	uint8_t ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode;
	uint8_t config_reg    = (t_sb << 5) | (filter << 2) | spi3w_en;
	
	//Serial.begin(115200);
	Wire.begin();
	
	//  writeReg(0xF2, ctrl_hum_reg);
	writeReg(0xF4, ctrl_meas_reg);
	writeReg(0xF5, config_reg);
	readTrim();                    //

	return;
}

void BMP280x::check()
{
	//double temp_act = 0.0, press_act = 0.0;
	signed long int temp_cal;
	//unsigned long int press_cal;
	//, press_Pa, press_mmHg;
	
	readData();
	
	temp_cal = calibration_T(temp_raw);
	Temp_C = (double)temp_cal / 100.0;
	
	//press_cal = calibration_P(pres_raw);
	Press_Pa = calibration_P(pres_raw);
	//Press_Pa = press_cal;
	Press_mmHg = Press_Pa * 0.0075;
	//  hum_act = (double)hum_cal / 1024.0;
#ifdef BMP280x_DEBUG
	Serial.print("TEMP : ");
	Serial.print(Temp_C);
	Serial.print(" DegC  PRESS : ");
	Serial.print(Press_Pa);
	Serial.print(" Pa | ");
	Serial.print(Press_mmHg);
	Serial.println(" mmHg");
#endif	
	//  delay(1000);
}

void BMP280x::readTrim()
{
	uint8_t data[24], i = 0;                   // Fix 2014/04/06
	Wire.beginTransmission(_bmp280_addr);
	Wire.write(0x88);
	Wire.endTransmission();
	Wire.requestFrom(_bmp280_addr, 24);      // Fix 2014/04/06
	while (Wire.available()) 
	{
		data[i] = Wire.read();
		i++;
	}
	
	Wire.beginTransmission(_bmp280_addr);    // Add 2014/04/06
	Wire.write(0xA1);                          // Add 2014/04/06
	Wire.endTransmission();                    // Add 2014/04/06
	Wire.requestFrom(_bmp280_addr, 1);       // Add 2014/04/06
	data[i] = Wire.read();                     // Add 2014/04/06
	i++;                                       // Add 2014/04/06
	
	Wire.beginTransmission(_bmp280_addr);
	Wire.write(0xE1);
	Wire.endTransmission();
	Wire.requestFrom(_bmp280_addr, 7);       // Fix 2014/04/06
	while (Wire.available()) 
	{
		data[i] = Wire.read();
		i++;
	}
	dig_T1 = (data[1] << 8) | data[0];
	dig_T2 = (data[3] << 8) | data[2];
	dig_T3 = (data[5] << 8) | data[4];
	dig_P1 = (data[7] << 8) | data[6];
	dig_P2 = (data[9] << 8) | data[8];
	dig_P3 = (data[11] << 8) | data[10];
	dig_P4 = (data[13] << 8) | data[12];
	dig_P5 = (data[15] << 8) | data[14];
	dig_P6 = (data[17] << 8) | data[16];
	dig_P7 = (data[19] << 8) | data[18];
	dig_P8 = (data[21] << 8) | data[20];
	dig_P9 = (data[23] << 8) | data[22];
}

void BMP280x::writeReg(uint8_t reg_address, uint8_t data)
{
	Wire.beginTransmission(_bmp280_addr);
	Wire.write(reg_address);
	Wire.write(data);
	Wire.endTransmission();
}

void BMP280x::readData()
{
	int i = 0;
	uint32_t data[8];
	Wire.beginTransmission(_bmp280_addr);
	Wire.write(0xF7);
	Wire.endTransmission();
	Wire.requestFrom(_bmp280_addr, 8);
	while (Wire.available()) 
	{
		data[i] = Wire.read();
		i++;
	}
	pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
	temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
}

signed long int BMP280x::calibration_T(signed long int adc_T)
{
	
	signed long int var1, var2, T;
	var1 = ((((adc_T >> 3) - ((signed long int)dig_T1 << 1))) * ((signed long int)dig_T2)) >> 11;
	var2 = (((((adc_T >> 4) - ((signed long int)dig_T1)) * ((adc_T >> 4) - ((signed long int)dig_T1))) >> 12) * ((signed long int)dig_T3)) >> 14;
	
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}

unsigned long int BMP280x::calibration_P(signed long int adc_P)
{
	signed long int var1, var2;
	unsigned long int P;
	var1 = (((signed long int)t_fine) >> 1) - (signed long int)64000;
	var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((signed long int)dig_P6);
	var2 = var2 + ((var1 * ((signed long int)dig_P5)) << 1);
	var2 = (var2 >> 2) + (((signed long int)dig_P4) << 16);
	var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((signed long int)dig_P2) * var1) >> 1)) >> 18;
	var1 = ((((32768 + var1)) * ((signed long int)dig_P1)) >> 15);
	if (var1 == 0)
	{
		return 0;
	}
	P = (((unsigned long int)(((signed long int)1048576) - adc_P) - (var2 >> 12))) * 3125;
	if (P < 0x80000000)
	{
		P = (P << 1) / ((unsigned long int) var1);
	}
	else
	{
		P = (P / (unsigned long int)var1) * 2;
	}
	var1 = (((signed long int)dig_P9) * ((signed long int)(((P >> 3) * (P >> 3)) >> 13))) >> 12;
	var2 = (((signed long int)(P >> 2)) * ((signed long int)dig_P8)) >> 13;
	P = (unsigned long int)((signed long int)P + ((var1 + var2 + dig_P7) >> 4));
	return P;
}


