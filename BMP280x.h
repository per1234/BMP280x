/*
	BMP280.h
	Bosch BMP280 pressure sensor library for the Arduino microcontroller.
	This library uses I2C connection.
	
	Uses floating-point equations from BMP280 datasheet.
	
	modified by mhafuzul islam
	
	version 1.01		 16/9/2014 initial version
	
	Our example code uses the "pizza-eating" license. You can do anything
	you like with this code. No really, anything. If you find it useful,
	buy me italian pizza someday.
*/

#ifndef BMP280x_h
#define BMP280x_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>

//#define BMP280x_DEBUG

class BMP280x
{
public:
	BMP280x(int bmp280_addr); // base type
	
	unsigned long Press_Pa;
	uint16_t Press_mmHg;
	float Temp_C;
	
	void init();
	void check();

private:
	unsigned long int temp_raw, pres_raw;
	signed long int t_fine;
	uint16_t dig_T1, dig_P1;
	int16_t dig_T2, dig_T3;
	int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
	int _bmp280_addr;
	
	void readTrim();
	void writeReg(uint8_t reg_address, uint8_t data);
	void readData();
	signed long int calibration_T(signed long int adc_T);
	unsigned long int calibration_P(signed long int adc_P);	
};

#endif
