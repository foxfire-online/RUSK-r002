#include "FOXFIRE_MPU9150.h"

////////////////////////////////////////////////////////////
///////// I2C functions to get all values easier ///////////
////////////////////////////////////////////////////////////

FOXFIRE_MPU9150::FOXFIRE_MPU9150()
{
	_addr_compass = 0x0C; //Digital Compass is at 0x0C
	_addr_motion = 0x68; //Default I2C address is 0x68, AD0 low = 0x68, AD0 high = 0x69
}

boolean FOXFIRE_MPU9150::begin(int addrI2C)
{
	//Initialize the I2C bus if not already enabled
	if (!Wire.isEnabled()) {
        	Wire.begin();
	}
	Wire.beginTransmission(addrI2C);
	return ( Wire.endTransmission(true) == 0 ? true : false );
}

int FOXFIRE_MPU9150::readSensor(int addrI2C, int addrL, int addrH){
	Wire.beginTransmission(addrI2C);
	Wire.write(addrL);
	Wire.endTransmission(false);

	Wire.requestFrom(addrI2C, 1, true);
	byte L = Wire.read();

	Wire.beginTransmission(addrI2C);
	Wire.write(addrH);
	Wire.endTransmission(false);

	Wire.requestFrom(addrI2C, 1, true);
	byte H = Wire.read();

	return (int16_t)((H<<8)+L);
}

int FOXFIRE_MPU9150::readSensor(int addrI2C, int addr){
	Wire.beginTransmission(addrI2C);
	Wire.write(addr);
	Wire.endTransmission(false);

	Wire.requestFrom(addrI2C, 1, true);

	return Wire.read();
}

int FOXFIRE_MPU9150::writeSensor(int addrI2C, int addr, int data){
	Wire.beginTransmission(addrI2C);
	Wire.write(addr);
	Wire.write(data);
	Wire.endTransmission(true);

	return 1;
}
