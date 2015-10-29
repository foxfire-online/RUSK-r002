#include "FOXFIRE_Si1132.h"

FOXFIRE_Si1132::FOXFIRE_Si1132()
{
	_addr = Si1132_ADDR;
}

boolean FOXFIRE_Si1132::begin(void)
{
	//Initialize the I2C bus if not already enabled
	if (!Wire.isEnabled()) {
		Wire.begin();
	}
	reset();

	// enable UVindex measurement coefficients!
	//write8(Si1132_REG_UCOEF0, 0x29);
	//write8(Si1132_REG_UCOEF1, 0x89);
	//write8(Si1132_REG_UCOEF2, 0x02);
	//write8(Si1132_REG_UCOEF3, 0x00);
	write8(Si1132_REG_UCOEF0, 0x7B);
	write8(Si1132_REG_UCOEF1, 0x6B);
	write8(Si1132_REG_UCOEF2, 0x01);
	write8(Si1132_REG_UCOEF3, 0x00);

	// enable UV, IR and Visible light sensors
	writeParam(Si1132_PARAM_CHLIST, Si1132_PARAM_CHLIST_ENUV | Si1132_PARAM_CHLIST_ENALSIR | Si1132_PARAM_CHLIST_ENALSVIS);

	write8(Si1132_REG_INTCFG, Si1132_REG_INTCFG_INTOE);
	write8(Si1132_REG_IRQEN, Si1132_REG_IRQEN_ALSEVERYSAMPLE);

  	writeParam(Si1132_PARAM_ALSIRADCMUX, Si1132_PARAM_ADCMUX_SMALLIR);  
	// fastest clocks, clock div 1
  	writeParam(Si1132_PARAM_ALSIRADCGAIN, 0x00);
	// take 511 clocks to measure
  	writeParam(Si1132_PARAM_ALSIRADCCOUNTER, Si1132_PARAM_ADCCOUNTER_511CLK);
	// in high range mode
  	writeParam(Si1132_PARAM_ALSIRADCMISC, Si1132_PARAM_ALSIRADCMISC_RANGE);

	// fastest clocks
  	writeParam(Si1132_PARAM_ALSVISADCGAIN, 0x00);
	// take 511 clocks to measure
  	writeParam(Si1132_PARAM_ALSVISADCCOUNTER, Si1132_PARAM_ADCCOUNTER_511CLK);
	// in high range mode (not normal signal)
  	writeParam(Si1132_PARAM_ALSVISADCMISC, Si1132_PARAM_ALSVISADCMISC_VISRANGE);

	// measure rate for auto, 32 * 31.25us = 1000us = 1ms
	write8(Si1132_REG_MEASRATE0, 0xFF);
	//write8(Si1132_REG_MEASRATE1, 0xFF);
	write8(Si1132_REG_COMMAND, Si1132_ALS_AUTO);

	return true;
}

uint16_t FOXFIRE_Si1132::readUV()
{
	return read16(Si1132_REG_UVINDEX0);
}

uint8_t FOXFIRE_Si1132::readUV0()
{
	return read8(Si1132_REG_UVINDEX0);
}

uint8_t FOXFIRE_Si1132::readUV1()
{
	return read8(Si1132_REG_UVINDEX1);
}

float FOXFIRE_Si1132::readIR()
{
	float lx = 0;

	for (int i = 0; i < 5; i++) {
		lx += read16(0x24);
		delay(20);
	}
	lx = lx/5;
	// adc offset
	if (lx > 256)
		lx -= 256;
	else
		lx = 0;

	return lx;
}

float FOXFIRE_Si1132::readVisible()
{
	float lx = 0;

	for (int i = 0; i < 5; i++) {
		lx += read16(0x22);
		delay(20);
	}
	lx = lx/5;
	// adc offset
	if (lx > 256)
		lx -= 256;
	else
		lx = 0;
	lx = lx*0.7;

	return lx;
}

void FOXFIRE_Si1132::reset()
{
	write8(Si1132_REG_COMMAND, Si1132_ALS_PAUSE);
	delay(10);

	write8(Si1132_REG_MEASRATE0, 0x00);
	write8(Si1132_REG_MEASRATE1, 0x00);
	write8(Si1132_REG_IRQEN, 0x00);
	write8(Si1132_REG_IRQMODE1, 0x00);
	write8(Si1132_REG_IRQMODE2, 0x00);
	write8(Si1132_REG_INTCFG, 0x00);
	write8(Si1132_REG_IRQSTAT, 0xFF);

	write8(Si1132_REG_COMMAND, Si1132_RESET);
	delay(10);

	write8(Si1132_REG_HWKEY, 0x17);
	delay(10);
}

uint8_t FOXFIRE_Si1132::read8(uint8_t reg)
{
	uint16_t val;
	Wire.beginTransmission(_addr);
	Wire.write((uint8_t)reg);
	Wire.endTransmission();

	Wire.requestFrom((uint8_t)_addr, (uint8_t)1);
	return Wire.read();
}

uint16_t FOXFIRE_Si1132::read16(uint8_t reg)
{
	uint16_t ret;

	Wire.beginTransmission(_addr);
	Wire.write(reg);
	Wire.endTransmission();

	Wire.requestFrom(_addr, (uint8_t)2);
	ret = Wire.read();
	ret |= (uint16_t)Wire.read() << 8;

	return ret;
}

void FOXFIRE_Si1132::write8(uint8_t reg, uint8_t val)
{
	Wire.beginTransmission(_addr);
	Wire.write(reg);
	Wire.write(val);
	Wire.endTransmission();
}

uint8_t FOXFIRE_Si1132::readParam(uint8_t p)
{
	write8(Si1132_REG_COMMAND, Si1132_NOP); // clear the register
	write8(Si1132_REG_COMMAND, p | Si1132_PARAM_QUERY);
	return read8(Si1132_REG_PARAMRD);
}

uint8_t FOXFIRE_Si1132::writeParam(uint8_t p, uint8_t v)
{
	write8(Si1132_REG_COMMAND, Si1132_NOP); // clear the register
	write8(Si1132_REG_PARAMWR, v);
	write8(Si1132_REG_COMMAND, p | Si1132_PARAM_SET);
	return read8(Si1132_REG_PARAMRD);
}
