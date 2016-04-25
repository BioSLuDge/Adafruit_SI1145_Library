/*************************************************** 
  This is a library for the Si1145 UV/IR/Visible Light Sensor

  Designed specifically to work with the Si1145 sensor in the
  adafruit shop
  ----> https://www.adafruit.com/products/1777

  These sensors use I2C to communicate, 2 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include "Adafruit_SI1145.h"

Adafruit_SI1145::Adafruit_SI1145() {
  _addr = SI1145_ADDR;
}


boolean Adafruit_SI1145::begin(void) {
  Wire.begin();
  reset();
 
  //  write the address (_addr) to the PARAM_WR I2C register (0x17)
  
  updateI2Caddr(0x17, _addr);

//  write a PARAM_SET command into the COMMAND register (0x18)
   updateI2Caddr(0x18, 0xA0);

/*  tell the chip you want to read the command response register.
    the first half is a "tell the chip what address I want by starting
    a write but not sending any data" transaction.. regrettably common 
    in I2C
*/
	quickWrite(0x2E);
	
	uint8_t paramVerify = quickRead(0x01);

	Serial.print( "return value from PARAM_SET command: 0x" );
	Serial.print( paramVerify, HEX );

/*  now, having done all that, execute a BUSADDR command to use the address
    we've stored in the PRAM
*/
	updateI2Caddr(0x18, 0x02);  //  BUSADDR command

//  this time we find the result in the RESPONSE register (0x20)

	quickWrite(0x20);
	paramVerify = quickRead(0x01);
	Serial.print( "return value from BUSADDR command: 0x" );
	Serial.print( paramVerify, HEX );
	
	uint8_t id = read8(SI1145_REG_PARTID);
  	if (id != 0x45) return false; // look for SI1145
  

  

    /***********************************/
  // enable UVindex measurement coefficients!
  write8(SI1145_REG_UCOEFF0, 0x29);
  write8(SI1145_REG_UCOEFF1, 0x89);
  write8(SI1145_REG_UCOEFF2, 0x02);
  write8(SI1145_REG_UCOEFF3, 0x00);

  // enable UV sensor
  writeParam(SI1145_PARAM_CHLIST, SI1145_PARAM_CHLIST_ENUV |
  SI1145_PARAM_CHLIST_ENALSIR | SI1145_PARAM_CHLIST_ENALSVIS |
  SI1145_PARAM_CHLIST_ENPS1);
  // enable interrupt on every sample
  write8(SI1145_REG_INTCFG, SI1145_REG_INTCFG_INTOE);  
  write8(SI1145_REG_IRQEN, SI1145_REG_IRQEN_ALSEVERYSAMPLE);  

/****************************** Prox Sense 1 */

  // program LED current
  write8(SI1145_REG_PSLED21, 0x03); // 20mA for LED 1 only
  writeParam(SI1145_PARAM_PS1ADCMUX, SI1145_PARAM_ADCMUX_LARGEIR);
  // prox sensor #1 uses LED #1
  writeParam(SI1145_PARAM_PSLED12SEL, SI1145_PARAM_PSLED12SEL_PS1LED1);
  // fastest clocks, clock div 1
  writeParam(SI1145_PARAM_PSADCGAIN, 0);
  // take 511 clocks to measure
  writeParam(SI1145_PARAM_PSADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK);
  // in prox mode, high range
  writeParam(SI1145_PARAM_PSADCMISC, SI1145_PARAM_PSADCMISC_RANGE|
    SI1145_PARAM_PSADCMISC_PSMODE);

  writeParam(SI1145_PARAM_ALSIRADCMUX, SI1145_PARAM_ADCMUX_SMALLIR);  
  // fastest clocks, clock div 1
  writeParam(SI1145_PARAM_ALSIRADCGAIN, 0);
  // take 511 clocks to measure
  writeParam(SI1145_PARAM_ALSIRADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK);
  // in high range mode
  writeParam(SI1145_PARAM_ALSIRADCMISC, SI1145_PARAM_ALSIRADCMISC_RANGE);



  // fastest clocks, clock div 1
  writeParam(SI1145_PARAM_ALSVISADCGAIN, 0);
  // take 511 clocks to measure
  writeParam(SI1145_PARAM_ALSVISADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK);
  // in high range mode (not normal signal)
  writeParam(SI1145_PARAM_ALSVISADCMISC, SI1145_PARAM_ALSVISADCMISC_VISRANGE);


/************************/

  // measurement rate for auto
  write8(SI1145_REG_MEASRATE0, 0xFF); // 255 * 31.25uS = 8ms
  
  // auto run
  write8(SI1145_REG_COMMAND, SI1145_PSALS_AUTO);

  return true;
}

void Adafruit_SI1145::reset() {
  updateI2Caddr(SI1145_REG_MEASRATE0, 0);
  updateI2Caddr(SI1145_REG_MEASRATE1, 0);
  updateI2Caddr(SI1145_REG_IRQEN, 0);
  updateI2Caddr(SI1145_REG_IRQMODE1, 0);
  updateI2Caddr(SI1145_REG_IRQMODE2, 0);
  updateI2Caddr(SI1145_REG_INTCFG, 0);
  updateI2Caddr(SI1145_REG_IRQSTAT, 0xFF);

  updateI2Caddr(SI1145_REG_COMMAND, SI1145_RESET);
  delay(10);
  updateI2Caddr(SI1145_REG_HWKEY, 0x17);
  
  delay(10);
}


//////////////////////////////////////////////////////

// returns the UV index * 100 (divide by 100 to get the index)
uint16_t Adafruit_SI1145::readUV(void) {
 return read16(0x2C); 
}

// returns visible+IR light levels
uint16_t Adafruit_SI1145::readVisible(void) {
 return read16(0x22); 
}

// returns IR light levels
uint16_t Adafruit_SI1145::readIR(void) {
 return read16(0x24); 
}

// returns "Proximity" - assumes an IR LED is attached to LED
uint16_t Adafruit_SI1145::readProx(void) {
 return read16(0x26); 
}

/*********************************************************************/

uint8_t Adafruit_SI1145::writeParam(uint8_t p, uint8_t v) {
  //Serial.print("Param 0x"); Serial.print(p, HEX);
  //Serial.print(" = 0x"); Serial.println(v, HEX);
  
  write8(SI1145_REG_PARAMWR, v);
  write8(SI1145_REG_COMMAND, p | SI1145_PARAM_SET);
  return read8(SI1145_REG_PARAMRD);
}

uint8_t Adafruit_SI1145::readParam(uint8_t p) {
  write8(SI1145_REG_COMMAND, p | SI1145_PARAM_QUERY);
  return read8(SI1145_REG_PARAMRD);
}

/*********************************************************************/

uint8_t  Adafruit_SI1145::read8(uint8_t reg) {
  uint16_t val;
    Wire.beginTransmission(_addr);
    Wire.write((uint8_t)reg);
    Wire.endTransmission();

    Wire.requestFrom((uint8_t)_addr, (uint8_t)1);  
    return Wire.read();
}

uint16_t Adafruit_SI1145::read16(uint8_t a) {
  uint16_t ret;

  Wire.beginTransmission(_addr); // start transmission to device 
  Wire.write(a); // sends register address to read from
  Wire.endTransmission(); // end transmission
  
  Wire.requestFrom(_addr, (uint8_t)2);// send data n-bytes read
  ret = Wire.read(); // receive DATA
  ret |= (uint16_t)Wire.read() << 8; // receive DATA

  return ret;
}

void Adafruit_SI1145::write8(uint8_t reg, uint8_t val) {

  Wire.beginTransmission(_addr); // start transmission to device 
  Wire.write(reg); // sends register address to write
  Wire.write(val); // sends value
  Wire.endTransmission(); // end transmission
}

/**************************************************************/
void Adafruit_SI1145::updateI2Caddr(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(0x60); // start transmission to device 
  Wire.write(reg); // sends register address to write
  Wire.write(val); // sends value
  Wire.endTransmission(); // end transmission
  }
  
void Adafruit_SI1145::quickWrite(uint8_t val) {
 	Wire.beginTransmission(0x60); // start transmission to device 
  	Wire.write(val); // sends value
  	Wire.endTransmission(); // end transmission
  }
  
uint8_t Adafruit_SI1145::quickRead(uint8_t reg) {
    Wire.beginTransmission(0x60);
    Wire.write((uint8_t)reg);
    Wire.endTransmission();
    
	Wire.requestFrom((uint8_t)0x60, (uint8_t)1);  // read one byte from the address just set
	return Wire.read();
}
