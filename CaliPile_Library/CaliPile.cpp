/* 06/16/2017 Copyright Tlera Corporation
 *  
 *  Created by Kris Winer
 *  
 This sketch uses SDA/SCL on pins 20/21 (back pads), respectively, and it uses the Butterfly STM32L4e33CU Breakout Board.
 The CaliPile is a simple thermipole but withon-board processing that makes it very versatile as well as low power; 
 ower consumption in only 20 microAmp
 
 Library may be used freely and without limit with attribution.
 
*/

#include "CaliPile.h"

  CALIPILE::CALIPILE(uint8_t intPin){
  pinMode(intPin, INPUT); // active LOW
  _intPin = intPin;
  }


  void CALIPILE::wake() {
  writeByte(0x00, 0x04, 0x00);  // issue general call and reload command
  delay(1);
  }


  void CALIPILE::readEEPROM()
  {
 
  uint8_t rawData[2] = {0, 0};
  /* Start of EEPROM operations, just have to do once *************************************************** */
 // Check EEPROM protocol number as a test of I2C communication 
  writeByte(CALIPILE_ADDRESS, CALIPILE_EEPROM_CONTROL, 0x80); // enable EEPROM read
   
  uint8_t c = readByte(CALIPILE_ADDRESS, CALIPILE_EEPROM_PROTOCOL);
  Serial.print("CaliPile EEPROM protocol number is "); Serial.println(c); 
  Serial.println("CaliPile EEPROM protocol number should be 3"); 

  uint8_t d = readByte(CALIPILE_ADDRESS, CALIPILE_SLAVE_ADDRESS);
  Serial.print("CaliPile EEPROM slave address is "); Serial.println(d); 
  Serial.println("CaliPile EEPROM slave address should be 140"); 
  Serial.println(" ");

  // Read the EEPROM calibration constants

  _LOOKUP = readByte(CALIPILE_ADDRESS, CALIPILE_EEPROM_LOOKUPNUM);
  Serial.print("CaliPile LookUpNumber is "); Serial.println(_LOOKUP);

  readBytes(CALIPILE_ADDRESS, CALIPILE_EEPROM_PTAT25, 2, &rawData[0]);
  _PTAT25 = ( (uint16_t) rawData[0] << 8) | rawData[1];
  Serial.print("CaliPile PTAT25 is "); Serial.println(_PTAT25);

  readBytes(CALIPILE_ADDRESS, CALIPILE_EEPROM_M, 2, &rawData[0]);
  _M = ( (uint16_t) rawData[0] << 8) | rawData[1];
  _M /= 100;
  Serial.print("CaliPile M is "); Serial.println(_M);

  readBytes(CALIPILE_ADDRESS, CALIPILE_EEPROM_U0, 2, &rawData[0]);
  _U0 = ( (uint16_t) rawData[0] << 8) | rawData[1];
  _U0 += 32768;
  Serial.print("CaliPile U0 is "); Serial.println(_U0);

  readBytes(CALIPILE_ADDRESS, CALIPILE_EEPROM_UOUT1, 2, &rawData[0]);
  _UOUT1 = ( (uint16_t) rawData[0] << 8) | rawData[1];
  _UOUT1 *= 2;
  Serial.print("CaliPile UOUT1 is "); Serial.println(_UOUT1);

  _TOBJ1 = readByte(CALIPILE_ADDRESS, CALIPILE_EEPROM_TOBJ1);
  Serial.print("CaliPile TOBJ1 is "); Serial.println(_TOBJ1);

  readBytes(CALIPILE_ADDRESS, CALIPILE_EEPROM_CHECKSUM, 2, &rawData[0]);
  _CHECKSUM = ( (uint16_t) rawData[0] << 8) | rawData[1];
  Serial.print("CaliPile CHECKSUM is supposed to be "); Serial.println(_CHECKSUM);

  // Calculate the checksum
  uint16_t sum = 0;
  for(int ii = 35; ii < 64; ii++)
  {
   sum += readByte(CALIPILE_ADDRESS, ii);
  }
  Serial.print("CaliPile CHECKSUM is "); Serial.println(sum + c);

  writeByte(CALIPILE_ADDRESS, CALIPILE_EEPROM_CONTROL, 0x00); // disable EEPROM read
  /* End of EEPROM operations, just have to do once *************************************************** */

  // Construct needed calibration constants (just need to calculate once)
  _k = ( (float) (_UOUT1 - _U0) )/(powf((float)(_TOBJ1 + 273.15f), 3.8f) - powf(25.0f + 273.15f, 3.8f) );

  }

    uint8_t  CALIPILE::checkIntStatus(){
    uint8_t  temp = readByte(CALIPILE_ADDRESS, CALIPILE_INTERRUPT_STATUS);
    return temp;
  }

  void CALIPILE::initMotion(uint8_t tcLP1, uint8_t tcLP2, uint8_t TPsource, uint8_t cycTime){
  // Initialize the sensor for motion and presence detection
  // Tthr (bit 4), presence (bit(3), motion (bit 2), amb shock (bit 1), timer (bit 0) interrupts allowed
  writeByte(CALIPILE_ADDRESS, CALIPILE_INT_MASK, 0x1C); 
  // time constant for LP1 (bits 0 - 3) and LP2 (bits 4 - 7)
  writeByte(CALIPILE_ADDRESS, CALIPILE_SLP12, tcLP2 << 4 | tcLP1);
  // select cycle time (bits 0 - 1) for motion detection, source (bits) 2 - 3) for presence detection
  uint8_t temp = readByte(CALIPILE_ADDRESS, CALIPILE_SRC_SELECT);
  writeByte(CALIPILE_ADDRESS, CALIPILE_SRC_SELECT, temp | TPsource << 2 | cycTime);  
  // select motion threshold
  writeByte(CALIPILE_ADDRESS, CALIPILE_TP_PRES_THLD, 0x22); // presence threshold, set at 50 counts
  writeByte(CALIPILE_ADDRESS, CALIPILE_TP_MOT_THLD, 0x0A); // motion threshold, set at 10 counts
  }

  void CALIPILE::initTempThr(uint16_t Tcounts){
  uint8_t rawData[2] = {0, 0};
  // specify the over temperature interrupt threshold (2 bytes)
  writeByte(CALIPILE_ADDRESS, CALIPILE_TPOT_THR, Tcounts); // 0x83 means 67,072 counts as threshold
  writeByte(CALIPILE_ADDRESS, (CALIPILE_TPOT_THR + 1), 0x00);
  uint8_t temp = readByte(CALIPILE_ADDRESS, CALIPILE_SRC_SELECT);
  writeByte(CALIPILE_ADDRESS, CALIPILE_SRC_SELECT, temp | 0x10); // interrupt on exceeding threshold
  // Verify threshold set
  readBytes(CALIPILE_ADDRESS, CALIPILE_TPOT_THR, 2, &rawData[0]);
  uint16_t TPOTTHR = ((uint16_t) rawData[0] << 8) | rawData[1];
  Serial.print("Overtemp threshold = "); Serial.println(TPOTTHR * 2);
  }

  uint16_t CALIPILE::getTPAMB(){
  uint8_t rawData[2] = {0, 0};
  readBytes(CALIPILE_ADDRESS, CALIPILE_TPAMBIENT, 2, &rawData[0]);
  uint16_t temp = ( (uint16_t)(rawData[0] & 0x7F) << 8) | rawData[1] ; 
  return temp;
  }

  uint32_t CALIPILE::getTPOBJ(){
  uint8_t rawData[3] = {0, 0, 0};
  readBytes(CALIPILE_ADDRESS, CALIPILE_TPOBJECT, 3, &rawData[0]);
  uint32_t temp = ( (uint32_t) ( (uint32_t)rawData[0] << 24) | ( (uint32_t)rawData[1] << 16) | ( (uint32_t)rawData[2] & 0x80) << 8) >> 15; 
  return temp;
  }

  
  uint32_t CALIPILE::getTPOBJLP1(){
  uint8_t rawData[3] = {0, 0, 0};
  readBytes(CALIPILE_ADDRESS, CALIPILE_TPOBJLP1, 3, &rawData[0]);
  uint32_t temp = ( ((uint32_t) rawData[0] << 16) | ((uint32_t) rawData[1] << 8) | ( (uint32_t)rawData[2] & 0xF0) ) >> 4;
  temp /= 8;
  return temp;
  }


  uint32_t CALIPILE::getTPOBJLP2(){
  uint8_t rawData[3] = {0, 0, 0};
  readBytes(CALIPILE_ADDRESS, CALIPILE_TPOBJLP2, 3, &rawData[0]);
  uint32_t temp = ((uint32_t) (rawData[0] & 0x0F) << 16) | ((uint32_t) rawData[1] << 8) | rawData[2] ;
  temp /= 8;
  return temp;
  }

  uint16_t CALIPILE::getTPAMBLP3(){
  uint8_t rawData[2] = {0, 0};
  readBytes(CALIPILE_ADDRESS, CALIPILE_TPAMBLP3, 2, &rawData[0]);
  uint16_t temp = ((uint16_t) rawData[0] << 8) | rawData[1];
  temp /= 2;
  return temp;
  }

  uint32_t CALIPILE::getTPOBJLP2FRZN(){
     uint8_t rawData[3] = {0, 0, 0};
     readBytes(CALIPILE_ADDRESS, CALIPILE_TPOBJLP2_FRZN, 3, &rawData[0]);
     uint32_t temp = ((uint32_t) rawData[0] << 16) | ((uint32_t) rawData[1] << 8) | rawData[2];
     temp /= 128;
     return temp;
  }

  uint8_t CALIPILE::getTPPRESENCE(){
      uint8_t temp = readByte(CALIPILE_ADDRESS, CALIPILE_TPPRESENCE);
      return temp;
  }

  
  uint8_t CALIPILE::getTPMOTION(){
      uint8_t temp = readByte(CALIPILE_ADDRESS, CALIPILE_TPMOTION);
      return temp;
  }

    uint8_t CALIPILE::getTPAMBSHK(){
      uint8_t temp = readByte(CALIPILE_ADDRESS, CALIPILE_TPAMB_SHOCK);
      return temp;
  }

    float CALIPILE::getTamb(uint16_t TPAMB){
      float temp = 298.15f + ((float)TPAMB - (float) _PTAT25) * (1.0f/(float) _M);
      return temp;
  }

    float CALIPILE::getTobj(uint32_t TPOBJ, float Tamb){
      float temp0 = powf(Tamb, 3.8f);
      float temp1 = ( ((float) TPOBJ) - ((float) _U0)  ) / _k ;
      float temp3 = powf( (temp0 + temp1), 0.2631578947f );
      return temp3;
  }
 
  // I2C scan function
  void CALIPILE::I2Cscan()
   {
  // scan for i2c devices
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmission to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
      
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
    
  }


  // I2C read/write functions for the BMP280 sensors
        void CALIPILE::writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
        uint8_t temp[2];
        temp[0] = subAddress;
        temp[1] = data;
        Wire.transfer(address, &temp[0], 2, NULL, 0); 
        }

        uint8_t CALIPILE::readByte(uint8_t address, uint8_t subAddress) {
        uint8_t temp[1];
        Wire.transfer(address, &subAddress, 1, &temp[0], 1);
        return temp[0];
        }

        void CALIPILE::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest) {
        Wire.transfer(address, &subAddress, 1, dest, count); 
        }
