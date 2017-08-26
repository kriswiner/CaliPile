/* 06/16/2017 Copyright Tlera Corporation
 *  
 *  Created by Kris Winer
 *  
 This sketch uses SDA/SCL on pins 20/21 (back pads), respectively, and it uses the Butterfly STM32L4e33CU Breakout Board.
 The CaliPile is a simple thermipole but withon-board processing that makes it very versatile as well as low power; 
 ower consumption in only 20 microAmp
 
 Library may be used freely and without limit with attribution.
 
*/
  
#ifndef CALIPILE_h
#define CALIPILE_h

#include "Arduino.h"
#include <Wire.h>

//https://www.pacer-usa.com/Assets/User/2077-CaliPile_TPiS_1S_1385_5029_27.04.2017.pdf
// CaliPile Registers
#define CALIPILE_TPOBJECT            1
#define CALIPILE_TPAMBIENT           3
#define CALIPILE_TPOBJLP1            5
#define CALIPILE_TPOBJLP2            7
#define CALIPILE_TPAMBLP3           10
#define CALIPILE_TPOBJLP2_FRZN      12
#define CALIPILE_TPPRESENCE         15
#define CALIPILE_TPMOTION           16
#define CALIPILE_TPAMB_SHOCK        17
#define CALIPILE_INTERRUPT_STATUS   18
#define CALIPILE_CHIP_STATUS        19
#define CALIPILE_SLP12              20
#define CALIPILE_SLP3               21
#define CALIPILE_TP_PRES_THLD       22
#define CALIPILE_TP_MOT_THLD        23
#define CALIPILE_TP_AMB_SHOCK_THLD  24
#define CALIPILE_INT_MASK           25
#define CALIPILE_SRC_SELECT         26
#define CALIPILE_TMR_INT            27
#define CALIPILE_TPOT_THR           28

// EEPROM Registers
#define CALIPILE_EEPROM_CONTROL     31
#define CALIPILE_EEPROM_PROTOCOL    32
#define CALIPILE_EEPROM_CHECKSUM    33
#define CALIPILE_EEPROM_LOOKUPNUM   41
#define CALIPILE_EEPROM_PTAT25      42
#define CALIPILE_EEPROM_M           44
#define CALIPILE_EEPROM_U0          46
#define CALIPILE_EEPROM_UOUT1       48
#define CALIPILE_EEPROM_TOBJ1       50
#define CALIPILE_SLAVE_ADDRESS      63

// I2C address when AD0 = AD1 = 0 (default)
#define CALIPILE_ADDRESS 0x0C

// Low-Pass time constants
#define TC_512s    0x00
#define TC_256s    0x01
#define TC_128s    0x02
#define TC_64s     0x03
#define TC_32s     0x04
#define TC_16s     0x05
#define TC_8s      0x08
#define TC_4s      0x09
#define TC_2s      0x0A
#define TC_1s      0x0B
#define TC_0_50s   0x0C
#define TC_0_25s   0x0D

// Sources
#define src_TPOBJ_TPOBJLP2         0x00
#define src_TPOBJLP1_TPOBJLP2      0x01
#define src_TPOBJ_TPOBJLP2_FRZN    0x02
#define src_TPOBJLP1_TPOBJLP2_FRZN 0x03

// Cycle times
#define cycTime_30ms  0x00
#define cycTime_60ms  0x01
#define cycTime_120ms 0x02
#define cycTime_140ms 0x03


class CALIPILE
{
  public: 
  CALIPILE(uint8_t intPin);
  uint8_t getChipID();
  void wake();
  void readEEPROM();
  uint8_t checkIntStatus();
  void initMotion(uint8_t tcLP1, uint8_t tcLP2, uint8_t TPsource, uint8_t cycTime);
  void initTempThr(uint16_t Tcounts);
  uint16_t getTPAMB();
  uint32_t getTPOBJ();
  uint32_t getTPOBJLP1();
  uint32_t getTPOBJLP2();
  uint16_t getTPAMBLP3();
  uint32_t getTPOBJLP2FRZN();
  uint8_t getTPPRESENCE();
  uint8_t getTPMOTION();
  uint8_t getTPAMBSHK();
  float getTamb(uint16_t TPAMB);
  float getTobj(uint32_t TPOBJ, float Tamb);
  void I2Cscan();
  void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
  uint8_t readByte(uint8_t address, uint8_t subAddress);
  void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);
  private:
  // Register read variables
  uint16_t _PTAT25, _M, _U0, _CHECKSUM;  
  uint32_t _UOUT1;  
  uint8_t  _TOBJ1,  _LOOKUP;
  float _k;
  uint8_t _intPin;
  };

#endif
