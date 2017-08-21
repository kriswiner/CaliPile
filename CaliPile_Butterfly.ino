/* 08/16/2017 Copyright Tlera Corporation
 *  
 *  Created by Kris Winer
 *  
 This sketch is to operate Excelitas' CaliPile TPiS1S1385/5029 IR Thermopile
 https://media.digikey.com/pdf/Data%20Sheets/Excelitas%20PDFs/TPiS_1S_1385.pdf 
 
 The sketch uses default SDA/SCL pins on 20/21 of the Butterfly development board.
 The CaliPile breakout board has 4K7 pullus on SDA and SCL.

 Library may be used freely and without limit with attribution.
 
  */
  
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

// Define pins
#define intPin 9
#define myLed1 13  // red led
#define myLed2 26  // green led
#define myLed3 38  // blue led

bool serialDebug = true, presSign = false, motSign = false;

uint8_t lookUp, rawData[3] = {0, 0, 0}, intStatus, chipStatus, temp;

// Register read variables
uint16_t PTAT25, M, U0, CHECKSUM, TPAMB, TPAMBLP3;
uint32_t TPOBJ, UOUT1, TPOBJLP1, TPOBJLP2, TPOBJLP2FRZN;
uint8_t  TOBJ1, TPPRESENCE, TPMOTION, TPAMBSHK;

// output data for comparisons
float Tamb, Tamblp3, Tobj, Tobjlp1, Tobjlp2, Tobjlp2frzn, Tpres, Tmot, Tambshk, k;

bool newInt = false;

void setup() {
 
  Serial.begin(115200);
  delay(4000);

  pinMode(intPin, INPUT);
 
  pinMode(myLed1, OUTPUT);
  digitalWrite(myLed1, HIGH);  // Start with leds off, active LOW on Butterfly
  pinMode(myLed2, OUTPUT);
  digitalWrite(myLed2, HIGH);
  pinMode(myLed3, OUTPUT);
  digitalWrite(myLed3, HIGH);
  
  Wire.begin(TWI_PINS_20_21); // set master mode 
  Wire.setClock(400000); // I2C frequency at 400 kHz  
  delay(1000);
 
  I2Cscan();

  //Initiate I2C transcations with general call/reload command
  writeByte(0x00, 0x04, 0x00);
  delay(1);

  I2Cscan();

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

  lookUp = readByte(CALIPILE_ADDRESS, CALIPILE_EEPROM_LOOKUPNUM);
  Serial.print("CaliPile LookUpNumber is "); Serial.println(lookUp);

  readBytes(CALIPILE_ADDRESS, CALIPILE_EEPROM_PTAT25, 2, &rawData[0]);
  PTAT25 = ( (uint16_t) rawData[0] << 8) | rawData[1];
  Serial.print("CaliPile PTAT25 is "); Serial.println(PTAT25);

  readBytes(CALIPILE_ADDRESS, CALIPILE_EEPROM_M, 2, &rawData[0]);
  M = ( (uint16_t) rawData[0] << 8) | rawData[1];
  M /= 100;
  Serial.print("CaliPile M is "); Serial.println(M);

  readBytes(CALIPILE_ADDRESS, CALIPILE_EEPROM_U0, 2, &rawData[0]);
  U0 = ( (uint16_t) rawData[0] << 8) | rawData[1];
  U0 += 32768;
  Serial.print("CaliPile U0 is "); Serial.println(U0);

  readBytes(CALIPILE_ADDRESS, CALIPILE_EEPROM_UOUT1, 2, &rawData[0]);
  UOUT1 = ( (uint16_t) rawData[0] << 8) | rawData[1];
  UOUT1 *= 2;
  Serial.print("CaliPile UOUT1 is "); Serial.println(UOUT1);

  TOBJ1 = readByte(CALIPILE_ADDRESS, CALIPILE_EEPROM_TOBJ1);
  Serial.print("CaliPile TOBJ1 is "); Serial.println(TOBJ1);

  readBytes(CALIPILE_ADDRESS, CALIPILE_EEPROM_CHECKSUM, 2, &rawData[0]);
  CHECKSUM = ( (uint16_t) rawData[0] << 8) | rawData[1];
  Serial.print("CaliPile CHECKSUM is supposed to be "); Serial.println(CHECKSUM);

  // Calculate the checksum
  uint16_t sum = 0;
  for(int ii = 35; ii < 64; ii++)
  {
   sum += readByte(CALIPILE_ADDRESS, ii);
  }
  Serial.print("CaliPile CHECKSUM is "); Serial.println(sum + c);

  writeByte(CALIPILE_ADDRESS, CALIPILE_EEPROM_CONTROL, 0x00); // disable EEPROM read

  // Initialize the sensor for motion and presence detection
  // Tthr (bit 4), presence (bit(3), motion (bit 2), amb shock (bit 1), timer (bit 0) interrupts allowed
  writeByte(CALIPILE_ADDRESS, CALIPILE_INT_MASK, 0x1C); 
  // time constant for LP1 (bits 0 - 3) and LP2 (bits 4 - 7)
  writeByte(CALIPILE_ADDRESS, CALIPILE_SLP12, TC_8s << 4 | TC_1s);
  // select cycle time (bits 0 - 1) for motion detection, source (bits) 2 - 3) for presence detection
  temp = readByte(CALIPILE_ADDRESS, CALIPILE_SRC_SELECT);
  writeByte(CALIPILE_ADDRESS, CALIPILE_SRC_SELECT, temp | src_TPOBJLP1_TPOBJLP2 << 2 | cycTime_30ms);  
  // select motion threshold
  writeByte(CALIPILE_ADDRESS, CALIPILE_TP_PRES_THLD, 0x22); // set at 50 counts

  // specify the over temperature interrupt threshold (2 bytes)
  writeByte(CALIPILE_ADDRESS, CALIPILE_TPOT_THR, 0x83); // choose 67,072 counts as threshold
  writeByte(CALIPILE_ADDRESS, (CALIPILE_TPOT_THR + 1), 0x00);
  temp = readByte(CALIPILE_ADDRESS, CALIPILE_SRC_SELECT);
  writeByte(CALIPILE_ADDRESS, CALIPILE_SRC_SELECT, temp | 0x10); // interrupt on exceeding threshold
  // Verify threshole set
  readBytes(CALIPILE_ADDRESS, CALIPILE_TPOT_THR, 2, &rawData[0]);
  uint16_t TPOTTHR = ((uint16_t) rawData[0] << 8) | rawData[1];
  Serial.print("Overtemp threshold = "); Serial.println(TPOTTHR * 2);
  

  // Construct needed calibration constants (just need to calculate once)
  k = ( (float) (UOUT1 - U0) )/(powf((float)(TOBJ1 + 273.15f), 3.8f) - powf(25.0f + 273.15f, 3.8f) );

  attachInterrupt(intPin, myinthandler, FALLING);  // define interrupt for INT pin output of CaliPile

  // read interrupt status register(s) to unlatch interrupt before entering main loop
  intStatus  = readByte(CALIPILE_ADDRESS, CALIPILE_INTERRUPT_STATUS);
  Serial.print("Int status = "); Serial.println(intStatus, HEX);
  
  /* end of setup */
}


void loop() {

   if(newInt == true)  // handle interrupt on receipt
   {
    newInt = false;

    // read interrupt status register(s) to clear interrupt
    intStatus  = readByte(CALIPILE_ADDRESS, CALIPILE_INTERRUPT_STATUS);
 
    if(intStatus & 0x08)
    {
      Serial.println("Presence detected!");
      digitalWrite(myLed2, LOW); delay(50); digitalWrite(myLed2, HIGH);  // flash green led
      if(intStatus & 0x80) presSign = true;
      else presSign = false;
    }

    if(intStatus & 0x04) 
    {
      Serial.println("Motion detected!");
      digitalWrite(myLed3, LOW); delay(50); digitalWrite(myLed3, HIGH); // flash blue led
      if(intStatus & 0x40) motSign = true;
      else motSign = false;
      }

    if(intStatus & 0x10)
    {
      Serial.println("Temp threshold exceeded!");
    }

    /* end of interrupt handling */
    }

    
  // read the ambient temperature
  readBytes(CALIPILE_ADDRESS, CALIPILE_TPAMBIENT, 2, &rawData[0]);
  TPAMB = ( (uint16_t)(rawData[0] & 0x7F) << 8) | rawData[1] ; 

  Tamb = 298.15f + ((float)TPAMB - (float)PTAT25) * (1.0f/(float) M);

  // read the object temperature
  readBytes(CALIPILE_ADDRESS, CALIPILE_TPOBJECT, 3, &rawData[0]);
  TPOBJ = ( (uint32_t) ( (uint32_t)rawData[0] << 24) | ( (uint32_t)rawData[1] << 16) | (rawData[2] & 0x80) << 8) >> 15; 
  
  float temp0 = powf(Tamb, 3.8f);
  float temp1 = ( ((float) TPOBJ) - ((float) U0)  ) / k ;
  Tobj = powf( (temp0 + temp1), 0.2631578947f );

  // Read the time-integrated registers

  // 20-bit wide, divide by 8 to compare with TPOBJ
  readBytes(CALIPILE_ADDRESS, CALIPILE_TPOBJLP1, 3, &rawData[0]);
  TPOBJLP1 = ( ((uint32_t) rawData[0] << 16) | ((uint32_t) rawData[1] << 8) | (rawData[2] & 0xF0) ) >> 4;
  TPOBJLP1 /= 8;
  
  // 20-bit wide, divide by 8 to compare with TPOBJ
  readBytes(CALIPILE_ADDRESS, CALIPILE_TPOBJLP2, 3, &rawData[0]);
  TPOBJLP2 = ( ((uint32_t) rawData[0] << 16) | ((uint32_t) rawData[1] << 8) | (rawData[2] & 0xF0) ) >> 4;
  TPOBJLP2 /= 8;
  
  // 16-bit wide, divide by 2 to compare with TPAMB
  readBytes(CALIPILE_ADDRESS, CALIPILE_TPAMBLP3, 2, &rawData[0]);
  TPAMBLP3 = ((uint16_t) rawData[0] << 8) | rawData[1];
  TPAMBLP3 /= 2;
  
  // 24-bit wide, divide by 128 to compare with TPOBJ
  readBytes(CALIPILE_ADDRESS, CALIPILE_TPOBJLP2_FRZN, 3, &rawData[0]);
  TPOBJLP2FRZN = ((uint32_t) rawData[0] << 16) | ((uint32_t) rawData[1] << 8) | rawData[2];
  TPOBJLP2FRZN /= 128;
  
  TPPRESENCE = readByte(CALIPILE_ADDRESS, CALIPILE_TPPRESENCE);
  TPMOTION   = readByte(CALIPILE_ADDRESS, CALIPILE_TPMOTION);
  TPAMBSHK   = readByte(CALIPILE_ADDRESS, CALIPILE_TPAMB_SHOCK);

  if(serialDebug)
  {
    Serial.print("Tambient = "); Serial.print(Tamb, 2); Serial.println(" K");
    Serial.print("TPAMP = "); Serial.println(TPAMB);  
    Serial.print("TAMBLP3 = "); Serial.println(TPAMBLP3);  
    Serial.println(" ");
  
    Serial.print("Tobj = "); Serial.print(Tobj, 2); Serial.println(" K");
    Serial.print("TPOBJ = "); Serial.println(TPOBJ);  
    Serial.print("TPOBJLP1 = "); Serial.println(TPOBJLP1);  
    Serial.print("TPOBJLP2 = "); Serial.println(TPOBJLP2);  
    Serial.print("TPOBJLP2FRZN = "); Serial.println(TPOBJLP2FRZN);
    Serial.println(" ");

     if(presSign) 
    {
      Serial.print("TPPRESENCE = ");   Serial.println(-1 * TPPRESENCE);  
    }
    else
    {
       Serial.print("TPPRESENCE = ");   Serial.println(TPPRESENCE);
    }
    
    if(motSign) 
    {
      Serial.print("TPMOTION = ");   Serial.println(-1 * TPMOTION);  
    }
    else
    {
       Serial.print("TPMOTION = ");   Serial.println(TPMOTION);
    }
    
    Serial.print("TAMBSHK = ");    Serial.println(TPAMBSHK);  
    Serial.println(" ");
  }

//  Serial.print((Tamb - 273.15));  Serial.print("  "); Serial.print((Tobj - 273.15)); Serial.println("  ");

   Serial.print((TPAMB)); Serial.print("  "); Serial.print((TPAMBLP3)); Serial.println("  ");

//   Serial.print((TPOBJ)); Serial.print("  "); Serial.print((TPOBJLP1)); Serial.print("  ");
//   Serial.print((TPOBJLP2)); Serial.print("  "); Serial.print((TPOBJLP2FRZN)); Serial.println("  ");

  digitalWrite(myLed1, LOW); delay(10); digitalWrite(myLed1, HIGH);  delay(500);
  
  /* end of main loop */
}


/* Useful functions */
void myinthandler()
{
  newInt = true;
}


// I2C scan function
void I2Cscan()
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

  void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

  uint8_t readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data; // `data` will store the register data   
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, 1);            // Read one byte from slave register address 
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}

  void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
  Wire.beginTransmission(address);   // Initialize the Tx buffer
  Wire.write(subAddress);            // Put slave register address in Tx buffer
  Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  Wire.requestFrom(address, count);  // Read bytes from slave register address 
  while (Wire.available()) {dest[i++] = Wire.read(); } // Put read results in the Rx buffer
}

