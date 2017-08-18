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
#define CALIPILE_SLP1               20
#define CALIPILE_SLP2               20
#define CALIPILE_SLP3               21
#define CALIPILE_TP_PRES_THLD       22
#define CALIPILE_TP_MOT_THLD        23
#define CALIPILE_TP_AMB_SHOCK_THLD  24
#define CALIPILE_INT_MASK           25
#define CALIPILE_MOTION             26
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

#define intPin 9
#define myLed 13

uint8_t lookUp, TOBJ1, rawData[3] = {0, 0, 0}, intStatus;
uint16_t PTAT25, M, U0, UOUT, UOUT1, CHECKSUM, TPAMB;
uint32_t TPOBJ;
float Tamb, Tobj;

bool newInt = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(4000);

  pinMode(intPin, INPUT);
 
  pinMode(myLed, INPUT);
  digitalWrite(myLed, HIGH);
  
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
  Serial.print("CaliPile M is "); Serial.println(M);

  readBytes(CALIPILE_ADDRESS, CALIPILE_EEPROM_U0, 2, &rawData[0]);
  U0 = ( (uint16_t) rawData[0] << 8) | rawData[1];
  Serial.print("CaliPile U0 is "); Serial.println(U0);

  readBytes(CALIPILE_ADDRESS, CALIPILE_EEPROM_UOUT1, 2, &rawData[0]);
  UOUT1 = ( (uint16_t) rawData[0] << 8) | rawData[1];
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
  uint8_t lookUp = readByte(CALIPILE_ADDRESS, CALIPILE_EEPROM_PROTOCOL);

 
  // Initialize the sensor
  // only motion (bit 2) and presence (bit 3) interrupts allowed
  writeByte(CALIPILE_ADDRESS, CALIPILE_INT_MASK, 0x0C); 


 attachInterrupt(intPin, myinthandler, FALLING);  // define interrupt for INT pin output of CaliPile
/* end of setup */
}


void loop() {
  if(newInt)
  {
    newInt = false;
    intStatus = readByte(CALIPILE_ADDRESS, CALIPILE_INTERRUPT_STATUS);


    if(intStatus & 0x08) 
    {
      Serial.println("Presence detected!");
    }

    if(intStatus & 0x04) 
    {
      Serial.println("Motion detected!");
    }
    
  }

  // read the ambient temperature
  readBytes(CALIPILE_ADDRESS, CALIPILE_TPAMBIENT, 2, &rawData[0]);
  TPAMB = ( (uint16_t)(rawData[0] & 0x7F) << 8) | rawData[1] ; 

  Tamb = (25.0f + 273.15f) + ((float)TPAMB - (float)PTAT25) * (1.0f/(float) M);
  Serial.print("Tambient = "); Serial.print(Tamb, 2); Serial.println(" K");

  // read the object temperature
  readBytes(CALIPILE_ADDRESS, CALIPILE_TPOBJECT, 3, &rawData[0]);
  TPOBJ = ( (uint32_t) ( (uint32_t)rawData[0] << 24) | ( (uint32_t)rawData[1] << 16) | (rawData[2] & 0x80) << 8) >> 15; 
  
  float k = ( (float)UOUT1 - (float)U0)/(powf((float)TOBJ1, 3.8f) - powf(298.15f, 3.8f) );
  Serial.print("k = "); Serial.println(k, 10);
  float temp0 = powf(Tamb, 3.8f);
  float temp1 = ( (float)(TPOBJ - U0)/k );
  float temp2 = ( temp1 + temp0 );
  Tobj = powf( (float) temp2, 0.2631578947f );
  Serial.print(" Tobject = "); Serial.print(Tobj, 2); Serial.println(" K");
  
  digitalWrite(myLed, 0); delay(1000); digitalWrite(myLed, 1);
  
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
