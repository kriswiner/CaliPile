/* 08/16/2017 Copyright Tlera Corporation
 *  
 *  Created by Kris Winer
 *  
 This sketch is to operate Excelitas' CaliPile TPiS1S1385/5029 IR Thermopile
 https://media.digikey.com/pdf/Data%20Sheets/Excelitas%20PDFs/TPiS_1S_1385.pdf 
 
 The sketch uses default SDA/SCL pins on 20/21 of the Butterfly development board.
 The CaliPile breakout board has 4K7 pullups on SDA and SCL.
 
 Library may be used freely and without limit with attribution.
  */
  
#include <Wire.h>
#include "CaliPile.h"

// Define pins
#define intPin 10
#define myLed1 13  // red led
#define myLed2 26  // green led
#define myLed3 38  // blue led

#define pin_VCC 31
#define pin_GND 30
#define pin_ADO0 9
#define pin_ADO1 8

bool serialDebug = true, presSign = false, motSign = false, oldpresSign = false;

uint8_t rawData[3] = {0, 0, 0}, intStatus, chipStatus, temp;

/*
Choices for time constants are TC_0_25s, TC_0_50s, TC_1s, TC_2s, TC_4s, ..., TC_512s
Sources = src_TPOBJ_TPOBJLP2, src_TPOBJLP1_TPOBJLP2, src_TPOBJ_TPOBJLP2_FRZN, src_TPOBJLP1_TPOBJLP2_FRZN
cycle times = cycTime_30ms, cycTime_60ms, cycTime_120ms, cycTime_140ms
*/
uint8_t tcLP1 = TC_1s, tcLP2 = TC_2s, LPsource = src_TPOBJ_TPOBJLP2, cycTime = cycTime_30ms;

uint16_t Tcounts = 0x83;  // set threshold for over temperature interrupt, 0x83 == 67072 counts

// output data for comparisons
float Tamb, Tamblp3, Tobj, Tobjlp1, Tobjlp2, Tobjlp2frzn, Tpres, Tmot, Tambshk;
uint32_t egress = 0, ingress = 0;

// Read from registers
uint16_t TPAMB, TPAMBLP3;
uint32_t TPOBJ, UOUT1, TPOBJLP1, TPOBJLP2, TPOBJLP2FRZN;
uint8_t  TOBJ1, TPPRESENCE, TPMOTION, TPAMBSHK;

bool newInt = false;

CALIPILE CALIPILE(intPin);

void setup() {

  Serial.begin(115200);  // USB UART output to Arduino IDE serial monitor
  Serial1.begin(115200); // BLE UART bridge to smart device serial console or BLE gateway
  delay(2000);

  pinMode(myLed1, OUTPUT);
  digitalWrite(myLed1, HIGH);  // Start with leds off, active LOW on Butterfly
  pinMode(myLed2, OUTPUT);
  digitalWrite(myLed2, HIGH);
  pinMode(myLed3, OUTPUT);
  digitalWrite(myLed3, HIGH);

  // power pins for CaliPile as a Butterfly add-on
  pinMode(pin_GND, OUTPUT);
  digitalWrite(pin_GND, LOW);

  pinMode(pin_VCC, OUTPUT);
  digitalWrite(pin_VCC, HIGH);

  // address pins for CaliPile
  pinMode(pin_ADO0, OUTPUT);
  digitalWrite(pin_ADO0, LOW);

  pinMode(pin_ADO1, OUTPUT);
  digitalWrite(pin_ADO1, LOW);
  
  Wire.begin(TWI_PINS_20_21); // set master mode 
  Wire.setClock(400000); // I2C frequency at 400 kHz  
  delay(1000);
 
  CALIPILE.I2Cscan(); // CaliPile should not respond to first I2C scan until woken with general call

  //Initiate I2C transcations with general call/reload command
  CALIPILE.wake();

  CALIPILE.I2Cscan();  // Scan again to see if CaliPile responds

  CALIPILE.readEEPROM(); // Verify protocol number and checksum and get calibration constants

  CALIPILE.initMotion(tcLP1, tcLP2, LPsource, cycTime); // configure presence and motion interrupts

  CALIPILE.initTempThr(Tcounts);  // choose something ~5% above TPAMB

  attachInterrupt(intPin, myinthandler, FALLING);  // define interrupt for INT pin output of CaliPile

  // read interrupt status register(s) to unlatch interrupt before entering main loop
  intStatus  = CALIPILE.checkIntStatus();
  Serial.print("Int status = "); Serial.println(intStatus, HEX);
  
  /* end of setup */
}


void loop() {

   if(newInt == true)  // handle interrupt on receipt
   {
    newInt = false;

    // read interrupt status register(s) to clear interrupt
    intStatus  = CALIPILE.checkIntStatus();
 
    /* Handle interrupts */
    if(intStatus & 0x08)
    {
      if(serialDebug) {Serial.println("Presence detected!");}
      if(intStatus & 0x80) 
      {
        digitalWrite(myLed2, LOW); delay(10); digitalWrite(myLed2, HIGH);  // flash green led
        egress++;
        presSign = true;
      }
      else 
      {
        digitalWrite(myLed3, LOW); delay(10); digitalWrite(myLed3, HIGH);  // flash blue led
        ingress++;
        presSign = false;
      }

    if(serialDebug) {Serial.print("ingress = "); Serial.println(ingress);
    Serial.print("egress = "); Serial.println(egress);}
    
    // output to BLE UART
    Serial1.print("ingress = "); Serial1.println(ingress);
    Serial1.print("egress = "); Serial1.println(egress);
    Serial1.flush();
    }

    if(intStatus & 0x04) 
    {
      if(serialDebug) {Serial.println("Motion detected!");}
      if(intStatus & 0x40) motSign = true;
      else motSign = false;
    }

    if(intStatus & 0x10)
    {
     if(serialDebug) {Serial.println("Temp threshold exceeded!");}
    }

    /* end of interrupt handling */
    }
    
    
  // read the ambient temperature
  TPAMB = CALIPILE.getTPAMB();

  Tamb = CALIPILE.getTamb(TPAMB);

  // read the object temperature
  TPOBJ = CALIPILE.getTPOBJ(); 
  
  Tobj = CALIPILE.getTobj(TPOBJ, Tamb);

  // Read the time-integrated registers

  // 20-bit wide, divide by 8 to compare with TPOBJ
  TPOBJLP1 = CALIPILE.getTPOBJLP1();
  
  // 20-bit wide, divide by 8 to compare with TPOBJ
  TPOBJLP2 =  CALIPILE.getTPOBJLP2();
  
  // 16-bit wide, divide by 2 to compare with TPAMB
  TPAMBLP3 = CALIPILE.getTPAMBLP3();
  
  // 24-bit wide, divide by 128 to compare with TPOBJ
  TPOBJLP2FRZN = CALIPILE.getTPOBJLP2FRZN();

  
  TPPRESENCE = CALIPILE.getTPPRESENCE();
  TPMOTION   = CALIPILE.getTPMOTION();
  TPAMBSHK   = CALIPILE.getTPAMBSHK();

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
  }

     if(presSign) 
    {
     if(serialDebug) {Serial.print("TPPRESENCE = ");   Serial.println(-1 * TPPRESENCE);}
    }
    else
    {
       if(serialDebug) {Serial.print("TPPRESENCE = ");   Serial.println(TPPRESENCE);}
    }
    
    if(motSign) 
    {
      if(serialDebug) {Serial.print("TPMOTION = ");   Serial.println(-1 * TPMOTION);}  
    }
    else
    {
       if(serialDebug) {Serial.print("TPMOTION = ");   Serial.println(TPMOTION);}
    }
    
    if(serialDebug) {Serial.print("TAMBSHK = ");    Serial.println(TPAMBSHK);  
    Serial.println(" ");}

    // format output to plot with serial plotter 
//   Serial.print((Tamb - 273.15));  Serial.print("  "); Serial.print((Tobj - 273.15)); Serial.print("  ");
//   Serial.print((TPAMB)); Serial.print("  "); Serial.print((TPAMBLP3)); Serial.print("  ");
   Serial.print((TPOBJ)); Serial.print("  "); Serial.print((TPOBJLP1)); Serial.print("  ");
   Serial.print((TPOBJLP2)); Serial.print("  "); Serial.print((TPOBJLP2FRZN)); Serial.println("  ");

//  digitalWrite(myLed1, LOW); delay(1); digitalWrite(myLed1, HIGH);  
  
  delay(500);
  
  STM32.sleep();
}
 /* end of main loop */

/* Useful functions */
void myinthandler()
{
  newInt = true;
}
