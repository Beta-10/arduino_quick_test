/*
    AD7606_Example.ino
    Developed by Megan Anderson, THS Applications Engineering Intern
    May 2018

    Purpose: Interfaces Arduino Due with the AD7606 ADC module to measure the Analogue-Digital
    Converter

*/

#include <SPI.h>

// Device address
const int RESET = 5;
const int BUSY = 2;
const int CS = 3;
const int CONVST = 4;

uint16_t ADCData[8];

static int i = 0;
static int16_t datac[32768];
static bool flag = LOW;


/************************* Initialization Setup Function **************************/
void setup()
{
  pinMode(CS, OUTPUT);
  pinMode(CONVST, OUTPUT);
  pinMode(RESET, OUTPUT);
  pinMode(BUSY, INPUT);

  
  digitalWrite(CS, HIGH);                   // Disable SPI first
  digitalWrite(CONVST, HIGH);
  //digitalWrite(OS2, HIGH);
  //digitalWrite(OS1, HIGH);

  digitalWrite(RESET, HIGH);
  delayMicroseconds(50);
  digitalWrite(RESET, LOW);
  
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);

  Serial.begin(115200);
}

/************************* Infinite Loop Function **********************************/
void loop()
{
  digitalWrite(CONVST, LOW);
  delayMicroseconds(5);
  digitalWrite(CONVST, HIGH);

  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CS, LOW);

  
  ADCData[0] = SPI.transfer16(0);
  ADCData[1] = SPI.transfer16(0);
  ADCData[2] = SPI.transfer16(0);
  ADCData[3] = SPI.transfer16(0);
  ADCData[4] = SPI.transfer16(0);
  ADCData[5] = SPI.transfer16(0);
  ADCData[6] = SPI.transfer16(0);
  ADCData[7] = SPI.transfer16(0);
    
  digitalWrite(CS, HIGH);
  
  SPI.endTransaction();

  Serial.println(ADCData[0]);
  //Serial.print('\t');
  //Serial.print(ADCData[1]);
  //Serial.print('\t');
  //Serial.print(ADCData[2]);
  //Serial.print('\t');
  //Serial.println(ADCData[3]);
  
  delay(1);
}
