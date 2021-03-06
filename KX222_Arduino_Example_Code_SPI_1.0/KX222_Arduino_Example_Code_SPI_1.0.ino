/*
    KX222_Example.ino
    Developed by Megan Anderson, THS Applications Engineering Intern
    May 2018

    Purpose: Interfaces Arduino UNO with the TMP116 temperature sensor to measure the
    temperature and display it on an LCD Display. Device communicates information based
    on I2C protocol.

*/

#include <SPI.h>

// Device address

// Hexadecimal addresses for various TMP116 registers
const int XHP_L = 0x00;          // High pass filtered accelerometer output
const int XHP_H = 0x01;
const int YHP_L = 0x02;
const int YHP_H = 0x03;
const int ZHP_L = 0x04;
const int ZHP_H = 0x05;

const int XOUT_L = 0x06;         // Accelerometer output
const int XOUT_H = 0x07;
const int YOUT_L = 0x08;
const int YOUT_H = 0x09;
const int ZOUT_L = 0x0A;
const int ZOUT_H = 0x0B;

const int WHO_AM_I = 0x0F;       // Device ID

const int CNTL1 = 0x18;          // Control register
const int ODCNTL = 0x1B;         // ODR Control
const int BUF_CNTL2 = 0x3B;      // Buffer control
const int BUF_READ = 0x3F;       // Sample buffer
const int INS2 = 0x13;           // Interrupt source register with Data Ready (2^4 position) 
const int LP_CNTL = 0x35;        // Averaging Filter Control

const int INTPIN = 2;
const int CS = 10;
const int DATA_LENGTH = 32768;

static int i = 0;
static int16_t datac[DATA_LENGTH];
static bool flag = LOW;


/************************* Initialization Setup Function **************************/
void setup()
{
  
  pinMode(CS, OUTPUT);
  pinMode(INTPIN, INPUT);
  digitalWrite(CS, HIGH);                   // Disable SPI first

  
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  
  Serial.begin(115200);
  //Serial.print("Start\r\n");

  // Write to register through SPI
  KX222_writeConfig(CNTL1, 0x68);           // 16G, DATA READY ENABLED, STANDBY MODE
  //KX222_writeConfig(CNTL1, 0x28);           // 16G, DATA READY ENABLED, STANDBY MODE, LOW POWER MODE
  delay(10);
  KX222_writeConfig(ODCNTL, 0x4F);          // 4 = 200Hz; 5 = 400 Hz; 7 = 1600Hz; C = 3.2kHz; E = 12.8kHz sampling, LPF is ODR/2
  delay(10);
  KX222_writeConfig(0x1C, 0x38);            // Setting interrupt
  delay(10);
  KX222_writeConfig(0x1F, 0x10);            // Routing interrupt to pin 1 (Based on DATA READY)
  delay(10);
  //KX222_writeConfig(LP_CNTL, 0x70);         // 4 = 16 Samples averages; 111 = 128 Samples averages
  KX222_writeConfig(CNTL1, 0xE8);           // 16G, DATA READY ENABLED, OPERATING MODE
  //KX222_writeConfig(CNTL1, 0xA8);             // 16G, DATA READY ENABLED, OPERATING MODE, LOW POWER MODE
    
  //Serial.println("Setup complete");
  //Serial.println("Interrupt start");
  
  attachInterrupt(digitalPinToInterrupt(INTPIN), IntAcc, RISING);  
}

/************************* Infinite Loop Function **********************************/
void loop()
{
  if (i >= DATA_LENGTH)
  {
    detachInterrupt(digitalPinToInterrupt(INTPIN));
    for (int k = 0; k < DATA_LENGTH; k++)
    {
      Serial.println(datac[k]);
    }
    i = 0;
    //attachInterrupt(digitalPinToInterrupt(INTPIN), IntAcc, RISING); 
  }
}

/**************************** SPI Write Function ********************************/
void KX222_writeConfig(uint8_t addr, uint8_t value)
{
  // MSB = 0 for writing
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CS, LOW);
  SPI.transfer(addr);
  SPI.transfer(value);
  digitalWrite(CS, HIGH);
  SPI.endTransaction();
}

/**************************** SPI Read Function ********************************/
int16_t  KX222_readAcc(uint8_t addr)
{
  // MSB = 1 for writing
  uint8_t inByte[2];
  uint8_t flagByte = 0;
  byte ADDR_R = 0x80 | addr;
    
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CS, LOW);
  SPI.transfer(ADDR_R);
  
  inByte[0] = SPI.transfer(0);
  inByte[1] = SPI.transfer(0);
    
  digitalWrite(CS, HIGH);
  SPI.endTransaction();
  
  return(((inByte[1] << 8) | inByte[0]));
}

/**************************** Interrupt Function ********************************/
void IntAcc()
{
  //detachInterrupt(digitalPinToInterrupt(INTPIN));
  datac[i] = KX222_readAcc(XOUT_L);
  i++;
  //attachInterrupt(digitalPinToInterrupt(INTPIN), IntAcc, RISING); 
}
