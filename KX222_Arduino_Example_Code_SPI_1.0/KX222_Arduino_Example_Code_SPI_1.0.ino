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

const int INTPIN = 2;
const int CS = 10;

static int i = 0;
static int16_t datac[750];

/************************* Initialization Setup Function **************************/
void setup()
{
  
  pinMode(CS, OUTPUT);
  pinMode(INTPIN, INPUT);
  digitalWrite(CS, HIGH);                   // Disable SPI first

  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  
  Serial.begin(38400);
  Serial.print("Start\r\n");

  // Write to register through SPI
  //KX222_writeConfig(CNTL1, 0x60);           // 8G, DATA READY ENABLED, STANDBY MODE
  //delay(10);
  //KX222_writeConfig(ODCNTL, 0x8F);          // 5 = 400 Hz; 7 = 1600Hz; E = 12.8kHz sampling, LPF is ODR/2
  //delay(10);
  //KX222_writeConfig(0x1C, 0x38);            // Setting interrupt
  //delay(10);
  //KX222_writeConfig(0x1F, 0x10);            // Routing interrupt to pin 1 (Based on DATA READY)
  //delay(10);
  //KX222_writeConfig(CNTL1, 0xE0);           // 8G, DATA READY ENABLED, OPERATING MODE

  KX222_writeConfig(0x18, 0x40);
  KX222_writeConfig(0x1B, 0x02);
  KX222_writeConfig(0x18, 0xC0);
    
  Serial.println("Setup complete");
  Serial.println("Interrupt start");
  
  //attachInterrupt(digitalPinToInterrupt(INTPIN), IntAcc, RISING);  
}

/************************* Infinite Loop Function **********************************/
void loop()
{
  datac[0] = KX222_readAcc(XOUT_L, 2);
  //Serial.println(datac[0]);
}

/*********************** Read KX222 Function (Bulk)*********************************/
void ReadAcc_Bulk(void)
{
  
}

/**************************** SPI Write Function ********************************/
void KX222_writeConfig(uint8_t addr, uint8_t value)
{
  // MSB = 0 for writing
  digitalWrite(CS, LOW);
  SPI.transfer(addr);
  SPI.transfer(value);
  digitalWrite(CS, HIGH);
}

/**************************** SPI Read Function ********************************/
int16_t  KX222_readAcc(uint8_t addr, int bytesToRead)
{
  // MSB = 1 for writing
  uint8_t inByte[2];
  byte ADDR_R = 0x80 | addr;

  digitalWrite(CS, LOW);
  SPI.transfer(ADDR_R);
  
  inByte[0] = SPI.transfer(0);
  inByte[1] = SPI.transfer(0);

  Serial.println(inByte[0]);
  Serial.println(inByte[1]);
  
  bytesToRead--;
  
  digitalWrite(CS, HIGH);
  
  return(((inByte[1] << 8) | inByte[0]));
}

/**************************** Interrupt Function ********************************/
void IntAcc()
{

}
