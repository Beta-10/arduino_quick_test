/*
    KX222_Example.ino
    Developed by Megan Anderson, THS Applications Engineering Intern
    May 2018

    Purpose: Interfaces Arduino UNO with the TMP116 temperature sensor to measure the
    temperature and display it on an LCD Display. Device communicates information based
    on I2C protocol.

*/

#include <Wire.h>

// Device address
const int KX222_Address = 0x1E;  // With ADDR to GND

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
const int INS2 = 0x13;           // Interrupt source register with Data Ready (2^4 position) 

/************************* Initialization Setup Function **************************/
void setup() {

  // Initiate wire library and serial communication
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI
  Serial.begin(115200);
  Serial.print("Start\r\n");

  // Write to register
  I2Cwrite(KX222_Address, CNTL1, 0x60);     // 8G, DATA READY ENABLED, STANDBY MODE
  delay(10);
  I2Cwrite(KX222_Address, ODCNTL, 0x8F);    // 5 = 400 Hz; 7 = 1600Hz; E = 12.8kHz sampling, LPF is ODR/2
  delay(10);
  I2Cwrite(KX222_Address, 0x1C, 0x38);      // Setting interrupt
  delay(10);
  I2Cwrite(KX222_Address, 0x1F, 0x10);      // Routing interrupt to pin 1
  delay(10);
  I2Cwrite(KX222_Address, CNTL1, 0xE0);     // 8G, DATA READY ENABLED, OPERATING MODE

  Serial.println("Setup complete");
}

/************************* Infinite Loop Function **********************************/
void loop() {
  // Calls ReadSensor function to get temperature data

  ReadAcc_Bulk();

  //int16_t Acc = ReadAcc();
  //Serial.println(Acc);
}

/*********************** Read KX222 Function ***************************************/
int16_t ReadAcc(void) {

  // Data array to store 2-bytes from I2C line
  uint8_t data[2];
  // Combination of 2-byte data into 16-bit data
  int16_t datac;

  Wire.beginTransmission(KX222_Address);
  Wire.write(XOUT_L);
  Wire.endTransmission();
  
  // Requests 2-byte temperature data from device
  Wire.requestFrom(KX222_Address, 2);

  // Checks if data received matches the requested 2-bytes
  if (Wire.available() <= 2) {
    // Stores each byte of data from temperature register
    data[0] = Wire.read();
    data[1] = Wire.read();

    // Combines data to make 16-bit binary number
    datac = ((data[1] << 8) | data[0]);
    return datac;
  }
}


/*********************** Read KX222 Function (Bulk)*********************************/
void ReadAcc_Bulk(void) {

  // Data array to store 2-bytes from I2C line
  uint8_t flag = 0;
  uint8_t data[2];
  // Combination of 2-byte data into 16-bit data
  int16_t datac[750];
  int16_t number = 0;
  int i = 0;

  for (i = 0; i < 750; i++)
  {
    Wire.beginTransmission(KX222_Address);
    Wire.write(INS2);
    Wire.endTransmission();

    while (!flag)
    {
      Wire.requestFrom(KX222_Address, 1);
      flag = Wire.read();
      flag = ((flag >> 4) && 0x01);
    }
      
    Wire.beginTransmission(KX222_Address);
    Wire.write(XOUT_L);
    Wire.endTransmission();

    Wire.requestFrom(KX222_Address, 2);

    if (Wire.available() <= 2) {
      // Stores each byte of data from temperature register
      data[0] = Wire.read();
      data[1] = Wire.read();

      // Combines data to make 16-bit binary number
      datac[i] = ((data[1] << 8) | data[0]);
      flag = 0;
    }
  }
  
  for (i = 0; i < 750; i++)
  {
    Serial.println(datac[i]);
  }
}

/**************************** I2C Write Function ********************************/
double I2Cwrite(int dev, int reg, int data) {
  // Takes in 4 variables:
  // device address, register addres
  // high and low bytes of data to transmit
  Wire.beginTransmission(dev);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
  delay(10);
}
