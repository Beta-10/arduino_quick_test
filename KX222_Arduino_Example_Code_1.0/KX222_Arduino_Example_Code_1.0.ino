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
const int BUF_CNTL2 = 0x3B;      // Buffer control
const int BUF_READ = 0x3F;       // Sample buffer
const int INS2 = 0x13;           // Interrupt source register with Data Ready (2^4 position) 

const int INTPIN = 2;

static int i = 0;
static int16_t datac[750];

/************************* Initialization Setup Function **************************/
void setup()
{
  // Initiate wire library and serial communication
  Wire.setClock(400000);    //Low speed setup
  Wire.begin();
  Serial.begin(115200);
  Serial.print("Start\r\n");

  // Write to register
  I2Cwrite(KX222_Address, CNTL1, 0x60);     // 8G, DATA READY ENABLED, STANDBY MODE
  delay(10);
  I2Cwrite(KX222_Address, ODCNTL, 0x8F);    // 5 = 400 Hz; 7 = 1600Hz; E = 12.8kHz sampling, LPF is ODR/2
  delay(10);
  I2Cwrite(KX222_Address, 0x1C, 0x38);      // Setting interrupt
  delay(10);
  I2Cwrite(KX222_Address, 0x1F, 0x10);    // Routing interrupt to pin 1 (Based on DATA READY)
  delay(10);
  I2Cwrite(KX222_Address, 0x3B, 0xC0);      // Enable sample buffer, read 16-bit data, FILO
  delay(10);
  I2Cwrite(KX222_Address, CNTL1, 0xE0);     // 8G, DATA READY ENABLED, OPERATING MODE
  Serial.println("Setup complete");
  Serial.println("Interrupt start");
 
  pinMode(INTPIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INTPIN), IntAcc, RISING);  
}

/************************* Infinite Loop Function **********************************/
void loop()
{
  // Calls ReadSensor function to get temperature data

  if (i >= 340)
  {
    detachInterrupt(digitalPinToInterrupt(INTPIN));
    ReadAcc_Bulk();
    i = 0;
    attachInterrupt(digitalPinToInterrupt(INTPIN), IntAcc, RISING);
  }
}

/*********************** Read KX222 Function (Bulk)*********************************/
void ReadAcc_Bulk(void)
{
  uint8_t x_data[2];
  uint8_t y_data[2];
  uint8_t z_data[2];
  
  int k = 0;
  
  // Ready for sample buffer read
  Wire.beginTransmission(KX222_Address);
  Wire.write(BUF_READ);
  Wire.endTransmission();
  Wire.requestFrom(KX222_Address, 340);

  for (k = 0; k < 340; k++)
  {
    x_data[0] = Wire.receive();
    x_data[1] = Wire.read();
    y_data[0] = Wire.read();
    y_data[1] = Wire.read();
    z_data[0] = Wire.read();
    z_data[1] = Wire.read();

    datac[k] = ((x_data[1] << 8) | x_data[0]);
  }

  for (k = 0; k < 6; k++)
  {
    Serial.println(datac[k]);
  }
}

/**************************** I2C Write Function ********************************/
double I2Cwrite(int dev, int reg, int data)
{
  // Takes in 4 variables:
  // device address, register addres
  // high and low bytes of data to transmit
  Wire.beginTransmission(dev);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
  delay(10);
}

/**************************** Interrupt Function ********************************/
void IntAcc()
{ 
  i = i + 1;
}
