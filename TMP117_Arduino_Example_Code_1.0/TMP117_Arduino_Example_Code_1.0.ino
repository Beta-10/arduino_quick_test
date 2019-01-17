/* 
 *  TMP117_Example.ino
 *  Developed by Megan Anderson, THS Applications Engineering Intern
 *  May 2018
 *  
 *  Purpose: Interfaces Arduino UNO with the TMP116 temperature sensor to measure the 
 *  temperature and display it on an LCD Display. Device communicates information based 
 *  on I2C protocol.
 *  
 */

 #include <Wire.h>
 #include <LiquidCrystal.h>

 volatile int alarm;
 
 // Device address
 const int TMP117_Address = 0x48;    

 // Hexadecimal addresses for various TMP116 registers 
 const int Temp_Reg = 0x00;           // Temperature register
 const int Config_Reg = 0x01;         // Configuration register
 const int High_Lim_Reg = 0x02;       // High limit register
 const int Low_Lim_Reg = 0x03;        // Low limit register
 const int EEPROM_Unlock_Reg = 0x04;  // EEPROM unlock register
 const int Device_ID_Reg = 0x0F;      // Device ID register

 // TMP117 Offset Reg
 const int Offset_Reg = 0x07;
 const int offsetH = B00001100;
 const int offsetL = B10000000;

 // Set temperature threshold 
 const uint8_t highlimH = B00001101;   // High byte of high lim
 const uint8_t highlimL = B10000000;   // Low byte of high lim  - High 27 C
 const uint8_t lowlimH = B00001100;    // High byte of low lim
 const uint8_t lowlimL = B00000000;    // Low byte of low lim   - Low 24 C

 // Declare pin assignments for LCD in order: RS, E, D4, D5, D6, D7
 LiquidCrystal lcd(7,8,9,10,11,12); 
 
/************************* Initialization Setup Function **************************/
 void setup(){
  
  // Initiate wire library and serial communication
  Wire.begin();       
  Serial.begin(9600); 
  Serial.print("Start\r\n");
  
  // Initialize LCD
  //lcd.begin(16,2);  
  //lcd.home();        
  //lcd.print("Temp(C): ");

  // Write to register
  I2Cwrite(TMP117_Address, High_Lim_Reg, highlimH, highlimL);
  I2Cwrite(TMP117_Address, Low_Lim_Reg, lowlimH, lowlimL);
  I2Cwrite(TMP117_Address, Config_Reg,0x02, 0x20); 
  //I2Cwrite(TMP117_Address, Offset_Reg, offsetH, offsetL);
  I2Cwrite(TMP117_Address, Offset_Reg, 0x00, 0x00);

  // Sets Pin 13 as output, Pin 2 as input (active low)
  pinMode(13, OUTPUT); 
  pinMode(2, INPUT_PULLUP);
  
  // Sets up pin 2 to trigger "alert" ISR when pin changes H->L and L->H
  attachInterrupt(digitalPinToInterrupt(2), alert, CHANGE);

  alarm = digitalRead(2); // reads startup ALERT pin value
 }

/************************* Infinite Loop Function **********************************/
 void loop(){
  // Calls ReadSensor function to get temperature data
  double temp = ReadTempSensor(); 

  Serial.println(temp); 

  if (!alarm){ // If alarm is active low, trigger alert
    digitalWrite(13,HIGH);
    //lcd.setCursor(9,1);
    //lcd.print("ALERT");
    
    // Clear ALERT flag by I2C reading from config reg
    Wire.beginTransmission(TMP117_Address);
    Wire.write(Config_Reg);
    Wire.endTransmission();
    delay(10);
    Wire.requestFrom(TMP117_Address,2);
  }
  else{ // Turn alert off
    digitalWrite(13,LOW);
    lcd.setCursor(9,1);
    lcd.print("     ");
  }

  // Delay for 0.5 second
  delay(500);
 }

/*********************** Read Temperature Sensor Function **************************/
double ReadTempSensor(void){
     
  // Data array to store 2-bytes from I2C line
  uint8_t data[2]; 
  // Combination of 2-byte data into 16-bit data
  int16_t datac;   

  // Points to device & begins transmission
  Wire.beginTransmission(TMP117_Address); 
  // Points to temperature register to read/write data
  Wire.write(Temp_Reg); 
  // Ends data transfer and transmits data from register
  Wire.endTransmission(); 

  // Delay to allow sufficient conversion time
  delay(10); 

  // Requests 2-byte temperature data from device
  Wire.requestFrom(TMP117_Address,2); 

  // Checks if data received matches the requested 2-bytes
  if(Wire.available() <= 2){  
    // Stores each byte of data from temperature register
    data[0] = Wire.read(); 
    data[1] = Wire.read(); 

    // Combines data to make 16-bit binary number
    datac = ((data[0] << 8) | data[1]); 

    // Convert to Celcius (7.8125 mC resolution) and return
    return datac*0.0078125; 
    
  }
}

/**************************** I2C Write Function ********************************/
double I2Cwrite(int dev, int reg, int H, int L){
  // Takes in 4 variables:
  // device address, register addres
  // high and low bytes of data to transmit 
  Wire.beginTransmission(dev); 
  Wire.write(reg);
  Wire.write(H);
  Wire.write(L);
  Wire.endTransmission();
  delay(10);
}

/******************************* ALERT ISR **************************************/

void alert(){

  alarm = digitalRead(2);

}
