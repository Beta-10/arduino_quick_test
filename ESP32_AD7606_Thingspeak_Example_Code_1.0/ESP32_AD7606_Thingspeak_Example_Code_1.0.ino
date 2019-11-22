

#ifdef __cplusplus
extern "C" {
#endif
#ifdef __cplusplus
}
#endif

#include <WiFi.h>
#include <SPI.h>
#include <ThingSpeak.h>

static const int spiClk = 4000000; // 1 MHz

//uninitalised pointers to SPI objects
SPIClass * vspi = NULL;
SPIClass * hspi = NULL;

unsigned long ChannelNumber = 891944;
const char *apiKey = "G98MOJV78F3N58CB";                  //  Enter your Write API key from ThingSpeak
const char *ssid =  "AID-2017";                                    // replace with your wifi ssid and wpa2 key
const char *pass =  "ylta7777";
const char* server = "api.thingspeak.com";

WiFiClient client;

const int CS = 5;
const int RST = 34;
const int BUSYPIN = 35;
const int CONVST = 32;

const int DATA_LENGTH = 2048;

static uint16_t datac[DATA_LENGTH];
static uint16_t adc_data;
static long rms;
static bool flag = 0;

void setup() 
{
  Serial.begin(115200);
  delay(10);
  Serial.println("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.print(".");
  }

  ThingSpeak.begin(client);
  
  Serial.println("");
  Serial.println("WiFi connected");
  
  //initialise two instances of the SPIClass attached to VSPI and HSPI respectively
  vspi = new SPIClass(VSPI);
  hspi = new SPIClass(HSPI);
 
  //initialise vspi with default pins
  //SCLK = 18, MISO = 19, MOSI = 23, SS = 5
  vspi->begin();
  
  //alternatively route through GPIO pins of your choice
  //hspi->begin(0, 2, 4, 33); //SCLK, MISO, MOSI, SS
  
  //initialise hspi with default pins
  //SCLK = 14, MISO = 12, MOSI = 13, SS = 15
  //hspi->begin(); 
  
  //alternatively route through GPIO pins
  //hspi->begin(25, 26, 27, 32); //SCLK, MISO, MOSI, SS

  //set up slave select pins as outputs as the Arduino API
  //doesn't handle automatically pulling SS low
  pinMode(CS, OUTPUT); //VSPI SS
  //pinMode(15, OUTPUT); //HSPI SS
  pinMode(CONVST, OUTPUT); //CONVST
  pinMode(BUSYPIN, INPUT); //BUSYPIN
  pinMode(RST, OUTPUT); //RESET
  
  attachInterrupt(digitalPinToInterrupt(BUSYPIN), IntBUSY, FALLING);
  
  digitalWrite(CS, HIGH);
  digitalWrite(CONVST, HIGH);
  digitalWrite(RST, LOW);
  delayMicroseconds(5);
  digitalWrite(RST, HIGH);
  delayMicroseconds(50);
  digitalWrite(RST, LOW);
}

void loop() 
{
  int h = 0;
  int i = 0;
  float t =0;

  for (i = 0; i < DATA_LENGTH; i++)
  {
    digitalWrite(CONVST, LOW);
    delayMicroseconds(5);
    digitalWrite(CONVST, HIGH);

    delayMicroseconds(50);
    
    datac[i] = AD7606Read();
    delayMicroseconds(50);
    //Serial.println(datac[i]);   
  }
  
  rms = RMS_Calculate(DATA_LENGTH);
  
  Serial.print("RMS = ");
  Serial.println(rms);
  Serial.println("Writing data to the net");

  ThingSpeak.writeField(ChannelNumber, 1, rms, apiKey); //Update in ThingSpeak
  
  Serial.println("Wait for next cycle");
  delay(5 );
}

uint16_t AD7606Read ( void ) 
{
    digitalWrite(CS, LOW);                      //pull SS slow to prep other end for transfer
    uint16_t reg_data;                               // GPIO register
    vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
    reg_data = vspi->transfer16(0x0000);  
    vspi->transfer16(0x0000);  
    vspi->transfer16(0x0000);  
    vspi->transfer16(0x0000);
    vspi->transfer16(0x0000);  
    vspi->transfer16(0x0000);
    vspi->transfer16(0x0000);  
    vspi->transfer16(0x0000);
    
    vspi->endTransaction(); 
    digitalWrite(CS, HIGH);                //pull ss high to signify end of data transfer
    return reg_data; 
}

long RMS_Calculate(int size)
{
  double interim_val = 0;
  double rms_val = 0;
  int i = 0;
  
  for (i =0; i < size; i++)
  {
    interim_val = (datac[i] * datac[i])/size;
    rms_val += interim_val;
  }

  return sqrt(rms_val);
}

/**************************** Interrupt Function ********************************/
void IntBUSY()
{
  flag = false; 
}
