
#ifdef __cplusplus
extern "C" {
#endif
#ifdef __cplusplus
}
#endif

#include <WiFi.h>
#include <SPI.h>

static const int spiClk = 4000000; // 1 MHz

//uninitalised pointers to SPI objects
SPIClass * vspi = NULL;
SPIClass * hspi = NULL;

unsigned long ChannelNumber = 891944;
const char *ssid =  "AID-2017";                                    // replace with your wifi ssid and wpa2 key
const char *pass =  "ylta7777";

//Set web server port number to 80
WiFiServer server(80);

//Variable to store the HTTP request
String header;

//Variable for AD7606 controller
const int CS = 5;
const int RST = 34;
const int BUSYPIN = 35;
const int CONVST = 32;

const int DATA_LENGTH = 2048;

static uint16_t data_ch1[DATA_LENGTH];
static uint16_t data_ch2[DATA_LENGTH];
static uint16_t adc_data;
static uint16_t adc_array[8] = {0,0,0,0,0,0,0,0};
static long rms;
static bool flag = false;

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

  //Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin(); 
  
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
  
  WiFiClient client = server.available();

  if (!flag)
  {
    while (i < DATA_LENGTH)
    {
      digitalWrite(CONVST, LOW);
      delayMicroseconds(5);
      digitalWrite(CONVST, HIGH);

      delayMicroseconds(50);
      
      AD7606Read(adc_array);
      data_ch1[i] = adc_array[0];
      data_ch2[i] = adc_array[1];

      if (data_ch1[i] != 0)
      {
        i++;
      }
      
      delayMicroseconds(50);      
    }

    Serial.println("ADC data ready");
    flag = true;
  }

  if (client)                               // If a new client connects,
  {                             
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected())              // loop while the client's connected
    {            
      if (client.available())               // if there's bytes to read from the client,
      {             
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n')                      // if the byte is a newline character
        {                    
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0)
          {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
            
            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            // CSS to style the on/off buttons 
            // Feel free to change the background-color and font-size attributes to fit your preferences
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #555555;}</style></head>");
            
            // Web Page Heading
            client.println("<body><h1>ESP32 Web Server</h1>");

            client.println("<table>");
            client.println("<tr><th>Channel 1</th><th>Channel 2</th></tr>");

            for (i = 0; i < DATA_LENGTH; i++)
            {
              client.println("<tr>");
              client.println("<td>");
              client.print(data_ch1[i]);
              client.println("</td><td>");
              client.print(data_ch2[i]);
              client.println("</td></tr>");
            }

            client.println("</table>");
            client.println("</body></html>");
            
            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop 
            break;
          }
          else
          { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } 
        
        else if (c != '\r')
        {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
    flag = false;
  }


  /*
  for (i = 0; i < DATA_LENGTH; i++)
  {
    digitalWrite(CONVST, LOW);
    delayMicroseconds(5);
    digitalWrite(CONVST, HIGH);

    delayMicroseconds(50);
    
    AD7606Read(adc_array);
    data_ch1[i] = adc_array[0];
    delayMicroseconds(50);
    //Serial.println(data_ch1[i]);   
  }
  
  rms = RMS_Calculate(DATA_LENGTH);
  
  Serial.print("RMS = ");
  Serial.println(rms);
  Serial.println("Writing data to the net");

  ThingSpeak.writeField(ChannelNumber, 1, rms, apiKey); //Update in ThingSpeak
  
  Serial.println("Wait for next cycle");
  delay(5 );
  */
}

void AD7606Read (uint16_t* adc_value) 
{
    digitalWrite(CS, LOW);                                  //pull SS slow to prep other end for transfer
    int i = 0;
    
    vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));

    for (i = 0; i < 8; i++)
    {
      adc_value[i] = vspi->transfer16(0x0000);
    }

    vspi->endTransaction(); 
    digitalWrite(CS, HIGH);                //pull ss high to signify end of data transfer
}

long RMS_Calculate(int size)
{
  double interim_val = 0;
  double rms_val = 0;
  int i = 0;
  
  for (i =0; i < size; i++)
  {
    interim_val = (data_ch1[i] * data_ch1[i])/size;
    rms_val += interim_val;
  }

  return sqrt(rms_val);
}

/**************************** Interrupt Function ********************************/
void IntBUSY()
{
  flag = false; 
}
