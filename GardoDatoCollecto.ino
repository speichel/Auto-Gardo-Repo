
//TMP36 Pin Variables

#include <Servo.h> // Include servo library
int sensorPin = 0; //the analog pin the TMP36's Vout (sense) pin is connected to
//the resolution is 10 mV / degree centigrade with a
//500 mV offset to allow for negative temperatures
Servo servoRight;                            // Declare right ser
Servo servoLeft;                            // Declare right ser
int SHT_clockPin = 3;  // pin used for clock
int SHT_dataPin  = 2;  // pin used for data
int light1=1; //Value returned from the potentiometer
int pressure=2; //Value returned from the potentiometer
int light2=3; //Value returned from the potentiometer

int pirPin = 7; //digital 7
int pirPin2 = 6; //digital 6
int i=1;
int a=1;


/*
  Web Server
 
 A simple web server that shows the value of the analog input pins.
 using an Arduino Wiznet Ethernet shield. 
 
 Circuit:
 * Ethernet shield attached to pins 10, 11, 12, 13
 * Analog inputs attached to pins A0 through A5 (optional)
 
 created 18 Dec 2009
 by David A. Mellis
 modified 9 Apr 2012
 by Tom Igoe
 
 */
//2/20/2015 Organizing this file before migrating to master file

#include <SPI.h>
#include <Ethernet.h>

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = { 
  0x90, 0xA2, 0xDA, 0x00, 0x4A, 0xFD  };
IPAddress ip(192,168,0,177);
EthernetServer server(80);

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only

    pinMode(pirPin, INPUT);
    pinMode(pirPin2, INPUT);
  }


  // start the Ethernet connection and the server:
  Ethernet.begin(mac, ip);
  server.begin();
  Serial.print("server is at ");
  Serial.println(Ethernet.localIP());
}


void loop() {
  // listen for incoming clients
  EthernetClient client = server.available();
  if (client) {
    Serial.println("new client");
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c);
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          // send a standard http response header
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          client.println("Connection: close");  // the connection will be closed after completion of the response
          client.println("Refresh: .1");  // refresh the page automatically every 5 sec
          client.println();
          client.println("<?xml version=\"1.0\" encoding=\"UTF-8\"?>");
          client.println("<measurementdata>");

          int soilmoisture = analogRead(5);                 
          char soil[10];
          String soilAsString;
          String soilString;
          dtostrf(soilmoisture,1,2,soil);
          soilAsString = String(soil);
          soilString = soilAsString + ",";
          client.println(soilString);

          // Motion
          int pirVal = digitalRead(pirPin);                 
          int pirVal2 = digitalRead(pirPin2);
          if(pirVal == LOW)
          { //was motion detected
            client.println("1,"); 
            delay(500); 
          }
          else
          {
            client.println("0,"); 
          }

          if(pirVal2 == LOW)
          { //was motion detected
            client.println("1,"); 
            delay(500); 
            a=1+a;
            client.println(a);
          }  
          else
          {
            client.println("0,"); 
          }

          float temperature = getTemperature();
          float temperature2 = (temperature * 9.0 / 5.0) + 32.0;
          char temp[10];
          String tempAsString;
          String tempString;
          dtostrf(temperature2,1,2,temp);
          tempAsString = String(temp);
          tempString = tempAsString + ",";
          client.println(tempString);

          float humidity = getHumidity();
          char hum[10];
          String humAsString;
          String humidityString;
          dtostrf(humidity,1,2,hum);
          humAsString = String(hum);
          humidityString = humAsString + ",";
          client.println(humidityString);

          light1=analogRead(1); 
          char light[10];
          String lightAsString;
          String lightString;
          dtostrf(light1,1,2,light);
          lightAsString = String(light);
          lightString = lightAsString + ",";
          client.println(lightString);

          pressure=analogRead(2);
          char pres[10];
          String pressureAsString;
          String pressureString;
          dtostrf(pressure,1,2,pres);
          pressureAsString = String(pres);
          pressureString = pressureAsString + ",";
          client.println(pressureString);

          light2=analogRead(3);
          char lig[10];
          String light2AsString;
          String light2String;
          dtostrf(light2,1,2,lig);
          light2AsString = String(lig);
          light2String = light2AsString + ",";
          client.println(light2String);

          //getting the voltage reading from the temperature sensor
          int reading = analogRead(sensorPin); 
          float voltage = reading * 5.0;
          voltage /= 1024.0; 
          float temperatureC = (voltage - 0.5) * 100; 
          float temperatureF = (temperatureC * 9.0 / 5.0) + 32.0;                      
          char tempe[10];
          String temperatureFAsString;
          String temperatureFString;
          dtostrf(temperatureF,1,2,tempe);
          temperatureFAsString = String(tempe);
          temperatureFString = temperatureFAsString + ",";
          client.println(temperatureFString);


          client.println(soilString);   

          delay(5000);                                     //waiting a second

          String measurementdata;
          measurementdata = "</measurementdata>";
          client.println(measurementdata);



          break;



        }
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
        } 
        else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);
    // close the connection:
    client.stop();
    Serial.println("client disonnected");
  }
}


float getTemperature(){
  //Return Temperature in Celsius
  SHT_sendCommand(B00000011, SHT_dataPin, SHT_clockPin);
  SHT_waitForResult(SHT_dataPin);

  int val = SHT_getData(SHT_dataPin, SHT_clockPin);
  SHT_skipCrc(SHT_dataPin, SHT_clockPin);
  return (float)val * 0.01 - 40; //convert to celsius
}

float getHumidity(){
  //Return  Relative Humidity
  SHT_sendCommand(B00000101, SHT_dataPin, SHT_clockPin);
  SHT_waitForResult(SHT_dataPin);
  int val = SHT_getData(SHT_dataPin, SHT_clockPin);
  SHT_skipCrc(SHT_dataPin, SHT_clockPin);
  return -4.0 + 0.0405 * val + -0.0000028 * val * val; 
}


void SHT_sendCommand(int command, int dataPin, int clockPin){
  // send a command to the SHTx sensor
  // transmission start
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  digitalWrite(dataPin, HIGH);
  digitalWrite(clockPin, HIGH);
  digitalWrite(dataPin, LOW);
  digitalWrite(clockPin, LOW);
  digitalWrite(clockPin, HIGH);
  digitalWrite(dataPin, HIGH);
  digitalWrite(clockPin, LOW);

  // shift out the command (the 3 MSB are address and must be 000, the last 5 bits are the command)
  shiftOut(dataPin, clockPin, MSBFIRST, command);

  // verify we get the right ACK
  digitalWrite(clockPin, HIGH);
  pinMode(dataPin, INPUT);

  if (digitalRead(dataPin)) Serial.println("ACK error 0");
  digitalWrite(clockPin, LOW);
  if (!digitalRead(dataPin)) Serial.println("ACK error 1");
}


void SHT_waitForResult(int dataPin){
  // wait for the SHTx answer
  pinMode(dataPin, INPUT);

  int ack; //acknowledgement

  //need to wait up to 2 seconds for the value
  for (int i = 0; i < 1000; ++i){
    delay(2);
    ack = digitalRead(dataPin);
    if (ack == LOW) break;
  }

  if (ack == HIGH) Serial.println("ACK error 2");
}

int SHT_getData(int dataPin, int clockPin){
  // get data from the SHTx sensor

  // get the MSB (most significant bits)
  pinMode(dataPin, INPUT);
  pinMode(clockPin, OUTPUT);
  byte MSB = shiftIn(dataPin, clockPin, MSBFIRST);

  // send the required ACK
  pinMode(dataPin, OUTPUT);
  digitalWrite(dataPin, HIGH);
  digitalWrite(dataPin, LOW);
  digitalWrite(clockPin, HIGH);
  digitalWrite(clockPin, LOW);

  // get the LSB (less significant bits)
  pinMode(dataPin, INPUT);
  byte LSB = shiftIn(dataPin, clockPin, MSBFIRST);
  return ((MSB << 8) | LSB); //combine bits
}

void SHT_skipCrc(int dataPin, int clockPin){
  // skip CRC data from the SHTx sensor
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  digitalWrite(dataPin, HIGH);
  digitalWrite(clockPin, HIGH);
  digitalWrite(clockPin, LOW);
}
