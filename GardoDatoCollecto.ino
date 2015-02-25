//The Gardo Dato Collecto uses a bunch of sensors to collect environmental.
//Sensors included at this this time are the following:
//TMP36 
//PIR Motion Sensor www.sparkfun.com/products/8630 
//Mini Photocell


//Declaring variables

int sensorPin = 0;      //the analog pin the TMP36's Vout (sense) pin is connected to
			//the resolution is 10 mV / degree centigrade with a
			//500 mV offset to allow for negative temperatures
Servo servoRight;       //Declare right servo
Servo servoLeft;        //Declare left servo
int SHT_clockPin = 3;   //Pin used for clock
int SHT_dataPin  = 2;   //Pin used for data
int light1=1;           //Value returned from voltage divider 
int light2=3;           //Value returned from voltage divider 
int pressure=2;         //Value returned from voltage divider
int pirPin = 7;         //digital 7
int pirPin2 = 6;        //digital 6
int i=1;
int a=1;


/* Web Server
 A simple web server that shows the value of the analog input pins.
 using an Arduino Wiznet Ethernet shield. 
 
 Circuit:
 * Ethernet shield attached to pins 10, 11, 12, 13
 * Analog inputs attached to pins A0 through A5 (optional)
 
 created 18 Dec 2009 by David A. Mellis modified 9 Apr 2012 by Tom Igoe */

#include <SPI.h>
#include <Ethernet.h>

byte mac[] = { 0x90, 0xA2, 0xDA, 0x00, 0x4A, 0xFD  };  //The IP address local network 
IPAddress ip(192,168,0,177);
EthernetServer server(80);

void setup() {
	Serial.begin(9600);             // Open serial communications and wait for port to open:
	while (!Serial) 
		{ ;                     // wait for serial port to connect. Needed for Leonardo only
		pinMode(pirPin, INPUT);
		pinMode(pirPin2, INPUT);
		}
					
	Ethernet.begin(mac, ip);        // start the Ethernet connection and the server:
	server.begin();
	Ethernet.begin(mac, ip); 
	Serial.print("server is at ");
	Serial.println(Ethernet.localIP());
	}

void loop() {
	EthernetClient client = server.available();          // listen for incoming clients
	if (client) {
		Serial.println("new client");
		                                             // an http request ends with a blank line
		boolean currentLineIsBlank = true;
		while (client.connected()) {
			if (client.available()) {
				char c = client.read();      // if you've gotten to the end of the line (received a newline
				Serial.write(c);             // so you can send a reply
						             // send a standard http response header
				if (c == '\n' && currentLineIsBlank) {
					client.println("HTTP/1.1 200 OK");
					client.println("Content-Type: text/html");
					client.println("Connection: close");         // the connection will be closed after completion of the response
					client.println("Refresh: .1");               // refresh the page automatically every 5 sec
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

					if(pirVal == LOW) { 
						client.println("1,");             //was motion detected 
						delay(500); 
						} else {
							client.println("0,"); 
							}

					if(pirVal2 == LOW) { 
						client.println("1,");             //was motion detected
						delay(500); 
						a=1+a;
						client.println(a);
						} else {
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
					} else if (c != '\r') {
						// you've gotten a character on the current line
						currentLineIsBlank = false;
						}
				}
			}
		delay(1);          // give the web browser time to receive the data
		client.stop();     // close the connection:
		Serial.println("client disonnected");
		}
	}

float getTemperature(){
	SHT_sendCommand(B00000011, SHT_dataPin, SHT_clockPin);  //Return Temperature in Celsius
	SHT_waitForResult(SHT_dataPin);

	int val = SHT_getData(SHT_dataPin, SHT_clockPin);
	SHT_skipCrc(SHT_dataPin, SHT_clockPin);
	return (float)val * 0.01 - 40;                          //Convert to celsius
	}

float getHumidity(){
	SHT_sendCommand(B00000101, SHT_dataPin, SHT_clockPin);  //Return  Relative Humidity
	SHT_waitForResult(SHT_dataPin);
	int val = SHT_getData(SHT_dataPin, SHT_clockPin);
	SHT_skipCrc(SHT_dataPin, SHT_clockPin);
	return -4.0 + 0.0405 * val + -0.0000028 * val * val; 
	}

void SHT_sendCommand(int command, int dataPin, int clockPin){   //Send a command to the SHTx sensor
								//Transmission start
	pinMode(dataPin, OUTPUT);
	pinMode(clockPin, OUTPUT);
	digitalWrite(dataPin, HIGH);
	digitalWrite(clockPin, HIGH);
	digitalWrite(dataPin, LOW);
	digitalWrite(clockPin, LOW);
	digitalWrite(clockPin, HIGH);
	digitalWrite(dataPin, HIGH);
	digitalWrite(clockPin, LOW);
	shiftOut(dataPin, clockPin, MSBFIRST, command);         //Shift out the command (the 3 MSB are address and must be 000, the last 5 bits are the command)
	digitalWrite(clockPin, HIGH);                           //Verify we get the right ACK
	pinMode(dataPin, INPUT);

	if (digitalRead(dataPin)) Serial.println("ACK error 0");
		digitalWrite(clockPin, LOW);

	if (!digitalRead(dataPin)) Serial.println("ACK error 1")
		//No Action
	}


void SHT_waitForResult(int dataPin){
	pinMode(dataPin, INPUT);                               //Wait for the SHTx answer
	int ack;                                               //Acknowledgement

	for (int i = 0; i < 1000; ++i){
		delay(2);                                      //Need to wait up to 2 seconds for the value ack = digitalRead(dataPin);
		if (ack == LOW) break;
		}

	if (ack == HIGH) Serial.println("ACK error 2");
	}

int SHT_getData(int dataPin, int clockPin){                    //Get data from the SHTx sensor
	pinMode(dataPin, INPUT);                               //Get the MSB (most significant bits)
	pinMode(clockPin, OUTPUT);
	byte MSB = shiftIn(dataPin, clockPin, MSBFIRST);

	pinMode(dataPin, OUTPUT);                              //Send the required ACK
	pinMode(dataPin, OUTPUT);
	digitalWrite(dataPin, HIGH);
	digitalWrite(dataPin, LOW);
	digitalWrite(clockPin, HIGH);
	digitalWrite(clockPin, LOW);

	// get the LSB (less significant bits)
	pinMode(dataPin, INPUT);                               //Get the LSB (less significant bits)
	pinMode(dataPin, INPUT);
	byte LSB = shiftIn(dataPin, clockPin, MSBFIRST);
	return ((MSB << 8) | LSB);                             //Combine bits
	}

void SHT_skipCrc(int dataPin, int clockPin){
	// skip CRC data from the SHTx sensor
	pinMode(dataPin, OUTPUT);
	pinMode(clockPin, OUTPUT);
	digitalWrite(dataPin, HIGH);
	digitalWrite(clockPin, HIGH);
	digitalWrite(clockPin, LOW);
	}
