//Master Auto Gardo Project// 

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
#include "Arduino.h"
#include <Servo.h>                           // Include servo library
#include <SabertoothSimplified.h>

SabertoothSimplified ST; // We'll name the Sabertooth object ST.
                         // For how to configure the Sabertooth, see the DIP Switch Wizard for
                         //   http://www.dimensionengineering.com/datasheets/SabertoothDIPWizard/start.htm
                         // Be sure to select Simplified Serial Mode for use with this library.
                         // This sample uses a baud rate of 9600.
                         //
                         // Connections to make:
                         //   Arduino TX->1  ->  Sabertooth S1
                         //   Arduino GND    ->  Sabertooth 0V
                         //   Arduino VIN    ->  Sabertooth 5V (OPTIONAL, if you want the Sabertooth to power the Arduino)
                         //
                         // If you want to use a pin other than TX->1, see the SoftwareSerial example
const int kpl = -12;                         // Proportional control constants
const int kpr = -12;

int i = 0;
int distance_setting_turning=16;
int sonar_turning;
int distance_setting_front=6;
int sonar_front;
int msTime=20;
int a = 1;
int b = 0;
int c = 0;

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
int i_1=1;
int a_1=1;


class Ultrasonic {
	public:
	  Ultrasonic(int pin);
	  void DistanceMeasure(void);
	  long microsecondsToCentimeters(void);
	private:
	  int _pin;//pin number of Arduino that is connected with SIG pin of Ultrasonic Ranger.
	  long duration;// the Pulse time received;
	};

Ultrasonic::Ultrasonic(int pin) { _pin = pin; }

/*Begin the detection and get the pulse back signal*/
void Ultrasonic::DistanceMeasure(void)
	{
	pinMode(_pin, OUTPUT);
	digitalWrite(_pin, LOW);
	delayMicroseconds(2);
	digitalWrite(_pin, HIGH);
	delayMicroseconds(5);
	digitalWrite(_pin,LOW);
	pinMode(_pin,INPUT);
	duration = pulseIn(_pin,HIGH);
	}

/*The measured distance from the range 0 to 400 Centimeters*/
long Ultrasonic::microsecondsToCentimeters(void)
	{
	  return duration/29/2;	
	}

/*minimum movement in forward direction:    ST.drive(20);    delay(20);    ST.drive(0);    delay(800);*/
void maneuver(int distance_setting_turning, int calc_sonar_turning, int distance_setting_front, int sonar_front, int msTime, int a)
	{
        ST.drive(40*a);
        delay(600);
    
        ST.turn(calc_sonar_turning);
        delay(20);
        
        ST.drive(0);
        delay(100);
     
    
        if(msTime==-1)     // if msTime = -1
		{                                  
		  ST.turn(0);      // Stop servo signals
		  ST.drive(0);   
		}

        delay(msTime);     // Delay for msTime
      
        if (i >110)
		{
		  Serial.println("Inside Maneuver: ");
		  Serial.println("**Leaving Maneuver***");
		  i=0;
		} else
		i=i+1;


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

	//Sabertooth setup

	SabertoothTXPinSerial.begin(9600); // This is the baud rate you chose with the DIP switches.             
	ST.drive(0);                       // The Sabertooth won't act on mixed mode until
	ST.turn(0);                        // it has received power levels for BOTH throttle and turning, since it
		                           // So, we set both to zero initially.
        	                           // Mixed mode tips:
		                           //   drive() should go forward and back, turn() should go right and left.
		                           //     If this is reversed, swap M2A and M2B.
		                           //   Positive on drive() should go forward, negative should go backward.
		                           //     If this is reversed, swap A and B on both M1 and M2.
		                           //   Positive on turn() should go right, negative should go left.
		                           //     If this is reversed, swap M1 and M2.
	                                   // In this sample, the SLOW sweep (left-to-right) here is turning,
	                                   // and the FAST sweep (backwards-to-forwards) is throttle.
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
					client.println("Connection: close"); //Connection closes after response complete
					client.println("Refresh: .1");       //5 sec auto refresh page 
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

	//Insert section of code for navigation

	long RangeInCentimeters7;
	long RangeInCentimeters5;
	float distance7;
	float distance5;

	ultrasonic7.DistanceMeasure();// get the current signal time;
	ultrasonic5.DistanceMeasure();// get the current signal time;

	RangeInCentimeters7 = ultrasonic7.microsecondsToCentimeters();//convert the time to centimeters
	RangeInCentimeters5 = ultrasonic5.microsecondsToCentimeters();//convert the time to centimeters

							//Serial.println("The distance to obstacles in front is: ");
	distance7 = RangeInCentimeters7/2.54;
	distance5 = RangeInCentimeters5/2.54;
							//Serial.print(distance);//0~400cm and 0~157 inches
							//Serial.println(" inches-decimal");
	delay(10);

							// Left and right proportional control calculations
	sonar_turning = distance7;
	sonar_front = distance5;
	
	distance_setting_turning = 12;
	distance_setting_front = 18;  
	
	int calc_sonar_turning = (distance_setting_turning - sonar_turning) * kpr;

	if (sonar_front >= 24 && sonar_front <= 12)  
		{
		  a = a;
		} else if (sonar_front > 24) 
		{
		  a = 1;
		} else if (sonar_front<12)
		{
		  a = -1;
		}

	maneuver(distance_setting_turning, calc_sonar_turning, distance_setting_front, sonar_front, 20, a);       // Drive levels set speeds

	if ( i > 110 )
		{
		  Serial.println("Inside Main Program: ");
		  Serial.println("***Leaving Main Program*** ");
		} 
		else
		{
		//nothing
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
	shiftOut(dataPin, clockPin, MSBFIRST, command);         //Shift out the command 
                                                                //The 3 MSB are address and must be 000
                                                                //The last 5 bits are the command
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
		delay(2);                                      //Need 2 sec wait for value ack = digitalRead(dataPin);
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

	pinMode(dataPin, INPUT);                               //Get the LSB (less significant bits)
	pinMode(dataPin, INPUT);
	byte LSB = shiftIn(dataPin, clockPin, MSBFIRST);
	return ((MSB << 8) | LSB);                             //Combine bits
	}

void SHT_skipCrc(int dataPin, int clockPin){                   //Skip CRC data from the SHTx sensor
	pinMode(dataPin, OUTPUT);
	pinMode(clockPin, OUTPUT);
	digitalWrite(dataPin, HIGH);
	digitalWrite(clockPin, HIGH);
	digitalWrite(clockPin, LOW);
	}
