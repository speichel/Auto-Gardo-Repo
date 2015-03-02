//Master Auto Gardo Project// 

//The Gardo Dato Collecto uses a bunch of sensors to collect environmental.
//Sensors included at this this time are the following:
//TMP36 
//PIR Motion Sensor www.sparkfun.com/products/8630 
//Mini Photocell


//Declaring variables

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
#include <avr/pgmspace.h>

Servo myservo;           //Initialize servo 
Servo myservo2;          //Initialize servo2  

SabertoothSimplified ST; // We'll name the Sabertooth object ST.
                         // For how to configure the Sabertooth, see the DIP Switch Wizard for
                         // http://www.dimensionengineering.com/datasheets/SabertoothDIPWizard/start.htm
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

//Variables for Camo 
int pos = 90;    // variable to store the servo position 
const int maxDeg = 160;
const int minDeg = 5;

const int leftPin = 4;
const int rightPin = 2;
const int upPin = 8;
const int downPin = 12;

const int led1Pin = 6;      // indicator
const int led2Pin = 5;      // indicator
const int led3Pin = 10;     // indicator
const int led4Pin = 11;     // indicator

const int outputPin = 9;    // pwm function will be disabled on pin 9 and 10 if using servo
const int outputPin2 = 3; 

int leftPressed = 0;
int rightPressed = 0;
int upPressed = 0;
int downPressed = 0;


//Variables here for GartoCarto
int i = 0;
int distance_setting_turning=16;
int sonar_turning;
int distance_setting_front=6;
int sonar_front;
int msTime=20;
int a = 1;
int b = 0;
int c = 0;


//Variables here for AutoGardo
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

//Variables for Serever Buttons
char const string_0[] PROGMEM = "<html><body><h2>Controle de LED pela Internet</h2><font size= 4><form method=GET>";
char const string_1[] PROGMEM = "<br><input type=submit name=b1 value=ForwardMotion>";
char const string_2[] PROGMEM = "<br><input type=submit name=b2 value=BackwardMotion>";
char const string_3[] PROGMEM = "<br><input type=submit name=b3 value=RightMotion>";
char const string_4[] PROGMEM = "<br><input type=submit name=b4 value=LeftMotion>";
char const string_5[] PROGMEM = "";  //"<br>Insert your name here:";
char const string_6[] PROGMEM = "";  //"<input name=msg value=no_name MAXLENGTH=20>";
char const string_7[] PROGMEM = "</form></body></html>";
char const string_8[] PROGMEM = "Motion(ON)";
char const string_9[] PROGMEM = "Motion (OFF)";
char const string_10[] PROGMEM = "<meta http-equiv=refresh content=30 > ";   //Auto refresh

PGM_P const string_table[] PROGMEM = //Change "string_table" name to suit
{  
string_0,
string_1,
string_2,
string_3,
string_4,
string_5,
string_6,
string_7,
string_8,
string_9,
string_10
};

char buffer[85];    //Make sure large enough for the largest string

byte mac[] = { 0x90, 0xA2, 0xDA, 0x00, 0x4A, 0xFD  };
IPAddress ip(192,168,0,177);
byte gateway[] = { 192, 168, 1, 1 };
byte subnet[] = { 255, 255, 255, 0 };

String inString = String(35);

EthernetServer server(80);

boolean ForwardMotion = false;
boolean BackwardMotion = false;
boolean RightMotion = false;
boolean LeftMotion = false;

String msg="";
int tam=0;
int st1=9,st2=9,st3=9,st4=9;

//Subprograms, first chunk from Sabo Codo 
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
	}

//byte mac[] = { 0x90, 0xA2, 0xDA, 0x00, 0x4A, 0xFD  };  //The IP address local network 
//IPAddress ip(192,168,0,177);
//EthernetServer server(80);

Ultrasonic ultrasonic7(7);
Ultrasonic ultrasonic5(5);

void setup() {
	Serial.begin(9600);             // Open serial communications and wait for port to open:
	while (!Serial) 
		{ ;                     // wait for serial port to connect. Needed for Leonardo only
		pinMode(pirPin, INPUT);
		pinMode(pirPin2, INPUT);
		}
					
	Ethernet.begin(mac, ip,gateway,subnet);        //Start the Ethernet connection and the server:
	server.begin();                                //Recently added gateway and subnect, this was in ServerButto 
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
	//Camo Setup
	myservo.attach(outputPin);          // attaches the servo on pin 9 to the servo object 
	myservo2.attach(outputPin2);        // attaches the servo on pin 3 to the servo object 

	pinMode(leftPin, INPUT);
	pinMode(rightPin, INPUT);
	pinMode(upPin, INPUT);
	pinMode(downPin, INPUT);

	pinMode(led1Pin, OUTPUT);
	pinMode(led2Pin, OUTPUT);
	pinMode(led3Pin, OUTPUT);
	pinMode(led4Pin, OUTPUT);

	pinMode(4,OUTPUT);
	pinMode(5,OUTPUT);
	pinMode(6,OUTPUT);
	pinMode(7,OUTPUT);
	}

void loop() {
	EthernetClient client = server.available();          // listen for incoming clients

	int led=0;
	if (client) {
		Serial.println("new client");
		                                             // an http request ends with a blank line
		boolean current_line_is_blank = true;
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
				
				//Adding the button part of the code here
				if (inString.length() < 35) {
					inString.concat(c);
					}

				if (c == '\n' && current_line_is_blank) {
					if(inString.indexOf("b1")>0){
						if(ForwardMotion==false){
							st1=8;
							ForwardMotion=true;
							digitalWrite(4,HIGH);
							} else {
								st1=9;
								ForwardMotion=false;
								digitalWrite(4,LOW);
							}
						led=1;
						}
					if(inString.indexOf("b2")>0){
						if(BackwardMotion==false){
							st2=8;
							BackwardMotion=true;
							digitalWrite(5,HIGH);
							} else {
								st2=9;
								BackwardMotion=false;
								digitalWrite(5,LOW);
								}
						led=2;  
						}
					 if(inString.indexOf("b3")>0){
						if(RightMotion==false){
							 st3=8;
							 RightMotion=true;
							 digitalWrite(6,HIGH);
							 } else {
								 st3=9;
								 RightMotion=false;
								 digitalWrite(6,LOW);
								 }
						 led=3;
						 }
					if(inString.indexOf("b4")>0){
						 if(LeftMotion==false){
							 st4=8;
							 LeftMotion=true;
							 digitalWrite(7,HIGH);
							 } else {
								 st4=9;
								 LeftMotion=false;
								 digitalWrite(7,LOW);
								 }
						 led=4;
						 }
					/* if(inString.indexOf("msg")>0){
						 char charBuf1[50];
						 char charBuf2[50];
						 strcpy(msg,(char*)inString.substring(inString.indexOf("g")+2,inString.indexOf(" H")));                        
						 //Serial.print("msg: ");
						 Serial.println(msg);
						 } */

					// send a standard http response header
					client.println("HTTP/1.1 200 OK");
					client.println("Content-Type: text/html");
					client.println();

					strcpy_P(buffer, (char*)pgm_read_word(&(string_table[0]))); // Necessary casts and dereferencing, just copy.
					client.println( buffer );
					for (int i = 1; i < 8; i++) {
						strcpy_P(buffer, (char*)pgm_read_word(&(string_table[i]))); // Necessary casts and dereferencing, just copy.
						client.println( buffer );
						switch(i){
							case 1: strcpy_P(buffer, (char*)pgm_read_word(&(string_table[st1]))); client.println( buffer ); break;
							case 2: strcpy_P(buffer, (char*)pgm_read_word(&(string_table[st2]))); client.println( buffer ); break;
							case 3: strcpy_P(buffer, (char*)pgm_read_word(&(string_table[st3]))); client.println( buffer ); break;
							case 4: strcpy_P(buffer, (char*)pgm_read_word(&(string_table[st4]))); client.println( buffer ); break;
							}
						delay(30);
						}

					if(digitalRead(4)==HIGH){
						client.println("<br>ForwardMotion, ON</br>");
						servoRight.writeMicroseconds(2200);        //Right wheel clockwise
						servoLeft.writeMicroseconds(800);          //Left wheel clockwise
						delay(1000); 
						} else {
							client.println("<br>ForwardMotion, OFF</br>");
							servoRight.writeMicroseconds(1513);//Stay still
							servoLeft.writeMicroseconds(1500); //Stay stil
							delay(1);
							}
					if(digitalRead(5)== HIGH){
						client.println("<br>BackwardMotion, ON</br>");
						servoRight.writeMicroseconds(800);         //Right wheel clockwise
						servoLeft.writeMicroseconds(2200);         //Left wheel clockwise
						delay(1000); 
						} else {
							client.println("<br>BackwardMotion, OFF</br>");
							servoRight.writeMicroseconds(1513);//Stay still
							servoLeft.writeMicroseconds(1500); //Stay still
							delay(1);         
							}
					if(digitalRead(6)==HIGH){
						client.println("<br>Right, ON</br>");
						servoRight.writeMicroseconds(800);         //Right wheel clockwise
						servoLeft.writeMicroseconds(800);          //Left wheel clockwise
						delay(1000); 
						} else {
							client.println("<br>Right, OFF</br>");
							servoRight.writeMicroseconds(1513);//Stay still
							servoLeft.writeMicroseconds(1500); //Stay still
							delay(1);
							}
					if(digitalRead(7)==HIGH){
						client.println("<br>Left, ON</br>");
						servoRight.writeMicroseconds(2200);        //Right wheel clockwise
						servoLeft.writeMicroseconds(2200);         //Left wheel clockwise
						delay(1000); 
						servoRight.writeMicroseconds(1513);        //Stay still
						servoLeft.writeMicroseconds(1500);         //Stay still
						delay(1000);
						} else {
							client.println("<br>Left, OFF</br>");
							servoRight.writeMicroseconds(1513);//Stay still
							servoLeft.writeMicroseconds(1500); //Stay still
							delay(1);
							}
						//strcpy_P(buffer, (char*)pgm_read_word(&(string_table[10]))); client.println( buffer );
						break;
						}
					if (c == '\n') {
						// we're starting a new line
						current_line_is_blank = true;
						} else if (c != '\r') {
							// we've gotten a character on the current line
							current_line_is_blank = false;
							}
						}
				}
			}
		delay(1);          // give the web browser time to receive the data
		client.stop();     // close the connection:
		Serial.println("client disonnected");

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

	if (sonar_front >= 24 && sonar_front <= 12) {
		a = a;
		} else if (sonar_front > 24) {
		a = 1;
		} else if (sonar_front<12)
		a = -1;
	

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

	//Camo void loop section
	leftPressed = digitalRead(leftPin);
	rightPressed = digitalRead(rightPin);
	upPressed = digitalRead(upPin);
	downPressed = digitalRead(downPin);
	 
	if(leftPressed){
		if(pos < maxDeg) pos += 20; 
		myservo.write(pos);                 // tell servo to go to position in variable ‘pos’ 
		digitalWrite(led1Pin,HIGH);
		delay(5);
		myservo.write(90);
		pos -= 20;
		} else
		digitalWrite(led1Pin,LOW);
 
	if(rightPressed){
		if(pos > minDeg) pos -= 20;
		myservo.write(pos);              // tell servo to go to position in variable ‘pos’ 
		digitalWrite(led2Pin,HIGH);
		delay(5);
		myservo.write(90);
		pos += 20;
		} else
		digitalWrite(led2Pin,LOW);
		delay(15);                       // waits 15ms for the servo to reach the position 

	if(upPressed){
		if(pos < maxDeg) pos += 20;
		myservo2.write(pos);              // tell servo to go to position in variable ‘pos’ 
		digitalWrite(led3Pin,HIGH);
		delay(5);
		myservo2.write(90);
		pos -= 20;
		} else
		digitalWrite(led3Pin,LOW);
 
	if(downPressed){
		if(pos > minDeg) pos -= 20;
		myservo2.write(pos);              // tell servo to go to position in variable ‘pos’ 
		digitalWrite(led4Pin,HIGH);
		delay(5);
		myservo2.write(90);
		pos += 20;
		} else
		digitalWrite(led4Pin,LOW);
		delay(15);                       // waits 15ms for the servo to reach the position 
	
	//Servo Buttons
	int led=0;
	if (client) {
		// an http request ends with a blank line

		boolean current_line_is_blank = true;
		while (client.connected()) {
			if (client.available()) {
				char c = client.read();  //If end of the line, then received a newline character
							 //If the line is blank, then http request has ended
							 //and so we can send a reply
				if (inString.length() < 35) {
					inString.concat(c);
					}

				if (c == '\n' && current_line_is_blank) {
					if(inString.indexOf("b1")>0){
						if(ForwardMotion==false){
							st1=8;
							ForwardMotion=true;
							digitalWrite(4,HIGH);
							} else {
								st1=9;
								ForwardMotion=false;
								digitalWrite(4,LOW);
							}
						led=1;
						}
					if(inString.indexOf("b2")>0){
						if(BackwardMotion==false){
							st2=8;
							BackwardMotion=true;
							digitalWrite(5,HIGH);
							} else {
								st2=9;
								BackwardMotion=false;
								digitalWrite(5,LOW);
								}
						led=2;  
						}
					 if(inString.indexOf("b3")>0){
						if(RightMotion==false){
							 st3=8;
							 RightMotion=true;
							 digitalWrite(6,HIGH);
							 } else {
								 st3=9;
								 RightMotion=false;
								 digitalWrite(6,LOW);
								 }
						 led=3;
						 }
					if(inString.indexOf("b4")>0){
						 if(LeftMotion==false){
							 st4=8;
							 LeftMotion=true;
							 digitalWrite(7,HIGH);
							 } else {
								 st4=9;
								 LeftMotion=false;
								 digitalWrite(7,LOW);
								 }
						 led=4;
						 }
					/* if(inString.indexOf("msg")>0){
						 char charBuf1[50];
						 char charBuf2[50];
						 strcpy(msg,(char*)inString.substring(inString.indexOf("g")+2,inString.indexOf(" H")));                        
						 //Serial.print("msg: ");
						 Serial.println(msg);
						 } */

					// send a standard http response header
					client.println("HTTP/1.1 200 OK");
					client.println("Content-Type: text/html");
					client.println();

					strcpy_P(buffer, (char*)pgm_read_word(&(string_table[0]))); // Necessary casts and dereferencing, just copy.
					client.println( buffer );
					for (int i = 1; i < 8; i++) {
						strcpy_P(buffer, (char*)pgm_read_word(&(string_table[i]))); // Necessary casts and dereferencing, just copy.
						client.println( buffer );
						switch(i){
							case 1: strcpy_P(buffer, (char*)pgm_read_word(&(string_table[st1]))); client.println( buffer ); break;
							case 2: strcpy_P(buffer, (char*)pgm_read_word(&(string_table[st2]))); client.println( buffer ); break;
							case 3: strcpy_P(buffer, (char*)pgm_read_word(&(string_table[st3]))); client.println( buffer ); break;
							case 4: strcpy_P(buffer, (char*)pgm_read_word(&(string_table[st4]))); client.println( buffer ); break;
							}
						delay(30);
						}

					if(digitalRead(4)==HIGH){
						client.println("<br>ForwardMotion, ON</br>");
						servoRight.writeMicroseconds(2200);        //Right wheel clockwise
						servoLeft.writeMicroseconds(800);          //Left wheel clockwise
						delay(1000); 
						} else {
							client.println("<br>ForwardMotion, OFF</br>");
							servoRight.writeMicroseconds(1513);//Stay still
							servoLeft.writeMicroseconds(1500); //Stay stil
							delay(1);
							}
					if(digitalRead(5)== HIGH){
						client.println("<br>BackwardMotion, ON</br>");
						servoRight.writeMicroseconds(800);         //Right wheel clockwise
						servoLeft.writeMicroseconds(2200);         //Left wheel clockwise
						delay(1000); 
						} else {
							client.println("<br>BackwardMotion, OFF</br>");
							servoRight.writeMicroseconds(1513);//Stay still
							servoLeft.writeMicroseconds(1500); //Stay still
							delay(1);         
							}
					if(digitalRead(6)==HIGH){
						client.println("<br>Right, ON</br>");
						servoRight.writeMicroseconds(800);         //Right wheel clockwise
						servoLeft.writeMicroseconds(800);          //Left wheel clockwise
						delay(1000); 
						} else {
							client.println("<br>Right, OFF</br>");
							servoRight.writeMicroseconds(1513);//Stay still
							servoLeft.writeMicroseconds(1500); //Stay still
							delay(1);
							}
					if(digitalRead(7)==HIGH){
						client.println("<br>Left, ON</br>");
						servoRight.writeMicroseconds(2200);        //Right wheel clockwise
						servoLeft.writeMicroseconds(2200);         //Left wheel clockwise
						delay(1000); 
						servoRight.writeMicroseconds(1513);        //Stay still
						servoLeft.writeMicroseconds(1500);         //Stay still
						delay(1000);
						} else {
							client.println("<br>Left, OFF</br>");
							servoRight.writeMicroseconds(1513);//Stay still
							servoLeft.writeMicroseconds(1500); //Stay still
							delay(1);
							}
						//strcpy_P(buffer, (char*)pgm_read_word(&(string_table[10]))); client.println( buffer );
						break;
						}
					if (c == '\n') {
						// we're starting a new line
						current_line_is_blank = true;
						} else if (c != '\r') {
							// we've gotten a character on the current line
							current_line_is_blank = false;
							}
						}
				}
			// give the web browser time to receive the data
			delay(1);
			inString = "";
			client.stop();
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

	if (!digitalRead(dataPin)) Serial.println("ACK error 1");
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
