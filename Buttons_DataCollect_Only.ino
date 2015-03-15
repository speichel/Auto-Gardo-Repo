#include <Ethernet.h>
#include <SPI.h>
#include <avr/pgmspace.h>
#include <Servo.h> 

Servo servoRight;   // Declare right servo
Servo servoLeft;    // Declare left servo
 
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
char const string_10[] PROGMEM = "<meta http-equiv=refresh content=1 > ";   //Auto refresh

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

//ADD CODE FROM GardoDatoCollecto Here
int sensorPin = 0;      //the analog pin the TMP36's Vout (sense) pin is connected to
			//the resolution is 10 mV / degree centigrade with a
			//500 mV offset to allow for negative temperatures
int SHT_clockPin = 3;   //Pin used for clock
int SHT_dataPin  = 2;   //Pin used for data
int light1=1;           //Value returned from voltage divider 
int light2=3;           //Value returned from voltage divider 
int pressure=2;         //Value returned from voltage divider
int pirPin = 7;         //digital 7
int pirPin2 = 6;        //digital 6
int i=1;
int a=1;

void setup() {
	Serial.begin(9600);
	Ethernet.begin(mac, ip,gateway,subnet);
	server.begin();
	Serial.println("Serial READY");
	Serial.println("Ethernet READY");
	Serial.println("Server READY");
	pinMode(4,OUTPUT);
	pinMode(5,OUTPUT);
	pinMode(6,OUTPUT);
	pinMode(7,OUTPUT);

	servoRight.attach(8);    // Attach right signal to pin 12
	servoLeft.attach(9);
	}

void loop() {
	EthernetClient client = server.available();

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
  
                                	// send a standard http response header
					client.println("HTTP/1.1 200 OK");
					client.println("Content-Type: text/html");
                                        client.println("Connection: close"); //Connection closes after response complete
                                        client.println("Refresh: .1");       //5 sec auto refresh page 
                                        client.println();
                                        
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





                              client.println("<br>-----</br>");
                              client.println("<br>-----</br>");
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
