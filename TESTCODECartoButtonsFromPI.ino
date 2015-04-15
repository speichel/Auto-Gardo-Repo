/***************************************************************************/
//	Function: T Measure the distance to obstacles in front and print the distance
//			  value to the serial terminal.The measured distance is from 
//			  the range 0 to 400cm(157 inches).
//	Hardware: Ultrasonic Range sensor
//	Arduino IDE: Arduino-1.0
//	Author:	 LG		
//	Date: 	 Jan 17,2013
//	Version: v1.0 modified by FrankieChu
//	by www.seeedstudio.com
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
//
/*****************************************************************************/
#include "Arduino.h"
#include <Servo.h>                           // Include servo library
#include <SabertoothSimplified.h>

//SabertoothSimplified ST; // We'll name the Sabertooth object ST.
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
int distance_setting_turning=6;
int sonar_turning;
int distance_setting_front=8;
int sonar_front;
int msTime=20;
int a = 1;
int b = 0;
int c = 0;

//Camerial Variables, Remember that Pins 5 & 7 are being used by the ultrasonic

Servo myservo;  // create servo object, maximum of eight servo objects can be created 
Servo myservo2; 

int pos = 90;    // variable to store the servo position 
const int maxDeg = 160;
const int minDeg = 5;

int D02 = 2; 
int D04 = 4;
int D08 = 8;
int D12 = 12;
int A00 = 0;
int A01 = 1;

int D03 = 1;  //reserved for overide
int D06 = 6;
int D13 = 13;

const int outputPin = 9;    // pwm function will be disabled on pin 9 and 10 if using servo
const int outputPin2 = 3; 
int leftPressed = 0;
int rightPressed = 0;
int upPressed = 0;
int downPressed = 0;

typedef struct {
        int PI_instruction_D02;
        int PI_instruction_D03;
        int PI_instruction_D04;
        int PI_instruction_D06;
        int PI_instruction_D08;
        int PI_instruction_D12;
        int PI_instruction_D13;
        float PI_instruction_A00;
        } D_A_variables_PI_arduino;

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
	if (sonar_front <= 24 && sonar_front >= 8)  
		{
		  a = a;
        Serial.print("Stay the course: a = ");
        Serial.println(a);
		} else if (sonar_front > 24) 
		{
		  a = 1; //move forward
        Serial.print("Move Forward Now: a = 1");
        Serial.println(a);
		} else if (sonar_front < 8)
		{
        
		  a = -1;//move backward
        Serial.print("Move Backward Now: a = -1");
        Serial.println(a);
		}
  
  
        //ST.drive(40*a);
        Serial.print("ST.drive(40*a)= ");
        Serial.println(40*a);
        
        delay(600);
    
        //ST.turn(calc_sonar_turning);
        Serial.print("ST.turn(calc_sonar_turning)=");
        Serial.println(calc_sonar_turning);
        
        delay(20);
        
        //ST.drive(0);
        Serial.print("ST.drive(0)=");
        Serial.println(0);
        
        delay(100);
     
    
        if(msTime==-1)     // if msTime = -1
		{                                  
		  //ST.turn(0);      // Stop servo signals
		  //ST.drive(0);   
		}

        delay(msTime);     // Delay for msTime
      
        if (i >110){  Serial.println("Inside Maneuver: ");  Serial.println("**Leaving Maneuver***");  i=0;} else i=i+1;   }      
        
void setup()
	{
        Serial.begin(9600);
	//SabertoothTXPinSerial.begin(9600); // This is the baud rate you chose with the DIP switches.             
	//ST.drive(0);                       // The Sabertooth won't act on mixed mode until
	//ST.turn(0);                        // it has received power levels for BOTH throttle and turning, since it
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
       //three second delay for allowing the digitals to settle and go to zero
       delay(3000);
       
	pinMode(D02, INPUT);
	pinMode(D03, INPUT);
	pinMode(D04, INPUT);
	pinMode(D06, INPUT);
	pinMode(D08, INPUT);
	pinMode(D12, INPUT);
	pinMode(D13, INPUT);
	pinMode(A00, INPUT);
	pinMode(A01, INPUT);
	
	myservo.attach(outputPin);          // attaches the servo on pin 9 to the servo object 
	myservo2.attach(outputPin2);        // attaches the servo on pin 3 to the servo object 

	}
       
long make_D_A_reading(D_A_variables_PI_arduino &e);

long make_D_A_reading(D_A_variables_PI_arduino &e)
        {
        e.PI_instruction_D02 = digitalRead(D02);
        e.PI_instruction_D03 = digitalRead(D03);
        e.PI_instruction_D04 = digitalRead(D04);
        e.PI_instruction_D06 = digitalRead(D06);
        e.PI_instruction_D08 = digitalRead(D08);
        e.PI_instruction_D12 = digitalRead(D12);
        e.PI_instruction_D13 = digitalRead(D13);
        e.PI_instruction_A00 = analogRead(A00);
        }

Ultrasonic ultrasonic7(7);  //declaring here what pins are being used
Ultrasonic ultrasonic5(5);

void loop()
	{
	
	D_A_variables_PI_arduino X;
        make_D_A_reading(X);
        //Serial.println(X.PI_instruction_D02);
        //Serial.println(X.PI_instruction_D04);
        //Serial.println(X.PI_instruction_D08);
        //Serial.println(X.PI_instruction_D12);
        //Serial.println(X.PI_instruction_A00);
	
	leftPressed = digitalRead(X.PI_instruction_D04);
	rightPressed = digitalRead(X.PI_instruction_D02);
	upPressed = digitalRead(X.PI_instruction_D08);
	downPressed = digitalRead(X.PI_instruction_D12);
	 
	if(leftPressed){
		if(pos < maxDeg) pos += 20; 
		myservo.write(pos);                 // tell servo to go to position in variable ‘pos’ 
		delay(5);
		myservo.write(90);
		pos -= 20;
		} else
 
	if(rightPressed){
		if(pos > minDeg) pos -= 20;
		myservo.write(pos);              // tell servo to go to position in variable ‘pos’ 
		delay(5);
		myservo.write(90);
		pos += 20;
		} else
		delay(15);                       // waits 15ms for the servo to reach the position 

	if(upPressed){
		if(pos < maxDeg) pos += 20;
		myservo2.write(pos);              // tell servo to go to position in variable ‘pos’ 
		delay(5);
		myservo2.write(90);
		pos -= 20;
		} else
 
	if(downPressed){
		if(pos > minDeg) pos -= 20;
		myservo2.write(pos);              // tell servo to go to position in variable ‘pos’ 
		delay(5);
		myservo2.write(90);
		pos += 20;
		} else
		delay(15);                       // waits 15ms for the servo to reach the position 

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
        Serial.print("sonar_turning = distance7 = ");
        Serial.println(sonar_turning);
	
        Serial.print("sonar_front = distance5 = ");
        Serial.println(sonar_front);

	int calc_sonar_turning = (distance_setting_turning - sonar_turning) * kpr;




        
        if (X.PI_instruction_D03 == 0 && X.PI_instruction_D06 == 1)
                {
                  a = 1; //override and move forward
                } else {
                //do nothing
                };         
        if (X.PI_instruction_D03 == 1 && X.PI_instruction_D06 == 0)
                {
                  a = -1; //override and move backward
                } else {
                //do nothing
                }; 
        
        delay(500);

	maneuver(distance_setting_turning, calc_sonar_turning, distance_setting_front, sonar_front, 20, a);       // Drive levels set speeds


	if ( i > 110 ){ Serial.println("Inside Main Program: ");  Serial.println("***Leaving Main Program*** ");} else	{//nothing
		}
	}
