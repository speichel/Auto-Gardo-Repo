/***************************************************************************/
//	Function: Measure the distance to obstacles in front and print the distance
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

int D08 = 8;
int D09 = 9;
int D10 = 10;
int D11 = 11;
int A00 = 0;

typedef struct {
        int ethernet_instruction_D08;
        int ethernet_instruction_D09;
        int ethernet_instruction_D10;
        int ethernet_instruction_D11;
        float ethernet_instruction_A00;
        } D_A_variables_ethernet_arduino;

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
      
        if (i >110){  Serial.println("Inside Maneuver: ");  Serial.println("**Leaving Maneuver***");  i=0;} else i=i+1;   }      
        
void setup()
	{
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
       //three second delay for allowing the digitals to settle and go to zero
       delay(3000);
       
       pinMode(D08, INPUT);
       pinMode(D09, INPUT);
       pinMode(D10, INPUT);
       pinMode(D11, INPUT);
       pinMode(A00, INPUT);
       }
       
long make_D_A_reading(D_A_variables_ethernet_arduino &e);

long make_D_A_reading(D_A_variables_ethernet_arduino &e)
        {
        e.ethernet_instruction_D08 = digitalRead(D08);
        e.ethernet_instruction_D09 = digitalRead(D09);
        e.ethernet_instruction_D10 = digitalRead(D10);
        e.ethernet_instruction_D11 = digitalRead(D11);
        e.ethernet_instruction_A00 = analogRead(A00);
        }

Ultrasonic ultrasonic7(7);
Ultrasonic ultrasonic5(5);

void loop()
	{
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
		  a = 1; //move forward
		} else if (sonar_front < 12)
		{
		  a = -1;//move backward
		}

        D_A_variables_ethernet_arduino X;
        make_D_A_reading(X);
        //Serial.println(X.ethernet_instruction_D08);
        //Serial.println(X.ethernet_instruction_D09);
        //Serial.println(X.ethernet_instruction_D10);
        //Serial.println(X.ethernet_instruction_D11);
        //Serial.println(X.ethernet_instruction_A00);
        
        if (X.ethernet_instruction_D08 == 0 && X.ethernet_instruction_D09 == 1 && X.ethernet_instruction_D10 == 0 && X.ethernet_instruction_D11 == 1)
                {
                  a = 1; //override and move forward
                } else {
                //do nothing
                };         
        if (X.ethernet_instruction_D08 == 1 && X.ethernet_instruction_D09 == 0 && X.ethernet_instruction_D10 == 1 && X.ethernet_instruction_D11 == 0)
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
