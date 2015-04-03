#include <Servo.h> 
//Finally starting to get the hang of github. 

Servo myservo;  // create servo object, maximum of eight servo objects can be created 
Servo myservo2; 

int pos = 90;    // variable to store the servo position 
const int maxDeg = 160;
const int minDeg = 5;

const int rightPin = 2;
const int leftPin = 4;
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

void setup() { 
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
	} 

void loop() {
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
	}
