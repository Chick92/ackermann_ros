#include <Servo.h>


Servo myservo;
Servo lights;

void setup(){
	myservo.attach(9);
	myservo.write(90); // 57 = full left, 123 = full right, 90 = middle.
	lights.attach(10);
	lights.write(0);
	
}

voud loop(){
	for (int i = 0; i < 255; i++){
		lights.write(i);
		delay(100);
	}
	for (int i = 57; i < 123; i++){
		myservo.write(i);
		delay(100);
	}
	
}