#include <Servo.h>

Servo leftservo;
Servo rightservo; 
// create servo object to control a servo

void setup()
{
  leftservo.attach(9);
  rightservo.attach(10);
  // attaches the servo on pin 9 to servo object

}

void loop() {
  // put your main code here, to run repeatedly:
leftservo.write(45);
rightservo.write(45);

}
