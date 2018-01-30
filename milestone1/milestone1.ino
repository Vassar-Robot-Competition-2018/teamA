#include <Servo.h>

Servo myservo; 
// create servo object to control a servo

void setup()
{
  myservo.attach(9); 
  // attaches the servo on pin 9 to servo object

}

void loop() {
  // put your main code here, to run repeatedly:
myservo.write(45);

}
