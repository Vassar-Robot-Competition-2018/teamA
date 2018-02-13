#include <Servo.h>

Servo leftservo;
Servo rightservo;

void setup() {
  leftservo.attach(2);
  leftservo.write(0);
  
  rightservo.attach(3);
  rightservo.write(180);
}

void loop() {

}
