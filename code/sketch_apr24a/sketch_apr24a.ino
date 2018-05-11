
#include <Servo.h>

Servo leftServo;
int leftServoPin = 5;

Servo rightServo;
int rightServoPin = 6;

Servo leftBackServo;
int leftBackServoPin = 7;

Servo rightBackServo;
int rightBackServoPin = 8;

void setup() {
  Serial.begin(9600);
  Serial.print("Starting...\n");


  leftServo.attach(leftServoPin);
  rightServo.attach(rightServoPin);
  leftBackServo.attach(leftBackServoPin);
  rightBackServo.attach(rightBackServoPin);
}

void setSpeed(int speed) {
  setLeftSpeed(speed);
  setRightSpeed(speed);
}

void setSpeed(int left, int right) {
  setLeftSpeed(left);
  setRightSpeed(right);
}

int speedToDegrees(int speed) {
  int deg = (speed * 0.6) + 90;
  return deg;
}

// Reverse full speed -100, Forward full speed 100
void setLeftSpeed(int speed) {
  int degrees = 180 - speedToDegrees(speed) + 3;
//  Serial.print("Left Degrees: "); Serial.println(degrees);
  leftServo.write(degrees);
  leftBackServo.write(degrees);
}

// Reverse full speed -100, Forward full speed 100
void setRightSpeed(int speed) {
  int degrees = speedToDegrees(speed) + 3;
//  Serial.print("Right Degrees: "); Serial.println(degrees);
  rightServo.write(degrees);
  rightBackServo.write(degrees);
}

void loop() {
  setSpeed(100);

  delay(5000);

  setSpeed(100,-100);

  delay(5000);

  setSpeed(100,50);
  
  delay(5000);
}
