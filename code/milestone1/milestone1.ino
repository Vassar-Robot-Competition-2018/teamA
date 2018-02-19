#include <Servo.h>

int baud = 9600;

int switchPin = 10;
int servo1Pin = 22;
int servo2Pin = 23;

Servo servo1;
Servo servo2;

void setup() {
  Serial.begin(baud);
  pinMode(switchPin, INPUT);
  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);
}

void loop() {
  if (digitalRead(switchPin) == HIGH) {
    Serial.print("stop\n");
    servo1.write(90);
    servo2.write(90);
  } else {
    Serial.print("go\n");
    servo1.write(150);
    servo2.write(30);
  }
}
