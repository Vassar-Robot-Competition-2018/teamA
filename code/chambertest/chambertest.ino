// Servo module
#include <Servo.h>

int SERVO_PIN = 7;

Servo chamberServo;
int pos;

void setup() {
  Serial.begin(9600);
  Serial.print("Starting...\n");

  chamberServo.attach(SERVO_PIN);
}

void loop() {
  for (pos = 0; pos <= 180; pos += 1) {
    chamberServo.write(pos);
    Serial.println(pos);
    delay(15);
  }
  for (pos = 180; pos >= 0; pos -= 1) {
    chamberServo.write(pos);
    Serial.println(pos);
    delay(15);
  }
}
