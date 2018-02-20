#include <Servo.h>

int baud = 9600;

// Constants
int switchPin = 10; // ON / OFF switch

int leftServoPin = 22; // Continuous rotation servo for left wheel
int rightServoPin = 23; // Continuous rotation servo for right wheel

int leftSensorPin = A7; // Sensor under left wheel
int rightSensorPin = A6; // Sensor under right wheel

// The threshold for the boundary lines
// Setting to 100 temporarily. This will also catch the in-boundary lines,
// but we'll deal with that later
int leftSensorBoundaryThreshold = 100;
int rightSensorBoundaryThreshold = 500;

Servo leftServo;
Servo rightServo;

// Initial setup for the Arduino
void setup() {
  Serial.begin(baud);
  pinMode(switchPin, INPUT);
  leftServo.attach(leftServoPin);
  rightServo.attach(rightServoPin);
}

int stopped = 0;
// Code that continuously runs on Arduino
void loop() {
  // If switch is in the on position, stop doing everything
  if (digitalRead(switchPin) == HIGH) {
    stopDriving();
    return;
  }
  if (stopped) {
    return;
  }

  // Switch is in ON position. Keep doing everything...
  if (inBounds()) {
    drive(); // Drive if we're in bounds
  } else {
    stopped = 1;
    stopDriving(); // Stop driving if we've hit a boundary line
  }
}

// Returns 1 if the robot is inside the boundary lines, 0 otherwise
int inBounds() {
  int leftSensorVal = analogRead(leftSensorPin);
  int rightSensorVal = analogRead(rightSensorPin);
//    Serial.println(leftSensorVal);
//  Serial.println(analogRead(rightSensorPin));
  return leftSensorVal > leftSensorBoundaryThreshold && rightSensorVal > rightSensorBoundaryThreshold;
}

// Drive the robot forward at a constant speed
void drive() {
  leftServo.write(20);
  rightServo.write(160);
}

// Stop the robot from driving
void stopDriving() {
  leftServo.write(160);
  rightServo.write(20);
  delay(1000);
  leftServo.write(90);
  rightServo.write(90);
}

