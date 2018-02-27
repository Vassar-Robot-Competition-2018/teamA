#include <Servo.h>

// Make sure Pixy module is installed
#include <SPI.h>
#include <Pixy.h>

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

// Servos
Servo leftServo;
Servo rightServo;

// Pixy camera
Pixy pixy;

enum states {
  off,        // Switch off
  foundBlock, // Found a block
  searching,  // Searching for a block (driving straight)
  rotating .  // Rotating a random amount
};

// Initial setup for the Arduino
void setup() {
  Serial.begin(baud);

  // Initialize our on/off switch
  pinMode(switchPin, INPUT);

  // Initialize our continuous rotation servos
  leftServo.attach(leftServoPin);
  rightServo.attach(rightServoPin);

  // Initialize Pixy camera
  pixy.init();
}

// Code that continuously runs on Arduino
void loop() {
  // If switch is in the on position, stop doing everything
  if (digitalRead(switchPin) == HIGH) {
    stopDriving();
    return;
  }

  // Switch is in ON position. Keep doing everything...
  if (inBounds()) {
    drive();
    showCameraInfo();
  } else {
    stopDriving(); // Stop driving if we've hit a boundary line
  }
}

// Returns 1 if the robot is inside the boundary lines, 0 otherwise
int inBounds() {
  int leftSensorVal = analogRead(leftSensorPin);
  int rightSensorVal = analogRead(rightSensorPin);
  return (leftSensorVal > leftSensorBoundaryThreshold
          && rightSensorVal > rightSensorBoundaryThreshold);
}

// Drive the robot forward at a constant speed
void drive() {
//  leftServo.write(20);
//  rightServo.write(160);
}

// Stop the robot from driving
void stopDriving() {
//  leftServo.write(90);
//  rightServo.write(90);
}

// http://www.cmucam.org/projects/cmucam5/wiki/Hooking_up_Pixy_to_a_Microcontroller_(like_an_Arduino)
void showCameraInfo() {
  static int i = 0;
  int j;
  uint16_t blocks;
  char buf[32];

  // grab blocks!
  blocks = pixy.getBlocks();

  // If there are detect blocks, print them!;
  if (blocks) {
    i++;

    // do this (print) every 50 frames because printing every
    // frame would bog down the Arduino
    if (i % 50 == 0) {
      sprintf(buf, "Detected %d:\n", blocks);
      Serial.print(buf);
      for (j = 0; j < blocks; j++) {
        sprintf(buf, "  block %d: ", j);
        Serial.print(buf);
        pixy.blocks[j].print();
      }
    }
  }
}

