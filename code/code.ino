// Pixy Module
#include <SPI.h>
#include <Pixy.h>

// Color sensor module
#include <Adafruit_TCS34725.h>

// Servo module
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

// Servos
Servo leftServo;
Servo rightServo;

// Pixy camera
Pixy pixy;

// Color sensor in funnel
Adafruit_TCS34725 tcs = Adafruit_TCS34725();

// Initial setup for the Arduino
void setup() {
  Serial.begin(baud);

  // Initialize our on/off switch
  pinMode(switchPin, INPUT);

  // Initialize our continuous rotation servos
  leftServo.attach(leftServoPin);
  rightServo.attach(rightServoPin);

  // Initialize Pixy camera
  //  pixy.init();

  tcs.begin();
}

// Code that continuously runs on Arduino
void loop() {
  // If switch is in the off position, stop doing everything
  if (switchOff()) {
    printColors();
    stopDriving();
    return;
  }
  // Switch is in ON position. Keep doing everything...
  Serial.println("ON!");
  if (inBounds()) {
    Serial.println("DRIVE!");
    drive();
    printColors();
  } else {
    Serial.println("BOUNDARY!");
    backup();
    delay(1000);
    spin();
    int spinTime = random_int(1500, 3000);
    Serial.println("Spinning for " + String(spinTime) + " ms");
    delay(spinTime);
  }
}

bool switchOff() {
  return digitalRead(switchPin) == HIGH;
}

// Returns true if the robot is inside the boundary lines, false otherwise
bool inBounds() {
  int leftSensorVal = analogRead(leftSensorPin);
  int rightSensorVal = analogRead(rightSensorPin);
  return (leftSensorVal > ledeftSensorBoundaryThreshold
          && rightSensorVal > rightSensorBoundaryThreshold);
}

void printColors() {
  uint16_t r, g, b, c, colorTemp, lux;

  tcs.getRawData(&r, &g, &b, &c);
  colorTemp = tcs.calculateColorTemperature(r, g, b);

  if (colorTemp >= 8000 && colorTemp <=8500) {
    Serial.println("NO BLOCK!!!");
  } else {
    Serial.println("BLOCK!!!");
    stopDriving();
    delay(4000);
  }
}

// Drive the robot forward at a constant speed
void drive() {
  leftServo.write(150);
  rightServo.write(30);
}

// Stop the robot from driving
void stopDriving() {
  leftServo.write(90);
  rightServo.write(90);
}

// Backup
void backup() {
  leftServo.write(30);
  rightServo.write(150);
}

// Spin right
void spin() {
  leftServo.write(30);
  rightServo.write(30);
}

int random_int(int min, int max) {
  return min + rand() % (max + 1 - min);
}
