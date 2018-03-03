// Color sensor module which allows for multiple sensors
// https://github.com/Fire7/Adafruit_TCS34725_SoftI2C
#include <Wire.h>
#include <Adafruit_TCS34725softi2c.h>

// Pixy Module
#include <SPI.h>
#include <Pixy.h>

// Servo module
#include <Servo.h>

int baud = 9600;

// Constants
int switchPin = 10; // ON / OFF switch

int leftServoPin = 23; // Continuous rotation servo for left wheel
int rightServoPin = 22; // Continuous rotation servo for right wheel

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
Adafruit_TCS34725softi2c tcsFunnel = Adafruit_TCS34725softi2c(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_1X, 28 /* SDA */, 29 /* SCL */);

// Ground color sensors
Adafruit_TCS34725softi2c tcsRight = Adafruit_TCS34725softi2c(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_1X, 36 /* SDA */, 37 /* SCL */);
Adafruit_TCS34725softi2c tcsLeft = Adafruit_TCS34725softi2c(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_1X, 44 /* SDA */, 45 /* SCL */);

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

  // Init block color sensor
//  if (!tcsFunnel.begin()) {
//    Serial.println("ERROR: Couldn't find tcsFunnel");
//    delay(100000);
//  }

  // Init right wheel color sensor
  if (!tcsRight.begin()) {
    Serial.println("ERROR: Couldn't find tcsRight");
    delay(100000);
  }

  // Init left wheel color sensor
  if (!tcsLeft.begin()) {
    Serial.println("ERROR: Couldn't find tcsLeft");
    delay(100000);
  }
}

// Code that continuously runs on Arduino
void loop() {
  // If switch is in the off position, stop doing everything
  if (switchOff()) {
    Serial.println("OFF :(");
    inBounds();
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
    int spinTime = randomInt(1500, 3000);
    Serial.println("Spinning for " + String(spinTime) + " ms");
    delay(spinTime);
  }
}

bool switchOff() {
  return digitalRead(switchPin) == HIGH;
}

// Returns true if the robot is inside the boundary lines, false otherwise
bool inBounds() {
  uint16_t rR, gR, bR, cR;
  tcsRight.getRawData(&rR, &gR, &bR, &cR);
  Serial.print("R: "); Serial.print(rR, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(gR, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(bR, DEC); Serial.print(" ");
  Serial.print("C: "); Serial.print(cR, DEC); Serial.print(" ");
  Serial.println(" ");
  uint16_t rL, gL, bL, cL, colorTempL;
  tcsLeft.getRawData(&rL, &gL, &bL, &cL);

  return cR < 3000 && cL < 3000;
}

void printColors() {
  uint16_t r, g, b, c, colorTemp, lux;

  tcsFunnel.getRawData(&r, &g, &b, &c);
  colorTemp = calculateColorTemperature(r, g, b);
  return;
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

int randomInt(int min, int max) {
  return min + rand() % (max + 1 - min);
}

uint16_t calculateColorTemperature(uint16_t r, uint16_t g, uint16_t b) {
  float X, Y, Z;      /* RGB to XYZ correlation      */
  float xc, yc;       /* Chromaticity co-ordinates   */
  float n;            /* McCamy's formula            */
  float cct;

  /* 1. Map RGB values to their XYZ counterparts.    */
  /* Based on 6500K fluorescent, 3000K fluorescent   */
  /* and 60W incandescent values for a wide range.   */
  /* Note: Y = Illuminance or lux                    */
  X = (-0.14282F * r) + (1.54924F * g) + (-0.95641F * b);
  Y = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);
  Z = (-0.68202F * r) + (0.77073F * g) + ( 0.56332F * b);

  /* 2. Calculate the chromaticity co-ordinates      */
  xc = (X) / (X + Y + Z);
  yc = (Y) / (X + Y + Z);

  /* 3. Use McCamy's formula to determine the CCT    */
  n = (xc - 0.3320F) / (0.1858F - yc);

  /* Calculate the final CCT */
  cct = (449.0F * powf(n, 3)) + (3525.0F * powf(n, 2)) + (6823.3F * n) + 5520.33F;

  /* Return the results in degrees Kelvin */
  return (uint16_t)cct;
}

