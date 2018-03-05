// Color sensor module which allows for multiple sensors
// https://github.com/Fire7/Adafruit_TCS34725_SoftI2C
#include <Wire.h>
#include <Adafruit_TCS34725softi2c.h>

// Pixy Module
#include <SPI.h>
#include <Pixy.h>

// Servo module
#include <Servo.h>

// Helpful rgb struct to store rgb values
typedef struct rgb {
  int r;
  int g;
  int b;
} rgb_t;

int baud = 9600;

// Constants
int switchPin = 10; // ON / OFF switch

int leftServoPin = 22; // Continuous rotation servo for left wheel
int rightServoPin = 23; // Continuous rotation servo for right wheel

// The threshold for the boundary lines
// Setting to 100 temporarily. This will also catch the in-boundary lines,
// but we'll deal with that later
int leftSensorBoundaryThreshold = 100;
int rightSensorBoundaryThreshold = 500;

// Our RGB LED
int redPin = 2;
int greenPin = 3;
int bluePin = 4;

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

rgb_t lastRGBSeen;

// States
int currentState;
unsigned long stateStartTime;

// Driving state
int DRIVING = 1;

// Spin state info
int SPINNING = 2;
int spinTime;

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

  // Set RGB Output
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT); 
  setRGBLed(setLastRGBSeen(255,255,255));

  // Set initial state
  setState(DRIVING);
}

// Code that continuously runs on Arduino
void loop() {
  // Things to do every time no matter what
  setRGBLed(lastRGBSeen);
  
  // If switch is in the off position, stop doing everything
  if (switchOff()) {
    Serial.println("OFF :(");
    stopDriving();
    return;
  }
  
  // Switch is in ON position. Keep doing everything...
  Serial.println("ON!");
  if (currentState == SPINNING) {
    Serial.println("SPINNING!");
    Serial.print(millis());
    Serial.print(" ");
    Serial.println(stateStartTime);
    spin();
  } else if (inBounds()) {
    Serial.println("DRIVE!");
    drive();
    printColors();
  } else {
    Serial.println("BOUNDARY!");
    backup();
    delay(1000);
    startSpinning(randomInt(1500, 3000));
  }
}

bool switchOff() {
  return digitalRead(switchPin) == HIGH;
}

// Returns true if the robot is inside the boundary lines, false otherwise
bool inBounds() {
  uint16_t rR, gR, bR, cR, rL, gL, bL, cL;
  tcsRight.getRawData(&rR, &gR, &bR, &cR);
  tcsLeft.getRawData(&rL, &gL, &bL, &cL);

  return cR < 3000 && cL < 3000;
}

void printColors() {
  uint16_t rR, gR, bR, cR, rL, gL, bL, cL;
  tcsRight.getRawData(&rR, &gR, &bR, &cR);
  tcsLeft.getRawData(&rL, &gL, &bL, &cL);
  Serial.print("R: "); Serial.print(rL, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(gL, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(bL, DEC); Serial.print(" ");
  Serial.print("C: "); Serial.print(cL, DEC); Serial.print(" ");
  Serial.println(" ");
  Serial.print("R: "); Serial.print(rR, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(gR, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(bR, DEC); Serial.print(" ");
  Serial.print("C: "); Serial.print(cR, DEC); Serial.print(" ");
  Serial.println(" ");
  if (isGreen(rR,gR,bR) || isGreen(rL,gL,bL)) {
    Serial.println("GREEN!");
    setLastRGBSeen(0, 255, 0);
  } else if (isRed(rR,gR,bR) || isRed(rL,gL,bL)) {
    Serial.println("RED!");
    setLastRGBSeen(255, 0, 0);
  } else if (isBlue(rR,gR,bR) || isBlue(rL,gL,bL)) {
    Serial.println("BLUE!");
    setLastRGBSeen(0, 0, 255);
  } else if (isYellow(rR,gR,bR) || isYellow(rL,gL,bL)) {
    Serial.println("YELLOW!");
    setLastRGBSeen(255, 255, 0);
  }
}

boolean isRed(uint16_t r, uint16_t g, uint16_t b) {
  return r > 300 && g < 150 && b < 150;
}

boolean isBlue(uint16_t r, uint16_t g, uint16_t b) {
  return b > g && g > r;
}

boolean isYellow(uint16_t r, uint16_t g, uint16_t b) {
  return r > 700 && g > 600 && b < 500;
}

boolean isGreen(uint16_t r, uint16_t g, uint16_t b) {
  return g > 500 && b < 500 && r < 500;
}

// Drive the robot forward at a constant speed
void drive() {
  setSpeed(100);
}

// Stop the robot from driving
void stopDriving() {
  setSpeed(0);
}

// Backup
void backup() {
  setSpeed(-100);
}

// Start the robot spinning
void startSpinning(int time) {
  spinTime = time;
  spin();
}

// Continue spinning right
void spin() {
  int timeLeft = spinTime - (millis() - stateStartTime);
  Serial.print("TIME LEFT: ");
  Serial.println(timeLeft);
  if (timeLeft <= 0) {
    setState(DRIVING);
    return;
  }
  if (currentState != SPINNING) {
    setState(SPINNING);
  }
  setSpeed(-100, 100);
}

int randomInt(int min, int max) {
  return min + rand() % (max + 1 - min);
}

rgb_t setLastRGBSeen(uint16_t r, uint16_t g, uint16_t b) {
  lastRGBSeen.r = r;
  lastRGBSeen.g = g;
  lastRGBSeen.b = b;
  return lastRGBSeen;
}

void setRGBLed(rgb_t rgb) {
  // If you are using a common *ANODE LED* instead of common
  // CATHODE, connect the long pin to +5 instead of ground
  // We are using ANODE, invert the color values
  int red = 255 - rgb.r;
  int green = 255 - rgb.g;
  int blue = 255 - rgb.b;
  digitalWrite(redPin, red);
  digitalWrite(greenPin, green);
  digitalWrite(bluePin, blue);
}

void setSpeed(int speed) {
  setLeftSpeed(speed);
  setRightSpeed(speed);
}

void setSpeed(int left, int right) {
  setLeftSpeed(left);
  setRightSpeed(right);
}

// Reverse full speed -100, Forward full speed 100
void setLeftSpeed(int speed) {
  if (speed == 0) {
    leftServo.write(92);
  } else if (speed < 0) {
    leftServo.write(150); // backup
  } else {
    leftServo.write(40); // forward
  }
}

// Reverse full speed -100, Forward full speed 100
void setRightSpeed(int speed) {
  if (speed == 0) {
    rightServo.write(92);
  } else if (speed < 0) {
    rightServo.write(30); // backup
  } else {
    rightServo.write(154); // forward
  }
}

void setState(int state) {
  currentState = state;
  stateStartTime = millis();
}

