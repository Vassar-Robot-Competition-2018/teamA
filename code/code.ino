// Color sensor module which allows for multiple sensors
// https://github.com/Fire7/Adafruit_TCS34725_SoftI2C
#include <Adafruit_TCS34725softi2c.h>

// Pixy module
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
int leftFrontServoPin = 7; // Continuous rotation servo for left front wheel
int rightFrontServoPin = 9; // Continuous rotation servo for right front wheel
int leftBackServoPin = 6; // Continuous rotation servo for left back wheel
int rightBackServoPin = 8; // Continuous rotation servo for right back wheel
int sorterMechanismServoPin = 12; // Flippy Floppy McDoodle if Bad Block
int doorMechanismServoPin = 13; //Door says come in!! or Don't!!
int blockPresent = 0; // We start out without a block in the chamber.

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
Servo leftFrontServo;
Servo rightFrontServo;
Servo leftBackServo;
Servo rightBackServo;
Servo sorterMechanismServo;
Servo doorMechanismServo;

// Pixy camera
Pixy pixy;

// Color sensor underneith body
Adafruit_TCS34725softi2c tcsFunnel = Adafruit_TCS34725softi2c(
                                       TCS34725_INTEGRATIONTIME_24MS,
                                       TCS34725_GAIN_1X,
                                       38 /* SDA */, 39 /* SCL */);

// Ground color sensors
Adafruit_TCS34725softi2c tcsRight = Adafruit_TCS34725softi2c(
                                      TCS34725_INTEGRATIONTIME_24MS,
                                      TCS34725_GAIN_1X,
                                      36 /* SDA */, 37 /* SCL */);
Adafruit_TCS34725softi2c tcsLeft = Adafruit_TCS34725softi2c(
                                     TCS34725_INTEGRATIONTIME_24MS,
                                     TCS34725_GAIN_1X,
                                     34 /* SDA */, 35 /* SCL */);

rgb_t lastRGBSeen;

// States
int currentState;
unsigned long stateStartTime;

// Spin state info
int SPIN_BACKUP = 2;
int backupTime;
int SPIN_SPIN = 3;
int spinTime;

// Push state info
int PUSH = 4;

// Out of bound states
int LEFT_OUT = 40;
int RIGHT_OUT = 41;
int BOTH_OUT = 42;

void setup() {
  Serial.begin(baud);
  Serial.print("Starting...\n");

  // Initialize our continuous rotation servos
  leftFrontServo.attach(leftFrontServoPin);
  rightFrontServo.attach(rightFrontServoPin);
  leftBackServo.attach(leftBackServoPin);
  rightBackServo.attach(rightBackServoPin);

  sorterMechanismServo.attach(sorterMechanismServoPin);
  doorMechanismServo.attach(doorMechanismServoPin);

  setSorterClosed();
//  delay(1000);
//  setSorterOpen();
//  delay(10000);

  // Initialize Pixy camera
  //  pixy.init();

  // Init all color sensors
  if (tcsLeft.begin()) {
    Serial.println("tcsLeft working");
  } else {
    error("Couldn't find tcsLeft");
  }
  if (tcsRight.begin()) {
    Serial.println("tcsRight working");
  } else {
    error("Couldn't find tcsRight");
  }
  if (tcsFunnel.begin()) {
    Serial.println("tcsFunnel working");
  } else {
    error("Couldn't find tcsFunnel");
  }

  // Set RGB Output
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  setRGBLed(setLastRGBSeen(255, 255, 255));

  // Set initial state
  setState(PUSH);
}

//Code that continuously runs on Arduino
void loop() {
  // Things to do every time no matter what
  uint16_t rF, bF, gF, cF, rR, gR, bR, cR, rL, gL, bL, cL;
  tcsRight.getRawData(&rR, &gR, &bR, &cR);
  tcsLeft.getRawData(&rL, &gL, &bL, &cL);
  tcsFunnel.getRawData(&rF, &gF, &bF, &cF);

  printCurrentState();

  printBlockColors(rF, gF, bF, cF);

  int outOfBounds = isOutOfBounds(rL, gL, bL, cL, rR, gR, bR, cR);

  setRGBLed(lastRGBSeen);
  if (currentState == PUSH) {
    push(outOfBounds);
  

    if(blockPresent && isRedBlock(rF, gF, bF, cF)) {
      Serial.print("THERE IS A BLOCK");
      setDoorOpen();
      delay(1000);
//      setSorterOpen();
    } else if(blockPresent) {
      setSorterOpen();
      delay(500);
    } else {
      setSorterClosed();
      setDoorClosed();
    }
    
  } else if (isSpinning()) {
    if (currentState == SPIN_BACKUP) {
      spinBackup();
    } else if (currentState == SPIN_SPIN) {
      spin();
    } else {
      error("INVALID SPIN STATE");
    }
  } else {
    error("INVALID STATE");
  }
}

void error(String message) {
  Serial.print("ERROR: ");
  Serial.println(message);
  stopDriving();
  while (1); // Stop doing everything
}

void printCurrentState() {
  Serial.print("Current State: ");
  switch (currentState) {
    case 2:
      Serial.println("SPIN_BACKUP");
      break;
    case 3:
      Serial.println("SPIN_SPIN");
      break;
    case 4:
      Serial.println("PUSH");
      break;
    default:
      Serial.print(currentState);
      Serial.println(": INVALID STATE");
  }
}

bool isSpinning() {
  return currentState == SPIN_SPIN || currentState == SPIN_BACKUP;
}

// Returns true if the robot is inside the boundary lines, false otherwise
int isOutOfBounds(uint16_t rL, uint16_t gL, uint16_t bL, uint16_t cL,
                  uint16_t rR, uint16_t gR, uint16_t bR, uint16_t cR) {
  boolean leftOutOfBounds = cL > 1200;
  boolean rightOutOfBounds = cR > 1200;

  Serial.print("cL: "); Serial.print(cL); Serial.print("; cR: "); Serial.println(cR);

  if (leftOutOfBounds && rightOutOfBounds) {
    return BOTH_OUT;
  } else if (leftOutOfBounds) {
    return LEFT_OUT;
  } else if (rightOutOfBounds) {
    return RIGHT_OUT;
  } else {
    return 0;
  }
}

void printBlockColors(uint16_t rF, uint16_t gF, uint16_t bF, uint16_t cF) {
  // MAKE SURE YELLOW IS HECKED FIRST!!!
  
  
  if (isYellowBlock(rF, gF, bF, cF)) {
    Serial.println("YELLOW!");
    blockPresent = 1;
  } else if (isRedBlock(rF, gF, bF, cF)) {
    Serial.println("RED!");
    blockPresent = 1;
  } else if (isBlueBlock(rF, gF, bF, cF)) {
    Serial.println("BLUE!");
    blockPresent = 1;
  } else if (isGreenBlock(rF, gF, bF, cF)) {
    Serial.println("GREEN!");
    blockPresent = 1;
  } else {
    Serial.println("NO BLOCK...");
    blockPresent = 0;
  }

 
}

boolean isRedBlock(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  // red greater than green & blue
  // green & blue within 30 of eachother
  return r > g && r > b && c > 200;
}

boolean isBlueBlock(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  // blue is greatest
  // red is least
  return b > g && g > r && c > 400;
}

boolean isYellowBlock(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  // red most
  // green middle
  // blue least
  // red & green within 200 of eachother
  return r > g && g > b && c > 400;
}

boolean isGreenBlock(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  // green greatest
  // red & blue within 20 of eachother
  return g > b && g > r && c > 400;
}

//void printColors() {
//  uint16_t rR, gR, bR, cR, rL, gL, bL, cL;
//  tcsRight.getRawData(&rR, &gR, &bR, &cR);
//  tcsLeft.getRawData(&rL, &gL, &bL, &cL);
//  //  Serial.print("R: "); Serial.print(rL, DEC); Serial.print(" ");
//  //  Serial.print("G: "); Serial.print(gL, DEC); Serial.print(" ");
//  //  Serial.print("B: "); Serial.print(bL, DEC); Serial.print(" ");
//  //  Serial.print("C: "); Serial.print(cL, DEC); Serial.print(" ");
//  //  Serial.println(" ");
//  //  Serial.print("R: "); Serial.print(rR, DEC); Serial.print(" ");
//  //  Serial.print("G: "); Serial.print(gR, DEC); Serial.print(" ");
//  //  Serial.print("B: "); Serial.print(bR, DEC); Serial.print(" ");
//  //  Serial.print("C: "); Serial.print(cR, DEC); Serial.print(" ");
//  //  Serial.println(" ");
//  if (isGreen(rR, gR, bR) || isGreen(rL, gL, bL)) {
//    Serial.println("GREEN!");
//    setLastRGBSeen(0, 255, 0);
//  } else if (isRed(rR, gR, bR) || isRed(rL, gL, bL)) {
//    Serial.println("RED!");
//    setLastRGBSeen(255, 0, 0);
//  } else if (isBlue(rR, gR, bR) || isBlue(rL, gL, bL)) {
//    Serial.println("BLUE!");
//    setLastRGBSeen(0, 0, 255);
//  } else if (isYellow(rR, gR, bR) || isYellow(rL, gL, bL)) {
//    Serial.println("YELLOW!");
//    setLastRGBSeen(255, 255, 0);
//  }
//}

boolean isRed(uint16_t r, uint16_t g, uint16_t b) {
  // red greater than green & blue
  // green & blue within 30 of eachother
  return r > 300 && g < 150 && b < 150;
}

boolean isBlue(uint16_t r, uint16_t g, uint16_t b) {
  // blue is greatest
  // red is least
  return b > g && g > r;
}

boolean isYellow(uint16_t r, uint16_t g, uint16_t b) {
  // red most
  // green middle
  // blue least
  // red & green within 200 of eachother
  return r > 700 && g > 600 && b < 500;
}

boolean isGreen(uint16_t r, uint16_t g, uint16_t b) {
  // green greatest
  // red & blue within 20 of eachother
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
  //  setFrontFlapDown();
  spinTime = time;
  spin();
}

// Continue spinning right
void spin() {
  int timeLeft = spinTime - (millis() - stateStartTime);
  Serial.println("SPINNING!");
  Serial.print(millis());
  Serial.print(" ");
  Serial.println(stateStartTime);
  Serial.print("TIME LEFT: ");
  Serial.println(timeLeft);

  if (timeLeft <= 0) {
    setState(PUSH);
    return;
  }
  if (currentState != SPIN_SPIN) {
    setState(SPIN_SPIN);
  }
  setSpeed(-100, 100);
}

void startSpinBackup(int time) {
  //  setFrontFlapUp();
  backupTime = time;
  spinBackup();
}

void spinBackup() {
  if (currentState != SPIN_BACKUP) {
    setState(SPIN_BACKUP);
  }
  int timeLeft = backupTime - (millis() - stateStartTime);
  Serial.print("TIME LEFT: ");
  Serial.println(timeLeft);
  if (timeLeft <= 0) {
    startSpinning(randomInt(2000, 3500));
    return;
  }
  setSpeed(-100);
}

void push(int outOfBounds) {
  if (currentState != PUSH) {
    setState(PUSH);
  }

  if (!outOfBounds) {
    setSpeed(100);
  } else {
    startSpinBackup(1000);
  }
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

int speedToDegrees(int speed) {
  int deg = (speed * 0.6) + 90;
  return deg;
}

// Reverse full speed -100, Forward full speed 100
void setLeftSpeed(int speed) {
  int degrees = 180 - speedToDegrees(speed) + 3;
  //  Serial.print("Left Degrees: "); Serial.println(degrees);
  leftFrontServo.write(degrees);
  leftBackServo.write(degrees);
}

// Reverse full speed -100, Forward full speed 100
void setRightSpeed(int speed) {
  int degrees = speedToDegrees(speed) + 3;
  //  Serial.print("Right Degrees: "); Serial.println(degrees);
  rightFrontServo.write(degrees);
  rightBackServo.write(degrees);
}

void setState(int state) {
  currentState = state;
  stateStartTime = millis();
}


void setSorterClosed() {
  sorterMechanismServo.write(180);
}

void setSorterOpen() {
  sorterMechanismServo.write(0);
}

void setDoorClosed() {
  doorMechanismServo.write(0);
}

void setDoorOpen() {
  doorMechanismServo.write(180);
}

