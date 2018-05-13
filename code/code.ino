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

// Debug flag. Set to 1 if we want to print debug info
boolean DEBUG = 1;

// Constants
int leftFrontServoPin = 8; // Continuous rotation servo for left front wheel
int rightFrontServoPin = 10; // Continuous rotation servo for right front wheel
int leftBackServoPin = 9; // Continuous rotation servo for left back wheel
int rightBackServoPin = 11; // Continuous rotation servo for right back wheel
int sorterMechanismServoPin = 5; // Flippy Floppy McDoodle if Bad Block
int doorMechanismServoPin = 6; //Door says come in!! or Don't!!

int blockPresent = 0; // We start out without a block in the chamber.

int COLOR_NULL = 0;
int COLOR_RED = 1;
int COLOR_BLUE = 2;
int COLOR_YELLOW = 3;
int COLOR_GREEN = 4;

int homeColor = COLOR_NULL;
int lastColorLeft = COLOR_NULL;
int lastColorRight = COLOR_NULL;

// The threshold for the boundary lines
// Setting to 100 temporarily. This will also catch the in-boundary lines,
// but we'll deal with that later
int leftSensorBoundaryThreshold = 100;
int rightSensorBoundaryThreshold = 500;

// Our RGB LED
int redPin = 4;
int greenPin = 3;
int bluePin = 2;

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
                                       34 /* SDA */, 35 /* SCL */);

// Ground color sensors
Adafruit_TCS34725softi2c tcsRight = Adafruit_TCS34725softi2c(
                                      TCS34725_INTEGRATIONTIME_24MS,
                                      TCS34725_GAIN_1X,
                                      30 /* SDA */, 31 /* SCL */);
Adafruit_TCS34725softi2c tcsLeft = Adafruit_TCS34725softi2c(
                                     TCS34725_INTEGRATIONTIME_24MS,
                                     TCS34725_GAIN_1X,
                                     32 /* SDA */, 33 /* SCL */);

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

void debugln(String msg) {
  if (DEBUG) {
    Serial.println(msg);
  }
}

void debug(String msg) {
  if (DEBUG) {
    Serial.print(msg);
  }
}

void debugln(long msg) {
  if (DEBUG) {
    Serial.println(msg);
  }
}

void debug(long msg) {
  if (DEBUG) {
    Serial.print(msg);
  }
}

void error(String message) {
  Serial.print("ERROR: ");
  Serial.println(message);
}

void error(long msg) {
  Serial.print("ERROR: ");
  Serial.println(msg);
}

void fail() {
  stopDriving();
  while (1); // Stop doing everything
}

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

  // Initialize sorting mechanism
  setSorterClosed();
  setDoorClosed();

  // Initialize Pixy camera
  pixy.init();

  // Init all color sensors
  if (tcsLeft.begin()) {
    debugln("tcsLeft working");
  } else {
    error("Couldn't find tcsLeft");
    fail();
  }
  if (tcsRight.begin()) {
    debugln("tcsRight working");
  } else {
    error("Couldn't find tcsRight");
    fail();
  }
  if (tcsFunnel.begin()) {
    debugln("tcsFunnel working");
  } else {
    error("Couldn't find tcsFunnel");
    fail();
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

  setQuadrantColors(rR, gR, bR, cR, rL, gL, bL, cL);
  printBlockColors(rF, gF, bF, cF);

  int outOfBounds = isOutOfBounds(rL, gL, bL, cL, rR, gR, bR, cR);

  setRGBLed();
  if (currentState == PUSH) {
    push(outOfBounds);

    if (blockPresent && isHomeBlock(rF, gF, bF, cF)) {
      debug("THERE IS A BLOCK");
      setDoorOpen();
      delay(1000);
    } else if (blockPresent) {
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
      fail();
    }
  } else {
    error("INVALID STATE");
    fail();
  }
}

void printCurrentState() {
  debug("Current State: ");
  switch (currentState) {
    case 2:
      debugln("SPIN_BACKUP");
      break;
    case 3:
      debugln("SPIN_SPIN");
      break;
    case 4:
      debugln("PUSH");
      break;
    default:
      debug(currentState);
      debugln(": INVALID STATE");
  }
}

boolean isHomeBlock(int signature) {
  return signature == homeColor;
}

// Finds the biggest block and returns a pointer to it
Block* findBestBlock(Block** blocks, int len) {
  Block* result;
  int i;

  result = blocks[0];

  for (i = 1; i < len; i++) {
    if (isBlock(&pixy.blocks[i])) {
      int resultArea = result->width * result->height;
      int currentArea = pixy.blocks[i].width * pixy.blocks[i].height;
      if (resultArea < currentArea && isHomeBlock(pixy.blocks[i].signature)) {
        result = &pixy.blocks[i];
      }
    }
  }

  return result;
}

boolean isBlock(Block* block) {
  int width = block->width;
  int height = block->height;
  int x = block->x;
  int y = block->y;

  return (width >= 0.9 * height) && (width <= 1.5 * height);
}

bool isSpinning() {
  return currentState == SPIN_SPIN || currentState == SPIN_BACKUP;
}

// Returns true if the robot is inside the boundary lines, false otherwise
int isOutOfBounds(uint16_t rL, uint16_t gL, uint16_t bL, uint16_t cL,
                  uint16_t rR, uint16_t gR, uint16_t bR, uint16_t cR) {
  boolean leftOutOfBounds = cL > 1200;
  boolean rightOutOfBounds = cR > 1200;

  debug("cL: "); debug(cL); debug("; cR: "); debugln(cR);

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

// Set the home color and last seen left and right wheel colors
void setQuadrantColors(uint16_t rL, uint16_t gL, uint16_t bL, uint16_t cL,
                       uint16_t rR, uint16_t gR, uint16_t bR, uint16_t cR) {
  // Set home color if we haven't already
  if (homeColor == COLOR_NULL) {
    if (isYellowLineLeft(rL, gL, bL, cL) || isYellowLineRight(rR, gR, bR, cR)) {
      debugln("YELLOW HOME QUADRANT");
      homeColor = COLOR_YELLOW;
    } else if (isRedLineLeft(rL, gL, bL, cL) || isRedLineRight(rR, gR, bR, cR)) {
      debugln("RED HOME QUADRANT");
      homeColor = COLOR_RED;
    } else if (isBlueLineLeft(rL, gL, bL, cL) || isBlueLineRight(rR, gR, bR, cR)) {
      debugln("BLUE HOME QUADRANT");
      homeColor = COLOR_BLUE;
    } else if (isGreenLineLeft(rL, gL, bL, cL) || isGreenLineRight(rR, gR, bR, cR)) {
      if (isGreenLineLeft(rL, gL, bL, cL)) {
        debugln("GREEN LEFT");
        debug("rL: "); debug(rL); debug(" gL: "); debug(gL); debug(" bL: "); debug(bL); debug(" cL: "); debugln(cL);
      }
      if (isGreenLineRight(rR, gR, bR, cR)) {
        debugln("GREEN RIGHT");
        debug("rR: "); debug(rR); debug(" gR: "); debug(gR); debug(" bR: "); debug(bR); debug(" cR: "); debugln(cR);
      }
      debugln("GREEN HOME QUADRANT");
      homeColor = COLOR_GREEN;
    }
  }

  // Set the last color seen for left wheel
  if (isYellowLineLeft(rL, gL, bL, cL)) {
    lastColorLeft = COLOR_YELLOW;
  } else if (isGreenLineLeft(rL, gL, bL, cL)) {
    lastColorLeft = COLOR_GREEN;
  } else if (isBlueLineLeft(rL, gL, bL, cL)) {
    lastColorLeft = COLOR_BLUE;
  } else if (isRedLineLeft(rL, gL, bL, cL)) {
    lastColorLeft = COLOR_RED;
  }

  // Set last color seen for right wheel
  if (isYellowLineRight(rR, gR, bR, cR)) {
    lastColorRight = COLOR_YELLOW;
  } else if (isGreenLineRight(rR, gR, bR, cR)) {
    lastColorRight = COLOR_GREEN;
  } else if (isBlueLineRight(rR, gR, bR, cR)) {
    lastColorRight = COLOR_BLUE;
  } else if (isRedLineRight(rR, gR, bR, cR)) {
    lastColorRight = COLOR_RED;
  }
}

void printBlockColors(uint16_t rF, uint16_t gF, uint16_t bF, uint16_t cF) {
  if (isYellowBlock(rF, gF, bF, cF)) {
    debugln("YELLOW BLOCK!");
    blockPresent = 1;
  } else if (isRedBlock(rF, gF, bF, cF)) {
    debugln("RED BLOCK!");
    blockPresent = 1;
  } else if (isBlueBlock(rF, gF, bF, cF)) {
    debugln("BLUE BLOCK!");
    blockPresent = 1;
  } else if (isGreenBlock(rF, gF, bF, cF)) {
    debugln("GREEN BLOCK!");
    blockPresent = 1;
  } else {
    debugln("NO BLOCK...");
    blockPresent = 0;
  }
}

/** Color detection **/

boolean isRedBlock(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  // red greater than green & blue
  // green & blue within 30 of eachother
  return r > g && r > b && c > 200;
}

boolean isRedLineLeft(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  return isRedBlock(r, g, b, c);
}

boolean isRedLineRight(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  return isRedBlock(r, g, b, c);
}

boolean isBlueBlock(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  // blue is greatest
  // red is least
  return b > g && g > r && c > 400;
}

boolean isBlueLineLeft(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  return isBlueBlock(r, g, b, c);
}

boolean isBlueLineRight(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  return isBlueBlock(r, g, b, c);
}

boolean isYellowBlock(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  // red most
  // green middle
  // blue least
  // red & green within 200 of eachother
  return r > g && g > b && c > 400;
}

boolean isYellowLineLeft(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  return isYellowBlock(r, g, b, c);
}

boolean isYellowLineRight(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  return isYellowBlock(r, g, b, c);
}

boolean isGreenBlock(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  // green greatest
  // red & blue within 20 of eachother
  return g > b && g > r && c > 400 && g > (r + 75) && g > (b + 75);
}

boolean isGreenLineLeft(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  return isGreenBlock(r, g, b, c);
}

boolean isGreenLineRight(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  return isGreenBlock(r, g, b, c);
}

// Returns true if the given block colors are the same as the home
// quadrant color
boolean isHomeBlock(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  if (homeColor == COLOR_NULL) {
    debug("Home color has not been set");
  } else if (homeColor == COLOR_YELLOW && isYellowBlock(r, g, b, c)) {
    return 1;
  } else if (homeColor == COLOR_BLUE && isBlueBlock(r, g, b, c)) {
    return 1;
  } else if (homeColor == COLOR_GREEN && isGreenBlock(r, g, b, c)) {
    return 1;
  } else if (homeColor == COLOR_RED && isRedBlock(r, g, b, c)) {
    return 1;
  } else {
    // Input block is not a color
    return 0;
  }
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
  debugln("SPINNING!");
  debug(millis());
  debug(" ");
  debugln(stateStartTime);
  debug("TIME LEFT: ");
  debugln(timeLeft);

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
  debug("TIME LEFT: ");
  debugln(timeLeft);
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
    //    followBlocks(); // Uses Pixy
  } else {
    startSpinBackup(1000);
  }
}

// Follow blocks based on information from the Pixy camera
void followBlocks() {
  int j;
  int x;
  float leftSpeed;
  float rightSpeed;
  uint16_t blocks;

  blocks = pixy.getBlocks();

  if (blocks) {
    Block* block = findBestBlock(&pixy.blocks, blocks);
    if (isHomeBlock(block->signature)) {
      // block->print();
      // center is 160
      x = block->x - 160;
      debugln(x);
      if (x >= 0) {
        leftSpeed = min(100, 100 - 100 * (sqrt(x) / -sqrt(160.0)));
        rightSpeed = min(100, 100 - 100 * (sqrt(x) / sqrt(160.0)));
      } else {
        leftSpeed = min(100, 100 - 100 * (-sqrt(-x) / -sqrt(160.0)));
        rightSpeed = min(100, 100 - 100 * (-sqrt(-x) / sqrt(160.0)));
      }
      //    debug("X: "); debugln(x);
      //    debug("L Speed: "); debugln(leftSpeed);
      //    debug("R Speed: "); debugln(rightSpeed);
      //    debugln();
      setSpeed(leftSpeed, rightSpeed);
    } else {
      debugln("No home blocks in frame");
      setSpeed(100);
    }
  } else {
    debugln("No blocks");
    setSpeed(100);
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

void setRGBLed() {
  if (homeColor == COLOR_NULL) {
    setRGBLed(setLastRGBSeen(255,255,255));
  } else if (homeColor == COLOR_RED) {
    setRGBLed(setLastRGBSeen(255,0,0));
  } else if (homeColor == COLOR_GREEN) {
    setRGBLed(setLastRGBSeen(0,255,0));
  } else if (homeColor == COLOR_BLUE) {
    setRGBLed(setLastRGBSeen(0,0,255));
  } else if (homeColor == COLOR_YELLOW) {
    setRGBLed(setLastRGBSeen(255,255,0));
  } else {
    setRGBLed(setLastRGBSeen(0,0,0));
    error("lastColorLeft has invalid value");
    error(lastColorLeft);
    fail();
  }
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
  //  debug("Left Degrees: "); debugln(degrees);
  leftFrontServo.write(degrees);
  leftBackServo.write(degrees);
}

// Reverse full speed -100, Forward full speed 100
void setRightSpeed(int speed) {
  int degrees = speedToDegrees(speed) + 3;
  //  debug("Right Degrees: "); debugln(degrees);
  rightFrontServo.write(degrees);
  rightBackServo.write(degrees);
}

void setState(int state) {
  currentState = state;
  stateStartTime = millis();
}


void setSorterClosed() {
  sorterMechanismServo.write(175);
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

