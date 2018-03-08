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

int frontFlapServoPin = 47;

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

Servo frontFlapServo;

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

// Initial setup for the Arduino
void setup() {
  Serial.begin(baud);

  // Initialize our on/off switch
  pinMode(switchPin, INPUT);

  // Initialize our continuous rotation servos
  leftServo.attach(leftServoPin);
  rightServo.attach(rightServoPin);

  frontFlapServo.attach(frontFlapServoPin);

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
  setState(PUSH);

  setFrontFlapDown();
}

// Code that continuously runs on Arduino
void loop() {
  // Things to do every time no matter what
  printCurrentState();
  printColors();
  Serial.print("SERVO: ");
  Serial.println(frontFlapServo.read());
  setRGBLed(lastRGBSeen);
  
  // If switch is in the off position, stop doing everything
  if (switchOff()) {
//    Serial.println("OFF :(");
    setState(PUSH);
    stopDriving();
    return;
  }
  
  // Switch is in ON position. Keep doing everything...
  Serial.println("ON!");
  if (currentState == PUSH) {
    push();
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
  Serial.println(message);
  stopDriving();
  delay(2500);
}

void printCurrentState() {
  Serial.print("Current State: ");
  switch(currentState) {
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

bool switchOff() {
  return digitalRead(switchPin) == HIGH;
}

// Returns true if the robot is inside the boundary lines, false otherwise
int isOutOfBounds() {
  uint16_t rR, gR, bR, cR, rL, gL, bL, cL;
  tcsRight.getRawData(&rR, &gR, &bR, &cR);
  tcsLeft.getRawData(&rL, &gL, &bL, &cL);

  boolean leftOutOfBounds = cL > 4300;
  boolean rightOutOfBounds = cR > 3000;

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

void printColors() {
  uint16_t rR, gR, bR, cR, rL, gL, bL, cL;
  tcsRight.getRawData(&rR, &gR, &bR, &cR);
  tcsLeft.getRawData(&rL, &gL, &bL, &cL);
//  Serial.print("R: "); Serial.print(rL, DEC); Serial.print(" ");
//  Serial.print("G: "); Serial.print(gL, DEC); Serial.print(" ");
//  Serial.print("B: "); Serial.print(bL, DEC); Serial.print(" ");
//  Serial.print("C: "); Serial.print(cL, DEC); Serial.print(" ");
//  Serial.println(" ");
//  Serial.print("R: "); Serial.print(rR, DEC); Serial.print(" ");
//  Serial.print("G: "); Serial.print(gR, DEC); Serial.print(" ");
//  Serial.print("B: "); Serial.print(bR, DEC); Serial.print(" ");
//  Serial.print("C: "); Serial.print(cR, DEC); Serial.print(" ");
//  Serial.println(" ");
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
  setFrontFlapDown();
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
  setFrontFlapUp();
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

void push() {
  if (currentState != PUSH) {
    setState(PUSH);
  }

  int outOfBounds = isOutOfBounds();
  if (!outOfBounds) {
    setSpeed(100);
  } else if (outOfBounds == LEFT_OUT) {
    // Stop moving left wheel
    setSpeed(-100, 100);
  } else if (outOfBounds == RIGHT_OUT) {
    // Stop moving right wheel
    setSpeed(100, -100);
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

void setFrontFlapUp() {
  frontFlapServo.write(180);
}

void setFrontFlapDown() {
  frontFlapServo.write(150);
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

