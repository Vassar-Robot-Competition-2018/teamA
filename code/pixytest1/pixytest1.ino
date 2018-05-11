#include <SPI.h>
#include <Pixy.h>
#include <Servo.h>

Pixy pixy;

Servo leftServo;
int leftServoPin = 5;

Servo rightServo;
int rightServoPin = 6;

void setup() {
  Serial.begin(9600);
  Serial.print("Starting...\n");

  pixy.init();

  leftServo.attach(leftServoPin);
  rightServo.attach(rightServoPin);
}

// Finds the biggest block and returns a pointer to it
Block* biggestBlock(Block** blocks, int len) {
  Block* result;
  int i;

  result = blocks[0];

  for (i = 1; i < len; i++) {
    if (/*isBlock(*/&pixy.blocks[i]/*)*/) {
      int resultArea = result->width * result->height;
      int currentArea = pixy.blocks[i].width * pixy.blocks[i].height;
      if (resultArea < currentArea) {
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

  // Return true if the width of a block is between .9 and 1.6 the height
  return (width >= 0.9 * height) && (width <= 1.6 * height);
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
  leftServo.write(degrees);
}

// Reverse full speed -100, Forward full speed 100
void setRightSpeed(int speed) {
  int degrees = speedToDegrees(speed) + 3;
//  Serial.print("Right Degrees: "); Serial.println(degrees);
  rightServo.write(degrees);
}

void loop() {
  static int i = 0;
  int j;
  int x;
  float leftSpeed;
  float rightSpeed;
  uint16_t blocks;

  blocks = pixy.getBlocks();
  delay(30);
  i++;

  if (/*i % 40 == 0 && */blocks) {
    Block* block = biggestBlock(&pixy.blocks, blocks);
    // block->print();
    // center is 160
    x = block->x - 160;
    Serial.println(x);
    if (x >= 0) {
      leftSpeed = min(100, 100 - 100 * (sqrt(x) / -sqrt(160.0)));
      rightSpeed = min(100, 100 - 100 * (sqrt(x) / sqrt(160.0)));
    } else {
      leftSpeed = min(100, 100 - 100 * (-sqrt(-x) / -sqrt(160.0)));
      rightSpeed = min(100, 100 - 100 * (-sqrt(-x) / sqrt(160.0)));
    }
    //    Serial.print("X: "); Serial.println(x);
    //    Serial.print("L Speed: "); Serial.println(leftSpeed);
    //    Serial.print("R Speed: "); Serial.println(rightSpeed);
    //    Serial.println();
    setSpeed(leftSpeed, rightSpeed);
  } else {
    Serial.println("No blocks");
    setSpeed(0);
  }
}
