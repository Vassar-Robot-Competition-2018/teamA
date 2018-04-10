#include <SPI.h>
#include <Pixy.h>

Pixy pixy;

void setup() {
  Serial.begin(9600);
  Serial.print("Starting...\n");

  pixy.init();
}

// Finds the biggest block and returns a pointer to it
Block* biggestBlock(Block** blocks, int len) {
  Block* result;
  int i;

  result = blocks[0];

  for (i = 1; i < len; i++) {
    if (isBlock(&pixy.blocks[i])) {
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

void loop() {
  static int i = 0;
  int j;
  int x;
  float leftSpeed;
  float rightSpeed;
  uint16_t blocks;

  blocks = pixy.getBlocks();
  i++;

  if (i % 40 == 0 && blocks) {
    Block* block = biggestBlock(&pixy.blocks, blocks);
    block->print();
    // center is 160
    x = block->x - 160;
    leftSpeed = min(100, 100 - 100 * (x / -320.0));
    rightSpeed = min(100, 100 - 100 * (x / 320.0));
    Serial.print("X: "); Serial.println(x);
    Serial.print("L Speed: "); Serial.println(leftSpeed);
    Serial.print("R Speed: "); Serial.println(rightSpeed);
    Serial.println();
  }
}
