// Color sensor module which allows for multiple sensors
// https://github.com/Fire7/Adafruit_TCS34725_SoftI2C
#include <Adafruit_TCS34725softi2c.h>

Adafruit_TCS34725softi2c tcsFunnel = Adafruit_TCS34725softi2c(
                                       TCS34725_INTEGRATIONTIME_24MS,
                                       TCS34725_GAIN_1X,
                                       38 /* SDA */, 39 /* SCL */);

void setup() {
  Serial.begin(9600);
  Serial.print("Starting...\n");

  if (tcsFunnel.begin()) {
    Serial.println("tcsFunnel working");
  } else {
    error("Couldn't find tcsFunnel");
  }
}

void error(String message) {
  Serial.print("ERROR: ");
  Serial.println(message);
  while (1); // Stop doing everything
}

void loop() {
  // put your main code here, to run repeatedly:
  uint16_t rR, gR, bR, cR, rL, gL, bL, cL;
  tcsFunnel.getRawData(&rR, &gR, &bR, &cR);
  Serial.print("R: "); Serial.print(rR, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(gR, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(bR, DEC); Serial.print(" ");
  Serial.print("C: "); Serial.print(cR, DEC); Serial.print(" ");
  Serial.println(" ");
}
