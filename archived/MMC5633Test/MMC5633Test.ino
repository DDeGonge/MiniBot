#include <Arduino.h>
#include <Wire.h>
#include "mmc5633.h"

#define SDA_PIN 21 
#define SCL_PIN 22

MMC5633NJL mag;

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("Initializing MMC5633...");
  if (!mag.begin(SDA_PIN, SCL_PIN, 100000)) {
    Serial.println("ERROR: MMC5633 init failed. Check wiring.");
    while (true) delay(1000);
  }
  Serial.println("MMC5633 initialized.");

  Serial.print("Running self-test... ");
  if (mag.runSelfTest()) {
    Serial.println("PASSED");
  } else {
    Serial.println("FAILED");
  }
}

void loop() {
  if (mag.readMeasurement()) {
    Serial.printf("X: %.4f  Y: %.4f  Z: %.4f (Gauss)\n",
                  mag.getFieldGaussX(), mag.getFieldGaussY(), mag.getFieldGaussZ());
  } else {
    Serial.println("Measurement failed.");
  }
  delay(100);
}
