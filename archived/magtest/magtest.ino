#include <Arduino.h>

const int EMAG_CNT = 3;

const int EMAG_PINS_A[EMAG_CNT] = {
  6,
  8,
  18,
};
const int EMAG_PINS_B[EMAG_CNT] = {
  7,
  9,
  19,
};

void setup() {
  for (int i = 0; i < EMAG_CNT; i++) {
    pinMode(EMAG_PINS_A[i], OUTPUT);
    pinMode(EMAG_PINS_B[i], OUTPUT);
  }
  for (int i = 0; i < EMAG_CNT; i++) {
    digitalWrite(EMAG_PINS_A[i], LOW);
    digitalWrite(EMAG_PINS_B[i], LOW);
  }
}
 
void loop() {
  for (int i = 0; i < EMAG_CNT; i++) {
    digitalWrite(EMAG_PINS_A[i], HIGH);
    digitalWrite(EMAG_PINS_B[i], LOW);
  }
  delay(500);
  // for (int i = 0; i < EMAG_CNT; i++) {
  //   digitalWrite(EMAG_PINS_A[i], LOW);
  //   digitalWrite(EMAG_PINS_B[i], LOW);
  // }
  // delay(500);
  // for (int i = 0; i < EMAG_CNT; i++) {
  //   digitalWrite(EMAG_PINS_A[i], LOW);
  //   digitalWrite(EMAG_PINS_B[i], HIGH);
  // }
  // delay(500);
  // for (int i = 0; i < EMAG_CNT; i++) {
  //   digitalWrite(EMAG_PINS_A[i], LOW);
  //   digitalWrite(EMAG_PINS_B[i], LOW);
  // }
  // delay(500);
}
