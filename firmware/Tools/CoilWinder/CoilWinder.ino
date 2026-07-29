/*
  ESP32 Coil Winder Controller

  Active-low button controls a STEP/DIR stepper driver:
    - Hold    -> ramps up to top speed, keeps spinning
    - Release -> decelerates to a stop (driver stays enabled)
    - Press again -> resumes ramping from current speed
    - Turn limit reached -> decelerates, disables driver; release + press
      again to re-enable and start the next coil
*/

#include <Arduino.h>

// ------------------------- USER CONFIGURATION -------------------------

// --- Pins ---
const int PIN_STEP    = 25;
const int PIN_DIR     = 26;
const int PIN_ENABLE  = 33;
const int PIN_BUTTON  = 0;   // active LOW

const bool ENABLE_ACTIVE_LOW = true;   // most drivers enable on EN = LOW
const int WIND_DIRECTION = LOW;

// --- Motor / mechanical ---
const int STEPS_PER_REV  = 200;   // full steps/rev (1.8 deg motor)
const int MICROSTEPS     = 16;
const long STEPS_PER_TURN = (long)STEPS_PER_REV * MICROSTEPS;

// --- Speed / acceleration profile, in turns/sec and turns/sec^2 ---
const float TOP_SPEED_TPS   = 7.0;   // cruise speed
const float ACCEL_TPS2      = 1.5;   // acceleration ramping up
const float BRAKE_TPS2      = 25.0;  // deceleration if button is released before at limit
const float DECEL_TPS2      = 0.5;  // deceleration when reaching turn limit
const float START_SPEED_TPS = 0.1;   // initial speed on button press
const float MIN_SPEED_TPS   = 0.1;    // treat as stopped below this during deceleration

// Converted to steps/sec and steps/sec^2 for the step-timing code below
const float TOP_SPEED_SPS   = TOP_SPEED_TPS * STEPS_PER_TURN;
const float ACCEL_SPS2      = ACCEL_TPS2 * STEPS_PER_TURN;
const float BRAKE_SPS2      = BRAKE_TPS2 * STEPS_PER_TURN;
const float DECEL_SPS2      = DECEL_TPS2 * STEPS_PER_TURN;
const float START_SPEED_SPS = START_SPEED_TPS * STEPS_PER_TURN;
const float MIN_SPEED_SPS   = MIN_SPEED_TPS * STEPS_PER_TURN;

// --- Winding target ---
const long DESIRED_TURNS = 475;
const long TARGET_STEPS = DESIRED_TURNS * STEPS_PER_TURN;

// --- Misc ---
const unsigned int STEP_PULSE_US = 2;
const unsigned long DEBOUNCE_MS = 25;
const bool ENABLE_SERIAL_DEBUG = true;
const unsigned long SERIAL_BAUD = 115200;

// ------------------------ END USER CONFIGURATION -----------------------

enum WinderState { IDLE, ACCEL_RUN, DECEL_TO_IDLE, DECEL_TO_LIMIT, LIMIT_REACHED };
WinderState state = IDLE;

float currentSpeed = 0.0;   // steps/sec
unsigned long lastSpeedUpdateUs = 0;
unsigned long lastStepUs = 0;
long stepCount = 0;
bool releasedSinceLimit = false;

int stableButtonState = HIGH;
int lastRawButtonState = HIGH;
unsigned long lastDebounceTime = 0;

void setEnabled(bool enabled) {
  bool level = ENABLE_ACTIVE_LOW ? !enabled : enabled;
  digitalWrite(PIN_ENABLE, level ? HIGH : LOW);
}

void setup() {
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_ENABLE, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  digitalWrite(PIN_STEP, LOW);
  digitalWrite(PIN_DIR, WIND_DIRECTION);
  setEnabled(true);

  if (ENABLE_SERIAL_DEBUG) {
    Serial.begin(SERIAL_BAUD);
    Serial.println("Coil winder ready.");
    Serial.print("Target steps for ");
    Serial.print(DESIRED_TURNS);
    Serial.print(" turns: ");
    Serial.println(TARGET_STEPS);
  }

  lastSpeedUpdateUs = micros();
  lastStepUs = micros();
}

bool buttonPressed() {
  int raw = digitalRead(PIN_BUTTON);

  if (raw != lastRawButtonState) {
    lastDebounceTime = millis();
    lastRawButtonState = raw;
  }
  if ((millis() - lastDebounceTime) > DEBOUNCE_MS) {
    stableButtonState = raw;
  }
  return (stableButtonState == LOW);
}

void doStep() {
  digitalWrite(PIN_STEP, HIGH);
  delayMicroseconds(STEP_PULSE_US);
  digitalWrite(PIN_STEP, LOW);
  stepCount++;
}

// Steps needed to decelerate from a given speed to zero at DECEL_SPS2
float stepsToStopFrom(float speed) {
  return (speed * speed) / (2.0 * DECEL_SPS2);
}

void updateSpeed(unsigned long nowUs, float ratePerSec2, bool rampingUp) {
  unsigned long dtUs = nowUs - lastSpeedUpdateUs;
  lastSpeedUpdateUs = nowUs;
  float dt = dtUs;
  dt /= 1000000.0;

  if (rampingUp) {
    currentSpeed += ratePerSec2 * dt;
    if (currentSpeed > TOP_SPEED_SPS) currentSpeed = TOP_SPEED_SPS;
  } else {
    currentSpeed -= ratePerSec2 * dt;
    if (currentSpeed < 0.0) currentSpeed = 0.0;
  }
}

bool stepIfDue(unsigned long nowUs) {
  if (currentSpeed <= 0.0) return false;
  unsigned long stepIntervalUs = (unsigned long)(1000000.0 / currentSpeed);
  if (nowUs - lastStepUs >= stepIntervalUs) {
    lastStepUs = nowUs;
    doStep();
    return true;
  }
  return false;
}

void loop() {
  bool pressed = buttonPressed();
  unsigned long nowUs = micros();

  switch (state) {

    case IDLE: {
      currentSpeed = 0.0;
      if (pressed) {
        currentSpeed = START_SPEED_SPS;
        lastSpeedUpdateUs = nowUs;
        lastStepUs = nowUs;
        state = ACCEL_RUN;
        if (ENABLE_SERIAL_DEBUG) Serial.println("Button pressed: starting ramp-up.");
      }
      break;
    }

    case ACCEL_RUN: {
      if (!pressed) {
        lastSpeedUpdateUs = nowUs;
        state = DECEL_TO_IDLE;
        if (ENABLE_SERIAL_DEBUG) Serial.println("Button released: decelerating to a stop.");
        break;
      }

      updateSpeed(nowUs, ACCEL_SPS2, true);

      // Start slowing early enough to land on the turn target
      long remainingSteps = TARGET_STEPS - stepCount;
      if (remainingSteps <= (long)ceil(stepsToStopFrom(currentSpeed))) {
        lastSpeedUpdateUs = nowUs;
        state = DECEL_TO_LIMIT;
        if (ENABLE_SERIAL_DEBUG) Serial.println("Approaching turn limit: decelerating.");
        break;
      }

      stepIfDue(nowUs);
      break;
    }

    case DECEL_TO_IDLE: {
      if (pressed) {
        lastSpeedUpdateUs = nowUs;
        state = ACCEL_RUN;
        if (ENABLE_SERIAL_DEBUG) Serial.println("Button pressed again: resuming ramp-up.");
        break;
      }

      updateSpeed(nowUs, BRAKE_SPS2, false);
      if (currentSpeed <= MIN_SPEED_SPS) {
        currentSpeed = 0.0;
        state = IDLE;
        if (ENABLE_SERIAL_DEBUG) Serial.println("Motor stopped (driver still enabled).");
        break;
      }

      stepIfDue(nowUs);
      break;
    }

    case DECEL_TO_LIMIT: {
      // Ignore the button here; finish slowing down, then disable the driver
      updateSpeed(nowUs, DECEL_SPS2, false);

      if (currentSpeed <= MIN_SPEED_SPS || stepCount >= TARGET_STEPS) {
        currentSpeed = 0.0;
        setEnabled(false);
        releasedSinceLimit = !pressed;
        state = LIMIT_REACHED;
        if (ENABLE_SERIAL_DEBUG) {
          Serial.print("Turn limit reached at step ");
          Serial.print(stepCount);
          Serial.println(". Driver disabled.");
          Serial.println("Release the button, then press again to wind the next coil.");
        }
        break;
      }

      if (stepCount < TARGET_STEPS) {
        stepIfDue(nowUs);
      }
      break;
    }

    case LIMIT_REACHED: {
      currentSpeed = 0.0;

      if (!pressed) {
        releasedSinceLimit = true;
      } else if (releasedSinceLimit) {
        stepCount = 0;
        setEnabled(true);
        currentSpeed = START_SPEED_SPS;
        lastSpeedUpdateUs = nowUs;
        lastStepUs = nowUs;
        releasedSinceLimit = false;
        state = ACCEL_RUN;
        if (ENABLE_SERIAL_DEBUG) Serial.println("Driver re-enabled: winding next coil.");
      }
      break;
    }
  }
}
