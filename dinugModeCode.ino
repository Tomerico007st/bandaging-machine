#include <AccelStepper.h>

// === Motor Pins ===
#define MOTOREV_STEP_PIN 3
#define MOTOREV_DIR_PIN 4
#define MOTOREV_ENABLE_PIN 5

#define MOTORLI_STEP_PIN 6
#define MOTORLI_DIR_PIN 7
#define MOTORLI_ENABLE_PIN 8

#define STEPS_PER_REV 800

// === Rotary Encoder Pins ===
#define ENCODER_CLK A1
#define ENCODER_DT A2

// === Proximity Sensor Pin ===
#define PROXIMITY_PIN A3

// === Motor objects ===
AccelStepper motorrev(AccelStepper::DRIVER, MOTOREV_STEP_PIN, MOTOREV_DIR_PIN);
AccelStepper motorlli(AccelStepper::DRIVER, MOTORLI_STEP_PIN, MOTORLI_DIR_PIN);

// === State ===
volatile int encoderValue = 0;
bool objectDetected = false;
unsigned long lastDebug = 0;
const unsigned long debugInterval = 1000; // 1 sec debug print

void setup() {
  Serial.begin(9600);

  // Motor config
  motorrev.setMaxSpeed(1000);
  motorrev.setAcceleration(1000);
  pinMode(MOTOREV_ENABLE_PIN, OUTPUT);
  digitalWrite(MOTOREV_ENABLE_PIN, LOW);

  motorlli.setMaxSpeed(1000);
  motorlli.setAcceleration(1000);
  pinMode(MOTORLI_ENABLE_PIN, OUTPUT);
  digitalWrite(MOTORLI_ENABLE_PIN, LOW);

  // Sensor config
  pinMode(ENCODER_CLK, INPUT);
  pinMode(ENCODER_DT, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_DT), updateEncoder, CHANGE);

  pinMode(PROXIMITY_PIN, INPUT);
}

void loop() {
  // Stepper run
  motorrev.run();
  motorlli.run();

  // Proximity sensor
  objectDetected = digitalRead(PROXIMITY_PIN) == LOW;

  // Debug info every second
  if (millis() - lastDebug > debugInterval) {
    lastDebug = millis();
    Serial.print("Encoder: ");
    Serial.print(encoderValue);
    Serial.print(" | Proximity: ");
    Serial.println(objectDetected ? "DETECTED" : "CLEAR");
  }
}

void updateEncoder() {
  int MSB = digitalRead(ENCODER_CLK);
  int LSB = digitalRead(ENCODER_DT);
  int encoded = (MSB << 1) | LSB;
  static int lastEncoded = 0;
  int sum = (lastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    encoderValue++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    encoderValue--;

  lastEncoded = encoded;
}
