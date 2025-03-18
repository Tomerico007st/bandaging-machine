#include <Arduino.h>
#line 1 "C:\\Users\\david\\OneDrive\\Desktop\\bandaging-machine\\bandaging-machine.ino"
#include <AccelStepper.h>

#define MOTOR1_STEP_PIN 2
#define MOTOR1_DIR_PIN 5
#define MOTOR1_ENABLE_PIN 8

#define MOTOR2_STEP_PIN 3
#define MOTOR2_DIR_PIN 6
#define MOTOR2_ENABLE_PIN 8

#define STEPS_PER_REV 800

AccelStepper motor1(AccelStepper::DRIVER, MOTOR1_STEP_PIN, MOTOR1_DIR_PIN);
AccelStepper motor2(AccelStepper::DRIVER, MOTOR2_STEP_PIN, MOTOR2_DIR_PIN);

#line 16 "C:\\Users\\david\\OneDrive\\Desktop\\bandaging-machine\\bandaging-machine.ino"
void setup();
#line 32 "C:\\Users\\david\\OneDrive\\Desktop\\bandaging-machine\\bandaging-machine.ino"
void loop();
#line 16 "C:\\Users\\david\\OneDrive\\Desktop\\bandaging-machine\\bandaging-machine.ino"
void setup() {
  pinMode(MOTOR1_ENABLE_PIN, OUTPUT);
  pinMode(MOTOR2_ENABLE_PIN, OUTPUT);
  digitalWrite(MOTOR1_ENABLE_PIN, LOW);
  digitalWrite(MOTOR2_ENABLE_PIN, LOW);


  motor1.setMaxSpeed(1000);
  motor1.setAcceleration(1000);
  motor1.moveTo(STEPS_PER_REV);

  motor2.setMaxSpeed(1000);
  motor2.setAcceleration(1000);
  motor2.moveTo(STEPS_PER_REV);
}

void loop() {
  motor1.run();
  motor2.run();

  if (motor1.distanceToGo() == 0) {
    motor1.moveTo(-motor1.currentPosition());
  }
  if (motor2.distanceToGo() == 0) {
    motor2.moveTo(-motor2.currentPosition());
  }
}
