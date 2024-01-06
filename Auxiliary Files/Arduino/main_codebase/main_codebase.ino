//
// MIT License
//
// Copyright (c) 2023 Leander Stephen Desouza
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include <Servo.h>
#include <time.h>

const int ENC_K = 32; // Number of pulses per revolution
const int GEAR_RATIO = 120; // Gear ratio of the motor
const int leftPINA = 18; // Pin allocation for the encoder
const int leftPINB = 19;
const int rightPINA = 2;
const int rightPINB = 3;

const int pinBackAI1 = 25; // back left motor
const int pinBackAI2 = 23;
const int pinBackPWMA = 4;

const int pinBackBI1 = 27; // back right motor
const int pinBackBI2 = 29;
const int pinBackPWMB = 6;

const int pinBackStandBy = 5;

const int pinFrontAI1 = 33; // front left motor
const int pinFrontAI2 = 31;
const int pinFrontPWMA = 7;

const int pinFrontBI1 = 35; // front right motor
const int pinFrontBI2 = 37;
const int pinFrontPWMB = 9;

const int pinFrontStandBy = 8;

const int frontEchoPin = 39;
const int frontTrigPin = 41; // Ultrasonic sensor
const int rightFrontEchoPin = 43;
const int rightFrontTrigPin = 45;
const int rightBackEchoPin = 47;
const int rightBackTrigPin = 49;

const int bottomServoPin = 10;
const int topServoPin = 11;

// Constants won't change. They're used here to set pin numbers:
const int buttonPin = 21;       // The number of the pushbutton pin

// Variables will change:
int lastState = HIGH;            // The previous state from the input pin
int currentState;                // The current reading from the input pin
unsigned long lastDebounceTime = 0;  // The last time the input pin was toggled
unsigned long debounceDelay = 50;    // The debounce time; increase if the output flickers
bool start = false;

long leftEnc_count = 0; // Total pulses from the encoder
long rightEnc_count = 0;
float leftEnc_rev = 0;
float rightEnc_rev = 0;
float leftWheel_angle = 0; // Wheel angle in degrees
float rightWheel_angle = 0;

boolean backAI1 = false;
boolean backAI2 = false;
boolean backBI1 = false;
boolean backBI2 = false;
boolean backstandBy = false;

boolean frontAI1 = false;
boolean frontAI2 = false;
boolean frontBI1 = false;
boolean frontBI2 = false;
boolean frontstandBy = false;

float rightFrontDistance = 0;
float rightBackDistance = 0;
float frontDistance = 0;

float alignOffset = 1.0;
float firstRotThreshold = 11.55;
float secondRotThreshold = 8
.0;
float thirdRotThreshold = 12.1;
float fourthRotThreshold = 20.0;
float gateThreshold = 30.0;

Servo bottomServo;
Servo topServo;


void setup() {
  // Assign the digital I/O pin directions
  pinMode(pinBackAI1, OUTPUT);
  pinMode(pinBackAI2, OUTPUT);
  pinMode(pinBackPWMA, OUTPUT);
  pinMode(pinBackBI1, OUTPUT);
  pinMode(pinBackBI2, OUTPUT);
  pinMode(pinBackPWMB, OUTPUT);
  pinMode(pinBackStandBy, OUTPUT);

  pinMode(pinFrontAI1, OUTPUT);
  pinMode(pinFrontAI2, OUTPUT);
  pinMode(pinFrontPWMA, OUTPUT);
  pinMode(pinFrontBI1, OUTPUT);
  pinMode(pinFrontBI2, OUTPUT);
  pinMode(pinFrontPWMB, OUTPUT);
  pinMode(pinFrontStandBy, OUTPUT);

  pinMode(frontEchoPin, INPUT);
  pinMode(frontTrigPin, OUTPUT);
  pinMode(rightFrontEchoPin, INPUT);
  pinMode(rightFrontTrigPin, OUTPUT);
  pinMode(rightBackEchoPin, INPUT);
  pinMode(rightBackTrigPin, OUTPUT);

  pinMode(bottomServoPin, OUTPUT);
  pinMode(topServoPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(buttonPin), buttonInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(leftPINA), leftChannelA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(leftPINB), leftChannelB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightPINA), rightChannelA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightPINB), rightChannelB, CHANGE);

  // Initialize the serial port
  Serial.begin(9600);

  // Drive the standby pin high to enable the output
  backstandBy = true;
  frontstandBy = true;
  digitalWrite(pinBackStandBy, backstandBy);
  digitalWrite(pinFrontStandBy, frontstandBy);
}

void buttonInterrupt() {
  // Read the state of the switch/button:
  currentState = digitalRead(buttonPin);

  // Check to see if the switch/button has stabilized after the debounce delay:
  if (millis() - lastDebounceTime > debounceDelay) {
    // If the switch/button changed, due to noise or pressing:
    if (currentState != lastState) {
      // If the button state has changed:
      if (lastState == HIGH && currentState == LOW) {
        Serial.println("The button is pressed");
        start = true;
      } else if (lastState == LOW && currentState == HIGH) {
        Serial.println("The button is released");
        start = false;
      }

      // Save the last steady state
      lastState = currentState;
    }
  }
  // Update the last debounce time
  lastDebounceTime = millis();
}

void forward(float speed) {
  backAI1 = false;
  backAI2 = true;
  backBI1 = true;
  backBI2 = false;

  frontAI1 = true;
  frontAI2 = false;
  frontBI1 = false;
  frontBI2 = true;

  // Write the pwnValue to the PWM pin
  analogWrite(pinBackPWMA, int(speed * 255));
  analogWrite(pinBackPWMB, int(speed * 255));
  analogWrite(pinFrontPWMA, int(speed * 255));
  analogWrite(pinFrontPWMB, int(speed * 255));

  // Write direction values
  digitalWrite(pinBackAI1, backAI1);
  digitalWrite(pinBackAI2, backAI2);
  digitalWrite(pinBackBI1, backBI1);
  digitalWrite(pinBackBI2, backBI2);
  digitalWrite(pinFrontAI1, frontAI1);
  digitalWrite(pinFrontAI2, frontAI2);
  digitalWrite(pinFrontBI1, frontBI1);
  digitalWrite(pinFrontBI2, frontBI2);
}

void backward(float speed) {
  backAI1 = true;
  backAI2 = false;
  backBI1 = false;
  backBI2 = true;

  frontAI1 = false;
  frontAI2 = true;
  frontBI1 = true;
  frontBI2 = false;

  // Write the pwnValue to the PWM pin
  analogWrite(pinBackPWMA, int(speed * 255));
  analogWrite(pinBackPWMB, int(speed * 255));
  analogWrite(pinFrontPWMA, int(speed * 255));
  analogWrite(pinFrontPWMB, int(speed * 255));

  // Write direction values
  digitalWrite(pinBackAI1, backAI1);
  digitalWrite(pinBackAI2, backAI2);
  digitalWrite(pinBackBI1, backBI1);
  digitalWrite(pinBackBI2, backBI2);
  digitalWrite(pinFrontAI1, frontAI1);
  digitalWrite(pinFrontAI2, frontAI2);
  digitalWrite(pinFrontBI1, frontBI1);
  digitalWrite(pinFrontBI2, frontBI2);
}

void left(float speed) {
  backAI1 = false;
  backAI2 = true;
  backBI1 = false;
  backBI2 = true;

  frontAI1 = false;
  frontAI2 = true;
  frontBI1 = false;
  frontBI2 = true;

  // Write the pwnValue to the PWM pin
  analogWrite(pinBackPWMA, int(speed * 255));
  analogWrite(pinBackPWMB, int(speed * 255));
  analogWrite(pinFrontPWMA, int(speed * 255));
  analogWrite(pinFrontPWMB, int(speed * 255));

  // Write direction values
  digitalWrite(pinBackAI1, backAI1);
  digitalWrite(pinBackAI2, backAI2);
  digitalWrite(pinBackBI1, backBI1);
  digitalWrite(pinBackBI2, backBI2);
  digitalWrite(pinFrontAI1, frontAI1);
  digitalWrite(pinFrontAI2, frontAI2);
  digitalWrite(pinFrontBI1, frontBI1);
  digitalWrite(pinFrontBI2, frontBI2);
}

void right(float speed) {
  backAI1 = true;
  backAI2 = false;
  backBI1 = true;
  backBI2 = false;

  frontAI1 = true;
  frontAI2 = false;
  frontBI1 = true;
  frontBI2 = false;

  // Write the pwnValue to the PWM pin
  analogWrite(pinBackPWMA, int(speed * 255));
  analogWrite(pinBackPWMB, int(speed * 255));
  analogWrite(pinFrontPWMA, int(speed * 255));
  analogWrite(pinFrontPWMB, int(speed * 255));

  // Write direction values
  digitalWrite(pinBackAI1, backAI1);
  digitalWrite(pinBackAI2, backAI2);
  digitalWrite(pinBackBI1, backBI1);
  digitalWrite(pinBackBI2, backBI2);
  digitalWrite(pinFrontAI1, frontAI1);
  digitalWrite(pinFrontAI2, frontAI2);
  digitalWrite(pinFrontBI1, frontBI1);
  digitalWrite(pinFrontBI2, frontBI2);
}

void stop() {
  backAI1 = false;
  backAI2 = false;
  backBI1 = false;
  backBI2 = false;

  frontAI1 = false;
  frontAI2 = false;
  frontBI1 = false;
  frontBI2 = false;

  // Write the pwnValue to the PWM pin
  analogWrite(pinBackPWMA, 0);
  analogWrite(pinBackPWMB, 0);
  analogWrite(pinFrontPWMA, 0);
  analogWrite(pinFrontPWMB, 0);

  // Write direction values
  digitalWrite(pinBackAI1, backAI1);
  digitalWrite(pinBackAI2, backAI2);
  digitalWrite(pinBackBI1, backBI1);
  digitalWrite(pinBackBI2, backBI2);
  digitalWrite(pinFrontAI1, frontAI1);
  digitalWrite(pinFrontAI2, frontAI2);
  digitalWrite(pinFrontBI1, frontBI1);
  digitalWrite(pinFrontBI2, frontBI2);
}

void leftChannelA() {
  if (digitalRead(leftPINA) == digitalRead(leftPINB)) {
    leftEnc_count++;
  } else {
    leftEnc_count--;
  }
  leftEnc_rev = float(leftEnc_count) / float(ENC_K * GEAR_RATIO);
  leftWheel_angle = leftEnc_rev * 360;
}

void leftChannelB() {
  if (digitalRead(leftPINA) == digitalRead(leftPINB)) {
    leftEnc_count--;
  } else {
    leftEnc_count++;
  }
  leftEnc_rev = float(leftEnc_count) / float(ENC_K * GEAR_RATIO);
  leftWheel_angle = leftEnc_rev * 360;
}

void rightChannelA() {
  if (digitalRead(rightPINA) == digitalRead(rightPINB)) {
    rightEnc_count++;
  } else {
    rightEnc_count--;
  }
  rightEnc_rev = float(rightEnc_count) / float(ENC_K * GEAR_RATIO);
  rightWheel_angle = rightEnc_rev * 360;
}

void rightChannelB() {
  if (digitalRead(rightPINA) == digitalRead(rightPINB)) {
    rightEnc_count--;
  } else {
    rightEnc_count++;
  }
  rightEnc_rev = float(rightEnc_count) / float(ENC_K * GEAR_RATIO);
  rightWheel_angle = rightEnc_rev * 360;
}

void rotate_90(float speed) {
  float currentRightEnc_rev = rightEnc_rev;
  float targetRightEnc_rev = 0.825;

  Serial.print("Current right encoder: ");
  Serial.println(rightEnc_rev);

  while (abs(currentRightEnc_rev - rightEnc_rev) < targetRightEnc_rev) {
    left(speed);
    Serial.print("Current right encoder: ");
    Serial.println(rightEnc_rev);
  }
  stop();
}

void rotate_180(float speed) {
  float currentRightEnc_rev = rightEnc_rev;
  float targetRightEnc_rev = 1.65;

  Serial.print("Current right encoder: ");
  Serial.println(rightEnc_rev);

  while (abs(currentRightEnc_rev - rightEnc_rev) < targetRightEnc_rev) {
    left(speed);
    Serial.print("Current right encoder: ");
    Serial.println(rightEnc_rev);
  }
  stop();
}

void calculateRightBackDistance() {
  digitalWrite(rightBackTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(rightBackTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(rightBackTrigPin, LOW);
  rightBackDistance = pulseIn(rightBackEchoPin, HIGH) * 0.034 / 2;
}

void calculateRightFrontDistance() {
  digitalWrite(rightFrontTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(rightFrontTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(rightFrontTrigPin, LOW);
  rightFrontDistance = pulseIn(rightFrontEchoPin, HIGH) * 0.034 / 2;
}

void calculateFrontDistance() {
  digitalWrite(frontTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(frontTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(frontTrigPin, LOW);
  frontDistance = pulseIn(frontEchoPin, HIGH) * 0.034 / 2;
  Serial.print("Front distance: ");
  Serial.println(frontDistance);
}

void realign(float speed) {

  time_t start = millis();

  while (millis() - start < 5000) {
    calculateRightBackDistance();
    calculateRightFrontDistance();

    float diff = rightFrontDistance - rightBackDistance;

    if (abs(diff) < alignOffset)
      return;

    if (diff > alignOffset) {
      while (diff > alignOffset) {
        right(speed);
        calculateRightBackDistance();
        calculateRightFrontDistance();
        diff = rightFrontDistance - rightBackDistance;
        delay(10);
      }
      stop();
    } else if (diff < alignOffset) {
      while (diff < alignOffset) {
        left(speed);
        calculateRightBackDistance();
        calculateRightFrontDistance();
        diff = rightFrontDistance - rightBackDistance;
        delay(10);
      }
      stop();
    }
  }
}

void pause() {
  stop();
  delay(2000);
}

void followStraightLine(float speed, float threshold) {
  calculateFrontDistance();
  while (frontDistance > threshold) {

    calculateRightBackDistance();
    calculateRightFrontDistance();
    calculateFrontDistance();

    float diff = rightFrontDistance - rightBackDistance;

    if (abs(diff) < alignOffset)
      forward(0.2);

    else if (diff > alignOffset)
      right(0.2);

    else if (diff < alignOffset)
      left(0.2);
  }
}


void actuateArm() {

  bottomServo.attach(bottomServoPin);
  topServo.attach(topServoPin);

  bottomServo.write(170);
  topServo.write(45);

 for (int i = 170; i > 135; i--) {
  int topServoAngle = map(i, 170, 135, 0, 45);
  topServo.write(topServoAngle);

  bottomServo.write(i);
  delay(80);
 }

 for (int i = 135; i < 170; i++) {
  int topServoAngle = map(i, 135, 170, 45, 0);
  topServo.write(topServoAngle);

  bottomServo.write(i);
  delay(80);
 }

//  // detach the servo
  bottomServo.detach();
  topServo.detach();
}

void controller() {
  rotate_180(0.2);
  pause();

  realign(0.2);
  pause();

  followStraightLine(0.2, firstRotThreshold);
  pause();

  rotate_90(0.2);
  pause();

  realign(0.2);
  pause();

  calculateFrontDistance();
  while (frontDistance > gateThreshold) {
    calculateFrontDistance();
    forward(0.2);
  }
  pause();

  followStraightLine(0.2, secondRotThreshold);
  pause();

  realign(0.2);
  pause();

  actuateArm();
  pause();

  rotate_90(0.2);
  pause();

  realign(0.2);
  pause();

  followStraightLine(0.2, thirdRotThreshold);
  pause();

  rotate_90(0.2);
  pause();

  realign(0.2);
  pause();

  calculateFrontDistance();
  while (frontDistance > fourthRotThreshold) {
    calculateFrontDistance();
    forward(0.2);
  }
  pause();
}


void loop() {

  // Serial.println(start);

  // if (start) {
  //   controller();
  //   exit(0);
  // }

  controller();
  exit(0);

  delay(10);
}
