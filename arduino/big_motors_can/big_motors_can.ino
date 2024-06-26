#include "main_bus.hpp"
#include <Arduino_CAN.h>

// pins
#define LGND 9
#define RGND 5
#define LDIR 7
#define RDIR 8
#define LPWM 6
#define RPWM 4
#define LIHB 2
#define RIHB 3

// Min and max speeds
// TODO tune values
float minPWM = 0;
float maxPWM = 255;
float minSpd = 0;
float maxSpd = 30;
// float multiplier = 0.5; // Convert rpm to pwm

// Dont move if message not received in timeout millisecconds
unsigned long currentTime;
unsigned long lastRecv;
bool lockout = false;
int timeout = 3000;

// Store speed
float left_speed;
float right_speed;

void controlMotor(int ihbPin, int dirPin, int pwmPin, float speed);
void stopMotor(int ihbPin, int pwmPin);

void setup() {
  Serial.begin(115200);

  // Initialize CAN Bus
  if (!CAN.begin(CanBitRate::BR_500k)) {
    while (1) {
      Serial.println("Starting CAN failed!");
    }
  }
  Serial.println("Starting CAN successful");

  // Initialize Motor Control Pins
  pinMode(LGND, OUTPUT);
  pinMode(RGND, OUTPUT);
  pinMode(LDIR, OUTPUT);
  pinMode(RDIR, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(RPWM, OUTPUT);
  pinMode(LIHB, OUTPUT);
  pinMode(RIHB, OUTPUT);
  digitalWrite(LGND, LOW);
  digitalWrite(RGND, LOW);
}

void loop() {
  currentTime = millis();

  // Handle can
  if (CAN.available()) {
    // Serial.println("CAN Recieved");
    CanMsg const msg = CAN.read();

    switch (msg.id) {
    case (long unsigned int)can::FrameID::EStop:
      // TODO lockout until un-estop msg received
      // Right now it will estop until motor msg received
      lockout = true;
      lastRecv = currentTime;
    case (long unsigned int)can::FrameID::MotorCommands:
      lockout = false;
      can::MotorCommands cmd =
          can::MotorCommands_deserialize(can::from_buffer(msg.data));
      left_speed = cmd.left.speed;
      right_speed = cmd.right.speed;
      lastRecv = currentTime;
    }
  }

  // lockout if message not received
  if (currentTime - lastRecv > timeout) {
    lockout = true;
  }

  if (!lockout) {
    controlMotor(LIHB, LDIR, LPWM, left_speed);
    controlMotor(RIHB, RDIR, RPWM, right_speed);
  } else {
    stopMotor(LIHB, LPWM);
    stopMotor(RIHB, RPWM);
  }
}

void controlMotor(int ihbPin, int dirPin, int pwmPin, float speed) {
  digitalWrite(ihbPin, HIGH);
  int pwm = map(abs(speed), 0, maxSpd, 0, maxPWM);
  bool dir = speed > 0;
  if (dirPin == LDIR) {
    dir = !dir;
  }
  Serial.print(speed);
  Serial.print(" : ");
  Serial.print(pwm);
  Serial.println();
  if (pwm > maxPWM) {
    pwm = maxPWM;
  }

  if (pwm < minPWM && pwm != 0) {
    Serial.print("PWM ");
    Serial.print(pwm);
    Serial.print(" too low (minimum ");
    Serial.print(minPWM);
    Serial.println(")");
    // TODO send can error
    stopMotor(ihbPin, pwmPin);
  } else {
    digitalWrite(dirPin, dir);
    analogWrite(pwmPin, pwm);
  }
}

void stopMotor(int ihbPin, int pwmPin) {
  digitalWrite(ihbPin, LOW);
  analogWrite(pwmPin, 0);
}
