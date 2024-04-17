#include <Arduino_CAN.h>
#include "main_bus.hpp"

// Motor Pins
const int enA = 3;
const int in1 = 4;
const int in2 = 5;
const int enB = 6;
const int in3 = 7;
const int in4 = 8;

// Min and max speeds
int minPWM = 50;
int maxPWM = 255;
float multiplier = 0.5; // Convert rpm to pwm

// Dont move if message not received in timeout millisecconds
unsigned long currentTime;
unsigned long lastRecv;
bool lockout = false;
int timeout = 3000;

// Store speed
int left_speed;
int right_speed;

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
    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
}

void loop() {
    currentTime = millis();
    
    // Handle can
    if (CAN.available()) {
        Serial.println("CAN Recieved");
        CanMsg const msg = CAN.read();

        switch(msg.id) {
        case (long unsigned int) can::FrameID::EStop:
            // TODO lockout until un-estop msg received
            // Right now it will esotp until motor msg received
            lockout = true;
            lastRecv = currentTime;
        case (long unsigned int) can::FrameID::MotorCommands:
            lockout = false;
            can::MotorCommands cmd = can::MotorCommands_deserialize(can::from_buffer(msg.data));
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
        controlMotor(enA, in1, in2, right_speed);
        controlMotor(enB, in3, in4, left_speed);
    } else {
        stopMotor(enA, in1, in2);
        stopMotor(enB, in3, in4);
    }
}

void controlMotor(int enPin, int inPin1, int inPin2, int speed) {
    int pwm = abs(speed) * multiplier;
    bool dir = speed > 0;
    if (pwm > maxPWM) {
        pwm = maxPWM;
    }

    if (pwm < minPWM && pwm != 0) {
        Serial.print("PWM ");
        Serial.print(pwm);
        Serial.print(" too low (minimum ");
        Serial.print(minPWM);
        Serial.println(")");
        analogWrite(enPin, 0);
    } else {
        analogWrite(enPin, pwm);
        digitalWrite(inPin1, dir);
        digitalWrite(inPin2, !dir);
    }
}

void stopMotor(int enPin, int inPin1, int inPin2) {
    analogWrite(enPin, 0);
    digitalWrite(inPin1, false);
    digitalWrite(inPin2, false);
}