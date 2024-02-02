#include <Arduino_CAN.h>
#include "main_bus.hpp"

// Motor Pins
const int enA = 3;
const int in1 = 4;
const int in2 = 5;
const int enB = 6;
const int in3 = 7;
const int in4 = 8;

int minPWM = 50;
int maxPWM = 255;
float multiplier = 0.5;

void setup() {
    Serial.begin(115200);

    // Initialize CAN Bus
    if (!CAN.begin(CanBitRate::BR_500k)) {
        Serial.println("Starting CAN failed!");
        while (1);
    }

    // Initialize Motor Control Pins
    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
}

void loop() {
    if (CAN.available()) {
        CanMsg const msg = CAN.read();

        switch(msg.id) {
            case (long unsigned int) can::FrameID::EStop:
                // Left motor control
                controlMotor(enA, in1, in2, 0);
                // Right motor control
                controlMotor(enB, in3, in4, 0);
            case (long unsigned int) can::FrameID::MotorCommands:
                can::MotorCommands cmd = can::MotorCommands_deserialize(can::from_buffer(msg.data));
                controlMotor(enA, in1, in2, cmd.left.speed);
                controlMotor(enB, in3, in4, cmd.right.speed);
        }
        
    }
}

void controlMotor(int enPin, int inPin1, int inPin2, int speed) {
    int pwm = abs(speed) * multiplier;
    bool dir = speed > 0;
    if (pwm > maxPWM || pwm < minPWM && pwm != 0) {
        Serial.print("PWM ");
        Serial.print(pwm);
        Serial.println(" outside acceptable range");
        analogWrite(enPin, 0);
    } else {
        analogWrite(enPin, pwm);
        digitalWrite(inPin1, dir);
        digitalWrite(inPin2, !dir);
    }
}

