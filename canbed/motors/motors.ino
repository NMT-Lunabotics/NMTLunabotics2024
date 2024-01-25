#include <Arduino_CAN.h>

// Motor Pins
const int enA = 3;
const int in1 = 4;
const int in2 = 5;
const int enB = 6;
const int in3 = 7;
const int in4 = 8;

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
            case 0x000:
                // Left motor control
                controlMotor(enA, in1, in2, 0, 0);
                // Right motor control
                controlMotor(enB, in3, in4, 0, 0);
            case 0x100:
                controlMotor(enA, in1, in2, msg.data[0], msg.data[2]);
                controlMotor(enB, in3, in4, msg.data[1], msg.data[3]);
        }
        
    }
}

void controlMotor(int enPin, int inPin1, int inPin2, int speed, bool dir) {
    if (speed >= 0) {
        analogWrite(enPin, speed);
        digitalWrite(inPin1, dir);
        digitalWrite(inPin2, !dir);
    } else {
        analogWrite(enPin, -speed);
        digitalWrite(inPin1, !dir);
        digitalWrite(inPin2, dir);
    }
}

