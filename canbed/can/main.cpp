// receive a frame from can bus

#include <Arduino.h>

#include "arduino_lib.hpp"
#include "mcp_can.h"
#include <SPI.h>

enum class MotorCommand {
  Stop,
  Forward,
  Reverse,
};

class CANCommand {
public:
  unsigned int id;
  unsigned char buf[8];

  CANCommand(unsigned int id, const unsigned char *other_buf, int buf_len)
      : id(id) {
    for (int i = 0; i < sizeof(buf); i++)
      if (i < buf_len)
        buf[i] = other_buf[i];
      else
        buf[i] = 0;
  }

  bool is_left_motor() const { return id == 0x100; }
  bool is_right_motor() const { return id == 0x101; }

  MotorCommand command() const {
    switch (buf[0]) {
    case 1:
      return MotorCommand::Forward;
    case 2:
      return MotorCommand::Reverse;
    default:
      return MotorCommand::Stop;
    }
  }
};

class Wheels {
  OutPin left_fwd;
  OutPin left_rev;
  OutPin right_fwd;
  OutPin right_rev;

public:
  Wheels(int left_fwd, int left_rev, int right_fwd, int right_rev)
      : left_fwd(left_fwd), left_rev(left_rev), right_fwd(right_fwd),
        right_rev(right_rev) {}

  void dispatch(const CANCommand &command) {
    OutPin *fwd, *rev;
    if (command.is_left_motor()) {
      Serial.println("Left");
      fwd = &left_fwd;
      rev = &left_rev;
    } else if (command.is_right_motor()) {
      Serial.println("Right");
      fwd = &right_fwd;
      rev = &right_rev;
    } else {
      // not a wheel command, do nothing.
      Serial.print("Not a command because ");
      Serial.print(command.id);
      Serial.print(" is not ");
      Serial.print(0x100);
      Serial.print(" or ");
      Serial.println(0x101);
      return;
    }

    double max = 0.6;
    // Correct for discrepancy with right motor running twice as fast
    // as left motor.
    if (command.is_right_motor())
      max /= 1.0;

    switch (command.command()) {
    case MotorCommand::Stop:
      Serial.println("Stop");
      fwd->write_pwm(0);
      rev->write_pwm(0);
      break;
    case MotorCommand::Forward:
      Serial.println("Forward");
      fwd->write_pwm(max);
      rev->write_pwm(0);
      break;
    case MotorCommand::Reverse:
      Serial.println("Reverse");
      fwd->write_pwm(0);
      rev->write_pwm(max);
      break;
    }
  }
};

MCP_CAN CAN(17); // Set CS pin

void setup() {
  Serial.begin(115200);

  while (CAN_OK != CAN.begin(CAN_500KBPS)) // init can bus : baudrate = 500k
  {
    Serial.println("CAN BUS FAIL!");
    delay(100);
  }
  Serial.println("CAN BUS OK!");
}

void loop() {
  // Wheels wheels(8, 6, 4, 5);
  Wheels wheels(10, 6, 9, 5);

  unsigned char len;
  unsigned char buf[8];

  // Serial.println("looping");

  if (CAN.checkReceive() == CAN_MSGAVAIL) {
    CAN.readMsgBuf(&len, buf);
    unsigned long canId = CAN.getCanId();

    Serial.println("-----------------------------");
    Serial.print("Get data from ID: ");
    Serial.println(canId, HEX);

    // print the data
    for (int i = 0; i < len; i++) {
      Serial.print(buf[i], HEX);
      Serial.print("\t");
    }
    Serial.println();

    CANCommand cmd(canId, buf, len);
    wheels.dispatch(cmd);
  }
}
