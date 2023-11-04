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
  unsigned int id;
  unsigned char buf[8];

public:
  CANCommand(unsigned int id, const unsigned char *other_buf, int buf_len) {
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
      fwd = &left_fwd;
      rev = &left_rev;
    } else if (command.is_right_motor()) {
      fwd = &right_fwd;
      rev = &right_rev;
    } else {
      // not a wheel command, do nothing.
      return;
    }

    switch (command.command()) {
    case MotorCommand::Stop:
      *fwd = 0;
      *rev = 0;
      break;
    case MotorCommand::Forward:
      *fwd = 1;
      *rev = 0;
      break;
    case MotorCommand::Reverse:
      *fwd = 0;
      *rev = 1;
      break;
    }
  }
};

int main() {
  Serial.begin(115200);
  MCP_CAN CAN(17); // Set CS pin

  while (CAN_OK != CAN.begin(CAN_500KBPS)) // init can bus : baudrate = 500k
  {
    Serial.println("CAN BUS FAIL!");
    delay(100);
  }
  Serial.println("CAN BUS OK!");

  Wheels wheels(4, 5, 6, 8);

  while (true) {
    unsigned char len;
    unsigned char buf[8];

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
}
