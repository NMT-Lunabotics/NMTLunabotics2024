#include "helpers.hpp"
#include "main_bus.hpp"
#include <Arduino_CAN.h>

#define PIN_SPEED_LEFT 3
#define PIN_SPEED_RIGHT 5
#define PIN_DIRECTION_LEFT 4
#define PIN_DIRECTION_RIGHT 6
#define PIN_POTENTIOMETER_LEFT A0
#define PIN_POTENTIOMETER_RIGHT A1

int target_pos = 0; // in mm
int target_vel = 0; // in mm/s

int max_speed = 200; // 5 mm/s
float threshold = 1; // in mm

int stroke = 250;      // stroke length, in mm:
int potMin = 34;       // Calibrated, pot val at min stroke
int potMax = 945;      // Calibrated, pot val at max stroke
#define UPDATE_RATE 50 // hz

int max_error = 5;
int error_factor = 12;

#define MEDIAN_SIZE 15

class Actuator {
  OutPin speed;
  OutPin dir;
  SmoothedInput<MEDIAN_SIZE> pot;

  bool invert_direction;

public:
  Actuator(OutPin speed, OutPin dir, InPin pot, bool invert_direction)
      : speed(speed), dir(dir), pot(pot) {}

  int pos_mm() { return map(pot.read_analog_raw(), potMin, potMax, 0, stroke); }

  void set_speed(int signed_speed) {
    if (invert_direction)
      dir.write(signed_speed > 0);
    else
      dir.write(signed_speed < 0);
    speed.write_pwm_raw(abs(signed_speed));
  }
};

void setup() {
  Serial.begin(9600);
  while (!Serial)
    ;
  if (!CAN.begin(CanBitRate::BR_500k))
    panic("Can is not available");

  Actuator left(PIN_SPEED_LEFT, PIN_DIRECTION_LEFT, PIN_POTENTIOMETER_LEFT,
                false);
  Actuator right(PIN_SPEED_RIGHT, PIN_DIRECTION_RIGHT, PIN_POTENTIOMETER_RIGHT,
                 false);

  unsigned long current_time = millis();
  unsigned long last_time = current_time;

  int speed_l = 0;
  int speed_r = 0;

  bool estop = false;
  while (true) {
    if (CAN.available()) {
      CanMsg const msg = CAN.read();
      switch ((can::FrameID)msg.id) {
      case can::FrameID::EStop: {
        estop = true;
        break;
      }
      case can::FrameID::ActuatorPosCommands: {
        can::ActuatorPosCommands actuatorCmd =
            can::ActuatorPosCommands_deserialize(can::from_buffer(msg.data));
        target_pos = actuatorCmd.arm_pos;
        break;
      }
      case can::FrameID::ActuatorVelCommands: {
        can::ActuatorVelCommands actuatorCmd =
            can::ActuatorVelCommands_deserialize(can::from_buffer(msg.data));
        target_vel = actuatorCmd.arm_vel;
        target_pos = -1;
        // Serial.print("Got vel command: ");
        // Serial.print(actuatorCmd.arm_vel);
        // Serial.println();
        break;
      }
      default: {
        break;
      }
      }
    }

    if (estop) {
      Serial.println("Estopped");
    }

    current_time = millis();

    int dt = 1000 / UPDATE_RATE;

    if (current_time - last_time < dt) {
      continue;
    }
    last_time = current_time;

    int pos_l = left.pos_mm();
    int pos_r = right.pos_mm();

    // Serial.print(target_pos);
    // Serial.print(" :: ");

    // Serial.print(pos_l);
    // Serial.print(" ");
    // Serial.print(pos_r);
    // Serial.println();

    int error_lr = pos_l - pos_r;
    int error_l = 0;
    int error_r = 0;

    bool doomsday = false;
    if (error_lr > max_error) {
      doomsday = true;
      Serial.println("Doomsday");
    }

    // if (doomsday) {
    //   can::Error e = {.error_code = can::ErrorCode::ActuatorOutOfAlignment};
    //   uint8_t e_buff[8];
    //   can::to_buffer(e_buff, can::serialize(e));
    //   CanMsg const msg((long unsigned int)can::FrameID::Error,
    //   sizeof(e_buff),
    //                    e_buff);
    //   if (int const rc = CAN.write(msg); rc < 0) {
    //     String error = "CAN.write(...) failed with error code " + String(rc);
    //     panic(error.c_str());
    //   }
    // }

    can::ActuatorArmPos cmd = {.left_pos = (double)pos_l,
                               .right_pos = (double)pos_r};
    uint8_t buffer[8];
    can::to_buffer(buffer, can::serialize(cmd));

    CanMsg const msg((int)can::FrameID::ActuatorArmPos, sizeof(buffer), buffer);

    // if (int const rc = CAN.write(msg); rc < 0) {
    //   Serial.println("CAN.write(...) failed with error code " + String(rc));
    // }

    if (target_pos == -1) {
      speed_l = target_vel * 51;
      speed_r = target_vel * 51;
    } else {
      error_l = pos_l - target_pos;
      error_r = pos_r - target_pos;

      if (error_l > threshold) {
        speed_l = -max_speed;
      } else if (error_l < -threshold) {
        speed_l = max_speed;
      } else {
        speed_l = 0;
      }

      if (error_r > threshold) {
        speed_r = -max_speed;
      } else if (error_r < -threshold) {
        speed_r = max_speed;
      } else {
        speed_r = 0;
      }
    }

    int factor = error_factor * error_lr;
    speed_l -= factor;
    speed_r += factor;

    // Serial.print(error_l);
    // Serial.print(" : ");
    // Serial.print(speed_l);
    // Serial.print(", ");
    // Serial.print(error_r);
    // Serial.print(" : ");
    // Serial.print(speed_r);
    // Serial.print(", ");
    // Serial.print(error_lr);
    // Serial.println();

    speed_l = constrain(speed_l, -255, 255);
    speed_r = constrain(speed_r, -255, 255);

    Serial.print(speed_l);
    Serial.print('\t');
    Serial.print(speed_r);
    Serial.print('\t');
    Serial.print(pos_l);
    Serial.print('\t');
    Serial.print(pos_r);
    Serial.print('\t');
    Serial.print(factor);
    Serial.print('\n');

    // if (doomsday || estop) {
    //   left.set_speed(0);
    //   right.set_speed(0);
    // } else {
    left.set_speed(-speed_l);
    right.set_speed(-speed_r);
    // }
  }
}

void loop() {}
