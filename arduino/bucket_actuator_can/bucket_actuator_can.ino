#include "helpers.hpp"
#include "main_bus.hpp"
#include <Arduino_CAN.h>

#define PIN_SPEED 3
#define PIN_DIRECTION 4
#define PIN_POTENTIOMETER A0

int target_pos = -1; // in mm
int target_vel = 0; // in mm/s

int max_speed = 240; // 5 mm/s
float threshold = 1; // in mm
int min_pos = 17;
int max_pos = 270;

int stroke = 300; // stroke length, in mm:
int potMin = 34;  // Calibrated, pot val at min stroke
// TODO fix
int potMax = 945;      // Calibrated, pot val at max stroke
#define UPDATE_RATE 50 // hz

int max_error = 50;
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

  Actuator act(PIN_SPEED, PIN_DIRECTION, PIN_POTENTIOMETER, false);

  unsigned long current_time = millis();
  double linearDistance=15.164;
  unsigned long last_time = current_time;
  int speed = 0;

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
        target_pos = actuatorCmd.bucket_pos;
        linearDistance=15.164+target_pos;
        break;
      }
      case can::FrameID::ActuatorVelCommands: {
        can::ActuatorVelCommands actuatorCmd =
            can::ActuatorVelCommands_deserialize(can::from_buffer(msg.data));
        target_vel = actuatorCmd.bucket_vel;
        target_pos = -1;
        // Serial.print("Got vel command: ");
        // Serial.print(actuatorCmd.bucket_vel);
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

    int pos = act.pos_mm();
    can::ActuatorBucketPos cmd = {.pos = (double)pos};
    uint8_t buffer[8];
    can::to_buffer(buffer, can::serialize(cmd));

    CanMsg const msg(CanStandardId((int)can::FrameID::ActuatorBucketPos), sizeof(buffer), buffer);

    if (int const rc = CAN.write(msg); rc < 0) {
      Serial.println("CAN.write(...) failed with error code " + String(rc));
    }

    if (target_pos == -1) {
      speed = target_vel * 51;
    } else {
      int error = pos - target_pos;

      if (error > threshold) {
        speed = -max_speed;
      } else if (error < -threshold) {
        speed = max_speed;
      } else {
        speed = 1;
      }
    }

    speed = constrain(speed, -max_speed, max_speed);

    if (pos < max_pos || pos > min_pos || speed >= 0) {
      act.set_speed(-speed);
    } else {
      act.set_speed(0);
    }
  }
}

void loop() {}
