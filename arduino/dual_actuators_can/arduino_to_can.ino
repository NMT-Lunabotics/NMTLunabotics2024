#include "helpers.hpp"
#include "main_bus.hpp"
#include <Arduino.h>
#define PIN_SPEED_LEFT 6
#define PIN_SPEED_RIGHT 9
#define PIN_DIRECTION_LEFT 7
#define PIN_DIRECTION_RIGHT 10
#define PIN_POTENTIOMETER_LEFT A0
#define PIN_POTENTIOMETER_RIGHT A1

int target_pos = 0; // in mm
int target_vel = 0; // in mm/s

int max_speed = 200; // 5 mm/s
float threshold = 1; // in mm

int stroke = 250;     // stroke length, in mm:
int potMin = 34;      // Calibrated, pot val at min stroke
int potMax = 945;     // Calibrated, pot val at max stroke
int update_rate = 50; // hz

int max_error = 5;

void handleMessage(can_message_t msg) {}

void setup() {
  Serial.begin(9600);
  while (!Serial) {}
  if (!CAN.begin(CanBitRate::BR_250k)) {
    Serial.println("CAN.begin(...) failed.");
    for (;;) {
    }
  }

  OutPin speed_left(PIN_SPEED_LEFT);
  OutPin speed_right(PIN_SPEED_RIGHT);
  OutPin dir_left(PIN_DIRECTION_LEFT);
  OutPin dir_right(PIN_DIRECTION_RIGHT);
  InPin pot_left(PIN_POTENTIOMETER_LEFT);
  InPin pot_right(PIN_POTENTIOMETER_RIGHT);

  int vl = pot_left.read_analog_raw();
  int vr = pot_right.read_analog_raw();
  int median_size = 15;
  Median median_l(median_size);
  Median median_r(median_size);
  for (int i = 0; i < median_size; i++) {
    median_l.update(vl);
    median_r.update(vr);
  }

  unsigned long current_time = millis();
  unsigned long last_time = current_time;

  int speed_l = 0;
  int speed_r = 0;

  while (true) {
    if (CAN.available()) {
      CanMsg const msg = CAN.read();
      switch (msg.id) {
      case (long unsigned int)can::FrameID::ActuatorPosCommands:
        can::ActuatorCommands actuatorCmd =
            can::ActuatorCommands_deserialize(can::from_buffer(msg.data));
        target_pos = actuatorCmd.arm_pos;
        break;
      case (long unsigned int)can::FrameID::ActuatorVelCommands:
        can::ActuatorCommands actuatorCmd =
            can::ActuatorCommands_deserialize(can::from_buffer(msg.data));
        target_vel = actuatorCmd.arm_vel;
        target_pos = -1;
        break;
      }
    } else {
      Serial.println("CAN Not available");
      // TODO go into error mode
    }

    current_time = millis();

    int dt = 1000 / update_rate;

    if (current_time - last_time < dt) {
      continue;
    }
    last_time = current_time;

    int val_l = pot_left.read_analog_raw();
    int val_r = pot_right.read_analog_raw();

    int m_l = median_l.update(val_l);
    int m_r = median_r.update(val_r);

    float pos_l = map(m_l, potMin, potMax, 0, stroke);
    float pos_r = map(m_r, potMin, potMax, 0, stroke);

    float error_lr = pos_l - pos_r;
    bool doomsday = false;
    if (error_lr > max_error) {
      doomsday = true;
    }

    if (CAN.available()) {
      if (doomsday) {
        can::Error e = {.error_code = can::ErrorCode::ActuatorOutOfAlignment}
        uint8_t e_buff[8];
        can::to_buffer(e_buff, can::serialize(e));
        CanMsg const msg(CanStandardId(can::FrameID::ErrorCode),
                         sizeof(e_buff), e_buff);
      }

      can::ActuatorArmPos cmd = {.left_pos = m_l, .right_pos = m_r};
      uint8_t buffer[8];
      can::to_buffer(buffer, can::serialize(cmd));

      CanMsg const msg(CanStandardId(can::FrameID::ActuatorArmPos),
                       sizeof(buffer), buffer);

      if (int const rc = CAN.write(msg); rc < 0) {
        Serial.print("CAN.write(...) failed with error code ");
        Serial.println(rc);
        for (;;) {
        }
      }
    } else {
      // TODO handle can failiure
    }

    if (pos == -1) {
      speed_l = target_vel;
      speed_r = target_vel;
    } else {
      float error_l = pos_l - target_pos;
      float error_r = pos_r - target_pos;

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

    int factor = 12 * error_lr;
    speed_l -= factor;
    speed_r += factor;

    Serial.print(error_l);
    Serial.print(" : ");
    Serial.print(speed_l);
    Serial.print(", ");
    Serial.print(error_r);
    Serial.print(" : ");
    Serial.print(speed_r);
    Serial.print(", ");
    Serial.print(error_lr);
    Serial.println();

    speed_l = constrain(speed_l, -255, 255);
    speed_r = constrain(speed_r, -255, 255);

    if (doomsday) {
      speed_left.write_pwm_raw(0);
      speed_right.write_pwm_raw(0);
    } else {
      speed_left.write_pwm_raw(abs(speed_l));
      dir_left.write(speed_l < 0);
      speed_right.write_pwm_raw(abs(speed_r));
      dir_right.write(speed_r > 0);
    }
  }
}

void loop() {}
