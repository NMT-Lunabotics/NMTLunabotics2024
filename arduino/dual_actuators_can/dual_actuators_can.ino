#include <Arduino.h>
 #include "helpers.hpp"

#define PIN_SPEED_LEFT 6
#define PIN_SPEED_RIGHT 9
#define PIN_DIRECTION_LEFT 7
#define PIN_DIRECTION_RIGHT 10
#define PIN_POTENTIOMETER_LEFT A0
#define PIN_POTENTIOMETER_RIGHT A1

int target = 200; // in mm

int tgt_speed = 200; // speed, in pwm TODO change this to mm/s
float threshold = 1; // in mm

int stroke = 250;     // stroke length, in mm:
int potMin = 34;      // Calibrated, pot val at min stroke
int potMax = 945;     // Calibrated, pot val at max stroke
int update_rate = 50; // hz

void setup() {
  Serial.begin(9600);

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

    float error_l = pos_l - target;
    float error_r = pos_r - target;
    float error_lr = pos_l - pos_r;

    if (error_l > threshold) {
      speed_l = -tgt_speed;
    } else if (error_l < -threshold) {
      speed_l = tgt_speed;
    } else {
      speed_l = 0;
    }

    if (error_r > threshold) {
      speed_r = -tgt_speed;
    } else if (error_r < -threshold) {
      speed_r = tgt_speed;
    } else {
      speed_r = 0;
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

    speed_left.write_pwm_raw(abs(speed_l));
    dir_left.write(speed_l < 0);
    speed_right.write_pwm_raw(abs(speed_r));
    dir_right.write(speed_r > 0);
  }
}

void loop() {}
