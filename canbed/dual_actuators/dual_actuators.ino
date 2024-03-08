#include <Arduino.h>

#include "arduino_lib.hpp"

#define PIN_SPEED_LEFT 6
#define PIN_SPEED_RIGHT 9
#define PIN_DIRECTION_LEFT 7
#define PIN_DIRECTION_RIGHT 10
#define PIN_POTENTIOMETER_LEFT A0
#define PIN_POTENTIOMETER_RIGHT A1

int target = 100; // in mm

int pwm = 200;    // speed, in pwm TODO change this to mm/s
int stroke = 300; // stroke length, in mm
int potMin = 34;  // Calibrated, pot val at min stroke
int potMax = 945; // Calibrated, pot val at max stroke
float threshold = 1;

float p = 12;
float i = 0;
float d = .0;

class PID {
private:
  float error;
  float prev_error;
  float derivative;
  float integral;

  float p, i, d;

public:
  PID(float p, float i, float d) : p(p), i(i), d(d) {
    error = 0;
    prev_error = 0;
    derivative = 0;
    integral = 0;
  }

  float update(float error) {
    derivative = error - prev_error;
    integral += error;
    prev_error = error;
    return p * error + i * integral + d * derivative;
  }

  void resetIntegral() {
    integral = 0;
  }
};

class Median {
private:
  int history_size;
  int* history;
  int current_idx;

public:
  Median(int history_size) : history_size(history_size) {
    history = new int[history_size];
    current_idx = 0;
    memset(history, 0, sizeof(history));
  }

  ~Median() {
    delete[] history;
  }

  int update(int new_val) {
    history[current_idx] = new_val;
    current_idx++;
    current_idx %= history_size;

    int sorted[history_size];
    memcpy(sorted, history, history_size * sizeof(history[0]));

    qsort(sorted, history_size, sizeof(sorted[0]), [](const void *a, const void *b) {
      if (*(int *)a > *(int *)b)
        return 1;
      else if (*(int *)a < *(int *)b)
        return -1;
      return 0;
    });

    if (history_size % 2 == 1)
      return (sorted[history_size / 2] + sorted[history_size / 2 + 1]) / 2;
    else
      return sorted[history_size / 2];
  }
};

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

  PID pid_l(p, i, d);
  PID pid_r(p, i, d);

  while (true) {
    delay(50);

    int val_l = pot_left.read_analog_raw();
    int val_r = pot_right.read_analog_raw();

    int m_l = median_l.update(val_l);
    int m_r = median_r.update(val_r);

    int pos_l = map(m_l, potMin, potMax, 0, stroke);
    int pos_r = map(m_r, potMin, potMax, 0, stroke);

    float error_l = pos_l - target;
    float error_r = pos_r - target;

    // Serial.print(error_l);
    // Serial.print(" ");
    // Serial.println(error_r);

    if (abs(error_l) <= threshold) {
      speed_left.write(0);
    } else {
      int s_l = pid_l.update(error_l);
      s_l = constrain(s_l, -pwm, pwm);
      Serial.println(s_l);
      speed_left.write(s_l);
      dir_left.write(s_l > 0);
    }

    if (abs(error_r) <= threshold) {
      speed_right.write(0);
    } else {
      int s_r = pid_r.update(error_l);
      s_r = constrain(s_r, -pwm, pwm);
      speed_right.write(s_r);
      dir_right.write(s_r > 0);
    }
  }
}

void loop () {}
