#include <Arduino.h>

#include "arduino_lib.hpp"

// Define pin numbers
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

float kP = 12;
float kI = 0;
float kD = .0;
float derivative;
float error;
float prevError = 0;
float integral = 0;
float threshold = 1;
float i_threshold = 10;

static int median(const int *data, size_t nmemb);

void setup() {
  Serial.begin(9600);
  OutPin speed_left(PIN_SPEED_LEFT);
  OutPin speed_right(PIN_SPEED_LEFT);
  OutPin dir_left(PIN_DIRECTION_LEFT);
  OutPin dir_right(PIN_DIRECTION_RIGHT);
  InPin pot_left(PIN_POTENTIOMETER_LEFT);
  InPin pot_right(PIN_POTENTIOMETER_RIGHT);

  int history[15];
  memset(history, 0, sizeof(history));
  int v = pot_left.read_analog_raw();
  for (int i = 0; i < sizeof(history) / sizeof(history[0]); i++)
    history[i] = v;

  size_t head = 0;

  while (true) {
    // Handle new readings
    int newVal = pot_left.read_analog_raw();

    history[head] = newVal;
    head = (head + 1) % (sizeof(history) / sizeof(history[0]));

    newVal = median(history, sizeof(history) / sizeof(history[0]));
    int pos = map(newVal, potMin, potMax, 0, stroke);

    int output = 0;

    delay(50);
    error = target - pos;
    if (abs(error) >= threshold) {
      error = target - pos;
      derivative = error - prevError;
      prevError = error;
      if (abs(error) <= i_threshold && abs(error) != 0) {
        integral += error;
      } else {
        integral = 0;
      }
      output = kP * error + kI * integral + kD * derivative;
      output = constrain(output, -pwm, pwm);
    }

    Serial.print(output);
    Serial.print(" ");
    Serial.println(pos);
    // When here, output is between -pwm and pwm. Output 0 will stop;
    dir_left.write(output > 0);
    if (abs(error) <= threshold) {
      speed_left.write_pwm(0);
    } else {
      speed_right.write_pwm(abs(output));
    }
  }
}

void loop() {}

static int median(const int *data, size_t nmemb) {
  int my_data[nmemb];
  memcpy(my_data, data, nmemb * sizeof(my_data[0]));

  qsort(my_data, nmemb, sizeof(data[0]), [](const void *a, const void *b) {
    if (*(int *)a > *(int *)b)
      return 1;
    else if (*(int *)a < *(int *)b)
      return -1;
    return 0;
  });

  if (nmemb % 2 == 1)
    return (my_data[nmemb / 2] + my_data[nmemb / 2 + 1]) / 2;
  else
    return my_data[nmemb / 2];
}
