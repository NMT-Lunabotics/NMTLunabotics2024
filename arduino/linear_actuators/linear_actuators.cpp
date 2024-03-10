#include "arduino_lib.hpp"

// Define pin numbers
#define PIN_SPEED_LEFT 6
#define PIN_SPEED_RIGHT 9
#define PIN_DIRECTION_LEFT 7
#define PIN_DIRECTION_RIGHT 10
#define PIN_POTENTIOMETER_LEFT A0
#define PIN_POTENTIOMETER_RIGHT A1

#define target 100 // in mm

#define pwm 200    // speed, in pwm TODO change this to mm/s
#define stroke 300 // stroke length, in mm
#define potMin 34  // Calibrated, pot val at min stroke
#define potMax 945 // Calibrated, pot val at max stroke

#define kP 12
#define kI 0
#define kD 0
#define i_threshold 10
#define threshold 1

static int median(const int *data, size_t nmemb);

class SmoothedInput {
  InPin raw;
  int history[15];
  int head = 0;

public:
  SmoothedInput(InPin raw) : raw(raw) {
    int v = raw.read_analog_raw();
    for (int i = 0; i < length(history); i++)
      history[i] = v;
  }

  int read_analog_raw() {
    history[head] = raw.read_analog_raw();
    head = (head + 1) % length(history);
    return median(history, length(history));
  }
};

class LinearActuator {
  OutPin speed;
  OutPin dir;
  SmoothedInput pot;

  float prevError = 0;
  float integral = 0;

public:
  LinearActuator(OutPin speed, OutPin dir, InPin pot)
      : speed(speed), dir(dir), pot(pot) {}

  void update() {
    int newVal = pot.read_analog_raw();
    int pos = map(newVal, potMin, potMax, 0, stroke);

    float error = target - pos;
    if (abs(error) >= threshold) {
      float derivative = error - prevError;
      prevError = error;
      if (abs(error) <= i_threshold && abs(error) != 0) {
        integral += error;
      } else {
        integral = 0;
      }
      int output = kP * error + kI * integral + kD * derivative;
      output = constrain(output, -pwm, pwm);
      dir.write(output > 0);
      if (abs(error) <= threshold) {
        speed.write_pwm(0);
      } else {
        speed.write_pwm(abs(output));
      }
    }
  }
};

void setup() {
  Serial.begin(9600);

  LinearActuator left(PIN_SPEED_LEFT, PIN_DIRECTION_LEFT,
                      PIN_POTENTIOMETER_LEFT);
  LinearActuator right(PIN_SPEED_RIGHT, PIN_DIRECTION_RIGHT,
                       PIN_POTENTIOMETER_RIGHT);

  while (true) {
    // Handle new readings
    left.update();
    right.update();
    delay(50);
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
