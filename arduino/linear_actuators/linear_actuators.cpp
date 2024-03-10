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
#define i_threshold 10
#define threshold 1

static int median(const int *data, size_t nmemb);

class LinearActuator {
  OutPin speed;
  OutPin dir;
  SmoothedInput pot;

public:
  LinearActuator(OutPin speed, OutPin dir, InPin pot)
      : speed(speed), dir(dir), pot(pot) {}

  void update() {
    int newVal = pot.read_analog_raw();
    int pos = map(newVal, potMin, potMax, 0, stroke);

    float error = target - pos;
    if (abs(error) >= threshold) {
      int output = kP * error;
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
