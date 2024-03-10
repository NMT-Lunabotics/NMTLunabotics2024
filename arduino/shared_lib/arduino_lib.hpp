// Shared library code for Arduinos.
#ifndef ARDUINO_LIB_H
#define ARDUINO_LIB_H

#include <Arduino.h>

#define sign(i) (((i) > 0) - ((i) < 0))

#define length(array) (sizeof(array) / sizeof((array)[0]))

// Pinout reference:
// D4  RX
// D5* TX
// D6* SDA
// A3  SCL
// 12  D8
// A0  D9*
// A1  10*
// A2  11*
// 5V  GND
// (* = PWM)

// Labels for pins.
#define D4 4
#define D5 5
#define D6 6
#define D8 8
#define D9 9
#define D13 13

// Lock up the board, send an error message over serial, and flash the
// LED forever.
//
// This should *only* be used for hardware configuration errors that
// are completely unrecoverable.
inline void panic(const char *error_message);

// Output pins.
class OutPin {
  int num;
  bool allow_pwm;

public:
  OutPin(int num) : num(num) {
    pinMode(num, OUTPUT);

    // Only some pins are allowed to do PWM output. This only runs
    // once at init, so isn't much of a speed concern.
    switch (num) {
    case 3:
    case 5:
    case 6:
    case 9:
    case 10:
    case 11:
      allow_pwm = true;
      break;

    default:
      allow_pwm = false;
    }
  }

  void write(bool value) { digitalWrite(num, value ? HIGH : LOW); }
  void write_pwm(double duty_cycle) {
    if (!allow_pwm)
      panic("Attempted PWM on invalid PWM pin");

    analogWrite(num, min(duty_cycle, 1) * 255);
  }
  void write_pwm_raw(int duty_cycle) {
    if (!allow_pwm)
      panic("Attempted PWM on invalid PWM pin");

    analogWrite(num, min(duty_cycle, 255));
  }
};

// Input pins.
class InPin {
  int num;
  float threshold;

public:
  InPin(int num, float threshold = 0.8f) : num(num), threshold(threshold) {
    pinMode(num, INPUT);
  }
  bool read() { return digitalRead(num) == HIGH; }
  float read_analog() { return (analogRead(num) / 1023.0); }
  int read_analog_raw() { return analogRead(num); }
  bool read_threshold() { return read_analog() >= threshold; }
};

// Lock up the board, send an error message over serial, and flash the
// LED forever.
inline void panic(const char *error_message) {
  // Panic and make the LED blink forever.
  OutPin led(LED_BUILTIN);
  while (true) {
    Serial.println(error_message);
    led.write(false);
    delay(500);
    led.write(true);
    delay(500);
  }
}

// Relay controller.
struct Relay {
  OutPin left;
  OutPin right;
  OutPin common;

  Relay(int left, int right, int common)
      : left(left), right(right), common(common) {}

  // Sets the relay to output on its left pin, and returns a digital
  // output pin that is now connected to the left side of the relay.
  OutPin &output_left() {
    left.write(false);
    right.write(true);
    return common;
  }

  // Sets the relay to output on its right pin, and returns a digital
  // output pin that is now connected to the right side of the relay.
  OutPin &output_right() {
    left.write(true);
    right.write(false);
    return common;
  }
};

template <typename T> T median(const T *data, size_t nmemb) {
  T my_data[nmemb];
  memcpy(my_data, data, nmemb * sizeof(my_data[0]));

  qsort(my_data, nmemb, sizeof(data[0]), [](const void *a, const void *b) {
    if (*(T *)a > *(T *)b)
      return 1;
    else if (*(T *)a < *(T *)b)
      return -1;
    return 0;
  });

  if (nmemb % 2 == 1)
    return (my_data[nmemb / 2] + my_data[nmemb / 2 + 1]) / 2;
  else
    return my_data[nmemb / 2];
}

template <int history_len> class SmoothedInput {
  InPin raw;
  int history[history_len];
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

// // Captured data to be sent to interrupt handlers.
// static void *interrupt_data[EXTERNAL_NUM_INTERRUPTS];

// // Interrupt callback for a particular interrupt number and handler
// // function.
// template <uint8_t num, typename Func> inline void callback_interrupt() {
//   Func &f = *(Func *)interrupt_data[digitalPinToInterrupt(num)];
//   f();
// }

// // Attach an interrupt function to a particular pin. Use e.g.
// // attach_interrupt<8>([&]() { count++; }, FALLING) to count how many
// // times digital pin 8 falls.
// template <uint8_t num, typename Func>
// inline void attach_interrupt_lambda(Func &func, int mode) {
//   if (digitalPinToInterrupt(num) == NOT_AN_INTERRUPT)
//     panic("attempt to set up interrupts on a non-interrupt pin");

//   interrupt_data[digitalPinToInterrupt(num)] = (void *)&func;
//   attachInterrupt(digitalPinToInterrupt(num), callback_interrupt<num, Func>,
//                   mode);
// }

#endif // ARDUINO_LIB_H
