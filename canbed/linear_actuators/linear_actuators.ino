#include <Arduino.h>

// Define pin numbers
const int spd = 6;
const int dir = 7;
const int potPin = A0;

int tgt = 100; // in mm

int pwm = 200;      // speed, in pwm TODO change this to mm/s
int stroke = 300;   // stroke length, in mm
int potMin = 34;    // Calibrated, pot val at min stroke
int potMax = 945;   // Calibrated, pot val at max stroke
float weight = 0.2; // How much old readings influence new ones

float kP = .4;
float kI = .00;
float kD = .0;
float derivative;
float error;
float prevError = 0;
float integral = 0;
float threshold = 20;
float threshold2 = 1;

int median(const int *data, size_t nmemb);

void setup() {
  Serial.begin(9600);
  pinMode(spd, OUTPUT);
  pinMode(dir, OUTPUT);
  pinMode(potPin, INPUT);

  // int lastValue = analogRead(potPin);

  int history[15];
  memset(history, 0, sizeof(history));
  int v = analogRead(potPin);
  for (int i = 0; i < sizeof(history) / sizeof(history[0]); i++)
    history[i] = v;

  size_t head = 0;

  while (true) {
    // Handle new readings
    int newVal = analogRead(potPin);

    history[head] = newVal;
    head = (head + 1) % (sizeof(history) / sizeof(history[0]));

    Serial.print(newVal);
    Serial.print("\t|\t");
    newVal = median(history, sizeof(history) / sizeof(history[0]));
    int pos = map(newVal, potMin, potMax, 0, stroke);

    // Serial.print(newVal);
    // Serial.print("\t:\t");
    Serial.println(pos);

    delay(50);
    error = tgt - pos;
    if (error < threshold &&
        (error > error + threshold2 || error > error - threshold2)) {
      error = tgt - pos;
      derivative = error - prevError;
      prevError = error;
      integral += error;
      int output = kP * error + kI * integral + kD * derivative;
      output = constrain(output, -200, 200);
      Serial.println(output);

      // pos = 500;
      Serial.print(newVal);
      Serial.print("\t:\t");
      Serial.println(pos);

      delay(5);

      if (pos > tgt + 2) {
        digitalWrite(dir, LOW);
        analogWrite(spd, pwm);
      } else if (pos < tgt - 2) {
        digitalWrite(dir, HIGH);
        analogWrite(spd, pwm);
      } else {
        analogWrite(spd, 0);
      }

      // When here, output is between -200 and 200. Output 0 will stop;
      digitalWrite(dir, output > 0);
      if (output == 0) {
        analogWrite(spd, 0);
      } else {
        analogWrite(spd, map(abs(output), 0, 200, 30, 200));
      }
      delay(50);
    }
  }
}

void loop() {}
int median(const int *data, size_t nmemb) {
  int my_data[nmemb];
  memcpy(my_data, data, nmemb * sizeof(my_data[0]));

  // Serial.print('{');
  // for (int i = 0; i < nmemb; i++) {
  //   Serial.print(my_data[i]);
  //   Serial.print(", ");
  // }
  // Serial.print("}");

  qsort(my_data, nmemb, sizeof(data[0]), [](const void *a, const void *b) {
    if (*(int *)a > *(int *)b)
      return 1;
    else if (*(int *)a < *(int *)b)
      return -1;
    return 0;
  });

  // Serial.print(" -> {");
  // for (int i = 0; i < nmemb; i++) {
  //   Serial.print(my_data[i]);
  //   Serial.print(", ");
  // }
  // Serial.print("} ");

  if (nmemb % 2 == 1)
    return (my_data[nmemb / 2] + my_data[nmemb / 2 + 1]) / 2;
  else
    return my_data[nmemb / 2];
}
