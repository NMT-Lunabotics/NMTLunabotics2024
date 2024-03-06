#include <Arduino.h>

#include <stdlib.h>

// Define pin numbers
const int spd = 6;
const int dir = 7;
const int potPin = A0;

int tgt = 100; // in mm

int pwm = 200;      // speed, in pwm TODO change this to mm/s
int stroke = 250;   // stroke length, in mm
int potMin = 34;    // Calibrated, pot val at min stroke
int potMax = 945;   // Calibrated, pot val at max stroke
float weight = 0.2; // How much old readings influence new ones

void setup() {
  Serial.begin(9600);
  pinMode(spd, OUTPUT);
  pinMode(dir, OUTPUT);
  pinMode(potPin, INPUT);

  int lastValue = analogRead(potPin);
  while (true) {
    // Handle new readings
    int newVal = analogRead(potPin);

    if (abs(newVal - lastValue) > 20) {
      Serial.println("Ignoring jump");
      lastValue = newVal;
      delay(50);
      continue;
    }

    lastValue = newVal;

    int pos = map(newVal, potMin, potMax, 0, stroke);
    Serial.print(newVal);
    Serial.print("\t:\t");
    Serial.println(pos);

    delay(50);

    if (pos > tgt) {
      digitalWrite(dir, LOW);
      analogWrite(spd, pwm);
    } else if (pos < tgt) {
      digitalWrite(dir, HIGH);
      analogWrite(spd, pwm);
    } else {
      analogWrite(spd, 0);
    }
  }
}

void loop() {}

double median(const double *data, size_t nmemb) {
  double my_data[nmemb];
  memcpy(my_data, data, nmemb * sizeof(my_data[0]));

  qsort(my_data, nmemb, sizeof(data[0]), [](const void *a, const void *b) {
    if (*(double *)a > *(double *)b)
      return 1;
    else if (*(double *)a < *(double *)b)
      return -1;
    return 0;
  });

  if (nmemb % 2 == 1)
    return (my_data[nmemb / 2] + my_data[nmemb / 2 + 1]) / 2;
  else
    return my_data[nmemb / 2];
}
