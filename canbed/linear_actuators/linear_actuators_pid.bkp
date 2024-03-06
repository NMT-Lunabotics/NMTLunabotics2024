 #include <Arduino.h>

// Define pin numbers
const int spd = 6;
const int dir = 7;
const int potPin = A0;

int tgt = 50;      // in mm

int pwm = 200;      // speed, in pwm TODO change this to mm/s
int stroke = 300;   // stroke length, in mm
int potMin = 34;    // Calibrated, pot val at min stroke
int potMax = 945;   // Calibrated, pot val at max stroke

float kP=3;
float kI=1;
float kD=0;
float derivative;
float error;
float prevError=0;
float integral=0;
float threshold=1;
float i_threshold = 20;
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
      // Serial.println("Ignoring jump");
      lastValue = newVal;
      delay(50);
      continue;
    }

    lastValue = newVal;

    int pos = map(newVal, potMin, potMax, 0, stroke);
    // Serial.print(newVal);
    // Serial.print("\t:\t");
    // Serial.println(pos);

    int output;

    delay(50);
    error=tgt-pos;
    if(abs(error)>=threshold){
      error=tgt-pos;
      derivative=error-prevError;
      prevError=error;
      if (abs(error) <= i_threshold && abs(error) != 0) {
        integral+=error;
      } else {
        integral = 0;
      }
      output=kP*error+kI*integral+kD*derivative;
      output=constrain(output,-pwm,pwm);
    }

    Serial.print(output);
    Serial.print(" ");
    Serial.println(pos);
    // When here, output is between -pwm and pwm. Output 0 will stop;
  	digitalWrite(dir, output > 0);
    if (abs(error)<=threshold) {
      analogWrite(spd, 0);
    } else {
      analogWrite(spd, abs(output));
    }
    delay(50);
  }
}

void loop() {}
