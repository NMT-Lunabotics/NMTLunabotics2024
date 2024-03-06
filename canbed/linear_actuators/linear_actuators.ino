// Define pin numbers
const int spd = 6;
const int dir = 7;
const int potPin = A0;

int tgt = 100;      // in mm

int pwm = 200;      // speed, in pwm TODO change this to mm/s
int stroke = 250;   // stroke length, in mm
int potMin = 34;    // Calibrated, pot val at min stroke
int potMax = 945;   // Calibrated, pot val at max stroke
float weight = 0.2; // How much old readings influence new ones

float smooth = 0;   // Smoothed pot vals
int pos = 0;        // Position in mm
int tolerance = 1;  // Position tolerance
const int numStored = 10;
float readings[numStored];
int index = 0;
float total = 0;

void setup() {
  Serial.begin(9600);
  pinMode(spd, OUTPUT);
  pinMode(dir, OUTPUT);
  pinMode(potPin, INPUT);
}

void loop() {
  // Handle new readings
  int newVal = analogRead(potPin);
  // Serial.println(newVal);
  smooth = smooth * weight + newVal * (1 - weight);
  total -= readings[index];
  readings[index] = newVal;
  total += newVal;
  index = (index + 1) % numStored;
  float avg = total / numStored;

  pos = map(newVal, potMin, potMax, 0, stroke);
  // Serial.println(pos);

  delay(10);

  int a_spd = abs(pos - tgt);
  a_spd = a_spd > pwm ? pwm : a_spd;
  Serial.println(a_spd);

  if (pos > tgt) {
    digitalWrite(dir, LOW);
    analogWrite(spd, a_spd);
  } else if (pos < tgt) {
    digitalWrite(dir, HIGH);
    analogWrite(spd, a_spd);
  } else {
    analogWrite(spd, 0);
  }

  delay(5);
}
