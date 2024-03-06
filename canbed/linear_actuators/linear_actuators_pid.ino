 #include <Arduino.h>

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

float kP=.2;
float kI=.001;
float kD=.01;
float derivative;
float error;
float prevError=0;
float integral=0;
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
    
    error=tgt-pos;
    derivative=error-prevError;
    prevError=error;
    integral+=error;
    int output=kP*error+kI*integral+kD*derivative;
    output=constrain(output,0,250);
    if (output > 0) {
      digitalWrite(dir, LOW);
      analogWrite(spd, abs(output));
    } else if (output<0) {
      digitalWrite(dir, HIGH);
      analogWrite(spd, abs(output));
    } else {
      analogWrite(spd, 0);
    }
     
  }
}

void loop() {}
~                                                                                                        
~   
