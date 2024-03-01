int speed = 0;
int dir = 1;

int in = -255;
int ax = 255;

int f = 3;
int b = 5;
int i = 7;

const int encoderPin = 2; 
// Encoders
volatile int encoderCount = 0;
unsigned long lastMillis = 0; 
double rpm = 0;
int interval = 1000;

void setup() {
    Serial.begin(115200);
    pinMode(f, OUTPUT);
    pinMode(b, OUTPUT);
    pinMode(i, OUTPUT);
    pinMode(encoderPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(encoderPin), encoder, RISING); 
}

void loop() {
    digitalWrite(i, HIGH);
    analogWrite(f, speed);
    speed += dir;
    if (speed >= 255) {
        dir = 0;
    } else if (speed <= 0) {
        dir = 1;
    }
    delay(50);

    // Get rpm
    unsigned long currentMillis = millis();
    if (currentMillis - lastMillis >= 1000) { 
    noInterrupts(); 
    rpm = (encoderCount * 60 * (1000 / interval)) / 120.0; 
    // rpm = encoderCount / 360  * 60;
    encoderCount = 0; 
    interrupts(); 
    Serial.println("RPM: " + String(rpm));
    lastMillis = currentMillis;
    }
}

void encoder() {
  encoderCount++; 
}
