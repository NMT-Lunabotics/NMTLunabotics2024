int speed = 0;
int dir = 1;

int in = -255;
int ax = 255;

int f = 3;
int b = 5;

void setup() {
  Serial.begin(115200);
}

void loop() {
    if (speed >= 0) {
        analogWrite(f, speed);
        analogWrite(b, 0);
    } else {
        analogWrite(b, -speed);
        analogWrite(f, 0);
    }

    if (speed > ax) {
        dir = -1;
    } else if (speed < in) {
        dir = 1;
    }
    speed += dir;
    Serial.println(speed);
    delay(10);
}
