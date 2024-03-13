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

class LinearActuator {
  OutPin speed;
  OutPin dir;
  SmoothedInput<15> pot;

public:
  LinearActuator(OutPin speed, OutPin dir, InPin pot)
      : speed(speed), dir(dir), pot(pot) {}

  void update() {
    int newVal = pot.read_analog_raw();
    int pos = map(newVal, potMin, potMax, 0, stroke);
     
    //float error = target - pos;
    //if (abs(error) >= threshold) {
      //int output = kP * error;
      //output = constrain(output, -pwm, pwm);
      //dir.write(output > 0);
      //speed.write_pwm(abs(output));
    //} else {
      //speed.write_pwm(0);
    //}
    double vel=0;
    double maxSpeed=200;
    double maxAcc=20;
    for(double deltaT=0;deltaT<maxSpeed/maxAcc;deltaT+=.01){
      trapMotion(maxAcc,maxSpeed,deltaT,pos,vel);
      int ouput=vel;
      dir.write(output>0);
      speed.write_pwm(abs(output));
      delay(50);

    }
  }
  void trapMotion(double maxA,double maxV,double deltaT,double pos,double vel){
    double timeMaxV=maxV/maxA;
    double distMaxV-.5*maxA*timeMaxV*timeMaxV;
    if(deltaT<timeMaxV){
      vel=maxA*deltaT;
      pos=.5*maxA*deltaT*deltaT;

    }else if (deltaT<2*timeMaxV){
      double timeFromMaxVel=deltaT-timeMaxV;
      vel=maxV-maxA*timeMaxV;
      pos=distMaxV+maxV*timeFromMaxVel-.5*maxA*timeFromMaxVel*timeFromMaxVel;

    }else {
      vel=0;
      pos=2*distMaxV;
    }
  }

};
//maxV=200 which is 5m/s
//maxA .5m/s^2? so 20
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
    double pos=0; 
    double vel =0;
    double maxSpeed =200;
    double maxAcc=20;

    delay(50);

  }
}

void loop() {}
