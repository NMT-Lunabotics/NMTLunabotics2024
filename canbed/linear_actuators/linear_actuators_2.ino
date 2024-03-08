#include <Arduino.h>

#include "arduino_lib.hpp"
#include <Protothreads.h>
// Define pin numbers
#define PIN_SPEED_LEFT 6
#define PIN_SPEED_RIGHT 9
#define PIN_DIRECTION_LEFT 7
#define PIN_DIRECTION_RIGHT 10
#define PIN_POTENTIOMETER_LEFT A0
#define PIN_POTENTIOMETER_RIGHT A1

int target = 100; // in mm

int pwm = 200;    // speed, in pwm TODO change this to mm/s
int stroke = 300; // stroke length, in mm
int potMin = 34;  // Calibrated, pot val at min stroke
int potMax = 945; // Calibrated, pot val at max stroke

float kP = 12;
float kI = 0;
float kD = .0;
float derivative;
float errorL;
float prevErrorL = 0;
float integralL = 0;
float errorR;
float prevErrorR=0;
float integralR=0;
float threshold = 1;
float i_threshold = 10;
struct pt ptL,ptR;
static int median(const int *data, size_t nmemb);
  OutPin speed_left(PIN_SPEED_LEFT);
  OutPin speed_right(PIN_SPEED_RIGHT);
  OutPin dir_left(PIN_DIRECTION_LEFT);
  OutPin dir_right(PIN_DIRECTION_RIGHT);
  InPin pot_left(PIN_POTENTIOMETER_LEFT);
  InPin pot_right(PIN_POTENTIOMETER_RIGHT);

void PIDLeft(struct pt *pt){
  PT_BEGIN(pt);

  while(1){
    int newVal=analogRead(PIN_POTENTIOMETER_LEFT);
    int pos=map(newVal,potMin,potMax,0,stroke);
    errorL=target-pos;
    if(abs(errorL)>=threshold){
      derivative=errorL-prevErrorL;
      prevErrorL=errorL;
      if(abs(errorL)<=i_threshold&&abs(errorL)!=0){
        integralL+=errorL;
      } else {
        integralL=0;
      }
      int output=kP*errorL+kI*integralL+kD*derivative;
      output=constrain(output,-pwm,pwm);
      dir_left.write(output>0);
      if(abs(errorL)<=threshold){
        speed_left.write_pwm(0);
      } else {
        speed_left.write_pwm(abs(output));
      }
    }
    PT_WAIT_UNTIL(pt,millis()%50==0);
  }
  PT_END(pt);
}

void PIDRight(struct pt *pt){
  PT_BEGIN(pt);

  while(1){
    int newVal=analogRead(PIN_POTENTIOMETER_RIGHT);
    int pos=map(newVal,potMin,potMax,0,stroke);
    errorR=target-pos;
    if(abs(errorR)>=threshold){
      derivative=errorR-prevErrorR;
      prevErrorR=errorR;
      if(abs(errorR)<=i_threshold&&abs(errorR)!=0){
        integralR+=errorR;
      } else {
        integralR=0;
      }
      int output=kP*errorR+kI*integralR+kD*derivative;
      output=constrain(output,-pwm,pwm);
      dir_right.write(output>0);
      if(abs(errorR)<=threshold){
        speed_right.write_pwm(0);
      } else {
        speed_right.write_pwm(abs(output));
      }
    }
    PT_WAIT_UNTIL(pt,millis()%50==0);
  }
  PT_END(pt);
}

void setup() {
  Serial.begin(9600);
 

  int history[15];
  int history2[15];
  memset(history, 0, sizeof(history));
  memset(history2,0,sizeof(history2));
  int v = pot_left.read_analog_raw();
  int v2=pot_right.read_analog_raw();
  for (int i = 0; i < sizeof(history) / sizeof(history[0]); i++)
    history[i] = v;
  for (int i=0;i<sizeof(history2)/sizeof(history2[0]);i++)
    history2[i]=v2;
  size_t head = 0;
  size_t head2=0;
  PT_INIT(&ptL);
  PT_INIT(&ptR);
  while (true) {
    // Handle new readings
    

    delay(50);

    //Slight problem if you want to have each linear actuator have a pid and a pid of the error between them to chain them together you want the between pid to output a multiplier of faster or slower into the other pids which is multithreading and not possible on arduino
  

    Serial.print(output);
    Serial.print(" ");
    Serial.println(pos);
    // When here, output is between -pwm and pwm. Output 0 will stop;
  }
}

void loop() {}

static int median(const int *data, size_t nmemb) {
  int my_data[nmemb];
  memcpy(my_data, data, nmemb * sizeof(my_data[0]));

  qsort(my_data, nmemb, sizeof(data[0]), [](const void *a, const void *b) {
    if (*(int *)a > *(int *)b)
      return 1;
    else if (*(int *)a < *(int *)b)
      return -1;
    return 0;
  });

  if (nmemb % 2 == 1)
    return (my_data[nmemb / 2] + my_data[nmemb / 2 + 1]) / 2;
  else
    return my_data[nmemb / 2];

    PIDRight(&ptR);
    PIDLeft(&ptL);
}
