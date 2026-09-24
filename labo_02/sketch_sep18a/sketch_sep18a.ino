#include <MeAuriga.h>
#include <Wire.h>
#include <Arduino.h>

enum State {FORWARD, SLOW, STOP, BACKWARD, TURN};

State currentState = FORWARD;

//distance
int distance = 999;

int maxPwm = 255 * 0.6;
int halfPwm = maxPwm / 2;
int turnPwm = 150;

//Motor Left
const int m1_pwm = 11;
const int m1_in1 = 48; // M1 ENA
const int m1_in2 = 49; // M1 ENB

//Motor Right
const int m2_pwm = 10;
const int m2_in1 = 47; // M2 ENA
const int m2_in2 = 46; // M2 ENB

//Anneau lumineux
const int LEDNUM = 12;
const int LEDPIN = 44;

MeRGBLed led(0, LEDNUM);

//Detecteur ultrasonique

MeUltrasonicSensor ultraSensor(PORT_10);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  led.setpin(LEDPIN);

  pinMode(m1_pwm, OUTPUT);
  pinMode(m1_in1, OUTPUT);
  pinMode(m1_in2, OUTPUT);

  pinMode(m2_pwm, OUTPUT);
  pinMode(m2_in1, OUTPUT);
  pinMode(m2_in2, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:

  unsigned long currentTime = millis();

  readDistanceTask(currentTime);
  stateManager(currentTime);
  serialPrintTask(currentTime);
}


void readDistanceTask(unsigned long currentTime){
  static unsigned long previousTime = 0;
  const int RATE_READ = 100;

  if(currentTime - previousTime >= RATE_READ){
    previousTime = currentTime;

    distance = ultraSensor.distanceCm();
  }
}

void forwardState(unsigned long currentTime){             // N.A: 2449427
  static bool firstTime = true;
  bool transition = false;

  if(firstTime){
    firstTime = false;
    
    moveForward();
    led.setColor(0, 0, 0, 0);
    for(int i = 7; i <= 12; i++){
      led.setColor(i, 0, 255, 0);
    }
    led.show();
  }

  transition = (distance < 100);
  if(transition){
    firstTime = true;
    currentState = SLOW;
  }
}

void slowState(unsigned long currentTime){
  static bool firstTime = true;
  bool transition_forward = false;
  bool transition_stop = false;

  if(firstTime){
    firstTime = false;

    moveSlow();
    led.setColor(0, 0, 0, 0);
    for(int i = 7; i <= 12; i++){
      led.setColor(i, 255, 255, 0);
    }
    led.show();
  }

  transition_forward = (distance >= 100);
  transition_stop = (distance < 30);

  if (transition_forward){
    firstTime = true;
    currentState = FORWARD;
  }else if(transition_stop){
    firstTime = true;
    currentState = STOP;
  }
}

void stopState(unsigned long currentTime){
  static unsigned long exitTime = 0;
  static bool firstTime = true;
  const int STOP_RATE = 2000;
  bool transition = false;

  if(firstTime){
    firstTime = false;
    exitTime = currentTime + STOP_RATE;

    stop();
    led.setColor(0, 0, 0, 0);
    led.setColor(255, 0, 0);
    led.show();
  }

  transition = (currentTime >= exitTime);
  if(transition){
    firstTime = true;
    currentState = BACKWARD;
  }
}

void backwardState(unsigned long currentTime){
  static unsigned long exitTime = 0;
  static bool firstTime = true;
  const int BACKING_RATE = 1000;
  bool transition = false;

  if(firstTime){
    firstTime = false;
    exitTime = currentTime + BACKING_RATE;

    moveBackward();
  }

  transition = (currentTime >= exitTime);
  if(transition){
    firstTime = true;
    currentState = TURN;
  }
}

void turnState(unsigned long currentTime){
  static unsigned long exitTime = 0;
  static bool firstTime = true;
  const int TURNING_RATE = 1100;
  bool transition = false;

  if(firstTime){
    firstTime = false;
    exitTime = currentTime + TURNING_RATE;

    turnAround();
  }

  transition = (currentTime >= exitTime);
  if(transition){
    firstTime = true;
    currentState = FORWARD;
  }
}
  
void serialPrintTask(unsigned long currentTime){
  static unsigned long lastTime = 0;
  const int RATE = 250;

  if(currentTime - lastTime >= RATE){
    lastTime = currentTime;

    Serial.print("DISTANCE: ");
    Serial.print(distance);
    Serial.println(" cm");
  }
}

void stateManager(unsigned long currentTime){

  switch (currentState){
    case FORWARD:
      forwardState(currentTime);
      break;
    case SLOW:
      slowState(currentTime);
      break;
    case STOP:
      stopState(currentTime);
      break;
    case BACKWARD:
      backwardState(currentTime);
      break;
    case TURN:
      turnState(currentTime);
      break;
    default:
      currentState = STOP;
      break;
  }
}

void moveForward(){
  digitalWrite(m1_in1, HIGH);
  digitalWrite(m1_in2, LOW);
  analogWrite(m1_pwm, maxPwm);

  digitalWrite(m2_in1, HIGH);
  digitalWrite(m2_in2, LOW);
  analogWrite(m2_pwm, maxPwm);
}

void moveSlow(){
  digitalWrite(m1_in1, HIGH);
  digitalWrite(m1_in2, LOW);
  analogWrite(m1_pwm, halfPwm);

  digitalWrite(m2_in1, HIGH);
  digitalWrite(m2_in2, LOW);
  analogWrite(m2_pwm, halfPwm); 
}

void stop() {
  analogWrite(m1_pwm, 0);
  analogWrite(m2_pwm, 0);
}

void moveBackward(){
  digitalWrite(m1_in1, LOW);
  digitalWrite(m1_in2, HIGH);
  analogWrite(m1_pwm, maxPwm);

  digitalWrite(m2_in1, LOW);
  digitalWrite(m2_in2, HIGH);
  analogWrite(m2_pwm, maxPwm);
}

void turnAround(){
  digitalWrite(m1_in1, HIGH);
  digitalWrite(m1_in2, LOW);
  analogWrite(m1_pwm, turnPwm);

  digitalWrite(m2_in1, LOW);
  digitalWrite(m2_in2, HIGH);
  analogWrite(m2_pwm, turnPwm);
}
