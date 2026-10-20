#include <MeAuriga.h>
#include <Wire.h>
#include <Arduino.h>

int maxPwm = 255 * 0.6;
int halfPwm = maxPwm / 2;
int turnPwm = 150;

const int LEDNUM = 12;
const int ALL_LEDS = 0;
const int LEDPIN = 44;

const float TICKS_TOUR = 9 * 39.267;
const float WHEEL_DIAMETER = 6.5;
const float WHEEL_PERIMETER = WHEEL_DIAMETER * 3.1416;
const int DIST = 120;
float NB_LAPS = DIST / WHEEL_PERIMETER;
float NB_TICKS = NB_LAPS * TICKS_TOUR;

MeEncoderOnBoard encoder_1(SLOT1);
MeEncoderOnBoard encoder_2(SLOT2);

MeRGBLed led(0, LEDNUM);
MeGyro gyro(0, 0x69);  

//Detecteur ultrasonique

MeUltrasonicSensor ultraSensor(PORT_10);
int ultrasonicDistance = 0;

enum State {SETUP, ROAD_A, TURN_A, ROAD_B, TURN_B, ROAD_C, TURN_FULL, FINISH};
State currentState = SETUP;


void isr_process_encoder1(void)
{
  if(digitalRead(encoder_1.getPortB()) == 0) encoder_1.pulsePosMinus();
  else encoder_1.pulsePosPlus();;
}

void isr_process_encoder2(void)
{
  if(digitalRead(encoder_2.getPortB()) == 0) encoder_2.pulsePosMinus();
  else encoder_2.pulsePosPlus();
}

void encoderConfig() {
  attachInterrupt(encoder_1.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(encoder_2.getIntNum(), isr_process_encoder2, RISING);
  
  encoder_1.setPulse(9);
  encoder_2.setPulse(9);
  
  encoder_1.setRatio(39.267);
  encoder_2.setRatio(39.267);
  
  encoder_1.setPosPid(1.8,0,1.2);
  encoder_2.setPosPid(1.8,0,1.2);
  
  encoder_1.setSpeedPid(0.18,0,0);
  encoder_2.setSpeedPid(0.18,0,0);
  
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
}
void setup() {

  Serial.begin(115200);

  encoderConfig();
  led.setpin(LEDPIN);
  gyro.begin();

}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currentTime = millis();

  readDistanceTask(currentTime);
  encodersTask(currentTime);
  gyro.update();
  gestState(currentTime);
}

void encodersTask(unsigned long ct) {
  encoder_1.loop();
  encoder_2.loop();
}

void gestState(unsigned long currentTime){
  switch(currentState){
    case SETUP:
      setupState(currentTime);
      break;
    case ROAD_A:
      freeTripState();
      break;
    case TURN_A:
      turnState();
      break;
    case ROAD_B:
      obstacleTripState();
      break;
    case TURN_B:
      turnOppositeState();
      break;
    case ROAD_C:
      ledTripState();
      break;
    case TURN_FULL:
      turnFullState();
      break;
    case FINISH:
      finishState(currentTime);
      break;
    default:
      currentState = FINISH;
      break;
  }
}

void setupState(unsigned long currentTime){
  static unsigned long exitTime = 0;
  static bool firstTime = true;
  const int STOP_RATE = 3000;
  bool transition = false;

  if(firstTime){
    firstTime = false;
    exitTime = currentTime + STOP_RATE;

    Serial.println("Attente");
  }

  transition = (currentTime >= exitTime);
  if(transition){
    firstTime = true;
    currentState = ROAD_A;
  }
}

void goStraight(short speed = 100, short firstRun = 0, double kp = 12.0, double kd = 1.0) {
    static double zAngleGoal = 0.0;
    
    static double error = 0.0;
    static double previousError = 0.0;
    static double output = 0;
    
    //const double kp = 12.0;
    //const double ki = 1.0;
    //const double kd = 1.0;
    
    if (firstRun) {
      firstRun = 0;

      zAngleGoal = gyro.getAngleZ();
      Serial.println ("Setting speed");
      
      encoder_1.setTarPWM(-speed);
      encoder_2.setTarPWM(speed);
      
      return;
    }
    
    error = gyro.getAngleZ() - zAngleGoal;
    
    output = kp * error + kd * (error - previousError);
    
    previousError = error;        
    
    encoder_1.setTarPWM(-speed - output);
    encoder_2.setTarPWM(speed - output);
}

void freeTripState(){
  static bool firstTime = true;
  bool transition = false;

  if(firstTime){
    firstTime = false;
    goStraight(100, 1, 22.5, 10.0);

    led.setColor(0, 0, 0);
    //Serial.println("Ligne droite");
  }

  goStraight(100, 0, 22.5, 10.0);
  progressiveLeds(DIST);

  transition = (distanceParcourueCm() >= DIST);
  if(transition){
    firstTime = true;
    
    led.setColor(0, 0, 0);
    led.setColorAt(11, 0, 255, 0);
    led.show();
    
    stopEncoders();
    
    currentState = TURN_A;
  }
}

// Retourne true quand la rotation est terminée
bool spin(double angleDegres, short speed = 80, short firstRun = 0) {
  static double zStart = 0.0;
  const double TOLERANCE = 2.0;   // degrés

  if (firstRun) {
    zStart = gyro.getAngleZ();
    return false;
  }

  double fait = gyro.getAngleZ() - zStart;
  double reste = angleDegres - fait;

  // Serial.print("fait: ");
  // Serial.print(fait);
  // Serial.print(", reste: ");
  // Serial.println(reste);

  if (abs(reste) <= TOLERANCE) {
    stopEncoders();
    return true;
  }

  // Signe de la rotation selon le reste à faire
  short dir = (reste > 0) ? 1 : -1;

  // Ralentit près de la cible pour limiter le dépassement
  short vit = (abs(reste) < 20) ? speed / 2 : speed;

  encoder_1.setTarPWM(dir * vit);
  encoder_2.setTarPWM(dir * vit);
  return false;
}

void turnState(){
  const int ANGLE_ROTATION = -90;       // N.A: 2449427 (NA[6]: pair)
  int speed = 80;
  static bool angleMemorised = true;

  if(angleMemorised){
    spin(ANGLE_ROTATION, speed, 1);
    angleMemorised = false;
  }

  bool transition = spin(ANGLE_ROTATION, speed, 0);

  if(transition){
    angleMemorised = true;
    currentState = ROAD_B;
  }
}

void obstacleTripState(){
  const int STOP_DIST = 30;
  static bool firstTime = true;
  bool transition = false;

  if(firstTime){
    firstTime = false;
    
    goStraight(100, 1, 7.0, 15.0);
    led.setColor(0, 0, 0);
  }

  goStraight(100, 0, 7.0, 15.0);
  progressiveLeds(STOP_DIST);

  transition = (ultrasonicDistance <= STOP_DIST);
  if(transition){
    firstTime = true;

    led.setColor(0, 0, 0);
    led.setColorAt(10, 0, 255, 0);
    led.show();

    stopEncoders();
    currentState = TURN_B;
  }
}
void turnOppositeState(){
  const int ANGLE_ROTATION = 90;       // N.A: 2449427 (NA[6]: pair)
  int speed = 80;
  static bool angleMemorised = true;

  if(angleMemorised){
    spin(ANGLE_ROTATION, speed, 1);
    angleMemorised = false;
  }

  bool transition = spin(ANGLE_ROTATION, speed, 0);

  if(transition){
    angleMemorised = true;
    currentState = ROAD_C;
  }
}
void ledTripState(){
  static bool firstTime = true;
  bool transition = false;

  if(firstTime){
    firstTime = false;
    goStraight(100, 1, 22.5, 10.0);

    led.setColor(0, 0, 0);
    //Serial.println("Ligne droite");
  }

  goStraight(100, 0, 22.5, 10.0);
  progressiveLeds(DIST);

  transition = (distanceParcourueCm() >= DIST);
  if(transition){
    firstTime = true;
    
    led.setColor(0, 0, 0);
    led.setColorAt(9, 0, 255, 0);
    led.show();
    
    stopEncoders();
    
    currentState = TURN_FULL;
  }
}
void turnFullState(){
  const int ANGLE_ROTATION = 180;       
  int speed = 80;
  static bool angleMemorised = true;

  if(angleMemorised){
    spin(ANGLE_ROTATION, speed, 1);
    angleMemorised = false;
  }

  bool transition = spin(ANGLE_ROTATION, speed, 0);

  if(transition){
    angleMemorised = true;
    currentState = FINISH;
  }
}
void finishState(unsigned long currentTime){
  const int BLINK_RATE = 50; // (1000s / 20)
  static unsigned long lastTime = 0;
  static bool isOn = true;

  if(currentTime - lastTime >= BLINK_RATE){
    if(isOn){
      led.setColor(0, 0, 0);
      isOn = !isOn;
    }
    else{
      for(int i = 8; i <= 12; i++){
        led.setColor(i, 0, 255, 0);
      }
    }
    led.show();

    lastTime = currentTime;
  } 
}

float distanceParcourueCm() {
  // Moyenne des deux roues pour réduire l'erreur ; abs() car signes opposés
  float posMoy = (abs(encoder_1.getCurPos()) + abs(encoder_2.getCurPos())) / 2.0;
  return (posMoy / 360.0) * WHEEL_PERIMETER;
}

void readDistanceTask(unsigned long currentTime){
  static unsigned long previousTime = 0;
  const int RATE_READ = 100;

  if(currentTime - previousTime >= RATE_READ){
    previousTime = currentTime;

    ultrasonicDistance = ultraSensor.distanceCm();
  }
}

void progressiveLeds(int dist){
  float myDist = distanceParcourueCm();
  int nbLeds = map(myDist, 0, dist, 1, 7);

  for(int i = 1; i <= nbLeds; i++){
    led.setColorAt(i, 0, 255, 0);
  }
  led.show();
}

void stopEncoders(){
  encoder_1.setTarPWM(0);
  encoder_2.setTarPWM(0);
}