#include <MeAuriga.h>
#include <Wire.h>
#include <Arduino.h>

const int LEDNUM = 12;
const int LEDPIN = 44;

const float WHEEL_DIAMETER = 6.5;
const float WHEEL_PERIMETER = WHEEL_DIAMETER * 3.1416; 
const int DIST = 120; // Distance pour les segments A et C (en cm)

MeEncoderOnBoard encoder_1(SLOT1);
MeEncoderOnBoard encoder_2(SLOT2);

MeRGBLed led(0, LEDNUM);
MeGyro gyro(0, 0x69); 

// Détecteur ultrasonique
MeUltrasonicSensor ultraSensor(PORT_10);
int ultrasonicDistance = 0;

enum State {SETUP, ROAD_A, TURN_A, ROAD_B, TURN_B, ROAD_C, TURN_FULL, FINISH};
State currentState = SETUP;

void isr_process_encoder1(void) {
  if(digitalRead(encoder_1.getPortB()) == 0) encoder_1.pulsePosMinus();
  else encoder_1.pulsePosPlus();
}

void isr_process_encoder2(void) {
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

// 1. Attente de 3 secondes au démarrage[cite: 2]
void setupState(unsigned long currentTime){
  static unsigned long exitTime = 0;
  static bool firstTime = true;
  const int STOP_RATE = 3000;

  if(firstTime){
    firstTime = false;
    exitTime = currentTime + STOP_RATE;
    Serial.println("Attente de 3 secondes...");
  }

  if(currentTime >= exitTime){
    firstTime = true;
    currentState = ROAD_A;
  }
}

// Fonction de maintien de ligne droite via le gyroscope (avec réinitialisation de previousError)
void goStraight(short speed = 100, short firstRun = 0, double kp = 12.0, double kd = 1.0) {
    static double zAngleGoal = 0.0;
    static double error = 0.0;
    static double previousError = 0.0;
    static double output = 0;
    
    if (firstRun) {
      zAngleGoal = gyro.getAngleZ();
      previousError = 0.0; // Correction pour éviter le pic de dérivée au changement de segment
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

// Calcule la distance parcourue pour un segment spécifique (en partant de 0 à chaque segment)
float segmentDistanceCm(long start1, long start2) {
  float posMoy = (abs(encoder_1.getCurPos() - start1) + abs(encoder_2.getCurPos() - start2)) / 2.0;
  return (posMoy / 360.0) * WHEEL_PERIMETER;
}

// Gestion sécurisée de la barre de progression (LEDs 1 à 7)
void progressiveLeds(float myDist, float targetDist){
  int nbLeds = constrain(map(myDist, 0, targetDist, 1, 7), 1, 7);
  for(int i = 1; i <= nbLeds; i++){
    led.setColorAt(i, 0, 255, 0);
  }
  led.show();
}

// Trajet - Segment 1[cite: 2]
void freeTripState(){
  static bool firstTime = true;
  static long startPos1 = 0;
  static long startPos2 = 0;

  if(firstTime){
    firstTime = false;
    startPos1 = encoder_1.getCurPos();
    startPos2 = encoder_2.getCurPos();
    goStraight(100, 1, 22.5, 10.0);
    led.setColor(0, 0, 0);
  }

  goStraight(100, 0, 22.5, 10.0);
  float distParcourue = segmentDistanceCm(startPos1, startPos2);
  progressiveLeds(distParcourue, DIST);

  if(distParcourue >= DIST){
    firstTime = true;
    led.setColor(0, 0, 0);
    led.setColorAt(12, 0, 255, 0); // LED 12 s'allume et reste VERT à la fin du segment 1[cite: 2]
    led.show();
    stopEncoders();
    currentState = TURN_A;
  }
}

// Fonction de rotation précise
bool spin(double angleDegres, short speed = 80, short firstRun = 0) {
  static double zStart = 0.0;
  const double TOLERANCE = 4.0;   // Tolérance élargie à 4.0 degrés pour stabiliser l'arrêt

  if (firstRun) {
    zStart = gyro.getAngleZ();
    return false;
  }

  double fait = gyro.getAngleZ() - zStart;
  double reste = angleDegres - fait;

  if (abs(reste) <= TOLERANCE) {
    stopEncoders();
    return true;
  }

  short dir = (reste > 0) ? 1 : -1;
  short vit = (abs(reste) < 25) ? speed / 2 : speed; 

  encoder_1.setTarPWM(dir * vit);
  encoder_2.setTarPWM(dir * vit);
  return false;
}

// Pivot 1 (sens selon DA pair/impair, ex: -90° ou 90°)[cite: 2]
void turnState(){
  const int ANGLE_ROTATION = -90; 
  int speed = 80;
  static bool angleMemorised = true;

  if(angleMemorised){
    spin(ANGLE_ROTATION, speed, 1);
    angleMemorised = false;
  }

  if(spin(ANGLE_ROTATION, speed, 0)){
    angleMemorised = true;
    currentState = ROAD_B;
  }
}

// Trajet - Segment 2 (Arrêt à 30 cm de l'obstacle)[cite: 3]
void obstacleTripState(){
  const int STOP_DIST = 30;
  static bool firstTime = true;
  static long startPos1 = 0;
  static long startPos2 = 0;

  if(firstTime){
    firstTime = false;
    startPos1 = encoder_1.getCurPos();
    startPos2 = encoder_2.getCurPos();
    goStraight(100, 1, 7.0, 15.0);
    led.setColor(0, 0, 0);
  }

  goStraight(100, 0, 7.0, 15.0);
  float distParcourue = segmentDistanceCm(startPos1, startPos2);
  progressiveLeds(distParcourue, 150.0); 

  if(ultrasonicDistance <= STOP_DIST){
    firstTime = true;
    led.setColor(0, 0, 0);
    led.setColorAt(11, 0, 255, 0); // LED 11 s'allume et reste VERT à la fin du segment 2[cite: 3]
    led.show();
    stopEncoders();
    currentState = TURN_B;
  }
}

// Pivot 2 (Sens opposé du premier pivot)[cite: 3]
void turnOppositeState(){
  const int ANGLE_ROTATION = 90; 
  int speed = 80;
  static bool angleMemorised = true;

  if(angleMemorised){
    spin(ANGLE_ROTATION, speed, 1);
    angleMemorised = false;
  }

  if(spin(ANGLE_ROTATION, speed, 0)){
    angleMemorised = true;
    currentState = ROAD_C;
  }
}

// Trajet - Segment 3 (Ligne droite avec gyroscope et LED 10)[cite: 3]
void ledTripState(){
  static bool firstTime = true;
  static long startPos1 = 0;
  static long startPos2 = 0;
  bool transition = false;

  if(firstTime){
    firstTime = false;
    startPos1 = encoder_1.getCurPos();
    startPos2 = encoder_2.getCurPos();
    
    goStraight(100, 1, 12.0, 5.0); 

    led.setColor(0, 0, 0);
  }

  goStraight(100, 0, 12.0, 5.0);
  
  float distParcourue = segmentDistanceCm(startPos1, startPos2);
  progressiveLeds(distParcourue, DIST);

  transition = (distParcourue >= DIST);
  if(transition){
    firstTime = true;
    
    led.setColor(0, 0, 0);
    led.setColorAt(10, 0, 255, 0); // LED 10 s'allume et reste VERT en fin de segment 3[cite: 3]
    led.show();
    
    stopEncoders();
    currentState = TURN_FULL;
  }
}

// Pivot final de 180 degrés (Vitesse réduite à 50 pour éviter l'oscillation infinie)[cite: 3]
void turnFullState(){
  const int ANGLE_ROTATION = 180; 
  int speed = 50; 
  static bool angleMemorised = true;

  if(angleMemorised){
    spin(ANGLE_ROTATION, speed, 1);
    angleMemorised = false;
  }

  if(spin(ANGLE_ROTATION, speed, 0)){
    angleMemorised = true;
    currentState = FINISH;
  }
}

// FIN : LED 8 à 12 clignotent en VERT à 10 Hz[cite: 3]
void finishState(unsigned long currentTime){
  const int BLINK_RATE = 50; // 50ms allumé, 50ms éteint = période de 100ms (10 Hz)
  static unsigned long lastTime = 0;
  static bool isOn = true;

  if(currentTime - lastTime >= BLINK_RATE){
    if(isOn){
      led.setColor(0, 0, 0);
    } else {
      for(int i = 8; i <= 12; i++){
        led.setColor(i, 0, 255, 0);
      }
    }
    led.show();
    isOn = !isOn;
    lastTime = currentTime;
  } 
}

void readDistanceTask(unsigned long currentTime){
  static unsigned long previousTime = 0;
  const int RATE_READ = 100;

  if(currentTime - previousTime >= RATE_READ){
    previousTime = currentTime;
    ultrasonicDistance = ultraSensor.distanceCm();
  }
}

void stopEncoders(){
  encoder_1.setTarPWM(0);
  encoder_2.setTarPWM(0);
}