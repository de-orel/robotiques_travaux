// N.A: 2249427
// Labo 4 : suivi de ligne (5 capteurs IR) - version isolée, sans le labo 3

#include <MeAuriga.h>
#include <Wire.h>
#include <Arduino.h>
#include <Adafruit_seesaw.h>
#include <ArduinoJson.h>

// -- pivot (gyro)

const short SPEED_PIVOT = 50;
const short SPEED_PIVOT_FINAL = 37;
const double PIVOT_TOLERANCE = 2.0;          // degrés
const unsigned long PIVOT_SETTLE_MS = 300;   // pause après un pivot pour stabiliser le gyro

// -- leds

const int LEDNUM = 12;
const int LEDPIN = 44;

// -- objets

MeEncoderOnBoard encoder_1(SLOT1);   // moteur droit
MeEncoderOnBoard encoder_2(SLOT2);   // moteur gauche
MeRGBLed led(0, LEDNUM);
MeGyro gyro(0, 0x69);

// -- capteurs de ligne

#define NB_IR 5

const int ANGLE_CALIBRATION = 360;   // rotation cumulée pendant la calibration

float integral = 0;
float lastError = 0;

double capCentre = 0;           // dernier cap du robot quand il était bien centré sur la ligne
double angleDemiTour = 180.0;   // angle à parcourir pour le demi-tour (calculé quand la ligne disparaît)
bool spinStarted = false;       // état interne de spin(), global pour pouvoir l'annuler avec "stop"
bool isCalibrated = false;

Adafruit_seesaw ss;
int sensorValues[NB_IR];

int moveSpeed = 75;

struct Capteur{
  int valeurMin = 1023;
  int valeurMax = 0;
  int valeurLue;
  int valeurNormalisee;
};

Capteur capteurs[NB_IR];

// -- machine à états

enum State2 {BOOT, ATTENTE, CALIB, SUIVRE_LIGNE, CMD_INVALIDE, NON_CALIB, DEMI_TOUR};
State2 currentState2 = BOOT;

enum State {ATTENTE, CALIBRATION, DEMARRAGE, RECHERCHER_LIGNE, SUIVRE_LIGNE, VIRAGE, APPROCHE_COLIS, DEMI_TOUR, ATTENTIF, VERIFICATION, STATIONNEMENT, MANUEL, TERMINE, CMD_INVALIDE, NON_CALIB};
State currentState = ATTENTE;

// -- commandes série

String serialMessage = "";
bool messageRecu = false;

void setup() {
  Serial.begin(115200);

  encoderConfig();
  led.setpin(LEDPIN);
  gyro.begin();

  if (!ss.begin()) {
    Serial.println("Erreur de connexion au LyneTracker");
    while (1);
  }
  Serial.println("Connexion réussie au LyneTracker!");
}

void loop() {

  unsigned long currentTime = millis();

  encodersTask();
  gyro.update();
  gestStateLineDetector(currentTime);
}

void serialEvent() {
  if (!Serial.available()) return;

  serialMessage = Serial.readStringUntil('\n');
  serialMessage.trim();
  messageRecu = true;
}

void waitCommandTask() {
  static bool firstTime = true;

  if (firstTime) {
    firstTime = false;
    stopEncoders();
    Serial.println("En attente d'une commande..");
  }

  if (!messageRecu) return;
  messageRecu = false;

  if (serialMessage == "stop") {
    Serial.print("cmd recu:");
    Serial.println(serialMessage);

    firstTime = true;
    currentState2 = ATTENTE;

  } else if (serialMessage == "cal" || serialMessage == "C") {

    Serial.print("cmd recu:");
    Serial.println(serialMessage);

    firstTime = true;
    currentState2 = CALIB;

  } else if (serialMessage == "ligne") {

    Serial.print("cmd recu:");
    Serial.println(serialMessage);

    gyro.update();
    capCentre = gyro.getAngleZ();   // le robot est posé dans l'axe de la ligne
    resetPID();

    firstTime = true;
    currentState2 = SUIVRE_LIGNE;

  } else {
    Serial.println("cmd INCONNUE");
    firstTime = true;
    currentState2 = CMD_INVALIDE;
  }
}

// -- capteurs

void printSensorsValue(unsigned long currentTime){
  static unsigned long lastTime = 0;
  const int DELAY_PRINT = 100;

  if(currentTime - lastTime >= DELAY_PRINT){
    for (int i = 0; i < NB_IR; i++) {
      sensorValues[i] = ss.analogRead(i);
      Serial.print("IR"); Serial.print(i); Serial.print(":");
      Serial.print(sensorValues[i]);
      Serial.print("\t");
    }
    lastTime = currentTime;
    Serial.println();
  }
}

bool ligneVisible(){
  int somme = 0;
  for(int i = 0; i < NB_IR; i++) somme += 1000 - capteurs[i].valeurNormalisee;

  return somme >= 100;
}

void followLine(float adjustment){
  int leftSpeed = moveSpeed + adjustment;
  int rightSpeed = moveSpeed - adjustment;

  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  encoder_1.setTarPWM(-rightSpeed);
  encoder_2.setTarPWM(leftSpeed);
}

void calibrer(){

  for (int i = 0; i < NB_IR; i++){
    capteurs[i].valeurLue = ss.analogRead(i);
    if (capteurs[i].valeurLue < capteurs[i].valeurMin){
      capteurs[i].valeurMin = capteurs[i].valeurLue;
    }
    if (capteurs[i].valeurLue > capteurs[i].valeurMax){
      capteurs[i].valeurMax = capteurs[i].valeurLue;
    }
  }
}

float capteurLectureNormalisee(int index){
  return ((capteurs[index].valeurLue - capteurs[index].valeurMin) * 1.0) / (capteurs[index].valeurMax - capteurs[index].valeurMin) * 1000.0;
}

float calculerPositionLigne(){
  float numerateur = 0;
  float denominateur = 0;

  for(int i = 0; i < NB_IR; i++){
    float v = 1000 - capteurs[i].valeurNormalisee;   // noir = grand poids
    numerateur += v * (i - 2);
    denominateur += v;
  }

  if (denominateur < 100) return 0;   // ligne perdue
  return numerateur / (float)denominateur * 1000;   // environ ±2000
}

void resetPID(){integral = 0.0; lastError = 0.0;}  // on réinitialise l'intégrale et l'erreur précédente

float computePID(float position, float consigne = 0.0f) {
    // Ajuster les coefficients selon vos besoins
    static float kp = 0.05; // Coefficient proportionnel
    static float ki = 0.0;  // Coefficient intégral
    static float kd = 1;    // Coefficient dérivé

    static float derivative = 0;

    float error = position - consigne;

    integral += error;

    // Limiter l'intégrale pour éviter l'emballement intégral
    const float integralLimit = 500;
    integral = constrain(integral, -integralLimit, integralLimit);

    derivative = error - lastError;
    lastError = error;

    float output = kp * error + ki * integral + kd * derivative;

    return output;
}

void normaliserValeurs(){
  for(int i = 0; i < 5; i++){
    capteurs[i].valeurNormalisee = capteurLectureNormalisee(i);
  }
}

// -- états

void bootTask(unsigned long currentTime) {
  static unsigned long exitTime = 0;
  static bool firstTime = true;
  const unsigned long WAIT = 3000;
  bool transition = false;

  if (firstTime) {
    firstTime = false;
    exitTime = currentTime + WAIT;

    Serial.println("BOOT: attente 3 s");
  }

  transition = (currentTime >= exitTime);
  if (transition) {
    firstTime = true;
    currentState2 = ATTENTE;
  }
}

void calibrationTask(){

  gyro.update();

  while(!spin(ANGLE_CALIBRATION, millis())){
    gyro.update();
    encodersTask();
    calibrer();
  }

  isCalibrated = true;   // les capteurs sont calibrés
  currentState2 = ATTENTE;
}

void demiTourTask(unsigned long currentTime){  // lorsqu'on perd la ligne
  // angleDemiTour a été calculé au moment où la ligne a disparu (voir followLineTask)
  if(spin(angleDemiTour, currentTime)){
    capCentre = gyro.getAngleZ();   // nouveau cap de référence dans le nouveau sens de marche
    resetPID();
    currentState2 = SUIVRE_LIGNE;
  }
}

void followLineTask(unsigned long currentTime){

  if(isCalibrated){
    for (int i = 0; i < NB_IR; i++) {
      capteurs[i].valeurLue = ss.analogRead(i);
    }

    normaliserValeurs();

    if (!ligneVisible()) {
      stopEncoders();
      resetPID();

      // angle à parcourir : 180° moins ce dont le robot s'est déjà écarté depuis le dernier cap centré
      double dejaTourne = normalizeAngle(gyro.getAngleZ() - capCentre);
      angleDemiTour = 180.0 - dejaTourne;

      currentState2 = DEMI_TOUR;
      return;
    }
    else {
      float position = calculerPositionLigne();

      if (fabs(position) < 300) {
        capCentre = gyro.getAngleZ();   // robot bien centré : on retient son cap
      }

      float adjustment = computePID(position, 0.0f);
      followLine(adjustment);
    }

  }
  else{
    currentState2 = NON_CALIB;
  }
}

void InvalidCommandTask(unsigned long currentTime){
  const int BLINK_RATE = 250;
  const int DELAY = 3000;
  static unsigned long exitTime = 0;

  static unsigned long lastTimeBlink = 0;
  static bool isOn = false;
  static bool first = true;

  if(first){
    exitTime = currentTime + DELAY;
    lastTimeBlink = currentTime;
    first = false;
  }

  if(currentTime - lastTimeBlink >= BLINK_RATE){
    isOn = !isOn;
    if(isOn){
      led.setColor(0, 0, 0);
    }
    else{
      led.setColor(0, 255, 0, 0);   // rouge
    }

    led.show();
    lastTimeBlink = currentTime;
  }

  bool transition = (currentTime >= exitTime);

  if(transition){
    led.setColor(0, 0, 0);
    led.show();
    first = true;
    currentState2 = ATTENTE;
  }

}

void notCalibratedTask(unsigned long currentTime){
  const int BLINK_RATE = 250;
  const int DELAY = 3000;
  static unsigned long exitTime = 0;

  static unsigned long lastTimeBlink = 0;
  static bool isOn = false;
  static bool first = true;

  if(first){
    exitTime = currentTime + DELAY;
    lastTimeBlink = currentTime;
    first = false;
  }

  if(currentTime - lastTimeBlink >= BLINK_RATE){
    isOn = !isOn;
    if(isOn){
      led.setColor(0, 0, 0);
    }
    else{
      led.setColor(0, 255, 255, 0);   // jaune
    }

    led.show();
    lastTimeBlink = currentTime;
  }

  bool transition = (currentTime >= exitTime);

  if(transition){
    led.setColor(0, 0, 0);
    led.show();
    first = true;
    currentState2 = ATTENTE;
  }
}

void eteindreLeds(){
  for(int i = 1; i <= LEDNUM; i++){
    led.setColor(i, 0, 0, 0);
  }

  led.setColor(0, 0, 0, 0);
  led.show();
}

void gestStateLineDetector(unsigned long currentTime) {

  // "stop" est accepté dans tous les états sauf ATTENTE et BOOT
  if (messageRecu && serialMessage == "stop" && currentState2 != ATTENTE && currentState2 != BOOT) {
    messageRecu = false;
    stopEncoders();
    eteindreLeds();

    spinStarted = false;        // annule un pivot en cours, pour que le prochain reparte proprement
    currentState2 = ATTENTE;    // retourne à l'état d'attente
    Serial.print("cmd recu:");
    Serial.println(serialMessage);
    return;
  }

  switch (currentState2) {
    case BOOT: bootTask(currentTime); break;
    case ATTENTE: waitCommandTask(); break;
    case CALIB: calibrationTask(); break;
    case SUIVRE_LIGNE: followLineTask(currentTime); break;
    case CMD_INVALIDE: InvalidCommandTask(currentTime); break;
    case NON_CALIB: notCalibratedTask(currentTime); break;
    case DEMI_TOUR: demiTourTask(currentTime); break;
    default: currentState2 = ATTENTE; break;
  }
}

// -- encodeurs

void isr_process_encoder1(void)
{
  if (digitalRead(encoder_1.getPortB()) == 0) encoder_1.pulsePosMinus();
  else encoder_1.pulsePosPlus();
}

void isr_process_encoder2(void)
{
  if (digitalRead(encoder_2.getPortB()) == 0) encoder_2.pulsePosMinus();
  else encoder_2.pulsePosPlus();
}

void encoderConfig() {
  attachInterrupt(encoder_1.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(encoder_2.getIntNum(), isr_process_encoder2, RISING);

  encoder_1.setPulse(9);
  encoder_2.setPulse(9);

  encoder_1.setRatio(39.267);
  encoder_2.setRatio(39.267);

  encoder_1.setPosPid(1.8, 0, 1.2);
  encoder_2.setPosPid(1.8, 0, 1.2);

  encoder_1.setSpeedPid(0.18, 0, 0);
  encoder_2.setSpeedPid(0.18, 0, 0);

  // PWM 8 kHz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
}

void encodersTask() {
  encoder_1.loop();
  encoder_2.loop();
}

void stopEncoders() {
  encoder_1.setTarPWM(0);
  encoder_2.setTarPWM(0);
}

// -- gyro : angle et pivot

double normalizeAngle(double a) {
  while (a > 180.0) a -= 360.0;
  while (a <= -180.0) a += 360.0;
  return a;
}

// Pivot de angleDeg degrés (rotation cumulée, donc 360 fonctionne)
bool spin(double angleDeg, unsigned long currentTime) {
  static bool settling = false;
  static double lastAngle = 0;
  static double currentRotation = 0;
  static unsigned long settleEnd = 0;

  if (!spinStarted) {
    spinStarted = true;
    settling = false;
    lastAngle = gyro.getAngleZ();
    currentRotation = 0;
  }

  // Calcul de la rotation effectuée
  double currentAngle = gyro.getAngleZ();
  double delta = normalizeAngle(currentAngle - lastAngle);

  currentRotation += delta;
  lastAngle = currentAngle;

  // Calcul de l'erreur
  double reste = angleDeg - currentRotation;

  // Vérification après l'arrêt
  if (settling) {
    if (currentTime < settleEnd) return false;

    if (fabs(reste) <= PIVOT_TOLERANCE) {
      spinStarted = false;
      return true;
    }

    settling = false;
  }

  // Arrêt lorsque le robot est proche de la cible
  if (fabs(reste) <= PIVOT_TOLERANCE) {
    stopEncoders();
    settling = true;
    settleEnd = currentTime + PIVOT_SETTLE_MS;
    return false;
  }

  // Le signe de reste détermine le sens de correction
  short dir = (reste > 0) ? 1 : -1;

  short vit = SPEED_PIVOT;

  if (fabs(reste) < 30) vit = SPEED_PIVOT / 2;
  if (fabs(reste) < 10) vit = SPEED_PIVOT_FINAL;

  encoder_1.setTarPWM(dir * vit);
  encoder_2.setTarPWM(dir * vit);

  return false;
}

