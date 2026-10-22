#include <MeAuriga.h>
#include <Wire.h>
#include <Arduino.h>

const float DIST_X_CM     = 120.0;  
const int   STOP_DIST_CM  = 30;   

const int   DA_6 = 7;    
const bool  PIVOT_DROIT_1 = (DA_6 % 2 == 1);           
const double ANGLE_PIVOT_1 = PIVOT_DROIT_1 ? 90.0 : -90.0;   
const double ANGLE_PIVOT_2 = -ANGLE_PIVOT_1;            
const double ANGLE_DEMI_TOUR = 180.0;

// -- roues robots

const float WHEEL_DIAMETER  = 6.5;        
const float WHEEL_PERIMETER = WHEEL_DIAMETER * 3.1416;

// -- vitesses et réglages

const short SPEED = 100;   
const short SPEED_APPROCHE = 70;    // vitesse près de l'obstacle (segment B)
const short SPEED_PIVOT = 65;    
const short SPEED_PIVOT_FINAL = 37;    
const int   SLOW_ZONE_CM = 20;    // on ralentit à STOP_DIST_CM + SLOW_ZONE_CM
const float FREINAGE_CM = 0.0;   // distance d'inertie à anticiper (A et C), à calibrer
const double PIVOT_TOLERANCE = 2.0;   // degrés
const unsigned long PIVOT_SETTLE_MS = 300;   // pause après un pivot pour stabiliser le gyro

const double KP_AC = 25.5, KD_AC = 13.0;   // segments A et C
const double KP_B  = 9.0,  KD_B  = 18.0;   // segment B

// -- leds

const int LEDNUM = 12;
const int LEDPIN = 44;
const int PROGRESS_LEDS = 7;      
const int LED_SEG_A = 12;     // DEL verte laissée allumée à la fin de chaque segment
const int LED_SEG_B = 11;
const int LED_SEG_C = 10;
const unsigned long BLINK_HALF_PERIOD_MS = 50;  

// -- retour

const bool BONUS_RETOUR = true;                  
const unsigned long FIN_A_DUREE_MS = 3000;       // durée du clignotement final avant le retour

// -- objets

MeEncoderOnBoard encoder_1(SLOT1);   
MeEncoderOnBoard encoder_2(SLOT2);   
MeRGBLed led(0, LEDNUM);
MeGyro gyro(0, 0x69);
MeUltrasonicSensor ultraSensor(PORT_10);


float ultrasonicDistance = 400;      
float odoBaseL = 0, odoBaseR = 0;    // position des encodeurs au début d'un segment
int   progressShown = -1;            // nb de DEL affichées 
double headingGoal = 0.0;            // cap absolu à tenir 
float distSegB = 0;                  // distance réellement parcourue au segment B (pour le retour)
int   backStep = 0;                  // étape du retour

enum State {SETUP, ROAD_A, TURN_A, ROAD_B, TURN_B, ROAD_C, TURN_FULL, FINISH, BACK_TRIP, BACK_TURN, END};
State currentState = SETUP;

// -- fonctions d'interruption

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

// Distance parcourue (cm) depuis le début du segment (odoBaseL / odoBaseR)
float distanceParcourueCm() {
  float posMoy = (fabs(encoder_1.getCurPos() - odoBaseL) +
                  fabs(encoder_2.getCurPos() - odoBaseR)) / 2.0;
  return (posMoy / 360.0) * WHEEL_PERIMETER;
}

// -- capteur de distance

// force = true : lecture immédiate (départ du segment B)
void readDistanceTask(unsigned long currentTime, bool force) {
  static unsigned long previousTime = 0;
  const unsigned long RATE_READ = 50;

  if (force || currentTime - previousTime >= RATE_READ) {
    previousTime = currentTime;

    float d = ultraSensor.distanceCm();
    if (d >= 2.0) ultrasonicDistance = d;   // ignore les lectures invalides (0 = pas d'écho)
  }
}

// -- allumer les leds de manière progressive

void progressiveLeds(float ratio) {
  ratio = constrain(ratio, 0.0, 1.0);
  int nbLeds = (int)ceil(ratio * PROGRESS_LEDS);

  if (nbLeds == progressShown) return;
  progressShown = nbLeds;

  for (int i = 1; i <= PROGRESS_LEDS; i++) {
    if (i <= nbLeds) led.setColor(i, 0, 255, 0);
    else led.setColor(i, 0, 0, 0);
  }
  led.show();
}

// -- déplacement en ligne droite

double normalizeAngle(double a) {
  while (a > 180.0)   a -= 360.0;
  while (a <= -180.0) a += 360.0;
  return a;
}

void goStraight(short speed, short firstRun, double kp, double kd) {
  static double error = 0.0;
  static double previousError = 0.0;
  static double output = 0;

  if (firstRun) {
    previousError = 0.0;

    encoder_1.setTarPWM(-speed);
    encoder_2.setTarPWM(speed);
    return;
  }

  error = normalizeAngle(gyro.getAngleZ() - headingGoal);
  output = kp * error + kd * (error - previousError);
  previousError = error;

  encoder_1.setTarPWM(-speed - output);
  encoder_2.setTarPWM(speed - output);
}

// -- pivot du giroscope

bool spin(double angleDeg, unsigned long currentTime) {
  static bool started = false;
  static bool settling = false;
  static double targetHeading = 0.0;
  static unsigned long settleEnd = 0;

  if (!started) {
    started = true;
    settling = false;
    targetHeading = normalizeAngle(headingGoal + angleDeg);   // cap absolu visé
  }

  double reste = normalizeAngle(targetHeading - gyro.getAngleZ());

  if (settling) {
    if (currentTime < settleEnd) return false;

    // Robot immobile : on revérifie l'erreur résiduelle (dépassement dû à l'inertie)
    if (fabs(reste) <= PIVOT_TOLERANCE) {
      headingGoal = targetHeading;   // la référence est le cap visé, pas le cap mesuré
      started = false;
      return true;
    }
    settling = false;                // trop d'erreur : on corrige lentement
  }

  if (fabs(reste) <= PIVOT_TOLERANCE) {
    stopEncoders();
    settling = true;
    settleEnd = currentTime + PIVOT_SETTLE_MS;
    return false;
  }

  short dir = (reste > 0) ? 1 : -1;
  // Ralentit près de la cible pour limiter le dépassement
  short vit = SPEED_PIVOT;
  if (fabs(reste) < 30) vit = SPEED_PIVOT / 2;
  if (fabs(reste) < 10) vit = SPEED_PIVOT_FINAL;

  encoder_1.setTarPWM(dir * vit);
  encoder_2.setTarPWM(dir * vit);
  return false;
}


void setupState(unsigned long currentTime) {
  static unsigned long exitTime = 0;
  static bool firstTime = true;
  const unsigned long WAIT_MS = 3000;
  bool transition = false;

  if (firstTime) {
    firstTime = false;
    exitTime = currentTime + WAIT_MS;

    Serial.println("Attente 3 s");
  }

  transition = (currentTime >= exitTime);
  if (transition) {
    headingGoal = gyro.getAngleZ();   // cap de départ = référence pour tout le parcours
    firstTime = true;
    currentState = ROAD_A;
  }
}

// Segment 1 : ligne droite de X cm
void freeTripState(unsigned long currentTime) {
  static bool firstTime = true;
  bool transition = false;

  if (firstTime) {
    firstTime = false;
    progressShown = -1;                    // la barre repart de zéro
    odoBaseL = encoder_1.getCurPos();      // la distance se mesure à partir d'ici
    odoBaseR = encoder_2.getCurPos();
    goStraight(SPEED, 1, KP_AC, KD_AC);
  }

  goStraight(SPEED, 0, KP_AC, KD_AC);

  float d = distanceParcourueCm();
  progressiveLeds(d / DIST_X_CM);

  transition = (d >= DIST_X_CM - FREINAGE_CM);
  if (transition) {
    firstTime = true;

    stopEncoders();
    led.setColor(LED_SEG_A, 0, 255, 0);
    led.show();

    currentState = TURN_A;
  }
}

// Pivot 90° - DA[6]
void turnState(unsigned long currentTime) {
  if (spin(ANGLE_PIVOT_1, currentTime)) currentState = ROAD_B;
}

// Segment 2 : avance jusqu'à 30 cm de l'obstacle
void obstacleTripState(unsigned long currentTime) {
  static bool firstTime = true;
  static float distInit = 0;
  bool transition = false;

  if (firstTime) {
    firstTime = false;
    progressShown = -1;
    odoBaseL = encoder_1.getCurPos();      // mesurer la distance du segment B
    odoBaseR = encoder_2.getCurPos();
    readDistanceTask(currentTime, true);   // lecture fraîche au départ du segment
    distInit = ultrasonicDistance;
    goStraight(SPEED, 1, KP_B, KD_B);
  }

  readDistanceTask(currentTime, false);

  // Ralentit à l'approche pour s'arrêter précisément à 30 cm
  short speed = (ultrasonicDistance <= STOP_DIST_CM + SLOW_ZONE_CM) ? SPEED_APPROCHE : SPEED;
  goStraight(speed, 0, KP_B, KD_B);

  float span = distInit - STOP_DIST_CM;
  if (span > 0) progressiveLeds((distInit - ultrasonicDistance) / span);

  transition = (ultrasonicDistance <= STOP_DIST_CM);
  if (transition) {
    firstTime = true;

    distSegB = distanceParcourueCm();      // mémorisée pour le retour à la base

    stopEncoders();
    led.setColor(LED_SEG_B, 0, 255, 0);
    led.show();

    currentState = TURN_B;
  }
}

// Pivot 90° dans le sens opposé
void turnOppositeState(unsigned long currentTime) {
  if (spin(ANGLE_PIVOT_2, currentTime)) currentState = ROAD_C;
}

// Segment 3 : ligne droite de X cm
void ledTripState(unsigned long currentTime) {
  static bool firstTime = true;
  bool transition = false;

  if (firstTime) {
    firstTime = false;
    progressShown = -1;
    odoBaseL = encoder_1.getCurPos();
    odoBaseR = encoder_2.getCurPos();
    goStraight(SPEED, 1, KP_AC, KD_AC);
  }

  goStraight(SPEED, 0, KP_AC, KD_AC);

  float d = distanceParcourueCm();
  progressiveLeds(d / DIST_X_CM);

  transition = (d >= DIST_X_CM - FREINAGE_CM);
  if (transition) {
    firstTime = true;

    stopEncoders();
    led.setColor(LED_SEG_C, 0, 255, 0);
    led.show();

    currentState = TURN_FULL;
  }
}

// Demi-tour 180°
void turnFullState(unsigned long currentTime) {
  if (spin(ANGLE_DEMI_TOUR, currentTime)) currentState = FINISH;
}

// Fin A : DEL 8 à 12 clignotent en vert à 10 Hz, puis (bonus) retour à la base
void finishState(unsigned long currentTime) {
  static bool firstTime = true;
  static unsigned long lastTime = 0;
  static unsigned long startTime = 0;
  static bool isOn = false;

  if (firstTime) {
    firstTime = false;
    startTime = currentTime;
    stopEncoders();
    led.setColor(0, 0, 0, 0);
    led.show();
  }

  if (currentTime - lastTime >= BLINK_HALF_PERIOD_MS) {
    lastTime = currentTime;
    isOn = !isOn;

    for (int i = 8; i <= 12; i++) {
      led.setColor(i, 0, isOn ? 255 : 0, 0);
    }
    led.show();
  }

  if (BONUS_RETOUR && currentTime - startTime >= FIN_A_DUREE_MS) {
    firstTime = true;
    led.setColor(0, 0, 0, 0);
    led.show();
    backStep = 0;
    currentState = BACK_TRIP;
  }
}

// -- bonus: trajet retour

// Arc-en-ciel animé sur les 12 DEL (pendant tout le retour)
void rainbowTask(unsigned long currentTime) {
  static unsigned long lastTime = 0;
  static int offset = 0;
  const unsigned long RATE = 80;

  if (currentTime - lastTime < RATE) return;
  lastTime = currentTime;
  offset = (offset + 12) % 256;

  for (int i = 1; i <= LEDNUM; i++) {
    int pos = (offset + i * 256 / LEDNUM) % 256;   // position sur la roue des couleurs (0 à 255)
    int r, g, b;
    if (pos < 85) { r = pos * 3; g = 255 - pos * 3; b = 0; }
    else if (pos < 170) { pos -= 85;  r = 255 - pos * 3; g = 0; b = pos * 3; }
    else { pos -= 170; r = 0; g = pos * 3; b = 255 - pos * 3; }
    led.setColor(i, r, g, b);
  }
  led.show();
}

// Avance en ligne droite de distCm (cap = headingGoal). Retourne true quand la distance est atteinte.
bool returnTrip(float distCm, double kp, double kd) {
  static bool firstTime = true;

  if (firstTime) {
    firstTime = false;
    odoBaseL = encoder_1.getCurPos();
    odoBaseR = encoder_2.getCurPos();
    goStraight(SPEED, 1, kp, kd);
  }

  goStraight(SPEED, 0, kp, kd);

  if (distanceParcourueCm() >= distCm - FREINAGE_CM) {
    stopEncoders();
    firstTime = true;
    return true;
  }
  return false;
}

// Segments du retour : étape 0 (X cm), 2 (distance du segment B) et 4 (X cm)
void backTripState(unsigned long currentTime) {
  bool isB = (backStep == 2);
  float dist = isB ? distSegB : DIST_X_CM;
  double kp = isB ? KP_B : KP_AC;
  double kd = isB ? KD_B : KD_AC;

  if (returnTrip(dist, kp, kd)) {
    if (backStep == 4) {
      currentState = END;            // dernier segment terminé : retour à la case départ
    } else {
      backStep++;
      currentState = BACK_TURN;
    }
  }
}

// Pivots du retour : étape 1 (même sens que le 1er pivot) et 3 (sens opposé)
void backTurnState(unsigned long currentTime) {
  double angle = (backStep == 1) ? ANGLE_PIVOT_1 : ANGLE_PIVOT_2;

  if (spin(angle, currentTime)) {
    backStep++;
    currentState = BACK_TRIP;
  }
}

// Fin B : robot arrêté, DEL éteintes
void endState() {
  static bool firstTime = true;

  if (firstTime) {
    firstTime = false;
    stopEncoders();
    led.setColor(0, 0, 0, 0);
    led.show();
  }
}

void gestState(unsigned long currentTime) {
  static int lastState = -1;
  if (currentState != lastState) {          // trace de débogage à chaque changement d'état
    lastState = currentState;
    Serial.print("Etat: ");
    Serial.println(currentState);
  }

  switch (currentState) {
    case SETUP: setupState(currentTime); break;
    case ROAD_A: freeTripState(currentTime); break;
    case TURN_A: turnState(currentTime); break;
    case ROAD_B: obstacleTripState(currentTime); break;
    case TURN_B: turnOppositeState(currentTime); break;
    case ROAD_C: ledTripState(currentTime); break;
    case TURN_FULL: turnFullState(currentTime); break;
    case FINISH: finishState(currentTime); break;
    case BACK_TRIP: backTripState(currentTime); break;
    case BACK_TURN: backTurnState(currentTime); break;
    case END: endState(); break;
    default: currentState = FINISH; break;
  }
}

void setup() {
  Serial.begin(115200);

  encoderConfig();
  led.setpin(LEDPIN);
  gyro.begin();
}

void loop() {
  unsigned long currentTime = millis();

  encodersTask();
  gyro.update();
  gestState(currentTime);

  // Arc-en-ciel animé pendant tout le retour (bonus)
  if (currentState == BACK_TRIP || currentState == BACK_TURN) rainbowTask(currentTime);
}