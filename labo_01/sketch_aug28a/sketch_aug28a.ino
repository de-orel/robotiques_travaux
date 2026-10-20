#include <Wire.h>
#include <Arduino.h>
#include <HCSR04.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 
#define SCREEN_HEIGHT 64

#define OLED_RESET     -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const int TRIG_PIN = 12;
const int ECHO_PIN = 11;

float distance = 0;

HCSR04 hc(TRIG_PIN, ECHO_PIN);

void setup() {
  Serial.begin(115200);

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("Échec d'allocation SSD1306"));
    for(;;);
  }
}

void loop() {

  readDistance();
  printDistanceSerial();
  updateDisplay();
}

void readDistance(){
  static unsigned long lastTimeRead = 0;
  const int READ_DELAY = 100;
  unsigned long currentTime = millis();

  if(currentTime - lastTimeRead >= READ_DELAY){
    distance = hc.dist();
    lastTimeRead = currentTime;
  }
}

void printDistanceSerial(){
  static unsigned long lastTime = 0;
  const int REFRESH_DELAY = 250;
  unsigned long currentTime = millis();

  if(currentTime - lastTime >= REFRESH_DELAY){
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    lastTime = currentTime;
  }
}

void drawSun(int cx, int cy, int radius) {
  display.drawCircle(cx, cy, radius, SSD1306_WHITE);
  int rayLength = radius + 4;
  for (int angle = 0; angle < 360; angle += 45) {
    float rad = angle * PI / 180.0;
    int x1 = cx + cos(rad) * (radius + 1);   
    int y1 = cy + sin(rad) * (radius + 1);
    int x2 = cx + cos(rad) * rayLength;
    int y2 = cy + sin(rad) * rayLength;
    display.drawLine(x1, y1, x2, y2, SSD1306_WHITE);
  }
}

void drawCar(int x, int y) {
  display.drawRoundRect(x, y, 30, 10, 2, SSD1306_WHITE);
  display.drawRoundRect(x + 6, y - 6, 14, 8, 2, SSD1306_WHITE);
  display.fillCircle(x + 6, y + 10, 3, SSD1306_WHITE);
  display.fillCircle(x + 22, y + 10, 3, SSD1306_WHITE);
}

void drawName() {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(2, SCREEN_HEIGHT - 8);
  display.print("NGUEDIA");
}

int getCarX() {
  const int MIN_DIST = 10;
  const int MAX_DIST = 50;
  const int carWidth = 30;

  int xLeft = 0; 
  int xRight = SCREEN_WIDTH - carWidth; 

  float safeDist = constrain(distance, MIN_DIST, MAX_DIST);

  int carX = map(safeDist, MIN_DIST, MAX_DIST, xLeft, xRight);

  return carX;
}


void updateDisplay() {
  static unsigned long lastTime = 0;
  const int REFRESH_DELAY = 100;
  unsigned long currentTime = millis();

  if (currentTime - lastTime >= REFRESH_DELAY) {
    display.clearDisplay();

    drawSun(12, 12, 6);
    int carX = getCarX();
    drawCar(carX, SCREEN_HEIGHT - 16);
    drawName();

    display.display();
    lastTime = currentTime;
  }
}
