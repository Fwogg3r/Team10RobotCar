#include <LiquidCrystal_I2C.h> // by Frank de Brabander
//#include <Adafruit_NeoPixel.h> // Adafruit, 1.12.3
#include "sprites.h"
#include <SoftwareSerial.h>

#define LED_RED 13
#define LED_GREEN 12
#define Rx 19  // sets transmit pin on the Bluetooth to the Rx pin on the Arduino Mega
#define Tx 18  // sets receive pin on the Bluetooth to the Tx pin on the Arduino Mega
#define BLUETOOTH_BAUD_RATE 38400

#define encoderA 2
#define encoderB 3
#define MotorPWM_A 5  // Left motor PWM pin
#define MotorPWM_B 4  // Right motor PWM pin
#define INA1A 32
#define INA2A 34
#define INA1B 30
#define INA2B 36

LiquidCrystal_I2C lcd(0x27, 20, 2);
SoftwareSerial bluetooth(Rx, Tx);

int charsScrolled = 0;
bool initialize = true;

bool rightBlinkerActive = false;
bool rightBlinkerOn = false;
bool leftBlinkerActive = false;
bool leftBlinkerOn = false;

float BLINK_RATE = 250;           // rate at which the blinker will turn on and off (in ms)
unsigned long timeAtLastFrame = 0;  // the recorded time at the last frame in ms

void checkScrollLCDTextForIntro() {
  if (charsScrolled < 14) {
    delay(500);
    lcd.scrollDisplayLeft();
    charsScrolled++;
  } else if (initialize) {
    delay(2000);
    lcd.clear();
    Sprite::initLCD(lcd);
    initialize = false;
  }
}

static void activateBlinker(const char* side, int LED, bool blinkerOn) {
  if (strcmp(side, "left") == 0) {
    if (blinkerOn) {
      digitalWrite(LED, HIGH);
      Sprite::blinkLeft(lcd);
    } else {
      digitalWrite(LED, LOW);
      Sprite::blinkersOff(lcd);
    }
  } else if (strcmp(side, "right") == 0) {
    if (blinkerOn) {
      digitalWrite(LED, HIGH);
      Sprite::blinkRight(lcd);
    } else {
      digitalWrite(LED, LOW);
      Sprite::blinkersOff(lcd);
    }
  }
  else if (strcmp(side, "hazard") == 0) {
    if (blinkerOn) {
      digitalWrite(LED_RED, HIGH);
      digitalWrite(LED_GREEN, HIGH);
      Sprite::hazard(lcd);
    } else {
      digitalWrite(LED_RED, LOW);
      digitalWrite(LED_GREEN, LOW);
      Sprite::blinkersOff(lcd);
    }
  }
}

static void setAllBlinkersOff() {
  leftBlinkerActive = false;
  leftBlinkerOn = false;
  rightBlinkerActive = false;
  rightBlinkerOn = false;
  activateBlinker("left", LED_RED, false);
  activateBlinker("right", LED_GREEN, false);
}

// Motor control functions
void Forward(int speed) {
  analogWrite(MotorPWM_A, speed);  // Sets left motor speed
  analogWrite(MotorPWM_B, speed);  // Sets right motor speed

  digitalWrite(INA1A, HIGH);
  digitalWrite(INA2A, LOW);

  digitalWrite(INA1B, HIGH);
  digitalWrite(INA2B, LOW);
}

void Reverse(int speed) {
  analogWrite(MotorPWM_A, speed);  // Sets left motor speed
  analogWrite(MotorPWM_B, speed);  // Sets right motor speed

  digitalWrite(INA1A, LOW);
  digitalWrite(INA2A, HIGH);

  digitalWrite(INA1B, LOW);
  digitalWrite(INA2B, HIGH);
}

void Stop() {
  analogWrite(MotorPWM_A, 0);
  analogWrite(MotorPWM_B, 0);
}

void setup() {
  Serial.begin(38400);  // Monitor
  Serial1.begin(38400); // HC-05
  lcd.init();
  lcd.backlight();
  Sprite::initTeam10NameCredits(lcd);
  pinMode(Rx, INPUT);
  pinMode(Tx, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinmode(encoderA, INPUT);
  pinmode(encoderB, INPUT);
  pinMode(MotorPWM_A, OUTPUT);
  pinMode(MotorPWM_B, OUTPUT);
  pinMode(INA1A, OUTPUT);
  pinMode(INA2A, OUTPUT);
  pinMode(INA1B, OUTPUT);
  pinMode(INA2B, OUTPUT);
  
  timeAtLastFrame = millis();  // initialize time tracking
}

void loop() {
  checkScrollLCDTextForIntro();

  unsigned long currentTime = millis();

  if (Serial1.available()) {
    String command = Serial1.readStringUntil('\n');
    command.trim();

    Serial.print("Received: ");
    Serial.println(command);
    
    if (command.equalsIgnoreCase("right")) {
      setAllBlinkersOff();
      rightBlinkerActive = !rightBlinkerActive;
    } else if (command.equalsIgnoreCase("left")) {
      setAllBlinkersOff();
      leftBlinkerActive = !leftBlinkerActive;
    } else if (command.equalsIgnoreCase("off")) {
      setAllBlinkersOff();
    } else if (command.equalsIgnoreCase("forward")) {
      Forward(150);  // Move forward at medium speed (150)
    } else if (command.equalsIgnoreCase("reverse")) {
      Reverse(150);  // Move backward at medium speed (150)
    } else if (command.equalsIgnoreCase("stop")) {
      Stop();  // Stop both motors
    } else {
      Serial.println("Unknown command");
    }
  }

  if (leftBlinkerActive && currentTime - timeAtLastFrame >= BLINK_RATE) {
    timeAtLastFrame = currentTime;
    leftBlinkerOn = !leftBlinkerOn;
    activateBlinker("left", LED_RED, leftBlinkerOn);
  }

  if (rightBlinkerActive && currentTime - timeAtLastFrame >= BLINK_RATE) {
    timeAtLastFrame = currentTime;
    rightBlinkerOn = !rightBlinkerOn;
    activateBlinker("right", LED_GREEN, rightBlinkerOn);
  }
}
