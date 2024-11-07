#include <LiquidCrystal_I2C.h> // by Frank de Brabander
//#include <Adafruit_NeoPixel.h> // Adafruit, 1.12.3
#include "sprites.h"
#include <SoftwareSerial.h>

#define RGB_PIN 38
#define LED_RED 13
#define LED_GREEN 12
#define Rx 15  // sets transmit pin on the Bluetooth to the Rx pin on the Arduino
#define Tx 14  // sets receive pin on the Bluetooth to the Tx pin on the Arduino
#define BLUETOOTH_BAUD_RATE 38400

#define MotorPWM_A 46  // Left motor PWM pin
#define MotorPWM_B 44  // Right motor PWM pin
#define INA1A 32       // Left motor direction pin 1
#define INA2A 34       // Left motor direction pin 2
#define INA1B 30       // Right motor direction pin 1
#define INA2B 36       // Right motor direction pin 2

LiquidCrystal_I2C lcd(0x27, 20, 2);
SoftwareSerial bluetooth(Rx, Tx);

int charsScrolled = 0;
bool initialize = true;  // initialization values

bool rightBlinkerActive = false;  // if the blinker is set to execute logic in the loop
bool rightBlinkerOn = false;      // if the LED voltage is set to HIGH
bool leftBlinkerActive = false;
bool leftBlinkerOn = false;

float BLINK_RATE = 250;           // rate at which the blinker will turn on and off (in ms)
float timeAtLastFrame = 0;        // the recorded time at the last frame in ms
float timeUntilBlinkerChange = 500;  // the time until the blinker is scheduled to change

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

static void activateBlinker(char* side, int LED, bool blinkerOn) {
  if (side == "left") {
    if (blinkerOn) {
      digitalWrite(LED, HIGH);
      Sprite::blinkLeft(lcd);
    } else {
      digitalWrite(LED, LOW);
      Sprite::blinkersOff(lcd);
    }
  } else if (side == "right") {
    if (blinkerOn) {
      digitalWrite(LED, HIGH);
      Sprite::blinkRight(lcd);
    } else {
      digitalWrite(LED, LOW);
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
  analogWrite(MotorPWM_A, speed);
  analogWrite(MotorPWM_B, speed);

  // Set left motor to rotate clockwise
  digitalWrite(INA1A, HIGH);
  digitalWrite(INA2A, LOW);

  // Set right motor to rotate clockwise
  digitalWrite(INA1B, HIGH);
  digitalWrite(INA2B, LOW);
}

void Reverse(int speed) {
  analogWrite(MotorPWM_A, speed);
  analogWrite(MotorPWM_B, speed);

  // Set left motor to rotate counterclockwise
  digitalWrite(INA1A, LOW);
  digitalWrite(INA2A, HIGH);

  // Set right motor to rotate counterclockwise
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
  pinMode(MotorPWM_A, OUTPUT);
  pinMode(MotorPWM_B, OUTPUT);
  pinMode(INA1A, OUTPUT);
  pinMode(INA2A, OUTPUT);
  pinMode(INA1B, OUTPUT);
  pinMode(INA2B, OUTPUT);
}

void loop() {
  checkScrollLCDTextForIntro();
  timeUntilBlinkerChange -= millis() - timeAtLastFrame;
  timeAtLastFrame = millis();

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
      Forward(150);  // Move forward at medium speed
    } else if (command.equalsIgnoreCase("reverse")) {
      Reverse(150);  // Move backward at medium speed
    } else if (command.equalsIgnoreCase("stop")) {
      Stop();  // Stop both motors
    } else {
      Serial.println("Unknown command");
    }
  }

  if (leftBlinkerActive && timeUntilBlinkerChange <= 0) {
    timeUntilBlinkerChange = BLINK_RATE;
    leftBlinkerOn = !leftBlinkerOn;
    activateBlinker("left", LED_RED, leftBlinkerOn);
  }

  if (rightBlinkerActive && timeUntilBlinkerChange <= 0) {
    timeUntilBlinkerChange = BLINK_RATE;
    rightBlinkerOn = !rightBlinkerOn;
    activateBlinker("right", LED_GREEN, rightBlinkerOn);
  }
}
