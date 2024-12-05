#include <LiquidCrystal_I2C.h> // by Frank de Brabander
//#include <Adafruit_NeoPixel.h> // Adafruit, 1.12.3
#include "sprites.h"
#include <SoftwareSerial.h>
#include "motor.h"

#define LED_RED 13
#define LED_GREEN 12
#define Rx 19  // sets transmit pin on the Bluetooth to the Rx pin on the Arduino Mega
#define Tx 18  // sets receive pin on the Bluetooth to the Tx pin on the Arduino Mega
#define BLUETOOTH_BAUD_RATE 38400

#define ENCODERA 2
#define ENCODERB 3
static int countA = 0;
static int countB = 0;

LiquidCrystal_I2C lcd(0x27, 20, 2);
SoftwareSerial bluetooth(Rx, Tx);

int charsScrolled = 0;
bool initialize = true;

bool rightBlinkerActive = false;
bool rightBlinkerOn = false;
bool leftBlinkerActive = false;
bool leftBlinkerOn = false;
bool autoMode = false;
bool remoteMode = false;

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
}

static void setAllBlinkersOff() {
  leftBlinkerActive = false;
  leftBlinkerOn = false;
  rightBlinkerActive = false;
  rightBlinkerOn = false;
  activateBlinker("left", LED_RED, false);
  activateBlinker("right", LED_GREEN, false);
}

// Motor counter functions tied to crucial interrupts
void ISRA(){
  countA++;
}
void ISRB(){
  countB++;
}

void BlinkerLogic()
{
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
  pinMode(ENCODERA, INPUT_PULLUP);
  pinMode(ENCODERB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODERA), ISRA, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENCODERB), ISRB, FALLING);
  timeAtLastFrame = millis();  // initialize time tracking
}

void loop() {
  checkScrollLCDTextForIntro();
  unsigned long currentTime = millis();
  Serial.println(Motor::DetermineMotorSpeeds(&countA, &countB)); //Output motor information

  if (Serial1.available()) { //Read BT
    String command = Serial1.readStringUntil('\n');
    command.trim();

    Serial.print("Received: ");
    Serial.println(command);

    if(remoteMode)
    {

    }
    else if (command.equalsIgnoreCase("auto")) {
      autoMode = true;  // Enable auto mode
    }
    else if(command.equalsIgnoreCase("remote"))
    {
      remoteMode = true;
    }
    else if (command.equalsIgnoreCase("stop")) {
      autoMode = false;  // Disable auto mode
      //remoteMode = false;
      Motor::Stop();
    }
    else if (command.equalsIgnoreCase("forward")) {
      autoMode = false;
      Motor::Forward(150);
    } 
    else if (command.equalsIgnoreCase("reverse")) {
      autoMode = false;
      Motor::Reverse(150);
    }
    else if (command.equalsIgnoreCase("left")) {
      autoMode = false;
      Motor::Left(150);
    }
    else if (command.equalsIgnoreCase("right")) {
      autoMode = false;
      Motor::Right(150);
    }
  }

  BlinkerLogic();

  if (autoMode) {
    //TODO: print out timing, led logic, etc.
    Motor::pathfinding();  // Run pathfinding logic if auto mode is enabled
  }
}