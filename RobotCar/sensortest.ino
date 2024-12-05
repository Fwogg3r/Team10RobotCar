#include <LiquidCrystal_I2C.h> // by Frank de Brabander
#include "sprites.h"
#include <SoftwareSerial.h>

#define LED_RED 13
#define LED_GREEN 12
#define Rx 19  // Bluetooth Rx pin on the Arduino Mega
#define Tx 18  // Bluetooth Tx pin on the Arduino Mega
#define BLUETOOTH_BAUD_RATE 38400

#define encoderA 2
#define encoderB 3
#define MotorPWM_A 5  // Left motor PWM pin
#define MotorPWM_B 4  // Right motor PWM pin
#define INA1A 32
#define INA2A 34
#define INA1B 30
#define INA2B 36

#define R_S A7
#define M_S A6
#define L_S A5

#define echo A0       // Ultrasonic echo pin
#define trigger A1    // Ultrasonic trigger pin

LiquidCrystal_I2C lcd(0x27, 20, 2);
SoftwareSerial bluetooth(Rx, Tx);

int charsScrolled = 0;
bool initialize = true;

bool rightBlinkerActive = false;
bool rightBlinkerOn = false;
bool leftBlinkerActive = false;
bool leftBlinkerOn = false;
bool autoMode = false;

float BLINK_RATE = 250;           // Blinker rate (in ms)
unsigned long timeAtLastFrame = 0;  // Time tracking

// Ultrasonic distance measurement function
long readUltrasonic() {
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);

  long duration = pulseIn(echo, HIGH);
  long distance = duration * 0.034 / 2;  // Convert to centimeters
  return distance;
}

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

// Motor control functions
void Forward(int speed) {
  analogWrite(MotorPWM_A, speed);
  analogWrite(MotorPWM_B, speed);

  digitalWrite(INA1A, LOW);
  digitalWrite(INA2A, HIGH);

  digitalWrite(INA1B, LOW);
  digitalWrite(INA2B, HIGH);
}

void Left(int speed) {
  analogWrite(MotorPWM_A, speed / 2);  // Slow left motor
  analogWrite(MotorPWM_B, speed);      // Full speed on right motor

  digitalWrite(INA1A, LOW);
  digitalWrite(INA2A, HIGH);

  digitalWrite(INA1B, LOW);
  digitalWrite(INA2B, HIGH);
}

void Right(int speed) {
  analogWrite(MotorPWM_A, speed);      // Full speed on left motor
  analogWrite(MotorPWM_B, speed / 2);  // Slow right motor

  digitalWrite(INA1A, LOW);
  digitalWrite(INA2A, HIGH);

  digitalWrite(INA1B, LOW);
  digitalWrite(INA2B, HIGH);
}

void Reverse(int speed) {
  analogWrite(MotorPWM_A, speed);
  analogWrite(MotorPWM_B, speed);

  digitalWrite(INA1A, HIGH);
  digitalWrite(INA2A, LOW);

  digitalWrite(INA1B, HIGH);
  digitalWrite(INA2B, LOW);
}

void Stop() {
  analogWrite(MotorPWM_A, 0);
  analogWrite(MotorPWM_B, 0);
}

// New obstacle avoidance maneuver
void avoidObstacle() {
  Stop();
  delay(500);           // Brief pause

  Reverse(100);         // Move backward slightly
  delay(500);

  Left(100);            // Turn left
  delay(800);           // Adjust timing for a 90-degree turn

  Forward(100);         // Move forward to bypass obstacle
  delay(1500);          // Adjust depending on obstacle size

  Right(100);           // Turn right to realign with the line
  delay(800);

  Stop();               // Pause briefly before resuming pathfinding
  delay(500);
}

void pathfinding() {
  int leftSensorRead = analogRead(L_S);
  int middleSensorRead = analogRead(M_S);
  int rightSensorRead = analogRead(R_S);

  const int lowerThreshold = 950;
  const int upperThreshold = 1100;
  const int baseSpeed = 130;  // Normal speed

  if (middleSensorRead >= lowerThreshold && middleSensorRead <= upperThreshold) {
    Forward(baseSpeed);
  } else if (leftSensorRead >= lowerThreshold && leftSensorRead <= upperThreshold) {
    Left(baseSpeed);
  } else if (rightSensorRead >= lowerThreshold && rightSensorRead <= upperThreshold) {
    Right(baseSpeed);
  } else {
    recovery();
  }
}

void recovery() {
  const int recoverySpeed = 75;
  int turnDuration = 1000; // Adjust based on testing

  // Alternate turns for better search
  if (millis() % 2 == 0) {
    // Turn left in place
    digitalWrite(INA1A, HIGH);
    digitalWrite(INA2A, LOW);
    digitalWrite(INA1B, LOW);
    digitalWrite(INA2B, HIGH);
  } else {
    // Turn right in place
    digitalWrite(INA1A, LOW);
    digitalWrite(INA2A, HIGH);
    digitalWrite(INA1B, HIGH);
    digitalWrite(INA2B, LOW);
  }

  analogWrite(MotorPWM_A, recoverySpeed);
  analogWrite(MotorPWM_B, recoverySpeed);

  delay(turnDuration);
  Stop();
  delay(200);
}

void setup() {
  Serial.begin(38400);  // Monitor
  Serial1.begin(38400); // HC-05
  lcd.init();
  lcd.backlight();
  Sprite::initTeam10NameCredits(lcd);

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(trigger, OUTPUT);
  pinMode(echo, INPUT);

  pinMode(MotorPWM_A, OUTPUT);
  pinMode(MotorPWM_B, OUTPUT);
  pinMode(INA1A, OUTPUT);
  pinMode(INA2A, OUTPUT);
  pinMode(INA1B, OUTPUT);
  pinMode(INA2B, OUTPUT);

  timeAtLastFrame = millis();
}

void loop() {
  checkScrollLCDTextForIntro();

  if (autoMode) {
    long distance = readUltrasonic();
    Serial.print("Distance: ");
    Serial.println(distance);

    if (distance < 20) {  // Obstacle detected within 20 cm
      avoidObstacle();
    } else {
      pathfinding();
    }
  }

  // Bluetooth communication
  if (Serial1.available()) {
    String command = Serial1.readStringUntil('\n');
    command.trim();
    if (command.equalsIgnoreCase("auto")) {
      autoMode = true;
    } else if (command.equalsIgnoreCase("stop")) {
      autoMode = false;
      Stop();
    } else if (command.equalsIgnoreCase("forward")) {
      autoMode = false;
      Forward(150);
    } else if (command.equalsIgnoreCase("reverse")) {
      autoMode = false;
      Reverse(150);
    } else if (command.equalsIgnoreCase("left")) {
      autoMode = false;
      Left(150);
    } else if (command.equalsIgnoreCase("right")) {
      autoMode = false;
      Right(150);
    }
  }
}
