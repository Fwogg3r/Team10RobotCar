
#include <LiquidCrystal_I2C.h> // by Frank de Brabander
#include "sprites.h"
#include <SoftwareSerial.h>
#include <Servo.h>  // Add the Servo library
#include <protothreads.h> // by Ben Artin and Adam Dunkels

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

#define R_S A7 // Right Sensor
#define M_S A6 // Middle Sensor
#define L_S A5 // Left Sensor


//----------------------------------Servo-------------------------------
#define echo A0       // Ultrasonic echo pin
#define trigger A1    // Ultrasonic trigger pin
#define servoPin 10    // Pin for controlling the servo
Servo myServo;  // Create a servo object
//----------------------------------------------------------------------


//-------------------------------Sensor_Dist----------------------------
int Set=15;
int distance_F; 
//----------------------------------------------------------------------


//-------------------------------Servo Position--------------------------
Servo myservo;
int pos = 0;
//------------------------------------------------------------------------


//-------------------------------LCD-Display------------------------------
LiquidCrystal_I2C lcd(0x27, 20, 2);
SoftwareSerial bluetooth(Rx, Tx);

int charsScrolled = 0;
bool initialize = true;
//------------------------------------------------------------------------


//---------------------------------Blinker-Start-State----------------------------
bool rightBlinkerActive = false;
bool rightBlinkerOn = false;
bool leftBlinkerActive = false;
bool leftBlinkerOn = false;
bool autoMode = false;

float BLINK_RATE = 250;           // rate at which the blinker will turn on and off (in ms)
unsigned long timeAtLastFrame = 0;  // the recorded time at the last frame in ms

//-----------------------------------------------------------------------------------




//---------------------Check-for-intro-text----------------------------------------
void checkScrollLCDTextForIntro() {
  if (charsScrolled < 16) {
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

void servoSlew() {
  int leftDist = 0, rightDist = 0;
  long maxDist = 0;

  // Scan left (move the servo to 45 degrees)
  for (pos = 60; pos <= 150; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
  leftDist = Ultrasonic_read();  // Measure distance at left
  Serial.print("Left Distance: ");
  Serial.println(leftDist);      // Debugging output

  // Scan right (move the servo to 135 degrees)
  for (pos = 150; pos >= 60; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);
  }                       // waits 15 ms for the servo to reach the position
  rightDist = Ultrasonic_read();  // Measure distance at right
  Serial.print("Right Distance: ");
  Serial.println(rightDist);     // Debugging output
  myservo.write(90); //return to origin

  // Compare the distances and choose the direction with the greater distance
  if (leftDist > rightDist) {
    Serial.println("I'm going left!");
    moveLeft();  // Move left and perform obstacle avoidance maneuvers
  } else {
    Serial.println("I'm going right!");
    moveRight(); // Move right and perform obstacle avoidance maneuvers
  }
}
void moveLeft() {
  int shortDistance = 400; // Distance to travel in the short maneuver

  // Step 1: Turn 90° left
  hardLeft(75);
  delay(2000);  // adjust time for turn

  // Step 2: Move forward a short distance
  Forward(100);
  delay(1000);  // adjust time for turn

  // Step 3: Turn right
  hardRight(75);
  delay(2000);  // adjust time for turn

  // Step 4: Move forward the same short distance
  Forward(100);
  delay(1500);  // Adjust time to set distance

  // Step 5: Turn left to return to original orientation
  hardRight(75);
  delay(2000);  // adjust time for turn

  Forward(100);
  delay(1000);  // adjust time for turn

  hardLeft(75);
  delay(2000);  // adjust time for turn

  // Return to pathfinding
  pathfinding();  // Call pathfinding to continue
}

void moveRight() {
  int shortDistance = 400; // Distance to travel in the short maneuver

  // Step 1: Turn right
  hardRight(75);
  delay(2000);  // adjust for turn

  // Step 2: Move forward
  Forward(100);
  delay(1000);  // adjust time for distance

  // Step 3: Turn left
  hardLeft(75);
  delay(1500);  // Adjust time for turn

  // Step 4: Move forward
  Forward(100);
  delay(1500);  // Adjust time for distance

  // Step 5: turn left
  hardLeft(75);
  delay(1500);  // Adjust time for turn

  Forward(100);
  delay(1000);  // adjust time for distance

  hardRight(75);
  delay(2000);  // adjust for turn

  // Return to pathfinding
  pathfinding();  // Call pathfinding to continue
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

//--------------------- Motor control functions------------------------------------------
void Forward(int speed) {
  analogWrite(MotorPWM_A, speed);  // Sets left motor speed
  analogWrite(MotorPWM_B, speed);  // Sets right motor speed

  digitalWrite(INA1A, LOW);
  digitalWrite(INA2A, HIGH);

  digitalWrite(INA1B, LOW);
  digitalWrite(INA2B, HIGH);
}
//-----------------------------------------------------------------------------------------


//-------------------------------directional-controls--------------------------------------
void Left(int speed) {
  analogWrite(MotorPWM_A, speed / 3);  // Slow left motor
  analogWrite(MotorPWM_B, speed);      // Full speed on right motor

  digitalWrite(INA1A, LOW);
  digitalWrite(INA2A, HIGH);

  digitalWrite(INA1B, LOW);
  digitalWrite(INA2B, HIGH);
}

void hardRight(int speed) {
  analogWrite(MotorPWM_A, speed);     
  analogWrite(MotorPWM_B, speed);  

  digitalWrite(INA1A, LOW);
  digitalWrite(INA2A, HIGH);

  digitalWrite(INA1B, HIGH);
  digitalWrite(INA2B, LOW);
}
void hardLeft(int speed) {
  analogWrite(MotorPWM_A, speed);  
  analogWrite(MotorPWM_B, speed);      

  digitalWrite(INA1A, HIGH);
  digitalWrite(INA2A, LOW);

  digitalWrite(INA1B, LOW);
  digitalWrite(INA2B, HIGH);
}

void Right(int speed) {
  analogWrite(MotorPWM_A, speed);      // Full speed on left motor
  analogWrite(MotorPWM_B, speed / 3);  // Slow right motor

  digitalWrite(INA1A, LOW);
  digitalWrite(INA2A, HIGH);

  digitalWrite(INA1B, LOW);
  digitalWrite(INA2B, HIGH);
}
void Reverse(int speed) {
  analogWrite(MotorPWM_A, speed);  // Sets left motor speed
  analogWrite(MotorPWM_B, speed);  // Sets right motor speed

  digitalWrite(INA1A, HIGH);
  digitalWrite(INA2A, LOW);

  digitalWrite(INA1B, HIGH);
  digitalWrite(INA2B, LOW);
}

void Stop() {
  analogWrite(MotorPWM_A, 0);
  analogWrite(MotorPWM_B, 0);
}
//------------------------------------------------------------------------------------
int recoveryCounter = 0;  // Tracks consecutive failed line detections


//---------------------------------Path-Finding--------------------------------------------
void pathfinding() {
  long distance = Ultrasonic_read();  // Get distance from ultrasonic sensor

  // If an obstacle is within 20 cm, pause pathfinding
  if (distance <= 15) {
    Stop();  // Stop the robot
    Serial.println("Obstacle detected! Evasive manuevers!.");
    servoSlew();  // Start the slewing and distance comparison for evasive manuevers 
    return;
  }
  int leftSensorRead = analogRead(L_S);
  int middleSensorRead = analogRead(M_S);
  int rightSensorRead = analogRead(R_S);
  const int lowerThreshold = 950;
  const int upperThreshold = 1100;
  const int baseSpeed = 80;          // Normal speed

  if (middleSensorRead >= lowerThreshold && middleSensorRead <= upperThreshold) {
    Forward(baseSpeed);
    recoveryCounter = 0;  // Reset recovery on successful line detection
  } 
  else if (leftSensorRead >= lowerThreshold && leftSensorRead <= upperThreshold) {
    Left(baseSpeed);
    recoveryCounter = 0;
  } 
  else if (rightSensorRead >= lowerThreshold && rightSensorRead <= upperThreshold) {
    Right(baseSpeed);
    recoveryCounter = 0;
  } 
  else {
    // Initiate recovery mode if no line is detected
    recovery();
  }
}

void recovery() {
  const int recoverySpeed = 75;      // Slower speed during recovery
  int turnDuration = min(recoveryCounter * 150, 3000); // Increase duration, cap at 3s

  recoveryCounter++;  // Increment counter on each recovery attempt

  // Alternate between left and right turns for better search coverage
  if (recoveryCounter % 2 == 0) {
    // Turn left in place slowly
    digitalWrite(INA1A, HIGH);
    digitalWrite(INA2A, LOW);
    digitalWrite(INA1B, LOW);
    digitalWrite(INA2B, HIGH);
  } else {
    // Turn right in place slowly
    digitalWrite(INA1A, LOW);
    digitalWrite(INA2A, HIGH);
    digitalWrite(INA1B, HIGH);
    digitalWrite(INA2B, LOW);
  }

  analogWrite(MotorPWM_A, recoverySpeed);
  analogWrite(MotorPWM_B, recoverySpeed);

  delay(turnDuration);  // Perform recovery turn
  Stop();               // Brief stop between turns for stability
  delay(200);
}

long Ultrasonic_read(){
digitalWrite(trigger, LOW);
delayMicroseconds(2);
digitalWrite(trigger, HIGH);
delayMicroseconds(10);
long time = pulseIn (echo, HIGH);
return time / 29 / 2;
}



void setup() {
  Serial.begin(38400);  // Monitor
  Serial1.begin(38400); // HC-05
  lcd.init();
  lcd.backlight();
  Sprite::initTeam10NameCredits(lcd);

  myservo.attach(10); //attached servo to pin 10

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(MotorPWM_A, OUTPUT);
  pinMode(MotorPWM_B, OUTPUT);
  pinMode(INA1A, OUTPUT);
  pinMode(INA2A, OUTPUT);
  pinMode(INA1B, OUTPUT);
  pinMode(INA2B, OUTPUT);

  pinMode(echo, INPUT);
  pinMode(trigger, OUTPUT);

  timeAtLastFrame = millis();  // initialize time tracking
}

void loop() {
  checkScrollLCDTextForIntro();
  //servoSlew();

  // Print sensor values to Serial Monitor
  Serial.print("Left Sensor: ");
  Serial.println(analogRead(L_S));
  Serial.print("Middle Sensor: ");
  Serial.println(analogRead(M_S));
  Serial.print("Right Sensor: ");
  Serial.println(analogRead(R_S));
  Serial.println();

  // Print ultrasonic readings
  distance_F = Ultrasonic_read();
  Serial.print("D F=");Serial.println(distance_F);

  
  // Bluetooth communication monitoring
  if (Serial1.available()) {
    String command = Serial1.readStringUntil('\n');
    command.trim();

    // Print received command to the Serial Monitor
    Serial.print("Received: ");
    Serial.println(command);

    if (command.equalsIgnoreCase("auto")) {
      autoMode = true;  // Enable auto mode
    } else if (command.equalsIgnoreCase("stop")) {
      autoMode = false;  // Disable auto mode
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

  if (autoMode) {
    pathfinding();  // Run pathfinding logic if auto mode is enabled
  }
}
