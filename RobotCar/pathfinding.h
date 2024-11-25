/*#include <LiquidCrystal_I2C.h> // by Frank de Brabander
//#include <Adafruit_NeoPixel.h> // Adafruit, 1.12.3
#include "sprites.h"
#include <SoftwareSerial.h>

<<<<<<< Updated upstream
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

#define R_S A7
#define M_S A6
#define L_S A5 //7

LiquidCrystal_I2C lcd(0x27, 20, 2);
SoftwareSerial bluetooth(Rx, Tx);

int charsScrolled = 0;
bool initialize = true;
bool autoMode = false;  // Track whether the robot is in automatic mode

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
  } else if (strcmp(side, "hazard") == 0) {
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

static void setAllBlinkersOn() {
  leftBlinkerActive = false;  // Disable blinking activity
  rightBlinkerActive = false; // Disable blinking activity
  leftBlinkerOn = true;       // Turn left blinker solid
  rightBlinkerOn = true;      // Turn right blinker solid
  activateBlinker("hazard", 0, true); // Activate both blinkers solid
}

// Motor control functions
void Forward(int speed) {
  analogWrite(MotorPWM_A, speed);  // Sets left motor speed
  analogWrite(MotorPWM_B, speed);  // Sets right motor speed

  digitalWrite(INA1A, LOW);
  digitalWrite(INA2A, HIGH);

  digitalWrite(INA1B, LOW);
  digitalWrite(INA2B, HIGH);
}

void Left(int speed) {
  analogWrite(MotorPWM_A, speed);  // Sets left motor speed
  analogWrite(MotorPWM_B, speed);  // Sets right motor speed

  digitalWrite(INA1A, LOW);
  digitalWrite(INA2A, HIGH);

  digitalWrite(INA1B, LOW);
  digitalWrite(INA2B, LOW);
}

void Right(int speed) {
  analogWrite(MotorPWM_A, speed);  // Sets left motor speed
  analogWrite(MotorPWM_B, speed);  // Sets right motor speed

  digitalWrite(INA1A, LOW);
  digitalWrite(INA2A, LOW);

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

void pathfinding() {
  int leftSensorRead = analogRead(L_S);    // Read left sensor
  int middleSensorRead = analogRead(M_S);  // Read middle sensor
  int rightSensorRead = analogRead(R_S);   // Read right sensor

  // Determine which sensor has the lowest reading
  if (middleSensorRead <= leftSensorRead && middleSensorRead <= rightSensorRead) {
    // Middle sensor detects the line
    Forward(150);  // Move forward
  } 
  else if (leftSensorRead < middleSensorRead && leftSensorRead < rightSensorRead) {
    // Left sensor detects the line
    Left(150);  // Turn left
  } 
  else if (rightSensorRead < middleSensorRead && rightSensorRead < leftSensorRead) {
    // Right sensor detects the line
    Right(150);  // Turn right
  } 
  else {
    Stop();  // Stop in case of ambiguous readings
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
  pinMode(encoderA, INPUT);
  pinMode(encoderB, INPUT);
  pinMode(MotorPWM_A, OUTPUT);
  pinMode(MotorPWM_B, OUTPUT);
  pinMode(INA1A, OUTPUT);
  pinMode(INA2A, OUTPUT);
  pinMode(INA1B, OUTPUT);
  pinMode(INA2B, OUTPUT);
  pinMode(R_S, INPUT);
  pinMode(L_S, INPUT);

  timeAtLastFrame = millis();  // initialize time tracking
}

void loop() {
  checkScrollLCDTextForIntro();

  if (autoMode) {
    pathfinding();
  } else {
    // Manual control
    if (Serial1.available()) {
      String command = Serial1.readStringUntil('\n');
      command.trim();

      Serial.print("Received: ");
      Serial.println(command);

      if (command.equalsIgnoreCase("right")) {
        setAllBlinkersOff();
        rightBlinkerActive = !rightBlinkerActive;
        Right(150);
      } else if (command.equalsIgnoreCase("left")) {
        setAllBlinkersOff();
        leftBlinkerActive = !leftBlinkerActive;
        Left(150);
      } else if (command.equalsIgnoreCase("lights off")) {
        setAllBlinkersOff();
      } else if (command.equalsIgnoreCase("forward")) {
        Forward(150);  // Move forward at medium speed (150)
      } else if (command.equalsIgnoreCase("reverse")) {
        Reverse(150);  // Move backward at medium speed (150)
      } else if (command.equalsIgnoreCase("stop")) {
        Stop();  // Stop both motors
        setAllBlinkersOn();
      } else if (command.equalsIgnoreCase("auto")) {
        autoMode = true;  // Enable automatic mode
        Serial.println("Auto mode enabled");
      } else {
        Serial.println("Unknown command");
      }
    }
  }

  if (!autoMode) {
    unsigned long currentTime = millis();

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
}
=======
// without obstacle avoidance 
// https://github.com/Jeydori/Line-Follower-with-and-without-Obstacle-Avoidance-on-Specific-Track-?tab=readme-ov-file
#define enA 3
        #define in1 4 
        #define in2 5 
        #define in3 6  
        #define in4 7 
        #define enB 9 
        
        #define R_S A7
        #define M_S A6
        #define L_S A5 //7 
        
        void setup(){ 
        pinMode(R_S, INPUT); 
        pinMode(L_S, INPUT); 
        
        pinMode(enA, OUTPUT);  
        pinMode(in1, OUTPUT); 
        pinMode(in2, OUTPUT); 
        pinMode(in3, OUTPUT);   
        pinMode(in4, OUTPUT);  
        pinMode(enB, OUTPUT); 
        
        //adjust depending on the IR sensor's response time and calibrate it properly
        analogWrite(enA, 210); 
        analogWrite(enB, 210); 
        
        delay(500);
        }
        
        /*void loop(){
        //if Right Sensor and Left Sensor are at White color then it will call forword function
         if((digitalRead(R_S) == 0)&&(digitalRead(L_S) == 0)){
         forward(); 
         }
        //if Right Sensor is Black and Left Sensor is White then it will call turn Right function
        else if((digitalRead(R_S) == 1)&&(digitalRead(L_S) == 0)){
         turnRight();
         }
        //if Right Sensor is White and Left Sensor is Black then it will call turn Left function
        else if((digitalRead(R_S) == 0)&&(digitalRead(L_S) == 1)){
         turnLeft();
         } 
        }*/
/*
        int detectPathViaLineSensor()
        {
          int status = 0;
          int leftSensorRead = analogRead(L_S);
          int middleSensorRead = analogRead(M_S);
          int rightSensorRead = analogRead(R_S);

          if(leftSensorRead <= 0.5)
          {
            status -= 1;
          }

          if(rightSensorRead <= 0.5)
          {
            status += 1;
          }

          if(middleSensorRead <= 0.5)
          {
            status = -1;
          }
          return status;
        }
        
        void forward(){  //forword
        digitalWrite(in1, LOW); 
        digitalWrite(in2, HIGH); 
        digitalWrite(in3, HIGH); 
        digitalWrite(in4, LOW); 
        }
        
        void backword(){ //backword
        digitalWrite(in1, HIGH); 
        digitalWrite(in2, LOW);  
        digitalWrite(in3, LOW); 
        digitalWrite(in4, HIGH); 
        }
        
        void turnRight(){ //turnRight
        digitalWrite(in1, HIGH); 
        digitalWrite(in2, LOW); 
        digitalWrite(in3, HIGH);  
        digitalWrite(in4, LOW); 
        }
        
        void turnLeft(){ //turnLeft
        digitalWrite(in1, LOW);  
        digitalWrite(in2, HIGH);  
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH); 
        }
        
        void Stop(){ //stop
        digitalWrite(in1, LOW); 
        digitalWrite(in2, LOW); 
        digitalWrite(in3, LOW);  
        digitalWrite(in4, LOW);
        }
>>>>>>> Stashed changes*/
