/*#include <LiquidCrystal_I2C.h> //by Frank de Brabander
//#include <Adafruit_NeoPixel.h> //Adafruit, 1.12.3
#include "sprites.h"
#include "motor.h"
#include <SoftwareSerial.h>

#define RGB_PIN 38
#define LED_RED 13
#define LED_GREEN 12
#define Rx 15  //sets transmit pin on the bluetooth to the Rx pin on the arduino
#define Tx 14  //sets recieve pin on the bluetooth to the Tx pin on the arduino
#define BLUETOOTH_BAUD_RATE 38400
#define interruptPin 2

//Adafruit_NeoPixel pixels(NUMPIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800); //The RGB LED on the MEGA
LiquidCrystal_I2C lcd(0x27,20,2);
SoftwareSerial bluetooth(Rx,Tx);

int charsScrolled = 0;
bool initialize = true; //initialization values

bool rightBlinkerActive = false; //if the blinker is set to execute logic in the loop.
bool rightBlinkerOn = false; //if the LED voltage is set to HIGH.
bool leftBlinkerActive = false;
bool leftBlinkerOn = false;

float BLINK_RATE = 250; // rate at which the blinker will turn on and off (in ms)
float timeAtLastFrame = 0; //the recorded time at the last frame in ms (since initial loop first runtime)
float timeUntilBlinkerChange = 500; //the time until the blinker is scheduled to change (once negative blinker values flip.)

void checkScrollLCDTextForIntro()
{
  if(charsScrolled < 14)
  {
    delay(500);
    lcd.scrollDisplayLeft();
    charsScrolled++;
  }
  else if(initialize)
  {
    delay(2000);
    lcd.clear();
    Sprite::initLCD(lcd);
    initialize = false;
  }
}

static void activateBlinker(char* side, int LED, bool blinkerOn)
{
  if(side == "left")
  {
    if(blinkerOn)
    {
      digitalWrite(LED, HIGH);
      Sprite::blinkLeft(lcd);
    }
    else
    {
      digitalWrite(LED, LOW);
      Sprite::blinkersOff(lcd);
    }
  }
  else if(side == "right")
  {
    if(blinkerOn)
    {
      digitalWrite(LED, HIGH);
      Sprite::blinkRight(lcd);
    }
    else
    {
      digitalWrite(LED, LOW);
      Sprite::blinkersOff(lcd);
    }
  }
}

static void setAllBlinkersOff()
{
  leftBlinkerActive = false;
  leftBlinkerOn = false;
  rightBlinkerActive = false;
  rightBlinkerOn = false;
  activateBlinker("left", LED_RED, false);
  activateBlinker("right", LED_GREEN, false);
}

static void increaseInterruptCounter()
{
  count++;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400);  // Monitor
  Serial1.begin(38400); // HC-05
  lcd.init();                      // initialize the lcd 
  lcd.backlight();
  Sprite::initTeam10NameCredits(lcd);
  pinMode(Rx, INPUT);     //sets Rx as an input
  pinMode(Tx, OUTPUT);    //sets Tx as an output
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  //bluetooth.begin(BLUETOOTH_BAUD_RATE);     //starts bluetooth communication
  //Serial.println("Beginning serial on 9600.");

  pinMode(MotorPWM_A, OUTPUT);
  pinMode(MotorPWM_B, OUTPUT);
  pinMode(INA1A, OUTPUT);
  pinMode(INA2A, OUTPUT);
  pinMode(INA1B, OUTPUT);
  pinMode(INA2B, OUTPUT);

  pinMode(ENCODER, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), increaseInterruptCounter, FALLING);

}

void loop() {
  // put your main code here, to run repeatedly:
  //scroll 14 characters then end...
  checkScrollLCDTextForIntro();
  timeUntilBlinkerChange -= millis() - timeAtLastFrame; //not very optimal - unfortunate!
  timeAtLastFrame = millis();
  /*Serial.print("Time until Blinker Change: ");
  Serial.println(timeUntilBlinkerChange);
  Serial.print("Time at last frame: ");
  Serial.println(timeAtLastFrame);*/

/*
  if (Serial1.available()) {
    String command = Serial1.readStringUntil('\n'); // Read command from HC-05 using serial 1 which is pins 18 and 19
    command.trim(); // Remove any leading/trailing spaces, etc. This was an issue when I was sending the
    //color codes. I made them "red, green, blue" on my phone and serial monitor would show they were
    //received but not understood. I just changed these to right and left
    
    Serial.print("Received: ");
    Serial.println(command);
    
    // Handle the received command
    if (command.equalsIgnoreCase("right")) 
    {
      setAllBlinkersOff();
      rightBlinkerActive = !rightBlinkerActive;
    } 
    else if (command.equalsIgnoreCase("left"))
    {
      setAllBlinkersOff();
      leftBlinkerActive = !leftBlinkerActive;
    }
    else if(command.equalsIgnoreCase("off"))
    {
      setAllBlinkersOff();
    }
    else 
    {
      Serial.println("Unknown command"); // for when you push something that the board has no idea what it is
    }
  }

  if(leftBlinkerActive)
  {
    if(timeUntilBlinkerChange <= 0)
    {
      timeUntilBlinkerChange = BLINK_RATE;
      leftBlinkerOn = !leftBlinkerOn;
      activateBlinker("left", LED_RED, leftBlinkerOn);
    }
  }

  if(rightBlinkerActive)
  {
    if(timeUntilBlinkerChange <= 0)
    {
      timeUntilBlinkerChange = BLINK_RATE;
      rightBlinkerOn = !rightBlinkerOn;
      activateBlinker("right", LED_GREEN, rightBlinkerOn);
    }
  }

  
  Serial.print("RPM =");
  Serial.println(RPM);
}
*/
