#include <LiquidCrystal_I2C.h> //by Frank de Brabander
#include <Adafruit_NeoPixel.h> //Adafruit, 1.12.3
#include "sprites.h"
#include "bluetooth.h"
//using namespace SPRITES_H;
//using namespace BLUETOOTH_H;

#define NUMPIXELS 1
#define RGB_PIN 38
#define LED_RED 13
#define LED_GREEN 12
#define SCREEN_WIDTH
#define Rx 15  //sets transmit pin on the bluetooth to the Rx pin on the arduino
#define Tx 14  //sets recieve pin on the bluetooth to the Tx pin on the arduino
#define BLUETOOTH_BAUD_RATE 38400

Adafruit_NeoPixel pixels(NUMPIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800); //The RGB LED on the MEGA
LiquidCrystal_I2C lcd(0x27,20,2);
SoftwareSerial bluetooth(Rx,Tx);

int charsScrolled = 0;
bool initialize = true;
bool blinkerOn = false;
float timeAtLastFrame = 0;
float timeUntilBlinkerChange = 500;

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
  if(side == "Left")
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
  else if(side == "Right")
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

void setup() {
  // put your setup code here, to run once:
  //Serial.begin(9600);
  lcd.init();                      // initialize the lcd 
  lcd.backlight();
  Sprite::initTeam10NameCredits(lcd);
  pinMode(Rx, INPUT);     //sets Rx as an input
  pinMode(Tx, OUTPUT);    //sets Tx as an output
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  bluetooth.begin(BLUETOOTH_BAUD_RATE);     //starts bluetooth communication
  //Serial.println("Beginning serial on 9600.");
}

void loop() {
  // put your main code here, to run repeatedly:
  //scroll 14 characters then end...
  checkScrollLCDTextForIntro();
  if(bluetooth.available())
  {
    lcd.print("BLUETOOTH DETECTED ON INO");
  }
  String command = readCommand(bluetooth, lcd);
  timeUntilBlinkerChange -= millis() - timeAtLastFrame; //not very optimal - unfortunate!
  timeAtLastFrame = millis();
  if(command == "Left")
  {
    Serial.println("Left Command Identified...");
    if(timeUntilBlinkerChange <= 0)
    {
      timeUntilBlinkerChange = 500;
      activateBlinker("Left", LED_GREEN, !blinkerOn);
    }
  }
  else if(command == "Right")
  {
    Serial.println("Left Command Identified...");
    if(timeUntilBlinkerChange <= 0)
    {
      timeUntilBlinkerChange = 500;
      activateBlinker("Right", LED_RED, !blinkerOn);
    }
  }
}
