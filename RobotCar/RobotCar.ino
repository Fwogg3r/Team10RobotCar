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

Adafruit_NeoPixel pixels(NUMPIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800); //The RGB LED on the MEGA
LiquidCrystal_I2C lcd(0x27,20,2);

int charsScrolled = 0;
bool initialize = true;

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

void setup() {
  // put your setup code here, to run once:
  lcd.init();                      // initialize the lcd 
  lcd.backlight();
  Sprite::initTeam10NameCredits(lcd);
  pinMode(Rx, INPUT);     //sets Rx as an input
  pinMode(Tx, OUTPUT);    //sets Tx as an output
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  bluetooth.begin(BLUETOOTH_BAUD_RATE);     //starts bluetooth communication
}

void loop() {
  // put your main code here, to run repeatedly:
  //scroll 14 characters then end...
  checkScrollLCDTextForIntro();
  String command = readCommand();
  if(command == "Left")
  {
    Sprite::activateBlinker("Left", LED_GREEN, lcd);
  }
  else if(command == "Right")
  {
    Sprite::activateBlinker("Right", LED_RED, lcd);
  }
}
