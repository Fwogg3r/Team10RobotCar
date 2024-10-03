#include <LiquidCrystal_I2C.h> //by Frank de Brabander
#include <Adafruit_NeoPixel.h> //Adafruit, 1.12.3
#include "sprites.h"

#define NUMPIXELS 1
#define RGB_PIN 38
#define SCREEN_WIDTH
#if defined(ARDUINO) && ARDUINO >= 100
#define printByte(args)  write(args);
#else
#define printByte(args)  print(args,BYTE);
#endif

Adafruit_NeoPixel pixels(NUMPIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800); //The RGB LED on the MEGA
LiquidCrystal_I2C lcd(0x27,20,2);

int charsScrolled = 0;
bool initialize = true;

void initTeam10NameCredits()
{
  lcd.setCursor(0,0);
  lcd.print("TEAM 10: Thomas, Trevor, Ryan ");
}

void initLCD()
{
  //Create the characters associated with the LCD:
  //Left and right arrow:
  lcd.createChar(0, Sprite::leftArrow);
  lcd.createChar(1, Sprite::rightArrow);
  //Team 10 Sprite:
  lcd.createChar(2, Sprite::teamTenSpriteLeft);
  lcd.createChar(3, Sprite::teamTenSpriteRight);
  //Set the left arrow to the bottom left of the screen
  lcd.setCursor(0,1); //Left arrow
  lcd.printByte(0);

  lcd.setCursor(7, 1); //Bottom middle, team 10 logo
  lcd.printByte(2);
  lcd.printByte(3);

  lcd.setCursor(15, 1); //Bottom right, Right arrow
  lcd.printByte(1);
}

void setup() {
  // put your setup code here, to run once:
  lcd.init();                      // initialize the lcd 
  lcd.backlight();
  initTeam10NameCredits();
  //Generate sprite Characters from sprites.h:
  //Arrows:
}

void loop() {
  // put your main code here, to run repeatedly:
  //scroll 14 characters then end...
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
    initLCD();
    initialize = false;
  }
}
