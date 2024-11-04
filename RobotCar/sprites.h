/*
Thomas Bryant
9/30/2024
sprites.h | The sprites header designed to hold the data for the various sprites related to the robot car LCD Screen.
*/
#include "WString.h"
#include <stdint.h>
#include <LiquidCrystal_I2C.h>

#ifndef SPRITES_H
#define SPRITES_H
#if defined(ARDUINO) && ARDUINO >= 100
#define printByte(args)  write(args);
#else
#define printByte(args)  print(args,BYTE);
#endif

class Sprite {
private:
  bool leftArrowOn = true;
  bool rightArrowOn = true;
public:
  /*static uint8_t leftArrowTopLeft[8];
  static uint8_t leftArrowTopRight[8];
  static uint8_t leftArrowBottomLeft[8];
  static uint8_t leftArrowBottomRight[8];
  static uint8_t leftArrowWhole[16];
  static uint8_t rightArrowTopLeft[8];
  static uint8_t rightArrowTopRight[8];
  static uint8_t rightArrowBottomLeft[8];
  static uint8_t rightArrowBottomRight[8];
  static uint8_t rightArrowWhole[16];
  */
  static uint8_t leftArrow[8];
  static uint8_t rightArrow[8];
  static uint8_t teamTenSpriteLeft[8];
  static uint8_t teamTenSpriteRight[8];
  static uint8_t teamTenSprite[8];  //LCD Display is 8x5.. This sprite is 1 long by 2 wide.
  Sprite();
  static void initTeam10NameCredits(LiquidCrystal_I2C lcd);
  static void initLCD(LiquidCrystal_I2C lcd);
  static void blinkersOff(LiquidCrystal_I2C lcd);
  static void blinkLeft(LiquidCrystal_I2C lcd);
  static void blinkRight(LiquidCrystal_I2C lcd);
  //static void activateBlinker(char* side, int LED, LiquidCrystal_I2C &lcd, bool blinkerOn);
  /*void toggleOnSprite(byte sprite)
      {
        if(sprite == this->leftArrow)
        {
          this->leftArrowOn = !this->leftArrowOn;
          if(this->leftArrowOn)
          {
          //display
          }
          else 
          {
            //clear that segment
          }
        }
        else if(sprite == this->rightArrow)
        {
          thos->rightArrowOn = !rightArrowOn;
        }
      }*/
};
#endif