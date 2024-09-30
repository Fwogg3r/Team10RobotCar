/*
Thomas Bryant
9/30/2024
sprites.cpp | The cpp file associated with the sprites header. Defines functions.

*/
using namespace Sprites_N;
#include <LCD_I2C.h>

//TODO: Define sprite positions on LCD.

Sprite::displaySpriteToLcd(byte sprite, unsigned int cursorStart, unsigned int cursorEnd)
{

}
Sprite::clearSpriteFromLcd(byte sprite, unsigned int cursorStart, unsigned int cursorEnd)
{

}

Sprite::toggleOnSprite(byte sprite)
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
    rightArrowOn = !rightArrowOn;
  }
}