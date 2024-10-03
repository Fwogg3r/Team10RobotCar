/*
Thomas Bryant
9/30/2024
sprites.cpp | The cpp file associated with the sprites header. Defines functions.
*/

#include <LiquidCrystal_I2C.h>
#include "sprites.h"

//using namespace Sprites_N;
//TODO: Define sprite positions on LCD.

/*static uint8_t Sprite::leftArrowTopLeft[8] = {
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00001,
  0b00011,
  0b00111
};
static uint8_t Sprite::leftArrowTopRight[8] = {
  0b00001,
  0b00011,
  0b00111,
  0b01111,
  0b11111,
  0b11111,
  0b11111,
  0b11111
};
static uint8_t Sprite::leftArrowBottomLeft[8] = {
  0b00111,
  0b00011,
  0b00001,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000
};
static uint8_t Sprite::leftArrowBottomRight[8] = {
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b01111,
  0b00111,
  0b00011,
  0b00001
};
static uint8_t Sprite::leftArrowWhole[16] = {
  0b0000000001,  //16 height, 8 each hemisphere, 10 wide, 5 each hemisphere.
  0b0000000011,
  0b0000000111,
  0b0000001111,
  0b0000011111,
  0b0000111111,
  0b0001111111,
  0b0011111111,
  0b0011111111,
  0b0001111111,
  0b0000111111,
  0b0000011111,
  0b0000001111,
  0b0000000111,
  0b0000000011,
  0b0000000001
};*/

/*static uint8_t Sprite::rightArrowTopLeft[8] = {
  0b10000,
  0b11000,
  0b11100,
  0b11110,
  0b11111,
  0b11111,
  0b11111,
  0b11111
};
static uint8_t Sprite::rightArrowTopRight[8] = {
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b10000,
  0b11000
};
static uint8_t Sprite::rightArrowBottomLeft[8] = {
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11110,
  0b11100,
  0b11000,
  0b10000
};
static uint8_t Sprite::rightArrowBottomRight[8] = {
  0b11100,
  0b11000,
  0b10000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000
};

static uint8_t Sprite::rightArrowWhole[16] = {
  0b1000000000,
  0b1100000000,
  0b1110000000,
  0b1111000000,
  0b1111100000,
  0b1111110000,
  0b1111111000,
  0b1111111100,
  0b1111111100,
  0b1111111000,
  0b1111110000,
  0b1111100000,
  0b1111000000,
  0b1110000000,
  0b1100000000,
  0b1000000000,
};
*/

static uint8_t Sprite::leftArrow[8] =
{
  0b00011,
  0b00111,
  0b01111,
  0b11111,
  0b11111,
  0b01111,
  0b00111,
  0b00011,
};

static uint8_t Sprite::rightArrow[8] =
{
  0b11000,
  0b11100,
  0b11110,
  0b11111,
  0b11111,
  0b11110,
  0b11100,
  0b11000,
};

static uint8_t Sprite::teamTenSpriteLeft[8] = {
  0b11111,
  0b10100,
  0b10100,
  0b10100,
  0b10100,
  0b10100,
  0b11110,
  0b11111
};

static uint8_t Sprite::teamTenSpriteRight[8] = {
  0b11111,
  0b01001,
  0b10101,
  0b10101,
  0b10101,
  0b10101,
  0b01001,
  0b11111
};

static uint8_t Sprite::teamTenSprite[8] =  //LCD Display is 8x5.. This sprite is 1 long by 2 wide.
  {
    0b1111111111,
    0b1010001001,
    0b1010010101,
    0b1010010101,
    0b1010010101,
    0b1010010101,
    0b1111001001,
    0b1111111111,
  };