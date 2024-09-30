namespace Sprites_N
{
  class Sprite
  {
    private:
    bool leftArrowOn = true;
    bool rightArrowOn = true;
    public:
      Sprite();
      Sprite(unsigned int pin);
      byte leftArrow[14]
      {
        0b00000001,
        0b00000011,
        0b00000111,
        0b00001111,
        0b00011111,
        0b00111111,
        0b01111111,
        0b11111111,
        0b01111111,
        0b00111111,
        0b00011111,
        0b00001111,
        0b00000111,
        0b00000011,
        0b00000001
      };

      byte rightArrow[14]
      {
        0b10000000,
        0b11000000,
        0b11100000,
        0b11110000,
        0b11111000,
        0b11111100,
        0b11111110,
        0b11111111,
        0b11111110,
        0b11111100,
        0b11111000,
        0b11110000,
        0b11100000,
        0b11000000,
        0b10000000
      };

      byte teamTenSprite[8]
      {
        0b11111111,
        0b11000011,
        0b10100101,
        0b10011001,
        0b10011001,
        0b10100101,
        0b11000011,
        0b11111111,
      };

      void displaySpriteToLcd(byte sprite, unsigned int cursorStart, unsigned int cursorEnd);
      void clearSpriteFromLcd(byte sprite, unsigned int cursorStart, unsigned int cursorEnd);
      void toggleOnSprite(byte sprite);
  }
}