#include <LiquidCrystal_I2C.h>
#include <string.h>

class LCD
{
  public:
  /*static void LCD::Display(LiquidCrystal_I2C lcd, string text, bool setCursor = false, int x=0, int y=0, bool clear = false)
  {
    if(clear)
    {
      lcd.clear();
    }
    if(setCursor)
    {
      lcd.setCursor(x,y);
    }
    lcd.println(text);
  }*/
  static void LCD::DisplaySpeedAndDirection(LiquidCrystal_I2C lcd, float speed, String direction, float battery_voltage, float distance)
  {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(speed);
      lcd.setCursor(7, 0);
      lcd.print("BAT: ");
      lcd.print(battery_voltage);
      lcd.setCursor(0, 1);
      lcd.print(direction);
      lcd.setCursor(8, 1);
      lcd.print("DIST:");
      lcd.print(distance);
  }
};