//sketch created by Akshay Joseph
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x11, 16, 2);

void setup()
{

  lcd.begin();


  lcd.backlight();
  lcd.clear();
  lcd.setCursor(4,0);
  lcd.print("Hackster");
}

void loop()
{
}
