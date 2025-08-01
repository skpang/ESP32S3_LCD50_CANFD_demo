#include "Arduino.h"
#include "lcd_util.h"
#define Backlight_MAX   100 
#define LCD_Backlight_PIN   1 

void Set_Backlight_480(uint8_t Light)                        //
{
  if(Light > Backlight_MAX || Light < 0)
    printf("Set Backlight parameters in the range of 0 to 100 \r\n");
  else{
    uint32_t Backlight = Light*10;
    if(Backlight == 1000)
      Backlight = 1024;
    ledcWrite(LCD_Backlight_PIN, Backlight);
  }
}