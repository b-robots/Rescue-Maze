/*
 Name:		MainProgram.ino
 Created:	07.08.2019 14:16:07
 Author:	B.Robots
*/

// External Libs
#include <SPI.h>
#include <VL53L0X.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_AMG88xx.h>
#include <TPA81.h>
#include <U8glib.h> 
#include <string> 

// RobotLibrary
#include "JAFD/JAFD.h"
#include "JAFD/header/OLed.h"
using namespace JAFD::OLed;

// The setup function runs once when you press reset or power the board
;
void setup()
{
    Serial.begin(115200);
    
	// For testing
	 //Set font.
}

U8GLIB_SSD1306_128X32 u8g(U8G_I2C_OPT_NONE | U8G_I2C_OPT_DEV_1);
//0x3C OLED Adress
void loop()
{
    
   
    String time(millis()/1000);
    //OLed::OLed asd;
   
    //u8g.setFont(u8g_font_unifont);
    //for (int i = 0; i < 100; i++)
    //{
    //    String s(i);
    //    u8g.firstPage();
    //    do
    //    {
    //        u8g.drawBox(6*i+1, 5, 10, 20);
    //        u8g.drawStr(6*i+40, 16, s.c_str());
    //        u8g.drawPixel(6*i+100, 16);
    //        u8g.drawStr(6*i+17, 16, "??");
    //        
    //    } while (u8g.nextPage());
    //}
      u8g.firstPage();
      do
      {
          u8g.setFont(u8g_font_4x6);
          u8g.drawStr(92, 6, time.c_str());
 
      } while (u8g.nextPage());

      
 
   
    //Serial.println((millis()-time));

    
    //Delay before repeating the loop.
}