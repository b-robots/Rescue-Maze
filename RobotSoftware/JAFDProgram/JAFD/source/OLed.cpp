#include"../header/OLed.h"

namespace JAFD
{
	namespace OLed
	{
		
		namespace
		{
			U8GLIB_SSD1306_128X32 u8g(U8G_I2C_OPT_NONE | U8G_I2C_OPT_DEV_1);
			
			bool startset = true;

			struct OLedTime
			{
				uint8_t h = 0;
				uint8_t m = 0;
				uint8_t s = 0;

				uint32_t startTime = 0;
				uint32_t pauseTime = 0;
			};

			uint32_t flagTime = 0;

			struct OLedStringTime
			{
				String h;
				String m;
				String s;
			};

			OLedTime time;
			OLedTime relTime;
		}

		U8GLIB_SSD1306_128X32* getOLedInstance()
		{
			return &u8g;
		}

		void robotStopWatch(bool gamestart)
		{
			while (!gamestart)
			{
			}

			//Seting the starttime
			if (gamestart && startset)
			{
				time.startTime = millis();
				startset = false;
			}

			//Time Declearation

			if (time.s <= 60)
			{
				time.s = (millis() - (time.startTime + flagTime))/1000;
			}
			else if(time.s > 60)
			{
				time.s = 0;
				time.m++;
				flagTime = millis();
			}

			if (time.m > 60)
			{
				time.h++;
				time.m = 0;
			}
			
			//Stringyfy
			String s(time.s);
			String m(time.m);
			String h(time.h);
			String fg(flagTime);

			//Define Display Format
			String buffer = h + ":" + m + ":" + s;

			//Define Look + Position
			u8g.setFont(u8g_font_10x20);
			u8g.drawStr(1,26,buffer.c_str());
		}

		void robotAktiveStopWatch(bool gamestart)
		{
			if (gamestart && startset)
			{
				time.startTime = millis();
				startset = false;
			}
			else if (!gamestart)
			{
				time.pauseTime = millis();
			}
			else
			{
				u8g.setFont(u8g_font_4x6);
				time.startTime = millis();
			}
			
		}

		void robotGameTimer(bool gamestart)
		{
			
			u8g.setFont(u8g_font_4x6);
			time.startTime = millis();
		}
	}
}
