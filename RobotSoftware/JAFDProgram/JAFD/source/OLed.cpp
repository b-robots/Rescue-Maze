#include"../header/OLed.h"

namespace JAFD
{
	namespace OLed
	{
		
		namespace
		{
			U8GLIB_SSD1306_128X32 u8g(U8G_I2C_OPT_NONE | U8G_I2C_OPT_DEV_1);
			uint32_t startTime = 0;
			uint32_t pauseTime = 0;
			bool startset = true;
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

			if (gamestart && startset)
			{
				startTime = millis();
				startset = false;
			}

			u8g.setFont(u8g_font_4x6);
			u8g.drawStr(92,6,"we");
		}

		void robotAktiveStopWatch(bool gamestart)
		{
			if (gamestart && startset)
			{
				startTime = millis();
				startset = false;
			}
			else if (!gamestart)
			{
				pauseTime = millis();
			}
			else
			{
				u8g.setFont(u8g_font_4x6);
				startTime = millis();
			}
			
		}

		void robotGameTimer(bool gamestart)
		{
			
			u8g.setFont(u8g_font_4x6);
			startTime = millis();
		}
	}
}
