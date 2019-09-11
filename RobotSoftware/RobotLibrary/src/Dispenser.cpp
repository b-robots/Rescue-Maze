/*
This part of the Library is responsible for dispensing the rescue packages.
*/

#pragma once

#include "Dispenser.h"
#include "Servo.h"

namespace JAFTD
{
	namespace Dispenser
	{
		// Create Objekt instances
		Servo servoLeft;
		Servo servoRight;

		//rest and extendet angle in deegres (i am mayby making a calibration function)
		const int startAngle = 90;
		const int extendAngle = 180;

		// Set up the Dispenser System
		ReturnCode dispenserSetup(uint8_t srvPinLeft, uint8_t srvPinRight, uint8_t btnLeft, uint8_t btnRight)
		{

			//initialize the used pins 
			servoLeft.attach(srvPinLeft);
			servoRight.attach(srvPinRight);

			pinMode(btnLeft, INPUT);
			pinMode(btnRight, INPUT);

			//setting the servo to start angle
			servoLeft.write(startAngle);
			servoRight.write(startAngle);

			return ReturnCode::ok;
		}

		// Dispence items if side true = left
		ReturnCode dispense(uint8_t amount, bool side, uint8_t btnLeft, uint8_t btnRight, char *status)
		{
			// Auswerfen...
			uint8_t rightPacks = 6;
			uint8_t leftPacks = 6;
			uint8_t rightPackCount = 6;
			uint8_t leftPacksCount = 6;
			static uint8_t retrysLeft = 0;
			static uint8_t etrysRight = 0;


			if (side && retrysLeft <= 2)
			{
				for (uint8_t i = 0; i < amount; i++)
				{
					servoLeft.write(extendAngle);
					//mayby we need to experiment with the delay a little bitt
					delay(500);
					// we need to ensure hardware technicly that we dont have a to strong bounce 
					if (digitalRead() == LOW)
					{
						leftPacksCount--;
					}
					servoLeft.write(startAngle);
					leftPacks--;

					if (leftPacks != leftPackCount)
					{
						retrysLeft++;
						//R for Retry
						*status = 'R';
						return ReturnCode::aborted;
					}
				}
			}
			else if ()
			{
				for (uint8_t i = 0; i < amount; i++)
				{
					servoRight.write(extendAngle);
					//mayby we need to experiment with the delay a little bitt
					delay(500);
					servoRight.write(startAngle);
					RightPacks--;
				}
			}

			if (/*irgendwas geht schief*/0)
			{
				return ReturnCode::error;
			}

			return ReturnCode::ok;
		}
	}
}