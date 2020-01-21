/*
In this part are all interrupt handler
*/

#include "../header/AllDatatypes.h"
#include "../header/Interrupts.h"
#include "../header/MotorControl.h"
#include "../header/SmoothDriving.h"
#include "../header/SensorFusion.h"

void PIOA_Handler()
{
	JAFD::MotorControl::encoderInterrupt(JAFD::Interrupts::InterruptSource::pioA, PIOA->PIO_ISR);
}

void PIOB_Handler()
{
	JAFD::MotorControl::encoderInterrupt(JAFD::Interrupts::InterruptSource::pioB, PIOB->PIO_ISR);
}

void PIOC_Handler()
{
	JAFD::MotorControl::encoderInterrupt(JAFD::Interrupts::InterruptSource::pioC, PIOC->PIO_ISR);
}

void PIOD_Handler()
{
	JAFD::MotorControl::encoderInterrupt(JAFD::Interrupts::InterruptSource::pioD, PIOD->PIO_ISR);
}

// TC0 - TC2 are reserved for Arduino Framework

// 1kHz 
void TC3_Handler()
{
	TC1->TC_CHANNEL[0].TC_SR;
}

// 100Hz / 10Hz / 1Hz
void TC4_Handler()
{
	static uint8_t i = 0;

	TC1->TC_CHANNEL[1].TC_SR;

	i++;
	// 100Hz:

	if (i % 2 == 0)
	{
		// 50Hz:
	}

	if (i % 5 == 0)
	{
		// 20Hz:
		JAFD::SensorFusion::filter(20);
		JAFD::SmoothDriving::updateSpeeds(20);

		if (i % 10 == 0)
		{
			// 10Hz:
			JAFD::MotorControl::calcMotorSpeed(10);
			JAFD::MotorControl::speedPID(10);

			if (i % 20 == 0)
			{
				// 5Hz:

				if (i % 100 == 0)
				{
					i = 0;

					// 1Hz:
				}
			}
		}
	}
}