/*
In this part are all interrupt handler
*/

#include "../header/AllDatatypes.h"
#include "../header/Interrupts.h"
#include "../header/MotorControl.h"
#include "../header/SmoothDriving.h"
#include "../header/SensorFusion.h"
#include "../header/Bno055.h"
#include "../header/TCS34725.h"
#include "../header/DistanceSensors.h"

void handleISR(JAFD::Interrupts::InterruptSource interruptSrc, uint32_t isr)
{
	if (JAFD::MotorControl::encoderInterrupt(interruptSrc, isr)) {}
	else if (JAFD::ColorSensor::interrupt(interruptSrc, isr)) {}
}

void PIOA_Handler()
{
	handleISR(JAFD::Interrupts::InterruptSource::pioA, PIOA->PIO_ISR);
}

void PIOB_Handler()
{
	handleISR(JAFD::Interrupts::InterruptSource::pioB, PIOB->PIO_ISR);
}

void PIOC_Handler()
{
	handleISR(JAFD::Interrupts::InterruptSource::pioC, PIOC->PIO_ISR);
}

void PIOD_Handler()
{
	handleISR(JAFD::Interrupts::InterruptSource::pioD, PIOD->PIO_ISR);
}

// TC0 - TC2 are reserved for Arduino Framework

// 1kHz 
// Not active
void TC3_Handler()
{
	{
		volatile auto dummy = TC1->TC_CHANNEL[0].TC_SR;
	}
}

// 100Hz
// Not active
void TC4_Handler()
{
	static uint8_t i = 0;

	{
		volatile auto dummy = TC1->TC_CHANNEL[1].TC_SR;
	}

	i++;

	// 100Hz:

	if (i % 2 == 0)
	{
		// 50Hz:
	}
}

// 20Hz
// Active
void TC5_Handler()
{
	static uint8_t i = 0;

	{
		volatile auto dummy = TC1->TC_CHANNEL[2].TC_SR;
	}

	i++;

	// 20Hz:
	JAFD::MotorControl::calcMotorSpeed(20);
	JAFD::SensorFusion::sensorFiltering(20);
	JAFD::SmoothDriving::updateSpeeds(20);
	JAFD::MotorControl::speedPID(20);

	if (i % 2 == 0)
	{
		// 10Hz:

		if (i % 4 == 0)
		{
			// 5Hz:

			if (i % 20 == 0)
			{
				i = 0;

				// 1Hz:

			}
		}
	}
}