#include <Arduino.h>

#include "../SIAL.h"
#include "../header/DuePinMapping.h"
#include "../header/AllDatatypes.h"
#include "../header/Math.h"
#include "../header/Vector.h"
#include "../header/MotorControl.h"

#include <SPI.h>
#include <Wire.h>

namespace SIAL {
	void robotSetup() {
		// Setup the SPI-Bus
		SPI.begin();
		SPI.beginTransaction(SPISettings(10e+6, MSBFIRST, SPI_MODE0));

		// Enable clock for all pins for interrupts
		PMC->PMC_PCER0 = 1 << ID_PIOA | 1 << ID_PIOB | 1 << ID_PIOC | 1 << ID_PIOD;

		// Setup PWM - CLK A for motors - CLK B unused
		PMC->PMC_PCER1 = PMC_PCER1_PID36;
		PWM->PWM_CLK = PWM_CLK_PREB(0b111) | PWM_CLK_DIVB(1) | PWM_CLK_PREA(0) | PWM_CLK_DIVA(1);

		// Enable all Timer Counter
		PMC->PMC_PCER0 = PMC_PCER0_PID27 | PMC_PCER0_PID28 | PMC_PCER0_PID29 | PMC_PCER0_PID30 | PMC_PCER0_PID31;
		PMC->PMC_PCER1 = PMC_PCER1_PID32 | PMC_PCER1_PID33 | PMC_PCER1_PID34 | PMC_PCER1_PID35;

		// Initialise random number generator
		randomSeed(69420);

		//// Setup interrupts for all ports 
		//NVIC_EnableIRQ(PIOA_IRQn);
		//NVIC_SetPriority(PIOA_IRQn, 0);

		//NVIC_EnableIRQ(PIOB_IRQn);
		//NVIC_SetPriority(PIOB_IRQn, 0);

		//NVIC_EnableIRQ(PIOC_IRQn);
		//NVIC_SetPriority(PIOC_IRQn, 0);

		//NVIC_EnableIRQ(PIOD_IRQn);
		//NVIC_SetPriority(PIOD_IRQn, 0);

		// Setup of Motor Control
		if (MotorControl::setup() != ReturnCode::ok)
		{
			Serial.println("Error MotorControl!");
		}
	}

	void robotLoop() {
		MotorControl::calcMotorSpeed();
		MotorControl::speedPID();

		int soll = 30;

		//int i = (millis() / 4000) % 4;

		//if (i == 0) {
		//	soll = 40;
		//}
		//else if (i == 1) {
		//	soll = 15;
		//}
		//else if (i == 2) {
		//	soll = -15;
		//}
		//else {
		//	soll = -40;
		//}

		int ist = MotorControl::getFloatSpeeds().right;

		MotorControl::setSpeeds(WheelSpeeds{ soll, soll });

		delay(20);
	}
}