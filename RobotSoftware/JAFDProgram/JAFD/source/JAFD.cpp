/*
 Name:		RobotLibrary.cpp
 Created:	07.08.2019 13:20:59
 Author:	B.Robots
*/

#include "../JAFD.h"
#include "../header/Bno055.h"
#include "../header/Dispenser.h"
#include "../header/MazeMapping.h"
#include "../header/MotorControl.h"
#include "../header/SensorFusion.h"
#include "../header/SpiNVSRAM.h"
#include "../header/DistanceSensors.h"
#include "../header/AllDatatypes.h"

#include <SPI.h>

namespace JAFD
{
	// Just for testing...
	void robotSetup()
	{
		// Setup the SPI-Bus
		SPI.begin();
		SPI.beginTransaction(SPISettings(10e+6, MSBFIRST, SPI_MODE0));

		PMC->PMC_PCER0 = 1 << ID_PIOA | 1 << ID_PIOB | 1 << ID_PIOC | 1 << ID_PIOD;
		PMC->PMC_PCER1 = PMC_PCER1_PID36;

		// Setup PWM clock
		PWM->PWM_CLK = PWM_CLK_PREA(0) | PWM_CLK_DIVA(1) | PWM_CLK_PREB(0b111) | PWM_CLK_DIVB(1);

		// Setup TC6 for an interrupt every ms -> 1kHz (MCK / 32 / 2625)
		PMC->PMC_PCER0 = 1 << ID_TC6;

		TC2->TC_CHANNEL[0].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK3 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC;
		TC2->TC_CHANNEL[0].TC_RC = 2625;

		TC2->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
		TC2->TC_CHANNEL[0].TC_IDR = ~TC_IER_CPCS;

		NVIC_EnableIRQ(TC6_IRQn);

		TC2->TC_CHANNEL[0].TC_CCR = TC_CCR_SWTRG | TC_CCR_CLKEN;

		// Setup TC7 for an interrupt every 10ms -> 100Hz (MCK / 32 / 26250)
		PMC->PMC_PCER0 = 1 << ID_TC7;

		TC2->TC_CHANNEL[1].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK3 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC;
		TC2->TC_CHANNEL[1].TC_RC = 26250;

		TC2->TC_CHANNEL[1].TC_IER = TC_IER_CPCS;
		TC2->TC_CHANNEL[1].TC_IDR = ~TC_IER_CPCS;

		NVIC_EnableIRQ(TC7_IRQn);

		TC2->TC_CHANNEL[1].TC_CCR = TC_CCR_SWTRG | TC_CCR_CLKEN;

		// Setup of MazeMapper
		if (MazeMapping::setup() != ReturnCode::ok)
		{

		}

		// Setup of Dispenser
		if (Dispenser::setup() != ReturnCode::ok)
		{
			Serial.println("Error!!");
		}

		// Setup of Motor Control
		if (MotorControl::setup() != ReturnCode::ok)
		{

		}

		// Setup of SPI NVSRAM
		if (SpiNVSRAM::setup() != ReturnCode::ok)
		{

		}

		// Setup of Distance Sensors
		if (DistanceSensors::setup() != ReturnCode::ok)
		{

		}

		return;
	}

	void robotLoop()
	{
		return;
	}
}

