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
#include "../header/RobotLogic.h"

#include <SPI.h>

namespace JAFD
{
	// Just for testing...
	void robotSetup()
	{
		// Setup I2C
		Wire.begin();

		// Setup the SPI-Bus
		SPI.begin();
		SPI.beginTransaction(SPISettings(10e+6, MSBFIRST, SPI_MODE0));

		PMC->PMC_PCER0 = 1 << ID_PIOA | 1 << ID_PIOB | 1 << ID_PIOC | 1 << ID_PIOD;

		// Setup TC3 for an interrupt every ms -> 1kHz (MCK / 32 / 2625)
		PMC->PMC_PCER0 = 1 << ID_TC3;

		TC1->TC_CHANNEL[0].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK3 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC;
		TC1->TC_CHANNEL[0].TC_RC = 2625;

		TC1->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
		TC1->TC_CHANNEL[0].TC_IDR = ~TC_IER_CPCS;

		NVIC_EnableIRQ(TC3_IRQn);

		TC1->TC_CHANNEL[0].TC_CCR = TC_CCR_SWTRG | TC_CCR_CLKEN;

		// Setup TC4 for an interrupt every 10ms -> 100Hz (MCK / 32 / 26250)
		PMC->PMC_PCER0 = 1 << ID_TC4;

		TC1->TC_CHANNEL[1].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK3 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC;
		TC1->TC_CHANNEL[1].TC_RC = 26250;

		TC1->TC_CHANNEL[1].TC_IER = TC_IER_CPCS;
		TC1->TC_CHANNEL[1].TC_IDR = ~TC_IER_CPCS;

		NVIC_EnableIRQ(TC4_IRQn);

		TC1->TC_CHANNEL[1].TC_CCR = TC_CCR_SWTRG | TC_CCR_CLKEN;

		// INIT Distance Sensors

		// Setup of MazeMapper
		if (MazeMapping::setup() != ReturnCode::ok)
		{

		}

		// Setup of Dispenser
		if (Dispenser::setup() != ReturnCode::ok)
		{

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
		SensorFusion::untimedSensorUpdate();
		
		RobotLogic::loop();

		return;
	}
}

