/*
 Name:		RobotLibrary.cpp
 Created:	07.08.2019 13:20:59
 Author:	B.Robots
*/

#include "../JAFD.h"
#include "MazeMapping_private.h"
#include "Dispenser_private.h"
#include "MotorControl_private.h"
#include "SpiEeprom_private.h"

#include <SPI.h>

namespace JAFD
{
	// Just for testing...
	ReturnCode robotSetup()
	{
		// Setup the SPI-Bus
		SPI.begin();
		SPI.beginTransaction(SPISettings(10e+6, MSBFIRST, SPI_MODE0));

		PMC->PMC_PCER0 = 1 << ID_PIOA | 1 << ID_PIOB | 1 << ID_PIOC | 1 << ID_PIOD;

		// Setup TC3 for an interrupt every ms -> 1kHz (MCK / 32 / 2625)
		// ISR Priority = 5
		PMC->PMC_PCER0 = 1 << ID_TC3;

		TC1->TC_CHANNEL[0].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK3 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC;
		TC1->TC_CHANNEL[0].TC_RC = 2625;

		TC1->TC_CHANNEL[0].TC_CCR = TC_CCR_SWTRG;

		TC1->TC_CHANNEL[0].TC_IER = TC_IER_COVFS;
		TC1->TC_CHANNEL[0].TC_IDR = ~TC_IDR_COVFS;

		NVIC_EnableIRQ(TC3_IRQn);
		NVIC_SetPriority(TC3_IRQn, 6);

		// Setup TC4 for an interrupt every 10ms -> 100Hz (MCK / 32 / 26250)
		// ISR Priority = 6
		PMC->PMC_PCER0 = 1 << ID_TC4;

		TC1->TC_CHANNEL[1].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK3 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC;
		TC1->TC_CHANNEL[1].TC_RC = 26250;

		TC1->TC_CHANNEL[1].TC_CCR = TC_CCR_SWTRG;

		TC1->TC_CHANNEL[1].TC_IER = TC_IER_COVFS;
		TC1->TC_CHANNEL[1].TC_IDR = ~TC_IDR_COVFS;

		NVIC_EnableIRQ(TC4_IRQn);
		NVIC_SetPriority(TC4_IRQn, 6);

		// Setup of MazeMapper
		if (MazeMapping::mazeMapperSetup() != ReturnCode::ok)
		{
			return ReturnCode::fatalError;
		}

		// Setup of Dispenser
		if (Dispenser::dispenserSetup() != ReturnCode::ok)
		{
			return ReturnCode::fatalError;
		}

		// Setup of Motor Control
		if (MotorControl::motorControlSetup() != ReturnCode::ok)
		{
			return ReturnCode::fatalError;
		}

		// Setup of SPI EEPROM
		if (SpiEeprom::spiEepromSetup() != ReturnCode::ok)
		{
			return ReturnCode::fatalError;
		}

		return ReturnCode::ok;
	}

	ReturnCode robotLoop()
	{
		return ReturnCode::ok;
	}
}

