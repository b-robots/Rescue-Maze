/*
 Name:		RobotLibrary.cpp
 Created:	07.08.2019 13:20:59
 Author:	B.Robots
*/

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

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
#include "../header/SmoothDriving.h"
#include "../header/TCS34725.h"
#include "../header/TCA9548A.h"
#include "../header/HeatSensor.h"
#include "../header/SmallThings.h"
#include "../header/CamRec.h"
#include "../header/Math.h"
#include "../header/OLed.h"

#include <SPI.h>
#include <Wire.h>

namespace JAFD
{
	// Just for testing...
	void robotSetup()
	{
		// Setup the SPI-Bus
		SPI.begin();
		SPI.beginTransaction(SPISettings(10e+6, MSBFIRST, SPI_MODE0));

		// Setup PWM - CLK A for motors - CLK B unused
		PMC->PMC_PCER0 = 1 << ID_PIOA | 1 << ID_PIOB | 1 << ID_PIOC | 1 << ID_PIOD;
		PMC->PMC_PCER1 = PMC_PCER1_PID36;
		PWM->PWM_CLK = PWM_CLK_PREB(0b111) | PWM_CLK_DIVB(1) | PWM_CLK_PREA(0) | PWM_CLK_DIVA(1);

		// Nice
		randomSeed(69420);

		// Setup interrupts for all ports 
		NVIC_EnableIRQ(PIOA_IRQn);
		NVIC_SetPriority(PIOA_IRQn, 0);

		NVIC_EnableIRQ(PIOB_IRQn);
		NVIC_SetPriority(PIOB_IRQn, 0);

		NVIC_EnableIRQ(PIOC_IRQn);
		NVIC_SetPriority(PIOC_IRQn, 0);

		NVIC_EnableIRQ(PIOD_IRQn);
		NVIC_SetPriority(PIOD_IRQn, 0);

		// Setup of power LEDs
		if (PowerLEDs::setup() != ReturnCode::ok)
		{
			Serial.println("Error power LEDs");
		}

		// Setup I2C-Bus-Power
		if (I2CBus::setup() != ReturnCode::ok)
		{
			Serial.println("Error I2C bus power");
		}

		// Setup of SPI NVSRAM
		if (SpiNVSRAM::setup() != ReturnCode::ok)
		{
			Serial.println("Error SPI NVSRAM");
		}

		// Setup of MazeMapper
		if (MazeMapping::setup() != ReturnCode::ok)
		{
			Serial.println("Error Maze Mapping");
		}

		// Setup of Motor Control
		if (MotorControl::setup() != ReturnCode::ok)
		{
			Serial.println("Error Motor Control");
		}

		// Setup of I2C Multiplexer
		if (I2CMultiplexer::setup() != ReturnCode::ok)
		{
			Serial.println("Error I2C Multiplexer");

			if (I2CBus::resetBus() != ReturnCode::ok)
			{
				Serial.println("Error resetting I2C Bus");
			}
		}

		// Setup of Dispenser
		if (Dispenser::setup() != ReturnCode::ok)
		{
			Serial.println("Error Dispenser");
		}
		
		// Setup of Distance Sensors
		if (DistanceSensors::setup() != ReturnCode::ok)
		{
			Serial.println("Error Distance Sensor");

			if (I2CBus::resetBus() != ReturnCode::ok)
			{
				Serial.println("Error resetting I2C Bus");
			}
		}
		
		// Setup of Bno055
		if (Bno055::setup() != ReturnCode::ok)
		{
			Serial.println("Error BNO055");

			if (I2CBus::resetBus() != ReturnCode::ok)
			{
				Serial.println("Error resetting I2C Bus");
			}
		}
		
		// Setup of color sensor
		if (ColorSensor::setup() != ReturnCode::ok)
		{
			Serial.println("Error Color Sensor");

			if (I2CBus::resetBus() != ReturnCode::ok)
			{
				Serial.println("Error resetting I2C Bus");
			}
		}
		
		// Setup of heat sensors
		if (HeatSensor::setup() != ReturnCode::ok)
		{
			Serial.println("Error Heat Sensor");

			if (I2CBus::resetBus() != ReturnCode::ok)
			{
				Serial.println("Error resetting I2C Bus");
			}
		}

		// Setup communication with RasPI for camera recognition
		if (CamRec::setup() != ReturnCode::ok)
		{
			Serial.println("Error CamRec!");
		}

		// Clear all interrupts once
		{
			volatile auto temp = PIOA->PIO_ISR;
			temp = PIOB->PIO_ISR;
			temp = PIOC->PIO_ISR;
			temp = PIOD->PIO_ISR;
		}

		////Setup TC3 for an interrupt every ms -> 1kHz (MCK / 32 / 2625)
		//PMC->PMC_PCER0 = 1 << ID_TC3;

		//TC1->TC_CHANNEL[0].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK3 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC;
		//TC1->TC_CHANNEL[0].TC_RC = 2625;

		//TC1->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
		//TC1->TC_CHANNEL[0].TC_IDR = ~TC_IER_CPCS;

		//NVIC_EnableIRQ(TC3_IRQn);
		//NVIC_SetPriority(TC3_IRQn, 1);
		//TC1->TC_CHANNEL[0].TC_CCR = TC_CCR_SWTRG | TC_CCR_CLKEN;

		////Setup TC4 for an interrupt every 10ms -> 100Hz (MCK / 32 / 26250)
		//PMC->PMC_PCER0 = 1 << ID_TC4;

		//TC1->TC_CHANNEL[1].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK3 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC;
		//TC1->TC_CHANNEL[1].TC_RC = 26250;

		//TC1->TC_CHANNEL[1].TC_IER = TC_IER_CPCS;
		//TC1->TC_CHANNEL[1].TC_IDR = ~TC_IER_CPCS;

		//NVIC_EnableIRQ(TC4_IRQn);
		//NVIC_SetPriority(TC4_IRQn, 1);

		//TC1->TC_CHANNEL[1].TC_CCR = TC_CCR_SWTRG | TC_CCR_CLKEN;

		// Setup TC4 for an interrupt every ~50ms -> 19.9997Hz (MCK / 128 / 32813)
		PMC->PMC_PCER1 = PMC_PCER1_PID32;

		TC1->TC_CHANNEL[2].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK4 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC;
		TC1->TC_CHANNEL[2].TC_RC = 32813;

		TC1->TC_CHANNEL[2].TC_IER = TC_IER_CPCS;
		TC1->TC_CHANNEL[2].TC_IDR = ~TC_IER_CPCS;

		NVIC_EnableIRQ(TC5_IRQn);
		NVIC_SetPriority(TC5_IRQn, 1);
		TC1->TC_CHANNEL[2].TC_CCR = TC_CCR_SWTRG | TC_CCR_CLKEN;

		delay(500);

		//Set start for 9DOF
		Bno055::tare();

		delay(1000);
		
		return;
	}

	void robotLoop()
	{
		static float fps = 0.0f;

		auto time = millis();
		
		using namespace SmoothDriving;

		//static const TaskArray tasks[] = {
		//	TaskArray(Accelerate(30, 15), DriveStraight(30), Accelerate(0, 15), Stop()),
		//	TaskArray(Rotate(-2, -90), Stop()),
		//	TaskArray(Accelerate(30, 15), DriveStraight(30), Accelerate(0, 15), Stop())
		//};
		//
		//const static uint16_t numTasks = sizeof(tasks) / sizeof(*tasks);

		//static uint16_t i = 0;

		//if (SmoothDriving::isTaskFinished() && i < numTasks)
		//{
		//	if (SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(tasks[i]) != ReturnCode::ok)
		//	{
		//		Serial.println(i);
		//	}

		//	i++;
		//}

		SensorFusion::updateSensors();
		SensorFusion::untimedFusion();
		//RobotLogic::loop();
		
		auto fusedData = SensorFusion::getFusedData();
		fusedData.robotState.globalHeading;

		auto freeRam = MemWatcher::getFreeRam();

		if (fps < 0.01f) fps = 1000.0f / (millis() - time);
		else fps = fps * 0.4f + 600.0f / (millis() - time);

		return;
	}
}