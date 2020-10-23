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
#include "../header/SmoothDriving.h"
#include "../header/TCS34725.h"

#include <SPI.h>

namespace JAFD
{
	// Just for testing...
	void robotSetup()
	{
		// Setup I2C
		Wire.begin();
		Wire1.begin();

		// Setup the SPI-Bus
		SPI.begin();
		SPI.beginTransaction(SPISettings(10e+6, MSBFIRST, SPI_MODE0));

		// Setup PWM
		PMC->PMC_PCER0 = 1 << ID_PIOA | 1 << ID_PIOB | 1 << ID_PIOC | 1 << ID_PIOD;
		PMC->PMC_PCER1 = PMC_PCER1_PID36;
		PWM->PWM_CLK = PWM_CLK_PREB(0b111) | PWM_CLK_DIVB(1) | PWM_CLK_PREA(0) | PWM_CLK_DIVA(1);

		// Nice
		randomSeed(69420);

		// Setup interrupts for all ports 
		NVIC_EnableIRQ(PIOA_IRQn);
		NVIC_SetPriority(PIOA_IRQn, 1);

		NVIC_EnableIRQ(PIOB_IRQn);
		NVIC_SetPriority(PIOB_IRQn, 1);

		NVIC_EnableIRQ(PIOC_IRQn);
		NVIC_SetPriority(PIOC_IRQn, 1);

		NVIC_EnableIRQ(PIOD_IRQn);
		NVIC_SetPriority(PIOD_IRQn, 1);

		// Setup of MazeMapper
		if (MazeMapping::setup() != ReturnCode::ok)
		{
			Serial.println("Error Maze Mapping");
		}

		// Setup of Dispenser
		if (Dispenser::setup() != ReturnCode::ok)
		{
			Serial.println("Error Dispenser");
		}

		// Setup of Motor Control
		if (MotorControl::setup() != ReturnCode::ok)
		{
			Serial.println("Error Motor Control");
		}

		// Setup of SPI NVSRAM
		if (SpiNVSRAM::setup() != ReturnCode::ok)
		{
			Serial.println("Error SPI NVSRAM");
		}
		
		// Setup of Distance Sensors
		if (DistanceSensors::setup() != ReturnCode::ok)
		{
			Serial.println("Error Distance Sensors");
		}
		
		// Setup of Bno055
		if (Bno055::init() != ReturnCode::ok)
		{
			Serial.println("Error Bno055");
		}
		
		// Setup of color sensor
		if (ColorSensor::setup() != ReturnCode::ok)
		{
			Serial.println("Error Color-Sensor");
		}
		
		//Set start for 9DOF
		Bno055::setStartPoint();

		// Clear all interrupts once
		volatile auto temp = PIOA->PIO_ISR;
		temp = PIOB->PIO_ISR;
		temp = PIOC->PIO_ISR;
		temp = PIOD->PIO_ISR;

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

		return;
	}

	void robotLoop()
	{
		static float fps = 0;

		auto time = millis();

		constexpr uint16_t numTasks = 9;
		
		static const SmoothDriving::TaskArray tasks[numTasks] = {SmoothDriving::TaskArray(SmoothDriving::Stop(), SmoothDriving::Accelerate(30, 15.0f), SmoothDriving::Accelerate(0, 15.0f)),
			SmoothDriving::TaskArray(SmoothDriving::Stop(), SmoothDriving::Rotate(1.0f, 90.0f), SmoothDriving::Accelerate(30, 15.0f), SmoothDriving::Accelerate(0, 15.0f)),
			SmoothDriving::TaskArray(SmoothDriving::Stop(), SmoothDriving::Rotate(-1.0f, -90.0f), SmoothDriving::Accelerate(30, 15.0f), SmoothDriving::Accelerate(0, 15.0f)),
			SmoothDriving::TaskArray(SmoothDriving::Stop(), SmoothDriving::Accelerate(30, 15.0f), SmoothDriving::Accelerate(0, 15.0f)),
			SmoothDriving::TaskArray(SmoothDriving::Stop(), SmoothDriving::Rotate(-1.0f, -90.0f), SmoothDriving::Accelerate(30, 15.0f), SmoothDriving::Accelerate(0, 15.0f)),
			SmoothDriving::TaskArray(SmoothDriving::Stop(), SmoothDriving::Accelerate(50, 130.0f), SmoothDriving::Accelerate(0, 130.0f)),
			SmoothDriving::TaskArray(SmoothDriving::Stop(), SmoothDriving::Rotate(-1.0f, -90.0f), SmoothDriving::Accelerate(30, 15.0f), SmoothDriving::Accelerate(0, 15.0f)),
			SmoothDriving::TaskArray(SmoothDriving::Stop(), SmoothDriving::Rotate(-1.0f, -90.0f), SmoothDriving::Accelerate(30, 15.0f), SmoothDriving::Accelerate(0, 15.0f)),
			SmoothDriving::TaskArray(SmoothDriving::Stop(), SmoothDriving::Accelerate(30, 15.0f), SmoothDriving::Accelerate(0, 15.0f)),
		};
		
		static const bool dispL[numTasks] = { false, false, false, false, false, false, false, false, false };
		static const bool dispR[numTasks] = { false, false, false, false, false, false, false, false, true };
		
		static uint16_t i = 0;

		if (SmoothDriving::isTaskFinished())
		{
			//SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::Stop());

			while (!SmoothDriving::isTaskFinished());

			//if (dispR[i] == true) Dispenser::dispenseRight(1);
			//if (dispL[i] == true) Dispenser::dispenseLeft(1);

			//SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(tasks[i]);

			i++;
			i %= numTasks;
		}

		/*
		if (ColorSensor::dataIsReady())
		{
			uint16_t colorTemp = 0;
			uint16_t lux = 0;
			ColorSensor::getData(&colorTemp, &lux);
			Serial.println(lux);
		}
		*/

		SensorFusion::updateSensors();
		SensorFusion::untimedFusion();
		//RobotLogic::loop();

		if (fps < 0.01f) fps = 1000.0f / (millis() - time);
		else fps = fps * 0.7f + 300.0f / (millis() - time);

		Serial.print("----------\nFPS: ");
		Serial.println(fps);

		return;
	}
}