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

		randomSeed(69420);

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

		//Set start for 9DOF
		Bno055::setStartPoint();

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
		
		SensorFusion::updateDistSensor();
		SensorFusion::untimedFusion();

		//auto a = SensorFusion::getFusedData().distances.frontLong;
		//auto b = SensorFusion::getFusedData().distances.frontLeft;
		//auto c = SensorFusion::getFusedData().distances.frontRight;
		//auto d = SensorFusion::getFusedData().distances.leftFront;
		//auto e = SensorFusion::getFusedData().distances.leftBack;
		//auto f = SensorFusion::getFusedData().distances.rightFront;
		//auto g = SensorFusion::getFusedData().distances.rightBack; 

		//RobotLogic::loop();

		if (SmoothDriving::isTaskFinished())
		{
			//SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::Stop());

			//while (!SmoothDriving::isTaskFinished());

			//if (dispR[i] == true) Dispenser::dispenseRight(1);
			//if (dispL[i] == true) Dispenser::dispenseLeft(1);

			//SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(tasks[i]);

			//Serial.println(i);

			//i++;
			//i %= numTasks;
		}

		return;
	}
}