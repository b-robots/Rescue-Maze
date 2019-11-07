/*
 Name:		JAFD.h
 Created:	07.08.2019 13:20:59
 Author:	B.Robots
 Version:	1.1
*/

#pragma once

#if !defined(__SAM3X8E__)
#error This Library can only be used with the Arduino Due / ATSam3x8e
#endif

#define ROBOT_LIB_VERSION "1.3" 

// All public header files of Library
#include "implementation/Dispenser_public.h"
#include "implementation/ReturnCode_public.h"
#include "implementation/MazeMapping_public.h"
#include "implementation/MotorControl_public.h"
#include "implementation/SpiEeprom_public.h"

// Namespace for robot (including sensors, maze solving algorithm, and so on...)
// JAFD = Just Ask For Direction
namespace JAFD
{
	typedef struct
	{
		Dispenser::DispenserSettings dispenserSet;
		MazeMapping::MazeMapperSet mazeMapperSet;
		MotorControl::MotorControlSettings motorControlSet;
		SpiEeprom::SpiEepromSet spiEepromSet;
	} RobotSettings;

	// Setup & Loop for the Robot
	ReturnCode robotSetup(RobotSettings robotSettings);
	ReturnCode robotLoop();
};
