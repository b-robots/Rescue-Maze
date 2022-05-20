#pragma once

#if !defined(__SAM3X8E__)
#error This Library can only be used with the Arduino Due / ATSam3x8e
#endif

namespace SIAL
{
	// Setup & Loop for the Robot
	void robotSetup();
	void robotLoop();
};
