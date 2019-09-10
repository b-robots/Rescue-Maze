/*
All external includes needed for most of the library elements
*/

#pragma once


#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include <Servo.h>
#include <SpiRAM.h>