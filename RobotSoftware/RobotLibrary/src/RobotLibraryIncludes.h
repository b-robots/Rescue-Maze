/*
All external includes needed for every library Element
*/

#pragma once


#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include <Servo.h>
#include <cstdint>
#include <SpiRAM.h>
#include <functional>