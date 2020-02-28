/*
 Name:		MainProgram.ino
 Created:	07.08.2019 14:16:07
 Author:	B.Robots
*/

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

// RobotLibrary
#include "JAFD/JAFD.h"
#include "JAFD/header/MotorControl.h"
#include "JAFD/header/Bno055.h"
#include "JAFD/header/Interrupts.h"
#include "JAFD/header/Vector.h"
#include "JAFD/header/AllDatatypes.h"

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#include "JAFD/header/SpiNVSRAM.h"

using namespace JAFD::SpiNVSRAM;
using namespace JAFD::MotorControl;
using namespace JAFD;
using namespace JAFD::Bno055;

float x = 0.0f;

// The setup function runs once when you press reset or power the board

void setup() {
	// For testing
	Serial.begin(115200);

	JAFD::robotSetup();

	uint8_t page[256];
	uint8_t readp[256];

	page[0] = 42;
	page[1] = 69;
	page[2] = 33; //schauen ob die Werte invertiert das ausgegebene ergeben!!!

	writePage(3,page);

	Serial.println("Hello");
	
	readPage(3,readp);

	Serial.println(readp[0]);
	Serial.println(readp[1]);
	Serial.println(readp[2]);



}

// The loop function runs over and over again until power down or reset
void loop() {

	JAFD::robotLoop();

	

}



