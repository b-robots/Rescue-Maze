#include <Arduino.h>

// External Libs
#include <SPI.h>
#include <VL53L0X.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_AMG88xx.h>
#include <TPA81.h>

// RobotLibrary
#include "SIAL/SIAL.h"

// The setup function runs once when you press reset or power the board
void setup()
{
	// DEBUG
	Serial.begin(115200);

	SIAL::robotSetup();
}

// The loop function runs over and over again until power down or reset
void loop()
{
	SIAL::robotLoop();
}