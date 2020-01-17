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



using namespace JAFD::MotorControl;
using namespace JAFD;
using namespace JAFD::Bno055;

float x = 0.0f;

// The setup function runs once when you press reset or power the board

void setup() {
	// For testing
	Serial.begin(115200);

	JAFD::robotSetup();

	JAFD::Bno055::calibration();
}

// The loop function runs over and over again until power down or reset
void loop() {

	JAFD::robotLoop();

	JAFD::Bno055::update_sensorreadings();

	delay(500);

	auto absolute_orientation = JAFD::Bno055::get_absolute_orientation();

	auto linear_accel = JAFD::Bno055::get_linear_acceleration();

	auto  ang_velo = JAFD::Bno055::get_angular_velocity();

	auto grav_vec = JAFD::Bno055::get_gravity_vector();


	/*Serial.println("absolute_orientation.x");
	Serial.println(absolute_orientation.x);
	Serial.println("absolute_orientation.y");
	Serial.println(absolute_orientation.y);
	Serial.println("absolute_orientation.z");
	Serial.println(absolute_orientation.z);*/


	/*Serial.println("linear_Acceleration");
	Serial.println(linear_accel.x);
	Serial.println("linear_Acceleration.y");
	Serial.println(linear_accel.y);
	Serial.println("linear_Acceleration.z");
	Serial.println(linear_accel.z);*/

	/*Serial.println("ang_velocity.x");
	Serial.println(ang_velo.x);
	Serial.println("ang_velocity.y");
	Serial.println(ang_velo.y);
	Serial.println("ang_velocity.z");
	Serial.println(ang_velo.z);*/

	Serial.println("grav_vec.x");
	Serial.println(grav_vec.x);
	Serial.println("grav_vec.y");
	Serial.println(grav_vec.y);
	Serial.println("grav_vec.z");
	Serial.println(grav_vec.z);

}



