#include <Arduino.h>

#include "../SIAL.h"
#include "../SIALSettings.h"
#include "../header/DuePinMapping.h"
#include "../header/AllDatatypes.h"
#include "../header/Math.h"
#include "../header/Vector.h"
#include "../header/MotorControl.h"
#include "../header/HeatSensor.h"
#include "../header/I2CMultiplexer.h"
#include "../header/SmallThings.h"
#include "../header/DistanceSensors.h"
#include "../header/Gyro.h"
#include "../header/SpiNVSRAM.h"

#include <SPI.h>
#include <Wire.h>

namespace SIAL {
	void robotSetup() {
		// Setup the SPI-Bus
		SPI.begin();
		SPI.beginTransaction(SPISettings(10e+6, MSBFIRST, SPI_MODE0));

		// Setup I2C-Bus
		Wire.begin();
		Wire1.begin();

		// Start clock for PIO (debouncing)
		PMC->PMC_PCER0 = 1 << ID_PIOA | 1 << ID_PIOB | 1 << ID_PIOC | 1 << ID_PIOD;
		 
		// Setup PWM - CLK A for motors - CLK B unused
		PMC->PMC_PCER1 = PMC_PCER1_PID36;
		PWM->PWM_CLK = PWM_CLK_PREB(0b111) | PWM_CLK_DIVB(1) | PWM_CLK_PREA(0) | PWM_CLK_DIVA(1);

		// Enable all Timer Counter
		PMC->PMC_PCER0 = PMC_PCER0_PID27 | PMC_PCER0_PID28 | PMC_PCER0_PID29 | PMC_PCER0_PID30 | PMC_PCER0_PID31;
		PMC->PMC_PCER1 = PMC_PCER1_PID32 | PMC_PCER1_PID33 | PMC_PCER1_PID34 | PMC_PCER1_PID35;

		// Initialise random number generator
		randomSeed(69420);

		Switch::setup();

		// Setup of power LEDs
		if (PowerLEDs::setup() != ReturnCode::ok)
		{
			Serial.println("Error power LEDs");
		}

		// Setup of SPI NVSRAM
		if (SpiNVSRAM::setup() != ReturnCode::ok)
		{
			Serial.println("Error SPI NVSRAM");
		}

		// Setup of I2C Multiplexer
		if (I2CMultiplexer::setup() != ReturnCode::ok)
		{
			Serial.println("Error I2CMultiplexer");
		}

		// Setup of Distance Sensors
		if (DistanceSensors::setup() != ReturnCode::ok)
		{
			Serial.println("Error DistanceSensors");
		}

		// TESTING
		DistanceSensors::resetHardCodedCalib();

		// Setup of Gyro
		if (Gyro::setup() != ReturnCode::ok)
		{
			Serial.println("Error BNO055");
		}

		// Setup of MotorControl
		if (MotorControl::setup() != ReturnCode::ok)
		{
			Serial.println("Error MotorControl!");
		}

		// Setup of HeatSensor
		if (HeatSensor::setup() != ReturnCode::ok)
		{
			Serial.println("Error HeatSensor!");
		}

		Serial.println("Finished, setup!");

		// TESTING
		// Gyro::calibrate();

		//Serial.println("Wait for initial BNO055 calibration...");

		//uint8_t bno_sys = 0;
		//do {
		//	Gyro::updateValues();
		//	bno_sys = Gyro::getOverallCalibStatus();
		//	delay(100);
		//} while (bno_sys < 3);

		//Serial.println("BNO055 ready!");

		//PowerLEDs::setBrightness(SIALSettings::PowerLEDs::defaultPower);

		//while (!Switch::getState());

		Serial.println("Start!");

		//Set start for 9DOF
		Gyro::tare();
	}

	void robotLoop() {
		static uint32_t t = millis();

		// Chi: ICR factor coefficient
		constexpr float chi = 1.2f; //1.18f;
		float angle = (MotorControl::getDistance(Motor::right) - MotorControl::getDistance(Motor::left)) / (2.0f * SIALSettings::Mechanics::wheelDistToMiddle * chi);
		float dist = (MotorControl::getDistance(Motor::right) + MotorControl::getDistance(Motor::left)) / 2.0f;

		//MotorControl::setSpeeds(WheelSpeeds{ 35, -30});
		//static bool finished = false;

		////if (dist >= 50.0f) {
		////	finished = true;
		////}

		//if (angle <= -360.0f * DEG_TO_RAD) {
		//	finished = true;
		//}

		//if (finished) {
		//	MotorControl::setSpeeds(WheelSpeeds{ 0, 0 });
		//	Serial.println(angle);
		//}

		// TESTING
		HeatSensor::detectVictim(HeatSensorSide::left);
		HeatSensor::detectVictim(HeatSensorSide::right);
		DistanceSensors::updateDistSensors();
		Gyro::updateValues();

		// Serial.println(getGlobalHeading(Gyro::getForwardVec()));
		Serial.println(1000.0f / (millis() - t));
		t = millis();

		while (Serial.available()) { Serial.read(); }

		if (!I2CMultiplexer::checkI2C()) {
			Serial.println("I2C Multiplexer Error");
			
			while (!Serial.available());
			delay(10);
			while (Serial.available()) { Serial.read(); }

			Wire.end();

			pinMode(SCL, OUTPUT);
			pinMode(SDA, OUTPUT);

			for (int i = 0; i < 10; i++) {
				digitalWrite(SCL, LOW);
				delayMicroseconds(2);
				digitalWrite(SDA, LOW);
				delayMicroseconds(2);
				digitalWrite(SCL, HIGH);
				delayMicroseconds(4);
			}
			digitalWrite(SDA, HIGH);

			pinMode(SCL, INPUT);
			pinMode(SDA, INPUT);

			delay(10);
			Wire.begin();

			Wire.beginTransmission(0x1);
			Wire.endTransmission(true);

			while (!Serial.available());
			delay(10);
			while (Serial.available()) { Serial.read(); }
		}

		// delay(50);
	}
}