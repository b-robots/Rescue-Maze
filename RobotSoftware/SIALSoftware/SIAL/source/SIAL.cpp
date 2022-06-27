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
#include "../header/ColorSensor.h"
#include "../header/MazeMapping.h"
#include "../header/SensorFusion.h"
#include "../header/SmoothDriving.h"
#include "../header/RobotLogic.h"

#include <SPI.h>
#include <Wire.h>

namespace SIAL {
	void robotSetup() {
		// Setup the SPI-Bus
		SPI.begin();
		SPI.beginTransaction(SPISettings(10e+6, MSBFIRST, SPI_MODE0));

		// Setup I2C-Bus
		Wire.begin();
		Wire.setClock(400000);
		Wire1.begin();
		Wire1.setClock(400000);

		// Start clock for PIO (debouncing)
		PMC->PMC_PCER0 = 1 << ID_PIOA | 1 << ID_PIOB | 1 << ID_PIOC | 1 << ID_PIOD;

		// Setup PWM - CLK A for motors - CLK B unused
		PMC->PMC_PCER1 = PMC_PCER1_PID36;
		PWM->PWM_CLK = PWM_CLK_PREB(0b111) | PWM_CLK_DIVB(1) | PWM_CLK_PREA(0) | PWM_CLK_DIVA(1);

		// Enable all Timer Counter
		PMC->PMC_PCER0 = PMC_PCER0_PID27 | PMC_PCER0_PID28 | PMC_PCER0_PID29 | PMC_PCER0_PID30 | PMC_PCER0_PID31;
		PMC->PMC_PCER1 = PMC_PCER1_PID32 | PMC_PCER1_PID33 | PMC_PCER1_PID34 | PMC_PCER1_PID35;

		// Setup interrupts for all ports 
		NVIC_EnableIRQ(PIOA_IRQn);
		NVIC_EnableIRQ(PIOB_IRQn);
		NVIC_EnableIRQ(PIOC_IRQn);
		NVIC_EnableIRQ(PIOD_IRQn);

		{
			volatile auto temp = PIOA->PIO_ISR;
			temp = PIOB->PIO_ISR;
			temp = PIOC->PIO_ISR;
			temp = PIOD->PIO_ISR;
		}

		// Initialise random number generator
		randomSeed(69420);

		SmoothDriving::startNewTask(new SmoothDriving::Stop(), true);

		bool error = false;

		Switch::setup();
		PowerLEDs::setup();
		Bumper::setup();

		// Setup of SPI NVSRAM
		if (SpiNVSRAM::setup() != ReturnCode::ok)
		{
			Serial.println("Error SPI NVSRAM");
			error = true;
		}

		// Setup of I2C Multiplexer
		if (I2CMultiplexer::setup() != ReturnCode::ok)
		{
			Serial.println("Error I2CMultiplexer");
			error = true;
		}

		// Setup of Distance Sensors
		if (DistanceSensors::setup() != ReturnCode::ok)
		{
			Serial.println("Error DistanceSensors");
			error = true;
		}

		// TESTING
		DistanceSensors::resetHardCodedCalib();

		// Setup of Gyro
		if (Gyro::setup() != ReturnCode::ok)
		{
			Serial.println("Error BNO055");
			error = true;
		}

		// Setup of MotorControl
		if (MotorControl::setup() != ReturnCode::ok)
		{
			Serial.println("Error MotorControl!");
			error = true;
		}

		// Setup of HeatSensor
		if (HeatSensor::setup() != ReturnCode::ok)
		{
			Serial.println("Error HeatSensor!");
			error = true;
		}

		// Setup of ColorSensor
		if (ColorSensor::setup() != ReturnCode::ok)
		{
			Serial.println("Error ColorSensor!");
			error = true;
		}

		// Setup of MazeMapping
		if (MazeMapping::setup() != ReturnCode::ok)
		{
			Serial.println("Error MazeMapping!");
			error = true;
		}

		if (!error) {
			Serial.println("Finished setup!");
		}
		else {
			// TODO blink LEDs
			Serial.println("Error during setup!");
		}

		// TESTING
		// Gyro::calibrate();

		Serial.println("Wait for initial BNO055 calibration...");

		uint8_t bno_sys = 0;
		do {
			Gyro::updateValues();
			bno_sys = Gyro::getOverallCalibStatus();
			delay(100);
		} while (bno_sys < 3);

		Serial.println("BNO055 ready!");

		//PowerLEDs::setBrightness(SIALSettings::PowerLEDs::defaultPower);

		//while (!Switch::getState());

		Serial.println("Start!");
	}

	void robotLoop() {
		static float fps = 100.0f;
		static uint32_t t = 0;
		if (t > 0) {
			fps = 1000.0f / (millis() - t) * 0.8f + fps * 0.2f;
		}
		t = millis();

		Serial.print("fps: ");
		Serial.println(fps);

		SensorFusion::updateSensors();
		Serial.println(millis() - t);
		SensorFusion::distSensFusion();
		Serial.println(millis() - t);
		SensorFusion::sensorFusion();
		Serial.println(millis() - t);

		SmoothDriving::updateSpeeds();

		Serial.println(millis() - t);
		RobotLogic::loop();

		Serial.println(millis() - t);
		if (Bumper::left) {
			Bumper::left = false;
		}

		if (Bumper::right) {
			Bumper::right = false;
		}

		// TESTING
		//HeatSensor::detectVictim(HeatSensorSide::left);
		//HeatSensor::detectVictim(HeatSensorSide::right);

		auto data = SensorFusion::getFusedData();

		//Serial.print(data.robotState.position.x);
		//Serial.print("; ");
		//Serial.println(data.robotState.position.y);

		//const char* stateLookup[] = { "ok", "over", "under", "err" };
		//Serial.print(stateLookup[(int)data.distSensorState.leftFront]);
		//Serial.print("; ");
		//Serial.println(data.distances.leftFront);

		//auto forward = Gyro::getForwardVec();

		//Serial.print(getGlobalHeading(forward));
		//Serial.print("; ");
		//Serial.println(getPitch(forward));
	}
}