/*
This part of the Library is responsible for the 9 DOF-IMU (BNO055).
*/

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "../header/SpiNVSRAM.h"
#include <Adafruit_Sensor.h>
#include "../header/Bno055.h"
#include "../header/AllDatatypes.h"
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <utility/imumaths.h>

namespace JAFD
{
	namespace Bno055
	{
		namespace
		{
			constexpr uint32_t startAddress = 64 * 1024;

			Adafruit_BNO055 bno055;

			//Variables for getting the sensor values

			sensors_event_t	orientationEvent, angVelEvent, linearAccelEvent, gravityVecEvent;
		}

		void init()		//vorne steht das was die init Funktion zurückgibt und hinten das was wir ihr übergeben
		{
			bno055 = Adafruit_BNO055(55,0x28);

			if (!bno055.begin())
			{
				Serial.println("ERROR INIT ");
			}

			delay(1000);

			bno055.setExtCrystalUse(true);
		}


		void calibration()								// how to calibrate
		{
			bno055.setMode(bno055.OPERATION_MODE_NDOF_FMC_OFF);

			delay(30);

			uint8_t system, gyro, accel, mag; //system is 3 when accel and magn are 3


			//Calibration accelerometer

			Serial.println("Calibration accelerometer:");

			do {

				bno055.getCalibration(&system, &gyro, &accel, &mag);

			} while (!accel ); 

			Serial.print("accel:");
			Serial.println(accel);

			//Calibration magnetometer

			Serial.println("Calibration magnetometer:");

			do {

				bno055.getCalibration(&system, &gyro, &accel, &mag);

			} while (mag < 2);

			Serial.print("mag:");
			Serial.println(mag);

			//Calibration gyro

			Serial.println("Calibration gyro:");
			
			do {

				bno055.getCalibration(&system, &gyro, &accel, &mag);

			} while (gyro < 2);

			Serial.print("gyro:");
			Serial.println(gyro);

			Serial.println("Calibration complete!");


			//Values 

	/*		Serial.print("gyro: ");
			Serial.println(gyro);*/
			/*
			Serial.print("accel: ");
			Serial.println(accel);*/

			//Serial.print("magn: ");
			//Serial.println(mag);




		}


		void update_sensorreadings()					//gets values from the sensors
		{
			bno055.getEvent(&orientationEvent, Adafruit_BNO055::VECTOR_EULER);
			bno055.getEvent(&angVelEvent, Adafruit_BNO055::VECTOR_GYROSCOPE);
			bno055.getEvent(&linearAccelEvent, Adafruit_BNO055::VECTOR_LINEARACCEL);
			bno055.getEvent(&gravityVecEvent, Adafruit_BNO055::VECTOR_GRAVITY);
		}

		Vec3f get_linear_acceleration()
		{
			Vec3f linear_acceleration_values(0.0f, 0.0f, 0.0f);

			if (linearAccelEvent.type == SENSOR_TYPE_LINEAR_ACCELERATION)
			{

				linear_acceleration_values.x = linearAccelEvent.acceleration.x;
				linear_acceleration_values.y = linearAccelEvent.acceleration.y;
				linear_acceleration_values.z = linearAccelEvent.acceleration.z;
			}

			return linear_acceleration_values; 
		}

		Vec3f get_angular_velocity()
		{
			Vec3f angular_velocity_values(100.0f, 100.0f, 100.0f);

			if (angVelEvent.type == SENSOR_TYPE_ROTATION_VECTOR)
			{

				angular_velocity_values.x = angVelEvent.gyro.x;		//Links rechts
				angular_velocity_values.y = angVelEvent.gyro.y;		// auf ab 
				angular_velocity_values.z = angVelEvent.gyro.z;		//Seiten schief liegen
			}

			return angular_velocity_values;
		}

		Vec3f get_absolute_orientation()
		{
			Vec3f absolute_orientation_values(0.0f, 0.0f, 0.0f);

			if (orientationEvent.type == SENSOR_TYPE_ORIENTATION)
			{
				absolute_orientation_values.x = orientationEvent.orientation.x;
				absolute_orientation_values.y = orientationEvent.orientation.y;
				absolute_orientation_values.z = orientationEvent.orientation.z;
			}

			return absolute_orientation_values;
			
		}

		Vec3f get_gravity_vector()
		{
			Vec3f gravity_vector_values(100.0f, 100.0f, 100.0f);

			// Why the f**k is the gravity measurement an accelerometer sensor and not a gravity sensor?
			if (gravityVecEvent.type == SENSOR_TYPE_ACCELEROMETER)
			{

				gravity_vector_values.x = gravityVecEvent.acceleration.x;
				gravity_vector_values.y = gravityVecEvent.acceleration.y;
				gravity_vector_values.z = gravityVecEvent.acceleration.z;

			}

			return gravity_vector_values;
		}





		void Write_to_RAM()								//save Offsets
		{
			adafruit_bno055_offsets_t calib_data;	//Calibration data of bno055 structure

			bno055.getSensorOffsets(calib_data);		//write values in structure calib_data


			auto accel_offset_x = calib_data.accel_offset_x;
			auto accel_offset_y = calib_data.accel_offset_y;
			auto accel_offset_z = calib_data.accel_offset_z;
			auto accel_radius = calib_data.accel_radius;

			auto gyro_offset_x = calib_data.gyro_offset_x;
			auto gyro_offset_y = calib_data.gyro_offset_y;
			auto gyro_offset_z = calib_data.gyro_offset_z;

			auto mag_offset_x = calib_data.mag_offset_x;
			auto mag_offset_y = calib_data.mag_offset_y;
			auto mag_offset_z = calib_data.mag_offset_z;
			auto mag_radius = calib_data.mag_radius;

			SpiNVSRAM::writeByte(startAddress, (accel_offset_x >> 8));		//schreibt obere tetrade des 16 Bit int auf Startaddresse
			SpiNVSRAM::writeByte(startAddress + 1, (accel_offset_x & 0xFF));

			SpiNVSRAM::writeByte(startAddress + 2, (accel_offset_y >> 8));
			SpiNVSRAM::writeByte(startAddress + 3, (accel_offset_y & 0xFF));

			SpiNVSRAM::writeByte(startAddress + 4, (accel_offset_z >> 8));
			SpiNVSRAM::writeByte(startAddress + 5, (accel_offset_z & 0xFF));

			SpiNVSRAM::writeByte(startAddress + 6, (accel_radius >> 8));
			SpiNVSRAM::writeByte(startAddress + 7, (accel_radius & 0xFF));

			SpiNVSRAM::writeByte(startAddress + 8, (gyro_offset_x >> 8));		//schreibt obere tetrade des 16 Bit int auf Startaddresse
			SpiNVSRAM::writeByte(startAddress + 9, (gyro_offset_x & 0xFF));

			SpiNVSRAM::writeByte(startAddress + 10, (gyro_offset_y >> 8));
			SpiNVSRAM::writeByte(startAddress + 11, (gyro_offset_y & 0xFF));

			SpiNVSRAM::writeByte(startAddress + 12, (gyro_offset_z >> 8));
			SpiNVSRAM::writeByte(startAddress + 13, (gyro_offset_z & 0xFF));

			SpiNVSRAM::writeByte(startAddress + 14, (mag_offset_x >> 8));
			SpiNVSRAM::writeByte(startAddress + 15, (mag_offset_x & 0xFF));

			SpiNVSRAM::writeByte(startAddress + 16, (mag_offset_y >> 8));
			SpiNVSRAM::writeByte(startAddress + 17, (mag_offset_y & 0xFF));

			SpiNVSRAM::writeByte(startAddress + 18, (mag_offset_z >> 8));
			SpiNVSRAM::writeByte(startAddress + 19, (mag_offset_z & 0xFF));

			SpiNVSRAM::writeByte(startAddress + 20, (mag_radius >> 8));
			SpiNVSRAM::writeByte(startAddress + 21, (mag_radius & 0xFF));

		}


		void Read_from_RAM()			//get Offsets from RAM
		{

			adafruit_bno055_offsets_t calib_data;



			calib_data.accel_offset_x = (SpiNVSRAM::readByte(startAddress) << 8) + SpiNVSRAM::readByte(startAddress + 1);
			calib_data.accel_offset_y = (SpiNVSRAM::readByte(startAddress + 2) << 8) + SpiNVSRAM::readByte(startAddress + 3);
			calib_data.accel_offset_z = (SpiNVSRAM::readByte(startAddress + 4) << 8) + SpiNVSRAM::readByte(startAddress + 5);
			calib_data.accel_radius = (SpiNVSRAM::readByte(startAddress + 6) << 8) + SpiNVSRAM::readByte(startAddress + 7);


			calib_data.gyro_offset_x = (SpiNVSRAM::readByte(startAddress + 8) << 8) + SpiNVSRAM::readByte(startAddress + 9);
			calib_data.gyro_offset_y = (SpiNVSRAM::readByte(startAddress + 10) << 8) + SpiNVSRAM::readByte(startAddress + 11);
			calib_data.gyro_offset_z = (SpiNVSRAM::readByte(startAddress + 12) << 8) + SpiNVSRAM::readByte(startAddress + 13);

			calib_data.mag_offset_x = (SpiNVSRAM::readByte(startAddress + 14) << 8) + SpiNVSRAM::readByte(startAddress + 15);
			calib_data.mag_offset_y = (SpiNVSRAM::readByte(startAddress + 16) << 8) + SpiNVSRAM::readByte(startAddress + 17);
			calib_data.mag_offset_z = (SpiNVSRAM::readByte(startAddress + 18) << 8) + SpiNVSRAM::readByte(startAddress + 19);
			calib_data.mag_radius = (SpiNVSRAM::readByte(startAddress + 20) << 8) + SpiNVSRAM::readByte(startAddress + 21);


			bno055.setSensorOffsets(calib_data);	//write values to sensor offsets

		}



	}
}