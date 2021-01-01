/*
This part of the Library is responsible for the 9 DOF-IMU (BNO055).
*/

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "../../JAFDSettings.h"
#include "../header/SpiNVSRAM.h"
#include "../header/Bno055.h"
#include "../header/AllDatatypes.h"
#include "../header/Math.h"

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <utility/imumaths.h>

namespace JAFD
{
	namespace Bno055
	{
		namespace
		{
			Adafruit_BNO055 bno055;

			//Variables for getting the sensor values
			sensors_event_t	linearAccelEvent;
			imu::Quaternion quat;				// tared quaternion
			imu::Quaternion tareQuat;			// conjugate of quaternion to tare

			// Convert linear motion to the global axis based on the robot start orientation
			Vec3f toXYZ(Vec3f vec)
			{
				return Vec3f(-vec.z, -vec.x, -vec.y);
			}
		}

		ReturnCode setup()		//vorne steht das was die setup Funktion zurückgibt und hinten das was wir ihr übergeben
		{
			bno055 = Adafruit_BNO055(55, 0x28, &Wire1);

			if (!bno055.begin())
			{
				return ReturnCode::error;
			}

			delay(500);

			bno055.setExtCrystalUse(true);

			calibFromRAM();

			return ReturnCode::ok;
		}


		ReturnCode calibrate()								// how to calibrate
		{
			bno055.setMode(bno055.OPERATION_MODE_NDOF_FMC_OFF);

			delay(30);

			uint8_t system, gyro, accel, mag; //system is 3 when accel and magn are 3


			//Calibration accelerometer

			Serial.println("Calibration accelerometer:");

			do {

				bno055.getCalibration(&system, &gyro, &accel, &mag);

			} while (accel < 2); 

			//Calibration magnetometer

			Serial.println("Calibration magnetometer:");

			do {

				bno055.getCalibration(&system, &gyro, &accel, &mag);

			} while (mag < 2);

			//Calibration gyro

			Serial.println("Calibration gyro:");
			
			do {

				bno055.getCalibration(&system, &gyro, &accel, &mag);

			} while (gyro < 2);

			Serial.println("Calibration complete!");

			calibToRAM();
		}

		void tare()
		{
			tareQuat = bno055.getQuat().conjugate();
		}

		// Global heading in rad
		void tare(float globalHeading)
		{
			updateValues();

			auto isQuat = bno055.getQuat();

			imu::Quaternion shouldQuat;
			shouldQuat.fromAxisAngle(imu::Vector<3>(0, 1.0f, 0), globalHeading);

			tareQuat = isQuat * shouldQuat.conjugate();
		}

		void updateValues()					//gets values from the sensors
		{
			bno055.getEvent(&linearAccelEvent, Adafruit_BNO055::VECTOR_LINEARACCEL);

			quat = bno055.getQuat() * tareQuat;
		}

		Vec3f getLinAcc()
		{
			Vec3f linear_acceleration_values(0.0f, 0.0f, 0.0f);

			if (linearAccelEvent.type == SENSOR_TYPE_LINEAR_ACCELERATION)
			{

				linear_acceleration_values.x = linearAccelEvent.acceleration.x;
				linear_acceleration_values.y = linearAccelEvent.acceleration.y;
				linear_acceleration_values.z = linearAccelEvent.acceleration.z;
			}

			return toXYZ(linear_acceleration_values); 
		}

		// Forward vector
		Vec3f getForwardVec()
		{
			Vec3f forwardVec;

			forwardVec.x = 1.0f - 2.0f * (quat.x() * quat.x() + quat.z() * quat.z());
			forwardVec.y = -2.0f * (quat.x() * quat.y() - quat.w() * quat.z());
			forwardVec.z = -2.0f * (quat.y() * quat.z() + quat.w() * quat.x());

			return forwardVec;
		}


		void calibToRAM()								//save Offsets
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

			SpiNVSRAM::writeByte(JAFDSettings::SpiNVSRAM::bno055StartAddr, (accel_offset_x >> 8));		//schreibt obere tetrade des 16 Bit int auf Startaddresse
			SpiNVSRAM::writeByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 1, (accel_offset_x & 0xFF));

			SpiNVSRAM::writeByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 2, (accel_offset_y >> 8));
			SpiNVSRAM::writeByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 3, (accel_offset_y & 0xFF));

			SpiNVSRAM::writeByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 4, (accel_offset_z >> 8));
			SpiNVSRAM::writeByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 5, (accel_offset_z & 0xFF));

			SpiNVSRAM::writeByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 6, (accel_radius >> 8));
			SpiNVSRAM::writeByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 7, (accel_radius & 0xFF));

			SpiNVSRAM::writeByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 8, (gyro_offset_x >> 8));		//schreibt obere tetrade des 16 Bit int auf Startaddresse
			SpiNVSRAM::writeByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 9, (gyro_offset_x & 0xFF));

			SpiNVSRAM::writeByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 10, (gyro_offset_y >> 8));
			SpiNVSRAM::writeByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 11, (gyro_offset_y & 0xFF));

			SpiNVSRAM::writeByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 12, (gyro_offset_z >> 8));
			SpiNVSRAM::writeByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 13, (gyro_offset_z & 0xFF));

			SpiNVSRAM::writeByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 14, (mag_offset_x >> 8));
			SpiNVSRAM::writeByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 15, (mag_offset_x & 0xFF));

			SpiNVSRAM::writeByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 16, (mag_offset_y >> 8));
			SpiNVSRAM::writeByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 17, (mag_offset_y & 0xFF));

			SpiNVSRAM::writeByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 18, (mag_offset_z >> 8));
			SpiNVSRAM::writeByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 19, (mag_offset_z & 0xFF));

			SpiNVSRAM::writeByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 20, (mag_radius >> 8));
			SpiNVSRAM::writeByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 21, (mag_radius & 0xFF));
		}


		void calibFromRAM()			//get Offsets from RAM
		{
			adafruit_bno055_offsets_t calib_data;

			calib_data.accel_offset_x = (SpiNVSRAM::readByte(JAFDSettings::SpiNVSRAM::bno055StartAddr) << 8) + SpiNVSRAM::readByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 1);
			calib_data.accel_offset_y = (SpiNVSRAM::readByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 2) << 8) + SpiNVSRAM::readByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 3);
			calib_data.accel_offset_z = (SpiNVSRAM::readByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 4) << 8) + SpiNVSRAM::readByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 5);
			calib_data.accel_radius = (SpiNVSRAM::readByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 6) << 8) + SpiNVSRAM::readByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 7);

			calib_data.gyro_offset_x = (SpiNVSRAM::readByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 8) << 8) + SpiNVSRAM::readByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 9);
			calib_data.gyro_offset_y = (SpiNVSRAM::readByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 10) << 8) + SpiNVSRAM::readByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 11);
			calib_data.gyro_offset_z = (SpiNVSRAM::readByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 12) << 8) + SpiNVSRAM::readByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 13);

			calib_data.mag_offset_x = (SpiNVSRAM::readByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 14) << 8) + SpiNVSRAM::readByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 15);
			calib_data.mag_offset_y = (SpiNVSRAM::readByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 16) << 8) + SpiNVSRAM::readByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 17);
			calib_data.mag_offset_z = (SpiNVSRAM::readByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 18) << 8) + SpiNVSRAM::readByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 19);
			calib_data.mag_radius = (SpiNVSRAM::readByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 20) << 8) + SpiNVSRAM::readByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 21);

			bno055.setSensorOffsets(calib_data);	//write values to sensor offsets
		}
	}
}