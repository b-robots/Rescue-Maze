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
			volatile sensors_event_t linearAccelEvent;
			volatile sensors_event_t gravityEvent;
			volatile sensors_event_t rotSpeedEvent;
			volatile imu::Quaternion quat;				// tared quaternion
			volatile imu::Quaternion tareQuat;			// conjugate of quaternion to tare
			volatile uint8_t overallCalib = 0;

			// Convert linear motion to the global axis based on the robot start orientation
			Vec3f toXYZ(Vec3f vec)
			{
				return Vec3f(-vec.z, -vec.x, -vec.y);
			}
		}

		ReturnCode setup()		//vorne steht das was die setup Funktion zurückgibt und hinten das was wir ihr übergeben
		{
			bno055 = Adafruit_BNO055(55, 0x28, &Wire1);

			if (!bno055.begin(bno055.OPERATION_MODE_IMUPLUS))
			{
				return ReturnCode::error;
			}

			bno055.setMode(bno055.OPERATION_MODE_CONFIG);
			delay(25);
			Wire1.beginTransmission(0x28);
			Wire1.write(bno055.BNO055_AXIS_MAP_CONFIG_ADDR);
			Wire1.write(0b010010);
			Wire1.endTransmission();
			delay(10);
			bno055.setMode(bno055.OPERATION_MODE_IMUPLUS);
			delay(25);

			bno055.setExtCrystalUse(true);

			delay(100);

			calibFromRAM();

			return ReturnCode::ok;
		}

		ReturnCode calibrate()								// how to calibrate
		{
			bno055.setMode(bno055.OPERATION_MODE_IMUPLUS);

			delay(600);

			uint8_t system, gyro, accel, mag; //system is 3 when accel and magn are 3

			int num_calibrated = 0;
			do {
				bno055.getCalibration(&system, &gyro, &accel, &mag);
				overallCalib = fminl(gyro, accel);
				Serial.print("CALIBRATION: ");
				Serial.print(" Gyro=");
				Serial.print(gyro, DEC);
				Serial.print(" Accel=");
				Serial.println(accel, DEC);

				delay(200);

				if (overallCalib == 3) {
					num_calibrated++;
				}
				else {
					num_calibrated = 0;
				}

			} while (num_calibrated < 20);

			Serial.println("Calibration complete!");

			calibToRAM();
		}

		void tare()
		{
			auto isQuat = bno055.getQuat();
			while (fabsf(isQuat.magnitude() - 1) > 0.1) {
				isQuat = bno055.getQuat();
			}

			*(imu::Quaternion*)(&tareQuat) = isQuat.conjugate();
		}

		// Global heading in rad
		void tare(float globalHeading)
		{
			auto isQuat = bno055.getQuat();
			while (fabsf(isQuat.magnitude() - 1) > 0.1) {
				isQuat = bno055.getQuat();
			}

			imu::Quaternion a;
			a.fromAxisAngle(imu::Vector<3>(0.0f, 0.0f, 1.0f), globalHeading);

			*(imu::Quaternion*)(&tareQuat) = isQuat.conjugate() * a;
		}

		void updateValues()					//gets values from the sensors
		{
			bno055.getEvent((sensors_event_t*)&linearAccelEvent, Adafruit_BNO055::VECTOR_LINEARACCEL);
			bno055.getEvent((sensors_event_t*)&gravityEvent, Adafruit_BNO055::VECTOR_GRAVITY);

			auto isQuat = bno055.getQuat();
			while (fabsf(isQuat.magnitude() - 1) > 0.1) {
				isQuat = bno055.getQuat();
			}

			*(imu::Quaternion*)&quat = isQuat * (*(imu::Quaternion*)&tareQuat);

			bno055.getEvent((sensors_event_t*)&rotSpeedEvent, Adafruit_BNO055::VECTOR_GYROSCOPE);

			uint8_t system, gyro, accel, mag;
			bno055.getCalibration(&system, &gyro, &accel, &mag);
			overallCalib = fminl(gyro, accel);
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

			auto currQuat = *(imu::Quaternion*)&quat;
			currQuat.normalize();

			auto rotatedVec = currQuat.rotateVector(imu::Vector<3>(1.0f, 0.0f, 0.0f));

			forwardVec.x = rotatedVec.x();
			forwardVec.y = rotatedVec.y();
			forwardVec.z = rotatedVec.z();

			return forwardVec;
		}

		float getRotSpeed()
		{
			if (rotSpeedEvent.type == SENSOR_TYPE_GYROSCOPE || rotSpeedEvent.type == SENSOR_TYPE_ROTATION_VECTOR)
			{
				return rotSpeedEvent.gyro.y;
			}

			return 0.0;
		}

		uint8_t getOverallCalibStatus() {
			return overallCalib;
		}

		void calibToRAM()								//save Offsets
		{
			adafruit_bno055_offsets_t calib_data;	//Calibration data of bno055 structure

			bno055.setMode(bno055.OPERATION_MODE_CONFIG);
			delay(100);
			bno055.getSensorOffsets(calib_data);		//write values in structure calib_data
			
			bno055.setMode(bno055.OPERATION_MODE_IMUPLUS);
			delay(600);

			adafruit_bno055_offsets_t old_calib_data;

			old_calib_data.accel_offset_x = (SpiNVSRAM::readByte(JAFDSettings::SpiNVSRAM::bno055StartAddr) << 8) + SpiNVSRAM::readByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 1);
			old_calib_data.accel_offset_y = (SpiNVSRAM::readByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 2) << 8) + SpiNVSRAM::readByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 3);
			old_calib_data.accel_offset_z = (SpiNVSRAM::readByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 4) << 8) + SpiNVSRAM::readByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 5);
			old_calib_data.accel_radius = (SpiNVSRAM::readByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 6) << 8) + SpiNVSRAM::readByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 7);

			old_calib_data.gyro_offset_x = (SpiNVSRAM::readByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 8) << 8) + SpiNVSRAM::readByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 9);
			old_calib_data.gyro_offset_y = (SpiNVSRAM::readByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 10) << 8) + SpiNVSRAM::readByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 11);
			old_calib_data.gyro_offset_z = (SpiNVSRAM::readByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 12) << 8) + SpiNVSRAM::readByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 13);

			old_calib_data.mag_offset_x = (SpiNVSRAM::readByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 14) << 8) + SpiNVSRAM::readByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 15);
			old_calib_data.mag_offset_y = (SpiNVSRAM::readByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 16) << 8) + SpiNVSRAM::readByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 17);
			old_calib_data.mag_offset_z = (SpiNVSRAM::readByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 18) << 8) + SpiNVSRAM::readByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 19);
			old_calib_data.mag_radius = (SpiNVSRAM::readByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 20) << 8) + SpiNVSRAM::readByte(JAFDSettings::SpiNVSRAM::bno055StartAddr + 21);

			auto accel_offset_x = calib_data.accel_offset_x;	// -20027
			Serial.println(accel_offset_x);
			auto accel_offset_y = calib_data.accel_offset_y;	// 8
			Serial.println(accel_offset_y);
			auto accel_offset_z = calib_data.accel_offset_z;	// 18701
			Serial.println(accel_offset_z);
			auto accel_radius = calib_data.accel_radius;		// 16398
			Serial.println(accel_radius);

			auto gyro_offset_x = calib_data.gyro_offset_x;		// 20
			Serial.println(gyro_offset_x);
			auto gyro_offset_y = calib_data.gyro_offset_y;		// 0
			Serial.println(gyro_offset_y);
			auto gyro_offset_z = calib_data.gyro_offset_z;		// 6656
			Serial.println(gyro_offset_z);

			auto mag_offset_x = calib_data.mag_offset_x;		// 9
			Serial.println(mag_offset_x);
			auto mag_offset_y = calib_data.mag_offset_y;		// 4944
			Serial.println(mag_offset_y);
			auto mag_offset_z = calib_data.mag_offset_z;		// 8199
			Serial.println(mag_offset_z);
			auto mag_radius = calib_data.mag_radius;			// -19955
			Serial.println(mag_radius);

			if (fabsf(old_calib_data.accel_offset_x + 20000) < 5000 && fabsf(old_calib_data.mag_offset_y - 5000) < 2000 && fabsf(old_calib_data.accel_offset_z - 18700) < 5000) {
				accel_offset_x = (accel_offset_x + old_calib_data.accel_offset_x) / 2;
				accel_offset_y = (accel_offset_y + old_calib_data.accel_offset_y) / 2;
				accel_offset_z = (accel_offset_z + old_calib_data.accel_offset_z) / 2;
				accel_radius = (accel_radius + old_calib_data.accel_radius) / 2;

				mag_offset_x = (mag_offset_x + old_calib_data.mag_offset_x) / 2;
				mag_offset_y = (mag_offset_y + old_calib_data.mag_offset_y) / 2;
				mag_offset_z = (mag_offset_z + old_calib_data.mag_offset_z) / 2;
				mag_radius = (mag_radius + old_calib_data.mag_radius) / 2;

				gyro_offset_x = (gyro_offset_x + old_calib_data.gyro_offset_x) / 2;
				gyro_offset_y = (gyro_offset_y + old_calib_data.gyro_offset_y) / 2;
				gyro_offset_z = (gyro_offset_z + old_calib_data.gyro_offset_z) / 2;
			}

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

			bno055.setMode(bno055.OPERATION_MODE_CONFIG);
			delay(100);
			bno055.setSensorOffsets(calib_data);	//write values to sensor offsets

			bno055.setMode(bno055.OPERATION_MODE_IMUPLUS);
			delay(600);
		}
	}
}