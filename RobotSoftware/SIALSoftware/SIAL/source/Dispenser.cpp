#include "../header/Dispenser.h"
#include "../SIALSettings.h"
#include "../header/SmoothDriving.h"
#include "../header/DuePinMapping.h"
#include "../header/SensorFusion.h"
#include "../header/SmallThings.h"

namespace SIAL
{
	namespace Dispenser
	{
		namespace
		{
			constexpr auto rightPWMPin = PinMapping::MappedPins[SIALSettings::Dispenser::Right::servoPin];
			constexpr auto leftPWMPin = PinMapping::MappedPins[SIALSettings::Dispenser::Left::servoPin];
			constexpr auto rightPWMCh = PinMapping::getPWMChannel(rightPWMPin);
			constexpr auto leftPWMCh = PinMapping::getPWMChannel(leftPWMPin);
			uint8_t leftCubeCount = SIALSettings::Dispenser::Left::startCubeCount;
			uint8_t rightCubeCount = SIALSettings::Dispenser::Right::startCubeCount;
		}

		// Set up the Dispenser System
		ReturnCode setup()
		{
			// Check if PWM Pins and ADC Pins are correct
			if (!PinMapping::hasPWM(rightPWMPin) || !PinMapping::hasPWM(leftPWMPin))
			{
				return ReturnCode::fatalError;
			}

			// PWM 50Hz
			PWM->PWM_ENA = 1 << rightPWMCh | 1 << leftPWMCh;

			PWM->PWM_CH_NUM[rightPWMCh].PWM_CMR = PWM_CMR_CPRE_MCK_DIV_128;
			PWM->PWM_CH_NUM[rightPWMCh].PWM_CPRD = 13125; //  13125
			PWM->PWM_CH_NUM[rightPWMCh].PWM_CDTY = 0;

			if (PinMapping::getPWMStartState(rightPWMPin) == PinMapping::PWMStartState::high)
			{
				PWM->PWM_CH_NUM[rightPWMCh].PWM_CMR |= PWM_CMR_CPOL;
			}

			PWM->PWM_CH_NUM[leftPWMCh].PWM_CMR = PWM_CMR_CPRE_MCK_DIV_128;
			PWM->PWM_CH_NUM[leftPWMCh].PWM_CPRD = 13125;//  13125
			PWM->PWM_CH_NUM[leftPWMCh].PWM_CDTY = 0;

			if (PinMapping::getPWMStartState(leftPWMPin) == PinMapping::PWMStartState::high)
			{
				PWM->PWM_CH_NUM[leftPWMCh].PWM_CMR |= PWM_CMR_CPOL;
			}

			PWM->PWM_CH_NUM[rightPWMCh].PWM_CDTYUPD = SIALSettings::Dispenser::Right::endDty * PWM->PWM_CH_NUM[rightPWMCh].PWM_CPRD;
			PWM->PWM_CH_NUM[leftPWMCh].PWM_CDTYUPD = SIALSettings::Dispenser::Left::endDty * PWM->PWM_CH_NUM[leftPWMCh].PWM_CPRD;

			PWM->PWM_SCUC = PWM_SCUC_UPDULOCK;

			leftPWMPin.port->PIO_PDR = leftPWMPin.pin;
			rightPWMPin.port->PIO_PDR = rightPWMPin.pin;

			if (PinMapping::toABPeripheral(leftPWMPin))
			{
				leftPWMPin.port->PIO_ABSR |= leftPWMPin.pin;
			}
			else
			{
				leftPWMPin.port->PIO_ABSR &= ~leftPWMPin.pin;
			}

			if (PinMapping::toABPeripheral(rightPWMPin))
			{
				rightPWMPin.port->PIO_ABSR |= rightPWMPin.pin;
			}
			else
			{
				rightPWMPin.port->PIO_ABSR &= ~rightPWMPin.pin;
			}

			return ReturnCode::ok;
		}

		ReturnCode dispenseRight(uint8_t num)
		{
			//Max Packs to be allowed to get dispensed
			if (num > 3 || num < 1)
			{
				return ReturnCode::error;
			}
			else //if (getRightCubeCount() >= num)
			{
				for (int i = 0; i < num; i++)
				{
					PWM->PWM_CH_NUM[rightPWMCh].PWM_CDTYUPD = SIALSettings::Dispenser::Right::startDty * PWM->PWM_CH_NUM[rightPWMCh].PWM_CPRD;
					PWM->PWM_SCUC = PWM_SCUC_UPDULOCK;

					auto start = millis();
					while (millis() - start < SIALSettings::Dispenser::pause) {
						SensorFusion::sensorFusion();
						//PowerLEDs::setBrightness(sinf(millis() / SIALSettings::PowerLEDs::blinkPeriod * M_TWOPI) / 2.0f + 0.5f);
					}

					PWM->PWM_CH_NUM[rightPWMCh].PWM_CDTYUPD = SIALSettings::Dispenser::Right::endDty * PWM->PWM_CH_NUM[rightPWMCh].PWM_CPRD;
					PWM->PWM_SCUC = PWM_SCUC_UPDULOCK;

					start = millis();
					while (millis() - start < SIALSettings::Dispenser::pause) {
						SensorFusion::sensorFusion();

						//PowerLEDs::setBrightness(sinf(millis() / SIALSettings::PowerLEDs::blinkPeriod * M_TWOPI) / 2.0f + 0.5f);
					}

					rightCubeCount--;
				}

				return ReturnCode::ok;
			}
			/*else
			{
				uint16_t remainPacks = num - getRightCubeCount();

				for (int i = 0; i < getRightCubeCount(); i++)
				{
					PWM->PWM_CH_NUM[rightPWMCh].PWM_CDTYUPD = SIALSettings::Dispenser::Right::startDty * PWM->PWM_CH_NUM[rightPWMCh].PWM_CPRD;
					PWM->PWM_SCUC = PWM_SCUC_UPDULOCK;

					Wait::delayUnblocking(SIALSettings::Dispenser::pause);

					PWM->PWM_CH_NUM[rightPWMCh].PWM_CDTYUPD = SIALSettings::Dispenser::Right::endDty * PWM->PWM_CH_NUM[rightPWMCh].PWM_CPRD;
					PWM->PWM_SCUC = PWM_SCUC_UPDULOCK;

					Wait::delayUnblocking(SIALSettings::Dispenser::pause);

					rightCubeCount--;
				}

				SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::TaskArray(
					SmoothDriving::Stop(),
					SmoothDriving::Rotate(0.5f, 180.0f),
					SmoothDriving::AlignWalls(),
					SmoothDriving::Stop()));

				Wait::waitForFinishedTask();

				for (int i = 0; i < remainPacks; i++)
				{
					PWM->PWM_CH_NUM[leftPWMCh].PWM_CDTYUPD = SIALSettings::Dispenser::Left::startDty * PWM->PWM_CH_NUM[leftPWMCh].PWM_CPRD;
					PWM->PWM_SCUC = PWM_SCUC_UPDULOCK;

					Wait::delayUnblocking(SIALSettings::Dispenser::pause);

					PWM->PWM_CH_NUM[leftPWMCh].PWM_CDTYUPD = SIALSettings::Dispenser::Left::endDty * PWM->PWM_CH_NUM[leftPWMCh].PWM_CPRD;
					PWM->PWM_SCUC = PWM_SCUC_UPDULOCK;

					Wait::delayUnblocking(SIALSettings::Dispenser::pause);

					leftCubeCount--;
				}

				SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::TaskArray(
					SmoothDriving::Stop(),
					SmoothDriving::Rotate(-0.5f, -180.0f),
					SmoothDriving::AlignWalls(),
					SmoothDriving::Stop()));

				Wait::waitForFinishedTask();

				return ReturnCode::ok;
			}*/
			return ReturnCode::ok;
		}

		ReturnCode dispenseLeft(uint8_t num)
		{
			//Max Packs to be allowed to get dispensed
			if (num > 3 || num < 1)
			{
				return ReturnCode::error;
			}
			else// if (getLeftCubeCount() >= num)
			{
				for (int i = 0; i < num; i++)
				{
					PWM->PWM_CH_NUM[leftPWMCh].PWM_CDTYUPD = SIALSettings::Dispenser::Left::startDty * PWM->PWM_CH_NUM[leftPWMCh].PWM_CPRD;
					PWM->PWM_SCUC = PWM_SCUC_UPDULOCK;

					auto start = millis();
					while (millis() - start < SIALSettings::Dispenser::pause) {
						//PowerLEDs::setBrightness(sinf(millis() / SIALSettings::PowerLEDs::blinkPeriod * M_TWOPI) / 2.0f + 0.5f);
						SensorFusion::sensorFusion();
					}

					PWM->PWM_CH_NUM[leftPWMCh].PWM_CDTYUPD = SIALSettings::Dispenser::Left::endDty * PWM->PWM_CH_NUM[leftPWMCh].PWM_CPRD;
					PWM->PWM_SCUC = PWM_SCUC_UPDULOCK;

					start = millis();
					while (millis() - start < SIALSettings::Dispenser::pause) {
						SensorFusion::sensorFusion();

						//PowerLEDs::setBrightness(sinf(millis() / SIALSettings::PowerLEDs::blinkPeriod * M_TWOPI) / 2.0f + 0.5f);
					}

					leftCubeCount--;
				}

				return ReturnCode::ok;
			}
			/*
			else
			{
				uint16_t remainPacks = num - getLeftCubeCount();

				for (int i = 0; i < getLeftCubeCount(); i++)
				{
					PWM->PWM_CH_NUM[leftPWMCh].PWM_CDTYUPD = SIALSettings::Dispenser::Left::startDty * PWM->PWM_CH_NUM[leftPWMCh].PWM_CPRD;
					PWM->PWM_SCUC = PWM_SCUC_UPDULOCK;

					Wait::delayUnblocking(SIALSettings::Dispenser::pause);

					PWM->PWM_CH_NUM[leftPWMCh].PWM_CDTYUPD = SIALSettings::Dispenser::Left::endDty * PWM->PWM_CH_NUM[leftPWMCh].PWM_CPRD;
					PWM->PWM_SCUC = PWM_SCUC_UPDULOCK;

					Wait::delayUnblocking(SIALSettings::Dispenser::pause);

					leftCubeCount--;
				}

				SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::TaskArray(
					SmoothDriving::Stop(),
					SmoothDriving::Rotate(0.5f, 180.0f),
					SmoothDriving::AlignWalls(),
					SmoothDriving::Stop()));

				Wait::waitForFinishedTask();

				for (int i = 0; i < remainPacks; i++)
				{
					PWM->PWM_CH_NUM[rightPWMCh].PWM_CDTYUPD = SIALSettings::Dispenser::Right::startDty * PWM->PWM_CH_NUM[rightPWMCh].PWM_CPRD;
					PWM->PWM_SCUC = PWM_SCUC_UPDULOCK;

					Wait::delayUnblocking(SIALSettings::Dispenser::pause);

					PWM->PWM_CH_NUM[rightPWMCh].PWM_CDTYUPD = SIALSettings::Dispenser::Right::endDty * PWM->PWM_CH_NUM[rightPWMCh].PWM_CPRD;
					PWM->PWM_SCUC = PWM_SCUC_UPDULOCK;

					Wait::delayUnblocking(SIALSettings::Dispenser::pause);

					rightCubeCount--;
				}

				SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::TaskArray(
					SmoothDriving::Stop(),
					SmoothDriving::Rotate(-0.5f, -180.0f),
					SmoothDriving::AlignWalls(),
					SmoothDriving::Stop()));

				Wait::waitForFinishedTask();

				return ReturnCode::ok;
			}*/
			return ReturnCode::ok;
		}
	}
}