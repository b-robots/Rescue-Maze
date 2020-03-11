/*
This part is responsible for dispensing the rescue packages.
*/

#include "../header/Dispenser.h"
#include "../../JAFDSettings.h"
#include "../header/SmoothDriving.h"
#include "../header/DuePinMapping.h"

namespace JAFD
{
	namespace Dispenser
	{
		namespace
		{
			constexpr auto _rightPWMPin = PinMapping::MappedPins[JAFDSettings::Dispenser::Right::servoPinRight];
			constexpr auto _leftPWMPin = PinMapping::MappedPins[JAFDSettings::Dispenser::Left::servoPinLeft];
			constexpr auto _rightPWMCh = PinMapping::getPWMChannel(_rightPWMPin);
			constexpr auto _leftPWMCh = PinMapping::getPWMChannel(_leftPWMPin);
		}
		// Set up the Dispenser System
		ReturnCode setup()
		{
			// Check if PWM Pins and ADC Pins are correct
			if (!PinMapping::hasPWM(_rightPWMPin) || !PinMapping::hasPWM(_leftPWMPin))
			{
				return ReturnCode::fatalError;
			}

			PWM->PWM_ENA = 1 << _rightPWMCh | 1 << _leftPWMCh;

			PWM->PWM_CH_NUM[_rightPWMCh].PWM_CMR = PWM_CMR_CPRE_MCK_DIV_128;
			PWM->PWM_CH_NUM[_rightPWMCh].PWM_CPRD = 13125; //  13125
			PWM->PWM_CH_NUM[_rightPWMCh].PWM_CDTY = 0;

			if (PinMapping::getPWMStartState(_rightPWMPin) == PinMapping::PWMStartState::high)
			{
				PWM->PWM_CH_NUM[_rightPWMCh].PWM_CMR |= PWM_CMR_CPOL;
			}

			PWM->PWM_CH_NUM[_leftPWMCh].PWM_CMR = PWM_CMR_CPRE_MCK_DIV_128;
			PWM->PWM_CH_NUM[_leftPWMCh].PWM_CPRD = 13125;//  13125
			PWM->PWM_CH_NUM[_leftPWMCh].PWM_CDTY = 0;

			if (PinMapping::getPWMStartState(_leftPWMPin) == PinMapping::PWMStartState::high)
			{
				PWM->PWM_CH_NUM[_leftPWMCh].PWM_CMR |= PWM_CMR_CPOL;
			}

			_leftPWMPin.port->PIO_PDR = _leftPWMPin.pin;
			_rightPWMPin.port->PIO_PDR = _rightPWMPin.pin;

			if (PinMapping::toABPeripheral(_leftPWMPin))
			{
				_leftPWMPin.port->PIO_ABSR |= _leftPWMPin.pin;
			}
			else
			{
				_leftPWMPin.port->PIO_ABSR &= ~_leftPWMPin.pin;
			}

			if (PinMapping::toABPeripheral(_rightPWMPin))
			{
				_rightPWMPin.port->PIO_ABSR |= _rightPWMPin.pin;
			}
			else
			{
				_rightPWMPin.port->PIO_ABSR &= ~_rightPWMPin.pin;
			}

			return ReturnCode::ok;
		}

		uint16_t getRightCubeCount()
		{
			//DistanceSensors::packsRight.getDistance();

			return 5; //(DistanceSensors::packsRight.getDistance() - 29) / 10;
		}

		uint16_t getLeftCubeCount()
		{
			//DistanceSensors::packsLeft.getDistance();

			return 5; //(DistanceSensors::packsRight.getDistance() - 29) / 10;
		}

		ReturnCode dispenseRight(uint8_t num)
		{
			//Max Packs to be allowed to get dispensed
			if (num > 3 || num < 1)
			{
				return ReturnCode::error;
			}
			else if (num > (getRightCubeCount() + getLeftCubeCount()))
			{
				return ReturnCode::error;
			}
			else if (getRightCubeCount() >= num)
			{
				for (int i = 0; i < num; i++)
				{
					PWM->PWM_CH_NUM[_rightPWMCh].PWM_CDTYUPD = JAFDSettings::Dispenser::Right::startDYC * PWM->PWM_CH_NUM[_rightPWMCh].PWM_CPRD;
					PWM->PWM_SCUC = PWM_SCUC_UPDULOCK;
					delay(2000);
					PWM->PWM_CH_NUM[_rightPWMCh].PWM_CDTYUPD = JAFDSettings::Dispenser::Right::endDYC * PWM->PWM_CH_NUM[_rightPWMCh].PWM_CPRD;
					PWM->PWM_SCUC = PWM_SCUC_UPDULOCK;
					delay(2000);
				}

				return ReturnCode::ok;
			}
			else
			{
				uint16_t remainPacks = num - getRightCubeCount();
				
				for (int i = 0; i < getRightCubeCount(); i++)
				{
					PWM->PWM_CH_NUM[_rightPWMCh].PWM_CDTYUPD = JAFDSettings::Dispenser::Right::startDYC * PWM->PWM_CH_NUM[_rightPWMCh].PWM_CPRD;
					PWM->PWM_SCUC = PWM_SCUC_UPDULOCK;
					delay(500);
					PWM->PWM_CH_NUM[_rightPWMCh].PWM_CDTYUPD = JAFDSettings::Dispenser::Right::endDYC * PWM->PWM_CH_NUM[_rightPWMCh].PWM_CPRD;
					PWM->PWM_SCUC = PWM_SCUC_UPDULOCK;
					delay(500);
				}

				SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::Stop(), true);
				while (!SmoothDriving::isTaskFinished());
				SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::Rotate(4.0f, 180.0f));
				while (!SmoothDriving::isTaskFinished());
				SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::Stop());

				for (int i = 0; i < remainPacks; i++)
				{
					PWM->PWM_CH_NUM[_leftPWMCh].PWM_CDTYUPD = JAFDSettings::Dispenser::Left::startDYC * PWM->PWM_CH_NUM[_leftPWMCh].PWM_CPRD;
					PWM->PWM_SCUC = PWM_SCUC_UPDULOCK;
					delay(2000);
					PWM->PWM_CH_NUM[_leftPWMCh].PWM_CDTYUPD = JAFDSettings::Dispenser::Left::endDYC * PWM->PWM_CH_NUM[_leftPWMCh].PWM_CPRD;
					PWM->PWM_SCUC = PWM_SCUC_UPDULOCK;
					delay(2000);
				}

				return ReturnCode::ok;
			}
			return ReturnCode::ok;
		}

		ReturnCode dispenseLeft(uint8_t num)
		{
			//Max Packs to be allowed to get dispensed
			if (num > 3 || num < 1)
			{
				return ReturnCode::error;
			}
			else if (num > (getRightCubeCount() + getLeftCubeCount()))
			{
				return ReturnCode::error;
			}
			else if (getRightCubeCount() >= num)
			{
				for (int i = 0; i < num; i++)
				{
					PWM->PWM_CH_NUM[_leftPWMCh].PWM_CDTYUPD = JAFDSettings::Dispenser::Left::startDYC * PWM->PWM_CH_NUM[_leftPWMCh].PWM_CPRD;
					PWM->PWM_SCUC = PWM_SCUC_UPDULOCK;
					delay(2000);
					PWM->PWM_CH_NUM[_leftPWMCh].PWM_CDTYUPD = JAFDSettings::Dispenser::Left::endDYC * PWM->PWM_CH_NUM[_leftPWMCh].PWM_CPRD;
					PWM->PWM_SCUC = PWM_SCUC_UPDULOCK;
					delay(2000);
				}

				return ReturnCode::ok;
			}
			else
			{
				uint16_t remainPacks = num - getRightCubeCount();

				for (int i = 0; i < getRightCubeCount(); i++)
				{
					PWM->PWM_CH_NUM[_leftPWMCh].PWM_CDTYUPD = JAFDSettings::Dispenser::Left::startDYC * PWM->PWM_CH_NUM[_leftPWMCh].PWM_CPRD;
					PWM->PWM_SCUC = PWM_SCUC_UPDULOCK;
					delay(2000);
					PWM->PWM_CH_NUM[_leftPWMCh].PWM_CDTYUPD = JAFDSettings::Dispenser::Left::endDYC * PWM->PWM_CH_NUM[_leftPWMCh].PWM_CPRD;
					PWM->PWM_SCUC = PWM_SCUC_UPDULOCK;
					delay(2000);
				}

				SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::Stop(), true);
				while (!SmoothDriving::isTaskFinished());
				SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::Rotate(4.0f, 180.0f));
				while (!SmoothDriving::isTaskFinished());
				SmoothDriving::setNewTask<SmoothDriving::NewStateType::lastEndState>(SmoothDriving::Stop());

				for (int i = 0; i < remainPacks; i++)
				{
					PWM->PWM_CH_NUM[_rightPWMCh].PWM_CDTYUPD = JAFDSettings::Dispenser::Right::startDYC * PWM->PWM_CH_NUM[_rightPWMCh].PWM_CPRD;
					PWM->PWM_SCUC = PWM_SCUC_UPDULOCK;
					delay(2000);
					PWM->PWM_CH_NUM[_rightPWMCh].PWM_CDTYUPD = JAFDSettings::Dispenser::Right::endDYC * PWM->PWM_CH_NUM[_rightPWMCh].PWM_CPRD;
					PWM->PWM_SCUC = PWM_SCUC_UPDULOCK;
					delay(2000);
				}

				return ReturnCode::ok;
			}
			return ReturnCode::ok;
		}
	}
}