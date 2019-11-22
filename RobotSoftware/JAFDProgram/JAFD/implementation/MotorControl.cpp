/*
This part of the Library is responsible for driving the motors.
*/

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "../../JAFDSettings.h"
#include "MotorControl_private.h"
#include "Interrupts_private.h"
#include "../utility/DuePinMapping_private.h"

namespace JAFD
{
	namespace MotorControl
	{
		namespace
		{
			constexpr auto _mLPWM = PinMapping::MappedPins[JAFDSettings::MotorControl::Left::pwmPin];	// PWM pin left motor
			constexpr auto _mRPWM = PinMapping::MappedPins[JAFDSettings::MotorControl::Right::pwmPin];	// PWM pin right motor

			constexpr auto _mLDir = PinMapping::MappedPins[JAFDSettings::MotorControl::Left::dirPin];	// Direction pin left motor
			constexpr auto _mRDir = PinMapping::MappedPins[JAFDSettings::MotorControl::Right::dirPin];	// Direction pin right motor

			constexpr auto _mLFb = PinMapping::MappedPins[JAFDSettings::MotorControl::Left::fbPin];		// Current sense output left motor
			constexpr auto _mRFb = PinMapping::MappedPins[JAFDSettings::MotorControl::Right::fbPin];		// Current sense output right motor
			
			constexpr auto _mLEncA = PinMapping::MappedPins[JAFDSettings::MotorControl::Left::encA];		// Encoder Pin A left motor
			constexpr auto _mLEncB = PinMapping::MappedPins[JAFDSettings::MotorControl::Left::encB];		// Encoder Pin B left motor

			constexpr auto _mREncA = PinMapping::MappedPins[JAFDSettings::MotorControl::Right::encA];	// Encoder Pin A right motor
			constexpr auto _mREncB = PinMapping::MappedPins[JAFDSettings::MotorControl::Right::encB];	// Encoder Pin B right motor
		
			volatile int32_t _mLEncCnt = 0;		// Encoder count left motor
			volatile int32_t _mREncCnt = 0;		// Encoder count right motor

			volatile float _mLSpeed = 0.0f;		// Speed left motor (cm/s)
			volatile float _mRSpeed = 0.0f;		// Speed right motor (cm/s)
		}

		ReturnCode motorControlSetup()
		{
			// Check if PWM Pins and ADC Pins are correct
			if (!PinMapping::hasPWM(_mLPWM) || !PinMapping::hasPWM(_mRPWM) || !PinMapping::hasADC(_mLFb) || !PinMapping::hasADC(_mRFb))
			{
				return ReturnCode::fatalError;
			}

			// Set the pin modes for Dir - Pins / Encoder - Pins
			// Left Dir
			_mLDir.port->PIO_PER = _mLDir.pin;
			_mLDir.port->PIO_OER = _mLDir.pin;
			_mLDir.port->PIO_CODR = _mLDir.pin;
			
			// Right Dir
			_mRDir.port->PIO_PER = _mRDir.pin;
			_mRDir.port->PIO_OER = _mRDir.pin;
			_mRDir.port->PIO_CODR = _mRDir.pin;

			// Left Encoder A
			_mLEncA.port->PIO_PER = _mLEncA.pin;
			_mLEncA.port->PIO_ODR = _mLEncA.pin;
			_mLEncA.port->PIO_PUER = _mLEncA.pin;
			_mLEncA.port->PIO_IER = _mLEncA.pin;
			_mLEncA.port->PIO_AIMER = _mLEncA.pin;
			_mLEncA.port->PIO_ESR = _mLEncA.pin;
			_mLEncA.port->PIO_REHLSR = _mLEncA.pin;
			_mLEncA.port->PIO_DIFSR = _mLEncA.pin;
			_mLEncA.port->PIO_SCDR = PIO_SCDR_DIV(0);
			_mLEncA.port->PIO_IFER = _mLEncA.pin;

			// Left Encoder B
			_mLEncB.port->PIO_PER = _mLEncB.pin;
			_mLEncB.port->PIO_ODR = _mLEncB.pin;
			_mLEncB.port->PIO_PUER = _mLEncB.pin;
			_mLEncB.port->PIO_DIFSR = _mLEncB.pin;
			_mLEncB.port->PIO_SCDR = PIO_SCDR_DIV(0);
			_mLEncB.port->PIO_IFER = _mLEncB.pin;

			// Right Encoder A
			_mREncA.port->PIO_PER = _mREncA.pin;
			_mREncA.port->PIO_ODR = _mREncA.pin;
			_mREncA.port->PIO_PUER = _mREncA.pin;
			_mREncA.port->PIO_IER = _mREncA.pin;
			_mREncA.port->PIO_AIMER = _mREncA.pin;
			_mREncA.port->PIO_ESR = _mREncA.pin;
			_mREncA.port->PIO_REHLSR = _mREncA.pin;
			_mREncA.port->PIO_DIFSR = _mREncA.pin;
			_mREncA.port->PIO_SCDR = PIO_SCDR_DIV(0);
			_mREncA.port->PIO_IFER = _mREncA.pin;

			// Right Encoder B
			_mREncB.port->PIO_PER = _mREncB.pin;
			_mREncB.port->PIO_ODR = _mREncB.pin;
			_mREncB.port->PIO_PUER = _mREncB.pin;
			_mREncB.port->PIO_DIFSR = _mREncB.pin;
			_mREncB.port->PIO_SCDR = PIO_SCDR_DIV(0);
			_mREncB.port->PIO_IFER = _mREncB.pin;

			// Setup PWM - Controller (20kHz)
			const uint8_t mLPWMCh = PinMapping::getPWMChannel(_mLPWM);
			const uint8_t mRPWMCh = PinMapping::getPWMChannel(_mRPWM);

			PMC->PMC_PCER1 = PMC_PCER1_PID36;

			PWM->PWM_CLK = PWM_CLK_PREA(0) | PWM_CLK_DIVA(1);
			PWM->PWM_ENA = 1 << mLPWMCh | 1 << mRPWMCh;

			PWM->PWM_CH_NUM[mLPWMCh].PWM_CMR = PWM_CMR_CPRE_CLKA;
			PWM->PWM_CH_NUM[mLPWMCh].PWM_CPRD = 4200;
			PWM->PWM_CH_NUM[mLPWMCh].PWM_CDTY = 0;

			PWM->PWM_CH_NUM[mRPWMCh].PWM_CMR = PWM_CMR_CPRE_CLKA;
			PWM->PWM_CH_NUM[mRPWMCh].PWM_CPRD = 4200;
			PWM->PWM_CH_NUM[mRPWMCh].PWM_CDTY = 0;

			_mLPWM.port->PIO_PDR = _mLPWM.pin;
			_mRPWM.port->PIO_PDR = _mRPWM.pin;

			if (PinMapping::toABPeripheral(_mLPWM))
			{
				_mLPWM.port->PIO_ABSR |= _mLPWM.pin;
			}
			else
			{
				_mLPWM.port->PIO_ABSR &= ~_mLPWM.pin;
			}

			if (PinMapping::toABPeripheral(_mRPWM))
			{
				_mRPWM.port->PIO_ABSR |= _mRPWM.pin;
			}
			else
			{
				_mRPWM.port->PIO_ABSR &= ~_mRPWM.pin;
			}

			// Setup ADC (Freerunning mode / 21MHz)
			const uint8_t mLADCCh = PinMapping::getADCChannel(_mLFb);
			const uint8_t mRADCCh = PinMapping::getADCChannel(_mRFb);

			PMC->PMC_PCER1 = PMC_PCER1_PID37;

			ADC->ADC_MR = ADC_MR_FREERUN_ON | ADC_MR_PRESCAL(3) | ADC_MR_STARTUP_SUT896 | ADC_MR_SETTLING_AST5 | ADC_MR_TRACKTIM(0) | ADC_MR_TRANSFER(1);
			ADC->ADC_CGR = ADC_CGR_GAIN0(0b11);
			ADC->ADC_CHER = 1 << mLADCCh | 1 << mRADCCh;

			// Setup interrupts for rotary encoder pins
			NVIC_EnableIRQ(static_cast<IRQn_Type>(_mLEncA.portID));
			NVIC_EnableIRQ(static_cast<IRQn_Type>(_mREncA.portID));

			NVIC_SetPriority(static_cast<IRQn_Type>(_mLEncA.portID), 5);
			NVIC_SetPriority(static_cast<IRQn_Type>(_mREncA.portID), 5);

			// Setup TC for an interrupt every 100ms -> 10Hz (MCK / 128 / 65625) for motor speed calculation
			// Use TC2 - Channel 6
			PMC->PMC_PCER0 = PMC_PCER0_PID29;

			NVIC_EnableIRQ(TC2_IRQn);
			NVIC_SetPriority(TC2_IRQn, 6);

			TC2->TC_CHANNEL[0].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK4 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC;
			TC2->TC_CHANNEL[0].TC_RC = 65625;
			
			TC2->TC_CHANNEL[0].TC_CCR = TC_CCR_SWTRG;

			return ReturnCode::ok;
		}

		void calcMotorSpeed()
		{
			static int32_t lastLeftCnt = 0;
			static int32_t lastRightCnt = 0;

			_mLSpeed = (lastLeftCnt - _mLEncCnt) / (11.0f * 34.02f) * JAFDSettings::Mechanics::wheelDiameter * PI;
			_mRSpeed = (lastRightCnt - _mREncCnt) / (11.0f * 34.02f) * JAFDSettings::Mechanics::wheelDiameter * PI;

			lastLeftCnt = _mLEncCnt;
			lastRightCnt = _mREncCnt;
		}

		float getDistance(Motor motor)
		{
			if (motor == Motor::left)
			{
				return _mLEncCnt / (11.0f * 34.02f) * JAFDSettings::Mechanics::wheelDiameter * PI;
			}
			else
			{
				return _mREncCnt / (11.0f * 34.02f) * JAFDSettings::Mechanics::wheelDiameter * PI;
			}
		}

		void encoderInterrupt(Interrupts::InterruptSource source, uint32_t isr)
		{
			if (_mLEncA.portID == static_cast<uint8_t>(source) && (isr & _mLEncA.pin))
			{
				if (_mLEncB.port->PIO_PDSR & _mLEncB.pin)
				{
					_mLEncCnt++;
				}
				else
				{
					_mLEncCnt--;
				}
			}

			if (_mREncA.portID == static_cast<uint8_t>(source) && (isr & _mREncA.pin))
			{
				if (_mREncB.port->PIO_PDSR & _mREncB.pin)
				{
					_mREncCnt++;
				}
				else
				{
					_mREncCnt--;
				}
			}
		}

		void setSpeed(Motor motor, float speed)
		{
			static uint8_t pwmCh;

			if (motor == Motor::left)
			{
				pwmCh = PinMapping::getPWMChannel(_mLPWM);

				// Set Dir Pin
				if (speed > 0)
				{
					_mLDir.port->PIO_CODR = _mLDir.pin;
				}
				else
				{
					_mLDir.port->PIO_SODR = _mLDir.pin;
				}
			}
			else
			{
				pwmCh = PinMapping::getPWMChannel(_mRPWM);

				// Set Dir Pin
				if (speed > 0)
				{
					_mRDir.port->PIO_CODR = _mRDir.pin;
				}
				else
				{
					_mRDir.port->PIO_SODR = _mRDir.pin;
				}
			}

			// Set PWM Value
			PWM->PWM_CH_NUM[pwmCh].PWM_CDTYUPD = (PWM->PWM_CH_NUM[pwmCh].PWM_CPRD * abs(speed));
			PWM->PWM_SCUC = PWM_SCUC_UPDULOCK;
		}

		float getCurrent(Motor motor)
		{
			Serial.println(_mLEncCnt);
			const uint8_t mLADCCh = PinMapping::getADCChannel(_mLFb);
			const uint8_t mRADCCh = PinMapping::getADCChannel(_mRFb);

			float result = 0.0f;

			// Sample 10 values and return average
			for (uint8_t i = 0; i < 10; i++)
			{
				// Wait for end of conversion
				while (!(ADC->ADC_ISR & ADC_ISR_DRDY));

				if (motor == Motor::left)
				{
					result += (float)ADC->ADC_CDR[mLADCCh] * 3.3f * 1904.7619f / 4.0f / (float)(1 << 12 - 1);
				}
				else
				{
					result += (float)ADC->ADC_CDR[mRADCCh] * 3.3f * 1904.7619f / 4.0f / (float)(1 << 12 - 1);
				}
			}

			return result / 10.0f;
		}
	}
}