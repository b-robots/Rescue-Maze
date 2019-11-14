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
			constexpr uint8_t _mLPWM = JAFDSettings::MotorControl::Left::pwmPin;	// PWM pin left motor
			constexpr uint8_t _mRPWM = JAFDSettings::MotorControl::Right::pwmPin;	// PWM pin right motor

			constexpr uint8_t _mLDir = JAFDSettings::MotorControl::Left::dirPin;	// Direction pin left motor
			constexpr uint8_t _mRDir = JAFDSettings::MotorControl::Right::dirPin;	// Direction pin right motor

			constexpr uint8_t _mLFb = JAFDSettings::MotorControl::Left::fbPin;		// Current sense output left motor
			constexpr uint8_t _mRFb = JAFDSettings::MotorControl::Right::fbPin;		// Current sense output right motor
			
			constexpr uint8_t _mLEncA = JAFDSettings::MotorControl::Left::encA;		// Encoder Pin A left motor
			constexpr uint8_t _mLEncB = JAFDSettings::MotorControl::Left::encB;		// Encoder Pin B left motor

			constexpr uint8_t _mREncA = JAFDSettings::MotorControl::Right::encA;	// Encoder Pin A right motor
			constexpr uint8_t _mREncB = JAFDSettings::MotorControl::Right::encB;	// Encoder Pin B right motor
		
			volatile int32_t _mLEncCnt = 0;
			volatile int32_t _mREncCnt = 0;
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
			PinMapping::MappedPins[_mLDir].port->PIO_PER = PinMapping::MappedPins[_mLDir].pin;
			PinMapping::MappedPins[_mLDir].port->PIO_OER = PinMapping::MappedPins[_mLDir].pin;
			PinMapping::MappedPins[_mLDir].port->PIO_CODR = PinMapping::MappedPins[_mLDir].pin;
			
			// Right Dir
			PinMapping::MappedPins[_mRDir].port->PIO_PER = PinMapping::MappedPins[_mRDir].pin;
			PinMapping::MappedPins[_mRDir].port->PIO_OER = PinMapping::MappedPins[_mRDir].pin;
			PinMapping::MappedPins[_mRDir].port->PIO_CODR = PinMapping::MappedPins[_mRDir].pin;

			// Left Encoder A
			PinMapping::MappedPins[_mLEncA].port->PIO_PER = PinMapping::MappedPins[_mLEncA].pin;
			PinMapping::MappedPins[_mLEncA].port->PIO_ODR = PinMapping::MappedPins[_mLEncA].pin;
			PinMapping::MappedPins[_mLEncA].port->PIO_PUER = PinMapping::MappedPins[_mLEncA].pin;
			PinMapping::MappedPins[_mLEncA].port->PIO_IER = PinMapping::MappedPins[_mLEncA].pin;
			PinMapping::MappedPins[_mLEncA].port->PIO_AIMER = PinMapping::MappedPins[_mLEncA].pin;
			PinMapping::MappedPins[_mLEncA].port->PIO_ESR = PinMapping::MappedPins[_mLEncA].pin;
			PinMapping::MappedPins[_mLEncA].port->PIO_REHLSR = PinMapping::MappedPins[_mLEncA].pin;
			
			// Left Encoder B
			PinMapping::MappedPins[_mLEncB].port->PIO_PER = PinMapping::MappedPins[_mLEncB].pin;
			PinMapping::MappedPins[_mLEncB].port->PIO_ODR = PinMapping::MappedPins[_mLEncB].pin;
			PinMapping::MappedPins[_mLEncB].port->PIO_PUER = PinMapping::MappedPins[_mLEncB].pin;

			// Right Encoder A
			PinMapping::MappedPins[_mREncA].port->PIO_PER = PinMapping::MappedPins[_mREncA].pin;
			PinMapping::MappedPins[_mREncA].port->PIO_ODR = PinMapping::MappedPins[_mREncA].pin;
			PinMapping::MappedPins[_mREncA].port->PIO_PUER = PinMapping::MappedPins[_mREncA].pin;
			PinMapping::MappedPins[_mREncA].port->PIO_IER = PinMapping::MappedPins[_mREncA].pin;
			PinMapping::MappedPins[_mREncA].port->PIO_AIMER = PinMapping::MappedPins[_mREncA].pin;
			PinMapping::MappedPins[_mREncA].port->PIO_ESR = PinMapping::MappedPins[_mREncA].pin;
			PinMapping::MappedPins[_mREncA].port->PIO_REHLSR = PinMapping::MappedPins[_mREncA].pin;

			// Right Encoder B
			PinMapping::MappedPins[_mREncB].port->PIO_PER = PinMapping::MappedPins[_mREncB].pin;
			PinMapping::MappedPins[_mREncB].port->PIO_ODR = PinMapping::MappedPins[_mREncB].pin;
			PinMapping::MappedPins[_mREncB].port->PIO_PUER = PinMapping::MappedPins[_mREncB].pin;

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

			PinMapping::MappedPins[_mLPWM].port->PIO_PDR = PinMapping::MappedPins[_mLPWM].pin;
			PinMapping::MappedPins[_mRPWM].port->PIO_PDR = PinMapping::MappedPins[_mRPWM].pin;

			if (PinMapping::toABPeripheral(_mLPWM))
			{
				PinMapping::MappedPins[_mLPWM].port->PIO_ABSR |= PinMapping::MappedPins[_mLPWM].pin;
			}
			else
			{
				PinMapping::MappedPins[_mLPWM].port->PIO_ABSR &= ~PinMapping::MappedPins[_mLPWM].pin;
			}

			if (PinMapping::toABPeripheral(_mRPWM))
			{
				PinMapping::MappedPins[_mRPWM].port->PIO_ABSR |= PinMapping::MappedPins[_mRPWM].pin;
			}
			else
			{
				PinMapping::MappedPins[_mRPWM].port->PIO_ABSR &= ~PinMapping::MappedPins[_mRPWM].pin;
			}

			// Setup ADC (Freerunning mode / 21MHz)
			const uint8_t mLADCCh = PinMapping::getADCChannel(_mLFb);
			const uint8_t mRADCCh = PinMapping::getADCChannel(_mRFb);

			PMC->PMC_PCER1 = PMC_PCER1_PID37;

			ADC->ADC_MR = ADC_MR_FREERUN_ON | ADC_MR_PRESCAL(3) | ADC_MR_STARTUP_SUT896 | ADC_MR_SETTLING_AST5 | ADC_MR_TRACKTIM(0) | ADC_MR_TRANSFER(1);
			ADC->ADC_CGR = ADC_CGR_GAIN0(0b11);
			ADC->ADC_CHER = 1 << mLADCCh | 1 << mRADCCh;

			// Enable interrupts for rotary encoder pins
			NVIC_EnableIRQ(static_cast<IRQn_Type>(PinMapping::MappedPins[_mLEncA].portID));
			NVIC_EnableIRQ(static_cast<IRQn_Type>(PinMapping::MappedPins[_mLEncB].portID));
			
			NVIC_EnableIRQ(static_cast<IRQn_Type>(PinMapping::MappedPins[_mREncA].portID));
			NVIC_EnableIRQ(static_cast<IRQn_Type>(PinMapping::MappedPins[_mREncB].portID));

			// Set priority
			NVIC_SetPriority(static_cast<IRQn_Type>(PinMapping::MappedPins[_mLEncA].portID), 5);
			NVIC_SetPriority(static_cast<IRQn_Type>(PinMapping::MappedPins[_mLEncB].portID), 5);

			NVIC_SetPriority(static_cast<IRQn_Type>(PinMapping::MappedPins[_mREncA].portID), 5);
			NVIC_SetPriority(static_cast<IRQn_Type>(PinMapping::MappedPins[_mREncB].portID), 5);

			return ReturnCode::ok;
		}

		float getDistance(Motor motor)
		{
			if (motor == Motor::left)
			{
				//return _mLEncCnt / 341.2f * 
			}
			else
			{

			}
		}

		void encoderInterrupt(Interrupts::InterruptSource source, uint32_t isr)
		{
			if (PinMapping::MappedPins[_mLEncA].portID == static_cast<uint8_t>(source) && (isr & PinMapping::MappedPins[_mLEncA].pin))
			{
				if (PinMapping::MappedPins[_mLEncB].port->PIO_PDSR & PinMapping::MappedPins[_mLEncB].pin)
				{
					_mLEncCnt++;
				}
				else
				{
					_mLEncCnt--;
				}
			}

			if (PinMapping::MappedPins[_mREncA].portID == static_cast<uint8_t>(source) && (isr & PinMapping::MappedPins[_mREncA].pin))
			{
				if (PinMapping::MappedPins[_mREncB].port->PIO_PDSR & PinMapping::MappedPins[_mREncB].pin)
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
					PinMapping::MappedPins[_mLDir].port->PIO_CODR = PinMapping::MappedPins[_mLDir].pin;
				}
				else
				{
					PinMapping::MappedPins[_mLDir].port->PIO_SODR = PinMapping::MappedPins[_mLDir].pin;
				}
			}
			else
			{
				pwmCh = PinMapping::getPWMChannel(_mRPWM);

				// Set Dir Pin
				if (speed > 0)
				{
					PinMapping::MappedPins[_mRDir].port->PIO_CODR = PinMapping::MappedPins[_mRDir].pin;
				}
				else
				{
					PinMapping::MappedPins[_mRDir].port->PIO_SODR = PinMapping::MappedPins[_mRDir].pin;
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

			// Sample 4 values and return average
			for (uint8_t i = 0; i < 4; i++)
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

			return result / 4.0f;
		}
	}
}