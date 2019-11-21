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
			constexpr PinMapping::PinInformation x = PinMapping::MappedPins[JAFDSettings::MotorControl::Left::pwmPin];
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

			// Constexpr for pin and ports
			constexpr Pio* mLDirPort = PinMapping::MappedPins[_mLDir].port;
			constexpr Pio* mRDirPort = PinMapping::MappedPins[_mRDir].port;
			constexpr uint32_t mLDirPin = PinMapping::MappedPins[_mLDir].pin;
			constexpr uint32_t mRDirPin = PinMapping::MappedPins[_mRDir].pin;

			constexpr Pio* mLEncAPort = PinMapping::MappedPins[_mLEncA].port;
			constexpr Pio* mREncAPort = PinMapping::MappedPins[_mLEncA].port;
			constexpr uint32_t mLEncAPin = PinMapping::MappedPins[_mLEncA].pin;
			constexpr uint32_t mREncAPin = PinMapping::MappedPins[_mLEncA].pin;

			constexpr Pio* mLEncBPort = PinMapping::MappedPins[_mLEncB].port;
			constexpr Pio* mREncBPort = PinMapping::MappedPins[_mLEncB].port;
			constexpr uint32_t mLEncBPin = PinMapping::MappedPins[_mLEncB].pin;
			constexpr uint32_t mREncBPin = PinMapping::MappedPins[_mLEncB].pin;

			// Set the pin modes for Dir - Pins / Encoder - Pins
			// Left Dir
			mLDirPort->PIO_PER = mLDirPin;
			mLDirPort->PIO_OER = mLDirPin;
			mLDirPort->PIO_CODR = mLDirPin;
			
			// Right Dir
			mRDirPort->PIO_PER = mRDirPin;
			mRDirPort->PIO_OER = mRDirPin;
			mRDirPort->PIO_CODR = mRDirPin;

			// Left Encoder A
			mLEncAPort->PIO_PER = mLEncAPin;
			mLEncAPort->PIO_ODR = mLEncAPin;
			mLEncAPort->PIO_PUER = mLEncAPin;
			mLEncAPort->PIO_IER = mLEncAPin;
			mLEncAPort->PIO_AIMER = mLEncAPin;
			mLEncAPort->PIO_ESR = mLEncAPin;
			mLEncAPort->PIO_REHLSR = mLEncAPin;
			mLEncAPort->PIO_DIFSR = mLEncAPin;
			mLEncAPort->PIO_SCDR = PIO_SCDR_DIV(0);
			mLEncAPort->PIO_IFER = mLEncAPin;

			// Left Encoder B
			mLEncBPort->PIO_PER = mLEncBPin;
			mLEncBPort->PIO_ODR = mLEncBPin;
			mLEncBPort->PIO_PUER = mLEncBPin;
			mLEncBPort->PIO_DIFSR = mLEncBPin;
			mLEncBPort->PIO_SCDR = PIO_SCDR_DIV(0);
			mLEncBPort->PIO_IFER = mLEncBPin;

			// Right Encoder A
			mREncAPort->PIO_PER = mREncAPin;
			mREncAPort->PIO_ODR = mREncAPin;
			mREncAPort->PIO_PUER = mREncAPin;
			mREncAPort->PIO_IER = mREncAPin;
			mREncAPort->PIO_AIMER = mREncAPin;
			mREncAPort->PIO_ESR = mREncAPin;
			mREncAPort->PIO_REHLSR = mREncAPin;
			mREncAPort->PIO_DIFSR = mREncAPin;
			mREncAPort->PIO_SCDR = PIO_SCDR_DIV(0);
			mREncAPort->PIO_IFER = mREncAPin;

			// Right Encoder B
			mREncBPort->PIO_PER = mREncBPin;
			mREncBPort->PIO_ODR = mREncBPin;
			mREncBPort->PIO_PUER = mREncBPin;
			mREncBPort->PIO_DIFSR = mREncBPin;
			mREncBPort->PIO_SCDR = PIO_SCDR_DIV(0);
			mREncBPort->PIO_IFER = mREncBPin;

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

			// Setup interrupts for rotary encoder pins
			NVIC_EnableIRQ(static_cast<IRQn_Type>(PinMapping::MappedPins[_mLEncA].portID));
			NVIC_EnableIRQ(static_cast<IRQn_Type>(PinMapping::MappedPins[_mREncA].portID));

			NVIC_SetPriority(static_cast<IRQn_Type>(PinMapping::MappedPins[_mLEncA].portID), 5);
			NVIC_SetPriority(static_cast<IRQn_Type>(PinMapping::MappedPins[_mREncA].portID), 5);

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

		void inline calcMotorSpeed()
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