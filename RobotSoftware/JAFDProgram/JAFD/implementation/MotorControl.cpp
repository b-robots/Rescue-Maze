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
#include "../utility/DuePinMapping_private.h"

namespace JAFD
{
	namespace MotorControl
	{
		namespace
		{
			constexpr auto _lPWM = PinMapping::MappedPins[JAFDSettings::MotorControl::Left::pwmPin];	// PWM pin left motor
			constexpr auto _rPWM = PinMapping::MappedPins[JAFDSettings::MotorControl::Right::pwmPin];	// PWM pin right motor

			constexpr auto _lDir = PinMapping::MappedPins[JAFDSettings::MotorControl::Left::dirPin];	// Direction pin left motor
			constexpr auto _rDir = PinMapping::MappedPins[JAFDSettings::MotorControl::Right::dirPin];	// Direction pin right motor

			constexpr auto _lFb = PinMapping::MappedPins[JAFDSettings::MotorControl::Left::fbPin];		// Current sense output left motor
			constexpr auto _rFb = PinMapping::MappedPins[JAFDSettings::MotorControl::Right::fbPin];		// Current sense output right motor
			
			constexpr auto _lEncA = PinMapping::MappedPins[JAFDSettings::MotorControl::Left::encA];		// Encoder Pin A left motor
			constexpr auto _lEncB = PinMapping::MappedPins[JAFDSettings::MotorControl::Left::encB];		// Encoder Pin B left motor

			constexpr auto _rEncA = PinMapping::MappedPins[JAFDSettings::MotorControl::Right::encA];	// Encoder Pin A right motor
			constexpr auto _rEncB = PinMapping::MappedPins[JAFDSettings::MotorControl::Right::encB];	// Encoder Pin B right motor
			
			constexpr auto _kp = JAFDSettings::MotorControl::kp;	// Kp factor for PID speed controller
			constexpr auto _ki = JAFDSettings::MotorControl::ki;	// Ki factor for PID speed controller
			constexpr auto _kd = JAFDSettings::MotorControl::kd;	// Kd factor for PID speed controller

			constexpr uint8_t _lPWMCh = PinMapping::getPWMChannel(_lPWM);	// Left motor PWM channel
			constexpr uint8_t _rPWMCh = PinMapping::getPWMChannel(_rPWM);	// Right motor PWM channel

			constexpr uint8_t _lADCCh = PinMapping::getADCChannel(_lFb);	// Left motor ADC channel
			constexpr uint8_t _rADCCh = PinMapping::getADCChannel(_rFb);	// Right motor ADC channel

			volatile int32_t _lEncCnt = 0;		// Encoder count left motor
			volatile int32_t _rEncCnt = 0;		// Encoder count right motor

			volatile float _lSpeed = 0.0f;		// Speed left motor (cm/s)
			volatile float _rSpeed = 0.0f;		// Speed right motor (cm/s)
		}

		ReturnCode motorControlSetup()
		{
			// Check if PWM Pins and ADC Pins are correct
			if (!PinMapping::hasPWM(_lPWM) || !PinMapping::hasPWM(_rPWM) || !PinMapping::hasADC(_lFb) || !PinMapping::hasADC(_rFb))
			{
				return ReturnCode::fatalError;
			}

			// Set the pin modes for Dir - Pins / Encoder - Pins
			// Left Dir
			_lDir.port->PIO_PER = _lDir.pin;
			_lDir.port->PIO_OER = _lDir.pin;
			_lDir.port->PIO_CODR = _lDir.pin;
			
			// Right Dir
			_rDir.port->PIO_PER = _rDir.pin;
			_rDir.port->PIO_OER = _rDir.pin;
			_rDir.port->PIO_CODR = _rDir.pin;

			// Left Encoder A
			_lEncA.port->PIO_PER = _lEncA.pin;
			_lEncA.port->PIO_ODR = _lEncA.pin;
			_lEncA.port->PIO_PUER = _lEncA.pin;
			_lEncA.port->PIO_IER = _lEncA.pin;
			_lEncA.port->PIO_AIMER = _lEncA.pin;
			_lEncA.port->PIO_ESR = _lEncA.pin;
			_lEncA.port->PIO_REHLSR = _lEncA.pin;
			_lEncA.port->PIO_DIFSR = _lEncA.pin;
			_lEncA.port->PIO_SCDR = PIO_SCDR_DIV(0);
			_lEncA.port->PIO_IFER = _lEncA.pin;

			// Left Encoder B
			_lEncB.port->PIO_PER = _lEncB.pin;
			_lEncB.port->PIO_ODR = _lEncB.pin;
			_lEncB.port->PIO_PUER = _lEncB.pin;
			_lEncB.port->PIO_DIFSR = _lEncB.pin;
			_lEncB.port->PIO_SCDR = PIO_SCDR_DIV(0);
			_lEncB.port->PIO_IFER = _lEncB.pin;

			// Right Encoder A
			_rEncA.port->PIO_PER = _rEncA.pin;
			_rEncA.port->PIO_ODR = _rEncA.pin;
			_rEncA.port->PIO_PUER = _rEncA.pin;
			_rEncA.port->PIO_IER = _rEncA.pin;
			_rEncA.port->PIO_AIMER = _rEncA.pin;
			_rEncA.port->PIO_ESR = _rEncA.pin;
			_rEncA.port->PIO_REHLSR = _rEncA.pin;
			_rEncA.port->PIO_DIFSR = _rEncA.pin;
			_rEncA.port->PIO_SCDR = PIO_SCDR_DIV(0);
			_rEncA.port->PIO_IFER = _rEncA.pin;

			// Right Encoder B
			_rEncB.port->PIO_PER = _rEncB.pin;
			_rEncB.port->PIO_ODR = _rEncB.pin;
			_rEncB.port->PIO_PUER = _rEncB.pin;
			_rEncB.port->PIO_DIFSR = _rEncB.pin;
			_rEncB.port->PIO_SCDR = PIO_SCDR_DIV(0);
			_rEncB.port->PIO_IFER = _rEncB.pin;

			// Discard first interrupt
			_rEncA.port->PIO_ISR;
			_lEncA.port->PIO_ISR;

			// Setup interrupts for rotary encoder pins
			// ISR Priority = 4
			NVIC_EnableIRQ(static_cast<IRQn_Type>(_lEncA.portID));
			NVIC_EnableIRQ(static_cast<IRQn_Type>(_rEncA.portID));

			NVIC_SetPriority(static_cast<IRQn_Type>(_lEncA.portID), 4);
			NVIC_SetPriority(static_cast<IRQn_Type>(_rEncA.portID), 4);

			// Setup PWM - Controller (20kHz)
			PMC->PMC_PCER1 = PMC_PCER1_PID36;

			PWM->PWM_CLK = PWM_CLK_PREA(0) | PWM_CLK_DIVA(1);
			PWM->PWM_ENA = 1 << _lPWMCh | 1 << _rPWMCh;

			PWM->PWM_CH_NUM[_lPWMCh].PWM_CMR = PWM_CMR_CPRE_CLKA;
			PWM->PWM_CH_NUM[_lPWMCh].PWM_CPRD = 4200;
			PWM->PWM_CH_NUM[_lPWMCh].PWM_CDTY = 0;

			PWM->PWM_CH_NUM[_rPWMCh].PWM_CMR = PWM_CMR_CPRE_CLKA;
			PWM->PWM_CH_NUM[_rPWMCh].PWM_CPRD = 4200;
			PWM->PWM_CH_NUM[_rPWMCh].PWM_CDTY = 0;

			_lPWM.port->PIO_PDR = _lPWM.pin;
			_rPWM.port->PIO_PDR = _rPWM.pin;

			if (PinMapping::toABPeripheral(_lPWM))
			{
				_lPWM.port->PIO_ABSR |= _lPWM.pin;
			}
			else
			{
				_lPWM.port->PIO_ABSR &= ~_lPWM.pin;
			}

			if (PinMapping::toABPeripheral(_rPWM))
			{
				_rPWM.port->PIO_ABSR |= _rPWM.pin;
			}
			else
			{
				_rPWM.port->PIO_ABSR &= ~_rPWM.pin;
			}

			// Setup ADC (Freerunning mode / 21MHz)
			PMC->PMC_PCER1 = PMC_PCER1_PID37;

			ADC->ADC_MR = ADC_MR_FREERUN_ON | ADC_MR_PRESCAL(3) | ADC_MR_STARTUP_SUT896 | ADC_MR_SETTLING_AST5 | ADC_MR_TRACKTIM(0) | ADC_MR_TRANSFER(1);
			ADC->ADC_CGR = ADC_CGR_GAIN0(0b11);
			ADC->ADC_CHER = 1 << _lADCCh | 1 << _rADCCh;

			return ReturnCode::ok;
		}

		void calcMotorSpeed(const uint8_t freq)
		{
			static int32_t lastLeftCnt = 0;
			static int32_t lastRightCnt = 0;

			_lSpeed = (_lEncCnt - lastLeftCnt) / (11.0f * 34.02f) * JAFDSettings::Mechanics::wheelDiameter * PI * freq;
			_rSpeed = (_rEncCnt - lastRightCnt) / (11.0f * 34.02f) * JAFDSettings::Mechanics::wheelDiameter * PI * freq;

			lastLeftCnt = _lEncCnt;
			lastRightCnt = _rEncCnt;
		}

		void speedPID(const uint8_t dt)
		{
			static float _llastVel = 0.0f;
			static float _lVelSum = 0.0f;

			static float _rlastVel = 0.0f;
			static float _rVelSum = 0.0f;
		}

		float getVelocity(Motor motor)
		{
			if (motor == Motor::left)
			{
				return _lSpeed;
			}
			else
			{
				return _rSpeed;
			}
		}

		float getDistance(Motor motor)
		{
			if (motor == Motor::left)
			{
				return _lEncCnt / (11.0f * 34.02f) * JAFDSettings::Mechanics::wheelDiameter * PI;
			}
			else
			{
				return _rEncCnt / (11.0f * 34.02f) * JAFDSettings::Mechanics::wheelDiameter * PI;
			}
		}

		void encoderInterrupt(Interrupts::InterruptSource source, uint32_t isr)
		{
			if (_lEncA.portID == static_cast<uint8_t>(source) && (isr & _lEncA.pin))
			{
				if (_lEncB.port->PIO_PDSR & _lEncB.pin)
				{
					_lEncCnt++;
				}
				else
				{
					_lEncCnt--;
				}
			}

			if (_rEncA.portID == static_cast<uint8_t>(source) && (isr & _rEncA.pin))
			{
				if (_rEncB.port->PIO_PDSR & _rEncB.pin)
				{
					_rEncCnt++;
				}
				else
				{
					_rEncCnt--;
				}
			}
		}

		void setSpeed(Motor motor, float speed)
		{
			uint8_t pwmCh;

			if (motor == Motor::left)
			{
				pwmCh = PinMapping::getPWMChannel(_lPWM);

				// Set Dir Pin
				if (speed > 0)
				{
					_lDir.port->PIO_CODR = _lDir.pin;
				}
				else
				{
					_lDir.port->PIO_SODR = _lDir.pin;
				}
			}
			else
			{
				pwmCh = PinMapping::getPWMChannel(_rPWM);

				// Set Dir Pin
				if (speed > 0)
				{
					_rDir.port->PIO_CODR = _rDir.pin;
				}
				else
				{
					_rDir.port->PIO_SODR = _rDir.pin;
				}
			}

			// Set PWM Value
			PWM->PWM_CH_NUM[pwmCh].PWM_CDTYUPD = (PWM->PWM_CH_NUM[pwmCh].PWM_CPRD * abs(speed));
			PWM->PWM_SCUC = PWM_SCUC_UPDULOCK;
		}

		float getCurrent(Motor motor)
		{
			float result = 0.0f;

			// Sample 10 values and return average
			for (uint8_t i = 0; i < 10; i++)
			{
				// Wait for end of conversion
				while (!(ADC->ADC_ISR & ADC_ISR_DRDY));

				if (motor == Motor::left)
				{
					result += (float)ADC->ADC_CDR[_lADCCh] * 3.3f * 1904.7619f / 4.0f / (float)(1 << 12 - 1);
				}
				else
				{
					result += (float)ADC->ADC_CDR[_rADCCh] * 3.3f * 1904.7619f / 4.0f / (float)(1 << 12 - 1);
				}
			}

			return result / 10.0f;
		}
	}
}