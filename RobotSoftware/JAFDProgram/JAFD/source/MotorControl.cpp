/*
This part of the Library is responsible for driving the motors.
*/

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "../../JAFDSettings.h"
#include "../header/MotorControl.h"
#include "../header/DuePinMapping.h"
#include "../header/PIDController.h"
#include "../header/Math.h"

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

			constexpr auto _cmPSToPerc = JAFDSettings::MotorControl::cmPSToPerc;	// Conversion factor from cm/s to motor PWM duty cycle

			constexpr uint8_t _lPWMCh = PinMapping::getPWMChannel(_lPWM);	// Left motor PWM channel
			constexpr uint8_t _rPWMCh = PinMapping::getPWMChannel(_rPWM);	// Right motor PWM channel

			constexpr uint8_t _lADCCh = PinMapping::getADCChannel(_lFb);	// Left motor ADC channel
			constexpr uint8_t _rADCCh = PinMapping::getADCChannel(_rFb);	// Right motor ADC channel

			PIDController _leftPID(JAFDSettings::Controller::Motor::pidSettings);		// Left speed PID-Controller
			PIDController _rightPID(JAFDSettings::Controller::Motor::pidSettings);		// Right speed PID-Controller

			volatile int32_t _lEncCnt = 0;		// Encoder count left motor
			volatile int32_t _rEncCnt = 0;		// Encoder count right motor

			volatile FloatWheelSpeeds _speeds = { 0.0f, 0.0f };	// Current motor speeds (cm/s)

			volatile WheelSpeeds _desSpeeds = { 0.0f, 0.0f };	// Desired motor speed (cm/s)
		}

		ReturnCode setup()
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
			NVIC_EnableIRQ(static_cast<IRQn_Type>(_lEncA.portID));
			NVIC_EnableIRQ(static_cast<IRQn_Type>(_rEncA.portID));

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
			static FloatWheelSpeeds lastSpeeds;


			// Calculate speeds and apply 
			_speeds.left = ((_lEncCnt - lastLeftCnt) / (JAFDSettings::MotorControl::pulsePerRev) * JAFDSettings::Mechanics::wheelDiameter * PI * freq) * 0.95f + lastSpeeds.left * 0.05f;
			_speeds.right = ((_rEncCnt - lastRightCnt) / (JAFDSettings::MotorControl::pulsePerRev) * JAFDSettings::Mechanics::wheelDiameter * PI * freq) * 0.95f + lastSpeeds.right * 0.05f;

			lastLeftCnt = _lEncCnt;
			lastRightCnt = _rEncCnt;
			lastSpeeds = static_cast<FloatWheelSpeeds>(_speeds);
		}

		void speedPID(const uint8_t freq)
		{
			static FloatWheelSpeeds setSpeed;	// Speed calculated by PID
			
			// When speed isn't 0, do PID controller
			if (_desSpeeds.left == 0)
			{
				_leftPID.reset();
				setSpeed.left = 0.0f;
			}
			else
			{
				setSpeed.left = _leftPID.process(_desSpeeds.left, _speeds.left, 1.0f / freq);

				if (setSpeed.left < JAFDSettings::MotorControl::minSpeed && setSpeed.left > -JAFDSettings::MotorControl::minSpeed) setSpeed.left = JAFDSettings::MotorControl::minSpeed * sgn(_desSpeeds.left);
				
				setSpeed.left *= _cmPSToPerc;
			}

			if (_desSpeeds.right == 0)
			{
				_rightPID.reset();
				setSpeed.right = 0.0f;
			}
			else
			{
				setSpeed.right = _rightPID.process(_desSpeeds.right, _speeds.right, 1.0f / freq);

				if (setSpeed.right < JAFDSettings::MotorControl::minSpeed && setSpeed.right > -JAFDSettings::MotorControl::minSpeed) setSpeed.right = JAFDSettings::MotorControl::minSpeed * sgn(_desSpeeds.right);
			
				setSpeed.right *= _cmPSToPerc;
			}


			// Set left dir pin
			if (setSpeed.left > 0.0f)
			{
				_lDir.port->PIO_CODR = _lDir.pin;
			}
			else
			{
				_lDir.port->PIO_SODR = _lDir.pin;
			}

			// Set right dir pin
			if (setSpeed.right > 0.0f)
			{
				_rDir.port->PIO_CODR = _rDir.pin;
			}
			else
			{
				_rDir.port->PIO_SODR = _rDir.pin;
			}

			// Set PWM Value
			PWM->PWM_CH_NUM[_lPWMCh].PWM_CDTYUPD = (PWM->PWM_CH_NUM[_lPWMCh].PWM_CPRD * abs(setSpeed.left));
			PWM->PWM_CH_NUM[_rPWMCh].PWM_CDTYUPD = (PWM->PWM_CH_NUM[_rPWMCh].PWM_CPRD * abs(setSpeed.right));
			PWM->PWM_SCUC = PWM_SCUC_UPDULOCK;
		}

		WheelSpeeds getSpeeds()
		{
			return WheelSpeeds{ static_cast<int16_t>(_speeds.left), -static_cast<int16_t>(_speeds.right) };
		}

		FloatWheelSpeeds getFloatSpeeds()
		{
			return FloatWheelSpeeds{ _speeds.left, -_speeds.right };
		}

		float getDistance(const Motor motor)
		{
			if (motor == Motor::left)
			{
				return _lEncCnt / (11.0f * 34.02f) * JAFDSettings::Mechanics::wheelDiameter * PI;
			}
			else
			{
				return _rEncCnt / (11.0f * 34.02f) * JAFDSettings::Mechanics::wheelDiameter * PI * -1;
			}
		}

		void encoderInterrupt(const Interrupts::InterruptSource source, const uint32_t isr)
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

		void setSpeeds(const WheelSpeeds wheelSpeeds)
		{
			_desSpeeds.left = wheelSpeeds.left;
			_desSpeeds.right = -wheelSpeeds.right;

			if (_desSpeeds.left < JAFDSettings::MotorControl::minSpeed && _desSpeeds.left > -JAFDSettings::MotorControl::minSpeed) _desSpeeds.left = 0;

			if (_desSpeeds.right < JAFDSettings::MotorControl::minSpeed && _desSpeeds.right > -JAFDSettings::MotorControl::minSpeed) _desSpeeds.right = 0;
		}

		float getCurrent(const Motor motor)
		{
			float result = 0.0f;

			// Sample 10 values and return average
			for (uint8_t i = 0; i < 10; i++)
			{
				// Wait for end of conversion
				while (!(ADC->ADC_ISR & ADC_ISR_DRDY));

				if (motor == Motor::left)
				{
					result += ADC->ADC_CDR[_lADCCh] * 3.3f * 1904.7619f / 4.0f / (1 << 12 - 1);
				}
				else
				{
					result += ADC->ADC_CDR[_rADCCh] * 3.3f * 1904.7619f / 4.0f / (1 << 12 - 1);
				}
			}

			return result / 10.0f;
		}
	}
}