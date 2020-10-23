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
			constexpr auto lPWM = PinMapping::MappedPins[JAFDSettings::MotorControl::Left::pwmPin];		// PWM pin left motor
			constexpr auto rPWM = PinMapping::MappedPins[JAFDSettings::MotorControl::Right::pwmPin];	// PWM pin right motor

			constexpr auto lDir = PinMapping::MappedPins[JAFDSettings::MotorControl::Left::dirPin];		// Direction pin left motor
			constexpr auto rDir = PinMapping::MappedPins[JAFDSettings::MotorControl::Right::dirPin];	// Direction pin right motor

			constexpr auto lCurFb = PinMapping::MappedPins[JAFDSettings::MotorControl::Left::curFbPin];		// Current sense output left motor
			constexpr auto rCurFb = PinMapping::MappedPins[JAFDSettings::MotorControl::Right::curFbPin];	// Current sense output right motor
			
			constexpr auto lVoltFb = PinMapping::MappedPins[JAFDSettings::MotorControl::Left::voltFbPin];		// Voltage sense output left motor
			constexpr auto rVoltFb = PinMapping::MappedPins[JAFDSettings::MotorControl::Right::voltFbPin];		// Voltage sense output right motor

			constexpr auto lEncA = PinMapping::MappedPins[JAFDSettings::MotorControl::Left::encA];		// Encoder Pin A left motor
			constexpr auto lEncB = PinMapping::MappedPins[JAFDSettings::MotorControl::Left::encB];		// Encoder Pin B left motor

			constexpr auto rEncA = PinMapping::MappedPins[JAFDSettings::MotorControl::Right::encA];		// Encoder Pin A right motor
			constexpr auto rEncB = PinMapping::MappedPins[JAFDSettings::MotorControl::Right::encB];		// Encoder Pin B right motor

			constexpr auto cmPSToPerc = JAFDSettings::MotorControl::cmPSToPerc;		// Conversion factor from cm/s to motor PWM duty cycle

			constexpr uint8_t lPWMCh = PinMapping::getPWMChannel(lPWM);		// Left motor PWM channel
			constexpr uint8_t rPWMCh = PinMapping::getPWMChannel(rPWM);		// Right motor PWM channel

			constexpr uint8_t lCurADCCh = PinMapping::getADCChannel(lCurFv);	// Left motor ADC channel for current measurement
			constexpr uint8_t rCurADCCh = PinMapping::getADCChannel(rCurFv);	// Right motor ADC channel for current measurement

			constexpr uint8_t lVoltADCCh = PinMapping::getADCChannel(lVoltFv);	// Left motor ADC channel for voltage measurement
			constexpr uint8_t rVoltADCCh = PinMapping::getADCChannel(rVoltFv);	// Right motor ADC channel for voltage measurement

			PIDController leftPID(JAFDSettings::Controller::Motor::pidSettings);		// Left speed PID-Controller
			PIDController rightPID(JAFDSettings::Controller::Motor::pidSettings);		// Right speed PID-Controller

			volatile int32_t lEncCnt = 0;		// Encoder count left motor
			volatile int32_t rEncCnt = 0;		// Encoder count right motor

			volatile FloatWheelSpeeds speeds = { 0.0f, 0.0f };	// Current motor speeds (cm/s)

			volatile WheelSpeeds desSpeeds = { 0.0f, 0.0f };	// Desired motor speed (cm/s)

			// Get output voltage of motor
			float getVoltage(const Motor motor)
			{
				// Wait for end of conversion
				while (!(ADC->ADC_ISR & ADC_ISR_DRDY));

				if (motor == Motor::left)
				{
					return ADC->ADC_CDR[lVoltADCCh] * 3.3f / (1 << 12 - 1) * JAFDSettings::MotorControl::voltageSensFactor;
				}
				else
				{
					ADC->ADC_CDR[rVoltADCCh] * 3.3f / (1 << 12 - 1) * JAFDSettings::MotorControl::voltageSensFactor;
				}
			}
		}

		ReturnCode setup()
		{
			// Check if PWM Pins and ADC Pins are correct
			if (!PinMapping::hasPWM(lPWM) || !PinMapping::hasPWM(rPWM) || !PinMapping::hasADC(lCurFv) || !PinMapping::hasADC(rCurFv))
			{
				return ReturnCode::fatalError;
			}

			// Set the pin modes for Dir - Pins / Encoder - Pins
			// Left Dir
			lDir.port->PIO_PER = lDir.pin;
			lDir.port->PIO_OER = lDir.pin;
			lDir.port->PIO_CODR = lDir.pin;
			
			// Right Dir
			rDir.port->PIO_PER = rDir.pin;
			rDir.port->PIO_OER = rDir.pin;
			rDir.port->PIO_CODR = rDir.pin;

			// Left Encoder A
			lEncA.port->PIO_PER = lEncA.pin;
			lEncA.port->PIO_ODR = lEncA.pin;
			lEncA.port->PIO_PUER = lEncA.pin;
			lEncA.port->PIO_IER = lEncA.pin;
			lEncA.port->PIO_AIMER = lEncA.pin;
			lEncA.port->PIO_ESR = lEncA.pin;
			lEncA.port->PIO_REHLSR = lEncA.pin;
			lEncA.port->PIO_DIFSR = lEncA.pin;
			lEncA.port->PIO_SCDR = PIO_SCDR_DIV(0);
			lEncA.port->PIO_IFER = lEncA.pin;

			// Left Encoder B
			lEncB.port->PIO_PER = lEncB.pin;
			lEncB.port->PIO_ODR = lEncB.pin;
			lEncB.port->PIO_PUER = lEncB.pin;
			lEncB.port->PIO_DIFSR = lEncB.pin;
			lEncB.port->PIO_SCDR = PIO_SCDR_DIV(0);
			lEncB.port->PIO_IFER = lEncB.pin;

			// Right Encoder A
			rEncA.port->PIO_PER = rEncA.pin;
			rEncA.port->PIO_ODR = rEncA.pin;
			rEncA.port->PIO_PUER = rEncA.pin;
			rEncA.port->PIO_IER = rEncA.pin;
			rEncA.port->PIO_AIMER = rEncA.pin;
			rEncA.port->PIO_ESR = rEncA.pin;
			rEncA.port->PIO_REHLSR = rEncA.pin;
			rEncA.port->PIO_DIFSR = rEncA.pin;
			rEncA.port->PIO_SCDR = PIO_SCDR_DIV(0);
			rEncA.port->PIO_IFER = rEncA.pin;

			// Right Encoder B
			rEncB.port->PIO_PER = rEncB.pin;
			rEncB.port->PIO_ODR = rEncB.pin;
			rEncB.port->PIO_PUER = rEncB.pin;
			rEncB.port->PIO_DIFSR = rEncB.pin;
			rEncB.port->PIO_SCDR = PIO_SCDR_DIV(0);
			rEncB.port->PIO_IFER = rEncB.pin;

			// Setup PWM - Controller (20kHz)
			PWM->PWM_ENA = 1 << lPWMCh | 1 << rPWMCh;

			PWM->PWM_CH_NUM[lPWMCh].PWM_CMR = PWM_CMR_CPRE_CLKA;
			PWM->PWM_CH_NUM[lPWMCh].PWM_CPRD = 4200;
			PWM->PWM_CH_NUM[lPWMCh].PWM_CDTY = 0;

			if (PinMapping::getPWMStartState(lPWM) == PinMapping::PWMStartState::high)
			{
				PWM->PWM_CH_NUM[lPWMCh].PWM_CMR |= PWM_CMR_CPOL;
			}

			PWM->PWM_CH_NUM[rPWMCh].PWM_CMR = PWM_CMR_CPRE_CLKA;
			PWM->PWM_CH_NUM[rPWMCh].PWM_CPRD = 4200;
			PWM->PWM_CH_NUM[rPWMCh].PWM_CDTY = 0;

			if (PinMapping::getPWMStartState(rPWM) == PinMapping::PWMStartState::high)
			{
				PWM->PWM_CH_NUM[rPWMCh].PWM_CMR |= PWM_CMR_CPOL;
			}

			lPWM.port->PIO_PDR = lPWM.pin;
			rPWM.port->PIO_PDR = rPWM.pin;

			if (PinMapping::toABPeripheral(lPWM))
			{
				lPWM.port->PIO_ABSR |= lPWM.pin;
			}
			else
			{
				lPWM.port->PIO_ABSR &= ~lPWM.pin;
			}

			if (PinMapping::toABPeripheral(rPWM))
			{
				rPWM.port->PIO_ABSR |= rPWM.pin;
			}
			else
			{
				rPWM.port->PIO_ABSR &= ~rPWM.pin;
			}

			// Setup ADC (Freerunning mode / 21MHz); No Gain and Offset
			PMC->PMC_PCER1 = PMC_PCER1_PID37;

			ADC->ADC_MR = ADC_MR_FREERUN_ON | ADC_MR_PRESCAL(3) | ADC_MR_STARTUP_SUT896 | ADC_MR_SETTLING_AST5 | ADC_MR_TRACKTIM(0) | ADC_MR_TRANSFER(1);
			ADC->ADC_CHER = 1 << lCurADCCh | 1 << rCurADCCh | 1 << lVoltADCCh | 1 << rVoltADCCh;

			return ReturnCode::ok;
		}

		void calcMotorSpeed(const uint8_t freq)
		{
			static int32_t lastLeftCnt = 0;
			static int32_t lastRightCnt = 0;

			// Calculate speeds
			speeds.left = ((lEncCnt - lastLeftCnt) / (JAFDSettings::MotorControl::pulsePerRev) * JAFDSettings::Mechanics::wheelDiameter * PI * freq);
			speeds.right = ((rEncCnt - lastRightCnt) / (JAFDSettings::MotorControl::pulsePerRev) * JAFDSettings::Mechanics::wheelDiameter * PI * freq);

			lastLeftCnt = lEncCnt;
			lastRightCnt = rEncCnt;
		}

		void speedPID(const uint8_t freq)
		{
			FloatWheelSpeeds setSpeed;	// Speed calculated by PID

			static FloatWheelSpeeds lastPWMVal = { 0.0f, 0.0f };	// Last PWM values
			static float leftPWMReduction = JAFDSettings::MotorControl::initPWMReduction;	// PWM reduction to prevent overvoltage
			static float rightPWMReduction = JAFDSettings::MotorControl::initPWMReduction;	// PWM reduction to prevent overvoltage

			// Update PWM reduction factor with IIR
			leftPWMReduction = JAFDSettings::MotorControl::pwmRedIIRFactor * (lastPWMVal.left * 6.0f / getVoltage(Motor::left)) + (1 - JAFDSettings::MotorControl::pwmRedIIRFactor) * leftPWMReduction;
			rightPWMReduction = JAFDSettings::MotorControl::pwmRedIIRFactor * (lastPWMVal.right * 6.0f / getVoltage(Motor::right)) + (1 - JAFDSettings::MotorControl::pwmRedIIRFactor) * rightPWMReduction;

			// When speed isn't 0, do PID controller
			if (desSpeeds.left == 0)
			{
				leftPID.reset();
				setSpeed.left = 0.0f;
			}
			else
			{
				setSpeed.left = leftPID.process(desSpeeds.left, speeds.left, 1.0f / freq);

				if (setSpeed.left < JAFDSettings::MotorControl::minSpeed && setSpeed.left > -JAFDSettings::MotorControl::minSpeed) setSpeed.left = JAFDSettings::MotorControl::minSpeed * sgn(desSpeeds.left);
				
				setSpeed.left *= cmPSToPerc;
			}

			if (desSpeeds.right == 0)
			{
				rightPID.reset();
				setSpeed.right = 0.0f;
			}
			else
			{
				setSpeed.right = rightPID.process(desSpeeds.right, speeds.right, 1.0f / freq);

				if (setSpeed.right < JAFDSettings::MotorControl::minSpeed && setSpeed.right > -JAFDSettings::MotorControl::minSpeed) setSpeed.right = JAFDSettings::MotorControl::minSpeed * sgn(desSpeeds.right);
			
				setSpeed.right *= cmPSToPerc;
			}

			// Set left dir pin
			if (setSpeed.left > 0.0f)
			{
				lDir.port->PIO_CODR = lDir.pin;
			}
			else
			{
				lDir.port->PIO_SODR = lDir.pin;
			}

			// Set right dir pin
			if (setSpeed.right > 0.0f)
			{
				rDir.port->PIO_CODR = rDir.pin;
			}
			else
			{
				rDir.port->PIO_SODR = rDir.pin;
			}

			// Reduce PWM values
			setSpeed.left *= leftPWMReduction;
			setSpeed.right *= rightPWMReduction;

			// Update last PWM values
			lastPWMVal = setSpeed;

			// Set PWM Value
			PWM->PWM_CH_NUM[lPWMCh].PWM_CDTYUPD = (PWM->PWM_CH_NUM[lPWMCh].PWM_CPRD * fabs(setSpeed.left));
			PWM->PWM_CH_NUM[rPWMCh].PWM_CDTYUPD = (PWM->PWM_CH_NUM[rPWMCh].PWM_CPRD * fabs(setSpeed.right));
			PWM->PWM_SCUC = PWM_SCUC_UPDULOCK;
		}

		WheelSpeeds getSpeeds()
		{
			return WheelSpeeds{ static_cast<int16_t>(speeds.left), -static_cast<int16_t>(speeds.right) };
		}

		FloatWheelSpeeds getFloatSpeeds()
		{
			return FloatWheelSpeeds{ speeds.left, -speeds.right };
		}

		float getDistance(const Motor motor)
		{
			if (motor == Motor::left)
			{
				return lEncCnt / JAFDSettings::MotorControl::pulsePerRev * JAFDSettings::Mechanics::wheelDiameter * PI;
			}
			else
			{
				return rEncCnt / JAFDSettings::MotorControl::pulsePerRev * JAFDSettings::Mechanics::wheelDiameter * PI * -1;
			}
		}

		bool encoderInterrupt(const Interrupts::InterruptSource source, const uint32_t isr)
		{
			if (lEncA.portID == static_cast<uint8_t>(source) && (isr & lEncA.pin))
			{
				if (lEncB.port->PIO_PDSR & lEncB.pin)
				{
					lEncCnt--;
				}
				else
				{
					lEncCnt++;
				}

				return true;
			}
			else if (rEncA.portID == static_cast<uint8_t>(source) && (isr & rEncA.pin))
			{
				if (rEncB.port->PIO_PDSR & rEncB.pin)
				{
					rEncCnt--;
				}
				else
				{
					rEncCnt++;
				}

				return true;
			}
			else
			{
				return false;
			}
		}

		void setSpeeds(const WheelSpeeds wheelSpeeds)
		{
			desSpeeds.left = wheelSpeeds.left;
			desSpeeds.right = -wheelSpeeds.right;

			if (desSpeeds.left < JAFDSettings::MotorControl::minSpeed && desSpeeds.left > -JAFDSettings::MotorControl::minSpeed) desSpeeds.left = 0;

			if (desSpeeds.right < JAFDSettings::MotorControl::minSpeed && desSpeeds.right > -JAFDSettings::MotorControl::minSpeed) desSpeeds.right = 0;
		}

		float getCurrent(const Motor motor)
		{
			float result = 0.0f;

			// Sample values and return average
			for (uint8_t i = 0; i < JAFDSettings::MotorControl::currentADCSampleCount; i++)
			{
				// Wait for end of conversion
				while (!(ADC->ADC_ISR & ADC_ISR_DRDY));

				if (motor == Motor::left)
				{
					result += ADC->ADC_CDR[lCurADCCh] * 3.3f / (1 << 12 - 1) * JAFDSettings::MotorControl::currentSensFactor;
				}
				else
				{
					result += ADC->ADC_CDR[rCurADCCh] * 3.3f / (1 << 12 - 1) * JAFDSettings::MotorControl::currentSensFactor;
				}
			}

			return result / JAFDSettings::MotorControl::currentADCSampleCount;
		}
	}
}