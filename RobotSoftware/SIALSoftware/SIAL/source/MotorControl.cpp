#include "../SIALSettings.h"
#include "../header/MotorControl.h"
#include "../header/DuePinMapping.h"
#include "../header/PIDController.h"
#include "../header/Math.h"

// 40Hz
void TC5_Handler()
{
	{
		volatile auto dummy = TC1->TC_CHANNEL[2].TC_SR;
	}

	SIAL::MotorControl::calcMotorSpeed(40);
	SIAL::MotorControl::speedPID(40);
}

namespace SIAL {
	namespace MotorControl {
		namespace
		{
			constexpr auto lPWM = PinMapping::MappedPins[SIALSettings::MotorControl::Left::pwmPin];		// PWM pin left motor
			constexpr auto rPWM = PinMapping::MappedPins[SIALSettings::MotorControl::Right::pwmPin];	// PWM pin right motor

			constexpr auto lInA = PinMapping::MappedPins[SIALSettings::MotorControl::Left::inAPin];		// A input pin left motor
			constexpr auto lInB = PinMapping::MappedPins[SIALSettings::MotorControl::Left::inBPin];		// B input pin left motor

			constexpr auto rInA = PinMapping::MappedPins[SIALSettings::MotorControl::Right::inAPin];	// A input pin left motor
			constexpr auto rInB = PinMapping::MappedPins[SIALSettings::MotorControl::Right::inBPin];	// B input pin left motor

			constexpr auto lCurFb = PinMapping::MappedPins[SIALSettings::MotorControl::Left::curFbPin];		// Current sense output left motor
			constexpr auto rCurFb = PinMapping::MappedPins[SIALSettings::MotorControl::Right::curFbPin];	// Current sense output right motor

			constexpr auto lVoltFbA = PinMapping::MappedPins[SIALSettings::MotorControl::Left::voltFbPinA];	// Voltage sense output left motor / A
			constexpr auto lVoltFbB = PinMapping::MappedPins[SIALSettings::MotorControl::Left::voltFbPinB];	// Voltage sense output left motor / B

			constexpr auto rVoltFbA = PinMapping::MappedPins[SIALSettings::MotorControl::Right::voltFbPinA];	// Voltage sense output right motor / A
			constexpr auto rVoltFbB = PinMapping::MappedPins[SIALSettings::MotorControl::Right::voltFbPinB];	// Voltage sense output right motor / B

			constexpr auto lEncA = PinMapping::MappedPins[SIALSettings::MotorControl::Left::encA];		// Encoder Pin A left motor
			constexpr auto lEncB = PinMapping::MappedPins[SIALSettings::MotorControl::Left::encB];		// Encoder Pin B left motor
			constexpr auto lTC = SIALSettings::MotorControl::Left::tc;

			constexpr auto rEncA = PinMapping::MappedPins[SIALSettings::MotorControl::Right::encA];		// Encoder Pin A right motor
			constexpr auto rEncB = PinMapping::MappedPins[SIALSettings::MotorControl::Right::encB];		// Encoder Pin B right motor
			constexpr auto rTC = SIALSettings::MotorControl::Right::tc;

			constexpr auto cmPSToPerc = SIALSettings::MotorControl::cmPSToPerc;		// Conversion factor from cm/s to motor PWM duty cycle

			constexpr uint8_t lPWMCh = PinMapping::getPWMChannel(lPWM);		// Left motor PWM channel
			constexpr uint8_t rPWMCh = PinMapping::getPWMChannel(rPWM);		// Right motor PWM channel

			constexpr uint8_t lCurADCCh = PinMapping::getADCChannel(lCurFb);	// Left motor ADC channel for current measurement
			constexpr uint8_t rCurADCCh = PinMapping::getADCChannel(rCurFb);	// Right motor ADC channel for current measurement

			constexpr uint8_t lVoltADCChA = PinMapping::getADCChannel(lVoltFbA);	// Left motor ADC channel for voltage measurement / A
			constexpr uint8_t lVoltADCChB = PinMapping::getADCChannel(lVoltFbB);	// Left motor ADC channel for voltage measurement / B

			constexpr uint8_t rVoltADCChA = PinMapping::getADCChannel(rVoltFbA);	// Right motor ADC channel for voltage measurement / A
			constexpr uint8_t rVoltADCChB = PinMapping::getADCChannel(rVoltFbB);	// Right motor ADC channel for voltage measurement / B

			PIDController leftPID(SIALSettings::Controller::Motor::pidSettings);		// Left speed PID-Controller
			PIDController rightPID(SIALSettings::Controller::Motor::pidSettings);		// Right speed PID-Controller

			volatile FloatWheelSpeeds speeds = FloatWheelSpeeds{ 0.0f, 0.0f };		// Current motor speeds (cm/s)

			volatile WheelSpeeds desSpeeds = WheelSpeeds{ 0, 0 };				// Desired motor speed (cm/s)

			// Get output voltage of motor
			float getVoltage(const Motor motor)
			{
				// Wait for end of conversion
				while (!(ADC->ADC_ISR & ADC_ISR_DRDY));

				if (motor == Motor::left)
				{
					float voltA = ADC->ADC_CDR[lVoltADCChA] * 3.3f / ((1 << 12) - 1) * SIALSettings::MotorControl::voltageSensFactor;
					float voltB = ADC->ADC_CDR[lVoltADCChB] * 3.3f / ((1 << 12) - 1) * SIALSettings::MotorControl::voltageSensFactor;

					return fabsf(voltA - voltB);
				}
				else
				{
					float voltA = ADC->ADC_CDR[rVoltADCChA] * 3.3f / ((1 << 12) - 1) * SIALSettings::MotorControl::voltageSensFactor;
					float voltB = ADC->ADC_CDR[rVoltADCChB] * 3.3f / ((1 << 12) - 1) * SIALSettings::MotorControl::voltageSensFactor;

					return fabsf(voltA - voltB);
				}
			}

			int64_t getEncoderVal(const Motor motor) {
				Tc* tc;

				if (motor == Motor::left) {
					tc = lTC;
				}
				else {
					tc = rTC;
				}

				uint32_t posVal = tc->TC_CHANNEL[0].TC_CV;
				uint32_t negVal = ~((uint32_t)0) - tc->TC_CHANNEL[0].TC_CV;

				int64_t val;
				if (posVal < negVal) {
					val = (int64_t)posVal;
				}
				else {
					val = -((int64_t)negVal);
				}

				return val;
			}

			float batVoltage = 0.0f;
		}

		ReturnCode setup()
		{
			// Check if PWM Pins and ADC Pins are correct
			if (!PinMapping::hasPWM(lPWM) || !PinMapping::hasPWM(rPWM) ||
				!PinMapping::hasADC(lCurFb) || !PinMapping::hasADC(rCurFb) ||
				!PinMapping::hasADC(lVoltFbA) || !PinMapping::hasADC(lVoltFbB) ||
				!PinMapping::hasADC(lVoltFbA) || !PinMapping::hasADC(lVoltFbB))
			{
				return ReturnCode::fatalError;
			}

			// Set the pin modes for Dir - Pins / Encoder - Pins
			// Left Input A
			lInA.port->PIO_PER = lInA.pin;
			lInA.port->PIO_OER = lInA.pin;
			lInA.port->PIO_CODR = lInA.pin;

			// Right Input A
			rInA.port->PIO_PER = rInA.pin;
			rInA.port->PIO_OER = rInA.pin;
			rInA.port->PIO_CODR = rInA.pin;

			// Left Input B
			lInB.port->PIO_PER = lInB.pin;
			lInB.port->PIO_OER = lInB.pin;
			lInB.port->PIO_CODR = lInB.pin;

			// Right Input B
			rInB.port->PIO_PER = rInB.pin;
			rInB.port->PIO_OER = rInB.pin;
			rInB.port->PIO_CODR = rInB.pin;

			// Left Encoder A
			lEncA.port->PIO_PDR = lEncA.pin;
			lEncA.port->PIO_PUER = lEncA.pin;
			lEncA.port->PIO_IDR = lEncA.pin;
			lEncA.port->PIO_ABSR |= lEncA.pin;

			// Left Encoder B
			lEncB.port->PIO_PDR = lEncB.pin;
			lEncB.port->PIO_PUER = lEncB.pin;
			lEncB.port->PIO_IDR = lEncB.pin;
			lEncB.port->PIO_ABSR |= lEncB.pin;

			// Timer Counter Quadrature Decoder for left motor
			lTC->TC_CHANNEL[0].TC_CMR = TC_CMR_TCCLKS_XC0;

			lTC->TC_BMR = TC_BMR_QDEN | TC_BMR_POSEN | TC_BMR_EDGPHA | (SIALSettings::MotorControl::Left::swapTCInputs ? TC_BMR_SWAP : 0b0);
			lTC->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
			lTC->TC_CHANNEL[1].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
			lTC->TC_CHANNEL[2].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;

			// Right Encoder A
			rEncA.port->PIO_PDR = rEncA.pin;
			rEncA.port->PIO_PUER = rEncA.pin;
			rEncA.port->PIO_IDR = rEncA.pin;
			rEncA.port->PIO_ABSR |= rEncA.pin;

			// Right Encoder B
			rEncB.port->PIO_PDR = rEncB.pin;
			rEncB.port->PIO_PUER = rEncB.pin;
			rEncB.port->PIO_IDR = rEncB.pin;
			rEncB.port->PIO_ABSR |= rEncB.pin;

			// Timer Counter Quadrature Decoder for right motor
			rTC->TC_CHANNEL[0].TC_CMR = TC_CMR_TCCLKS_XC0;

			rTC->TC_BMR = TC_BMR_QDEN | TC_BMR_POSEN | TC_BMR_EDGPHA | (SIALSettings::MotorControl::Right::swapTCInputs ? TC_BMR_SWAP : 0b0);
			rTC->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
			rTC->TC_CHANNEL[1].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
			rTC->TC_CHANNEL[2].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;

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

			// Setup TC5 for an interrupt every 25ms -> 40Hz (MCK / 128 / 16406)
			TC1->TC_CHANNEL[2].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK4 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC;
			TC1->TC_CHANNEL[2].TC_RC = 16406;

			TC1->TC_CHANNEL[2].TC_IER = TC_IER_CPCS;
			TC1->TC_CHANNEL[2].TC_IDR = ~TC_IER_CPCS;

			NVIC_EnableIRQ(TC5_IRQn);

			TC1->TC_CHANNEL[2].TC_CCR = TC_CCR_SWTRG | TC_CCR_CLKEN;

			// Setup ADC (Freerunning mode / 21MHz); No Gain and Offset
			PMC->PMC_PCER1 = PMC_PCER1_PID37;

			ADC->ADC_MR = ADC_MR_FREERUN_ON | ADC_MR_PRESCAL(3) | ADC_MR_STARTUP_SUT896 | ADC_MR_SETTLING_AST5 | ADC_MR_TRACKTIM(0) | ADC_MR_TRANSFER(1);
			ADC->ADC_CHER = 1 << lCurADCCh | 1 << rCurADCCh | 1 << lVoltADCChA | 1 << rVoltADCChA | 1 << lVoltADCChB | 1 << rVoltADCChB;

			return ReturnCode::ok;
		}

		void calcMotorSpeed(uint8_t freq)
		{
			static int32_t lastLeftCnt = 0;
			static int32_t lastRightCnt = 0;
			static bool first = true;

			auto lEncCnt = getEncoderVal(Motor::left);
			auto rEncCnt = getEncoderVal(Motor::right);

			const float dt = 1.0f / freq;

			if (first) {
				speeds.left = 0.0f;
				speeds.right = 0.0f;
				first = false;

				lastLeftCnt = lEncCnt;
				lastRightCnt = rEncCnt;
				return;
			}

			// Calculate speeds
			speeds.left = ((lEncCnt - lastLeftCnt) / SIALSettings::MotorControl::pulsePerRev * SIALSettings::Mechanics::wheelDiameter * PI / dt) * SIALSettings::SensorFiltering::Encoder::IIRFac + (1.0f - SIALSettings::SensorFiltering::Encoder::IIRFac) * speeds.left;
			speeds.right = ((rEncCnt - lastRightCnt) / SIALSettings::MotorControl::pulsePerRev * SIALSettings::Mechanics::wheelDiameter * PI / dt) * SIALSettings::SensorFiltering::Encoder::IIRFac + (1.0f - SIALSettings::SensorFiltering::Encoder::IIRFac) * speeds.right;

			lastLeftCnt = lEncCnt;
			lastRightCnt = rEncCnt;
		}

		void speedPID(uint8_t freq)
		{
			const float dt = 1.0f / freq;

			FloatWheelSpeeds setSpeed;	// Speed calculated by PID

			static FloatWheelSpeeds lastPWMVal = FloatWheelSpeeds{ 0.0f, 0.0f };	// Last PWM values
			static float leftPWMReduction = SIALSettings::MotorControl::initPWMReduction;	// PWM reduction to prevent overvoltage
			static float rightPWMReduction = SIALSettings::MotorControl::initPWMReduction;	// PWM reduction to prevent overvoltage

			if ((lastPWMVal.left * leftPWMReduction > 0.1f) && (lastPWMVal.right * rightPWMReduction > 0.1f)) {
				batVoltage = (getVoltage(Motor::left) / (lastPWMVal.left * leftPWMReduction) + getVoltage(Motor::right) / (lastPWMVal.right * rightPWMReduction)) / 2.0f * 0.2f + 0.8f * batVoltage;
			}

			// Update PWM reduction factor with IIR
			if (lastPWMVal.left > 0.2)
			{
				leftPWMReduction = SIALSettings::MotorControl::pwmRedIIRFactor * (6.0f / getVoltage(Motor::left) * lastPWMVal.left * leftPWMReduction) + (1.0f - SIALSettings::MotorControl::pwmRedIIRFactor) * leftPWMReduction;
			}

			if (lastPWMVal.right > 0.2)
			{
				leftPWMReduction = SIALSettings::MotorControl::pwmRedIIRFactor * (6.0f / getVoltage(Motor::right) * lastPWMVal.right * rightPWMReduction) + (1.0f - SIALSettings::MotorControl::pwmRedIIRFactor) * rightPWMReduction;
			}

			if (leftPWMReduction > 2.0f * SIALSettings::MotorControl::initPWMReduction) leftPWMReduction = SIALSettings::MotorControl::initPWMReduction;
			if (rightPWMReduction > 2.0f * SIALSettings::MotorControl::initPWMReduction) rightPWMReduction = SIALSettings::MotorControl::initPWMReduction;

			// When speed isn't 0, do PID controller
			if (desSpeeds.left == 0)
			{
				leftPID.reset();
				setSpeed.left = 0.0f;
				lastPWMVal.left = 0.0f;
				lInA.port->PIO_SODR = lInA.pin;
				lInB.port->PIO_SODR = lInB.pin;
				PWM->PWM_CH_NUM[lPWMCh].PWM_CDTYUPD = 0;
			}
			else
			{
				setSpeed.left = leftPID.process(desSpeeds.left, speeds.left, dt);

				if (setSpeed.left < SIALSettings::MotorControl::minSpeed && setSpeed.left > -SIALSettings::MotorControl::minSpeed) setSpeed.left = SIALSettings::MotorControl::minSpeed * sgn(desSpeeds.left);

				setSpeed.left *= cmPSToPerc;

				if (setSpeed.left >= 1.0f) setSpeed.left = 1.0f;
				else if (setSpeed.left <= -1.0f) setSpeed.left = -1.0f;

				// Set direction of left motor
				if (setSpeed.left < 0.0f)
				{
					lInA.port->PIO_SODR = lInA.pin;
					lInB.port->PIO_CODR = lInB.pin;
				}
				else
				{
					lInA.port->PIO_CODR = lInA.pin;
					lInB.port->PIO_SODR = lInB.pin;
				}

				lastPWMVal.left = fabsf(setSpeed.left);

				setSpeed.left *= leftPWMReduction;

				PWM->PWM_CH_NUM[lPWMCh].PWM_CDTYUPD = (uint16_t)(PWM->PWM_CH_NUM[lPWMCh].PWM_CPRD * fabsf(setSpeed.left));
			}

			if (desSpeeds.right == 0)
			{
				rightPID.reset();
				setSpeed.right = 0.0f;
				lastPWMVal.right = 0.0f;
				rInA.port->PIO_SODR = rInA.pin;
				rInB.port->PIO_SODR = rInB.pin;
				PWM->PWM_CH_NUM[rPWMCh].PWM_CDTYUPD = 0;
			}
			else
			{
				setSpeed.right = rightPID.process(desSpeeds.right, speeds.right, dt);

				if (setSpeed.right < SIALSettings::MotorControl::minSpeed && setSpeed.right > -SIALSettings::MotorControl::minSpeed) setSpeed.right = SIALSettings::MotorControl::minSpeed * sgn(desSpeeds.right);

				setSpeed.right *= cmPSToPerc;

				if (setSpeed.right >= 1.0f) setSpeed.right = 1.0f;
				else if (setSpeed.right <= -1.0f) setSpeed.right = -1.0f;

				// Set direction of right motor
				if (setSpeed.right < 0.0f)
				{
					rInA.port->PIO_CODR = rInA.pin;
					rInB.port->PIO_SODR = rInB.pin;
				}
				else
				{
					rInA.port->PIO_SODR = rInA.pin;
					rInB.port->PIO_CODR = rInB.pin;
				}

				lastPWMVal.right = fabsf(setSpeed.right);

				setSpeed.right *= rightPWMReduction;

				PWM->PWM_CH_NUM[rPWMCh].PWM_CDTYUPD = (uint16_t)(PWM->PWM_CH_NUM[rPWMCh].PWM_CPRD * fabsf(setSpeed.right));
			}

			PWM->PWM_SCUC = PWM_SCUC_UPDULOCK;
		}

		WheelSpeeds getSpeeds()
		{
			return WheelSpeeds{ static_cast<int16_t>(speeds.left), static_cast<int16_t>(speeds.right) };
		}

		FloatWheelSpeeds getFloatSpeeds()
		{
			return FloatWheelSpeeds{ speeds.left, speeds.right };
		}

		float getDistance(const Motor motor)
		{
			if (motor == Motor::left)
			{
				auto lEncCnt = getEncoderVal(Motor::left);
				return lEncCnt / SIALSettings::MotorControl::pulsePerRev * SIALSettings::Mechanics::wheelDiameter * PI;
			}
			else
			{
				auto rEncCnt = getEncoderVal(Motor::right);
				return rEncCnt / SIALSettings::MotorControl::pulsePerRev * SIALSettings::Mechanics::wheelDiameter * PI;
			}
		}

		void setSpeeds(const WheelSpeeds wheelSpeeds)
		{
			// TESTING
			// Serial.println(batVoltage);

			desSpeeds.left = wheelSpeeds.left;
			desSpeeds.right = wheelSpeeds.right;

			if (desSpeeds.left < SIALSettings::MotorControl::minSpeed && desSpeeds.left > -SIALSettings::MotorControl::minSpeed) desSpeeds.left = 0;

			if (desSpeeds.right < SIALSettings::MotorControl::minSpeed && desSpeeds.right > -SIALSettings::MotorControl::minSpeed) desSpeeds.right = 0;
		}

		float getCurrent(const Motor motor)
		{
			float result = 0.0f;

			// Sample values and return average
			for (uint8_t i = 0; i < SIALSettings::MotorControl::currentADCSampleCount; i++)
			{
				// Wait for end of conversion
				while (!(ADC->ADC_ISR & ADC_ISR_DRDY));

				if (motor == Motor::left)
				{
					result += ADC->ADC_CDR[lCurADCCh] * 3.3f / (1 << 12 - 1) * SIALSettings::MotorControl::currentSensFactor;
				}
				else
				{
					result += ADC->ADC_CDR[rCurADCCh] * 3.3f / (1 << 12 - 1) * SIALSettings::MotorControl::currentSensFactor;
				}
			}

			return result / SIALSettings::MotorControl::currentADCSampleCount;
		}
	}
}