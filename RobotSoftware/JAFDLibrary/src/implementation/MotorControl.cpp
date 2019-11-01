/*
This part of the Library is responsible for driving the motors.
*/

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "MotorControl_private.h"
#include "../utility/DuePinMapping_private.h"

namespace JAFD
{
	namespace MotorControl
	{
		namespace
		{
			uint8_t _m1PWM; // PWM pin motor 1
			uint8_t _m2PWM; // PWM pin motor 2

			uint8_t _m1Dir; // Direction pin motor 1
			uint8_t _m2Dir; // Direction pin motor 2

			uint8_t _m1Fb; // Current sense output motor 1
			uint8_t _m2Fb; // Current sense output motor 2
		}

		ReturnCode motorControlSetup(MotorControlSettings settings)
		{
			_m1PWM = settings.m1PWM;
			_m2PWM = settings.m2PWM;
			_m1Dir = settings.m1Dir;
			_m2Dir = settings.m2Dir;
			_m1Fb = settings.m1Fb;
			_m2Fb = settings.m2Fb;

			// Check if PWM Pins and ADC Pins are correct
			if (!PinMapping::hasPWM(_m1PWM) || !PinMapping::hasPWM(_m2PWM) || !PinMapping::hasADC(_m1Fb) || !PinMapping::hasADC(_m2Fb))
			{
				return ReturnCode::fatalError;
			}

			// Set the pin modes for Dir - Pins
			PMC->PMC_PCER0 = 1 << PinMapping::MappedPins[_m1Dir].portID | 1 << PinMapping::MappedPins[_m2Dir].portID;
			
			PinMapping::MappedPins[_m1Dir].port->PIO_PER = PinMapping::MappedPins[_m1Dir].pin;
			PinMapping::MappedPins[_m1Dir].port->PIO_OER = PinMapping::MappedPins[_m1Dir].pin;
			PinMapping::MappedPins[_m1Dir].port->PIO_CODR = PinMapping::MappedPins[_m1Dir].pin;
			
			PinMapping::MappedPins[_m2Dir].port->PIO_PER = PinMapping::MappedPins[_m2Dir].pin;
			PinMapping::MappedPins[_m2Dir].port->PIO_OER = PinMapping::MappedPins[_m2Dir].pin;
			PinMapping::MappedPins[_m2Dir].port->PIO_CODR = PinMapping::MappedPins[_m2Dir].pin;

			// Setup PWM - Controller (20kHz)
			const uint8_t m1PWMCh = PinMapping::getPWMChannel(_m1PWM);
			const uint8_t m2PWMCh = PinMapping::getPWMChannel(_m2PWM);

			PMC->PMC_PCER1 = PMC_PCER1_PID36;

			PWM->PWM_CLK = PWM_CLK_PREA(0) | PWM_CLK_DIVA(1);

			PWM->PWM_ENA = 1 << m1PWMCh | 1 << m2PWMCh;

			PWM->PWM_CH_NUM[m1PWMCh].PWM_CMR = PWM_CMR_CPRE_CLKA;
			PWM->PWM_CH_NUM[m1PWMCh].PWM_CPRD = 4200;
			PWM->PWM_CH_NUM[m1PWMCh].PWM_CDTY = 0;

			PWM->PWM_CH_NUM[m2PWMCh].PWM_CMR = PWM_CMR_CPRE_CLKA;
			PWM->PWM_CH_NUM[m2PWMCh].PWM_CPRD = 4200;
			PWM->PWM_CH_NUM[m2PWMCh].PWM_CDTY = 0;

			PinMapping::MappedPins[_m1PWM].port->PIO_PDR = PinMapping::MappedPins[_m1PWM].pin;
			PinMapping::MappedPins[_m2PWM].port->PIO_PDR = PinMapping::MappedPins[_m2PWM].pin;

			if (PinMapping::toABPeripheral(_m1PWM))
			{
				PinMapping::MappedPins[_m1PWM].port->PIO_ABSR |= PinMapping::MappedPins[_m1PWM].pin;
			}
			else
			{
				PinMapping::MappedPins[_m1PWM].port->PIO_ABSR &= ~PinMapping::MappedPins[_m1PWM].pin;
			}

			if (PinMapping::toABPeripheral(_m2PWM))
			{
				PinMapping::MappedPins[_m2PWM].port->PIO_ABSR |= PinMapping::MappedPins[_m2PWM].pin;
			}
			else
			{
				PinMapping::MappedPins[_m2PWM].port->PIO_ABSR &= ~PinMapping::MappedPins[_m2PWM].pin;
			}

			// Setup ADC
			const uint8_t m1ADCCh = PinMapping::getADCChannel(_m1Fb);
			const uint8_t m2ADCCh = PinMapping::getADCChannel(_m2Fb);

			PMC->PMC_PCER1 = PMC_PCER1_PID37;

			ADC->ADC_MR = ADC_MR_LOWRES_BITS_10 | ADC_MR_PRESCAL(0) | ADC_MR_STARTUP_SUT0 | ADC_MR_SETTLING_AST3 | ADC_MR_TRACKTIM(0) | ADC_MR_TRANSFER(1);

			ADC->ADC_CHER = 1 << m1ADCCh | 1 << m2ADCCh;

			return ReturnCode::ok;
		}

		void setSpeed(uint8_t motor, float speed)
		{
			static uint8_t pwmCh;

			if (motor == 1)
			{
				pwmCh = PinMapping::getPWMChannel(_m1PWM);

				// Set Dir Pin
				if (speed > 0)
				{
					PinMapping::MappedPins[_m1Dir].port->PIO_CODR = PinMapping::MappedPins[_m1Dir].pin;
				}
				else
				{
					PinMapping::MappedPins[_m1Dir].port->PIO_SODR = PinMapping::MappedPins[_m1Dir].pin;
				}
			}
			else
			{
				pwmCh = PinMapping::getPWMChannel(_m2PWM);

				// Set Dir Pin
				if (speed > 0)
				{
					PinMapping::MappedPins[_m2Dir].port->PIO_CODR = PinMapping::MappedPins[_m2Dir].pin;
				}
				else
				{
					PinMapping::MappedPins[_m2Dir].port->PIO_SODR = PinMapping::MappedPins[_m2Dir].pin;
				}
			}

			// Set PWM Value
			PWM->PWM_CH_NUM[pwmCh].PWM_CDTYUPD = (PWM->PWM_CH_NUM[pwmCh].PWM_CPRD * abs(speed));
			PWM->PWM_SCUC = PWM_SCUC_UPDULOCK;
		}

		float getCurrent(uint8_t motor)
		{
			const uint8_t m1ADCCh = PinMapping::getADCChannel(_m1Fb);
			const uint8_t m2ADCCh = PinMapping::getADCChannel(_m2Fb);

			// Start conversion
			ADC->ADC_CR = ADC_CR_START;
			
			// Wait for end of conversion
			while (!(ADC->ADC_ISR & ADC_ISR_DRDY));

			if (motor == 1)
			{
				Serial.println(ADC->ADC_CDR[m1ADCCh]);
				return ((float)ADC->ADC_CDR[m1ADCCh] / (float)(1 << 10 - 1) * 3.3f * 525.0f);
			}
			else
			{
				return ((float)ADC->ADC_CDR[m2ADCCh] / (float)(1 << 10 - 1) * 3.3f * 525.0f);
			}
		}
	}
}