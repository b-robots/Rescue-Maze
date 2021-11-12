#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "../../JAFDSettings.h"
#include "../header/SmallThings.h"
#include "../header/DistanceSensors.h"
#include "../header/HeatSensor.h"
#include "../header/SensorFusion.h"
#include "../header/SmoothDriving.h"
#include "../header/DuePinMapping.h"
#include "../header/RobotLogic.h"
#include "../header/TCA9548A.h"
#include "../header/TCS34725.h"
#include "../header/Bno055.h"

#include <Wire.h>

namespace JAFD
{
	namespace PowerLEDs
	{
		namespace
		{
			constexpr auto lPWM = PinMapping::MappedPins[JAFDSettings::PowerLEDs::Left::pwmPin];	// PWM pin left LED
			constexpr auto rPWM = PinMapping::MappedPins[JAFDSettings::PowerLEDs::Right::pwmPin];	// PWM pin right LED
			
			constexpr uint8_t lPWMCh = PinMapping::getPWMChannel(lPWM);		// Left LED PWM channel
			constexpr uint8_t rPWMCh = PinMapping::getPWMChannel(rPWM);		// Left LED PWM channel
		}

		ReturnCode setup()
		{
			// Setup PWM - Controller (~200Hz)
			PWM->PWM_ENA = 1 << lPWMCh | 1 << rPWMCh;

			PWM->PWM_CH_NUM[lPWMCh].PWM_CMR = PWM_CMR_CPRE_MCK_DIV_64;
			PWM->PWM_CH_NUM[lPWMCh].PWM_CPRD = 6562;
			PWM->PWM_CH_NUM[lPWMCh].PWM_CDTY = 0;

			if (PinMapping::getPWMStartState(lPWM) == PinMapping::PWMStartState::high)
			{
				PWM->PWM_CH_NUM[lPWMCh].PWM_CMR |= PWM_CMR_CPOL;
			}

			PWM->PWM_CH_NUM[rPWMCh].PWM_CMR = PWM_CMR_CPRE_MCK_DIV_64;
			PWM->PWM_CH_NUM[rPWMCh].PWM_CPRD = 6562;
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

			// Make pins Open Drain
			lPWM.port->PIO_PUDR = lPWM.pin;
			rPWM.port->PIO_PUDR = rPWM.pin;
			lPWM.port->PIO_MDER = lPWM.pin;
			rPWM.port->PIO_MDER = rPWM.pin;

			setBrightness(JAFDSettings::PowerLEDs::defaultPower);

			return ReturnCode::ok;
		}

		void setBrightness(float perc)
		{
			PWM->PWM_CH_NUM[lPWMCh].PWM_CDTYUPD = static_cast<uint16_t>(PWM->PWM_CH_NUM[lPWMCh].PWM_CPRD * perc);
			PWM->PWM_CH_NUM[rPWMCh].PWM_CDTYUPD = static_cast<uint16_t>(PWM->PWM_CH_NUM[rPWMCh].PWM_CPRD * perc);
			PWM->PWM_SCUC = PWM_SCUC_UPDULOCK;
		}
	}

	namespace I2CBus
	{
		namespace
		{
			constexpr auto resetBusPin = PinMapping::MappedPins[JAFDSettings::I2CBus::powerResetPin];

			void manualClockingWire0()
			{
				Wire.end();

				TWI1->TWI_CR = TWI_CR_SVDIS | TWI_CR_MSDIS;

				PIOB->PIO_PER |= PIO_PER_P13;      // TWCK1 pin (SCL) back to GPIO
				PIOB->PIO_OER |= PIO_OER_P13;
				PIOB->PIO_OWER |= PIO_OWER_P13;

				PIOB->PIO_PER |= PIO_PER_P12;      // TWD1 pin (SDA)  back to GPIO
				PIOB->PIO_OER |= PIO_OER_P12;
				PIOB->PIO_OWER |= PIO_OWER_P12;

				// Generate 9 clock pulses
				for (uint8_t i = 0; i < 9; i++) {
					digitalWrite(21, HIGH);
					delay(10);
					digitalWrite(21, LOW);
					delay(10);
				}

				// Send a STOP
				digitalWrite(21, LOW);
				digitalWrite(20, LOW);
				delay(20);
				digitalWrite(21, HIGH);
				digitalWrite(20, HIGH);

				// Back to TWI
				PIOB->PIO_PDR |= PIO_PDR_P12 | PIO_PDR_P13;				// Enable peripheral control
				PIOB->PIO_ABSR &= ~(PIO_PB12A_TWD1 | PIO_PB13A_TWCK1);	// TWD1 & TWCK1 Peripherals type A

				TWI1->TWI_CR = TWI_CR_MSEN;

				delay(30);

				Wire.begin();
			}

			void manualClockingWire1()
			{
				Wire1.end();

				TWI0->TWI_CR = TWI_CR_SVDIS | TWI_CR_MSDIS;

				PIOA->PIO_PER |= PIO_PER_P18;      // TWCK0 pin (SCL1) back to GPIO
				PIOA->PIO_OER |= PIO_OER_P18;
				PIOA->PIO_OWER |= PIO_OWER_P18;

				PIOA->PIO_PER |= PIO_PER_P17;      // TWD0 pin (SDA1)  back to GPIO
				PIOA->PIO_OER |= PIO_OER_P17;
				PIOA->PIO_OWER |= PIO_OWER_P17;

				// Generate 9 clock pulses
				for (uint8_t i = 0; i < 9; i++) {
					digitalWrite(71, HIGH);
					delay(10);
					digitalWrite(71, LOW);
					delay(10);
				}

				// Send a STOP
				digitalWrite(71, LOW);
				digitalWrite(70, LOW);
				delay(20);
				digitalWrite(71, HIGH);
				digitalWrite(70, HIGH);

				// Back to TWI
				PIOA->PIO_PDR |= PIO_PDR_P17 | PIO_PDR_P18;				// Enable peripheral control
				PIOA->PIO_ABSR &= ~(PIO_PA17A_TWD0 | PIO_PA18A_TWCK0);	// TWD0 & TWCK0 Peripherals type A			

				TWI0->TWI_CR = TWI_CR_MSEN;

				delay(30);

				Wire1.begin();
			}
		}

		ReturnCode setup()
		{
			Wire.begin();
			Wire1.begin();

			manualClockingWire0();
			manualClockingWire1();

			for (uint8_t ch = 0; ch < I2CMultiplexer::maxCh; ch++)
			{
				I2CMultiplexer::selectChannel(ch);
				manualClockingWire0();
			}

			resetBusPin.port->PIO_PER = resetBusPin.pin;
			resetBusPin.port->PIO_OER = resetBusPin.pin;

			resetBusPin.port->PIO_SODR = resetBusPin.pin;

			return ReturnCode::ok;
		}

		ReturnCode resetBus()
		{
			Serial.println("Reset Bus!");

			static auto lastReset = millis();

			ReturnCode result = ReturnCode::ok;

			manualClockingWire1();
			manualClockingWire0();

			Wire.begin();
			Wire1.begin();

			if ((millis() - lastReset) > 1000)
			{
				for (uint8_t ch = 0; ch < I2CMultiplexer::maxCh; ch++)
				{
					I2CMultiplexer::selectChannel(ch);
					manualClockingWire0();
				}

				if (I2CMultiplexer::setup() == ReturnCode::ok)
				{
					lastReset = millis();

					return ReturnCode::ok;
				}
			}

			Wire.end();
			Wire1.end();

			resetBusPin.port->PIO_CODR = resetBusPin.pin;
			delay(5);
			resetBusPin.port->PIO_SODR = resetBusPin.pin;

			delay(10);

			Wire.begin();
			Wire1.begin();

			if (DistanceSensors::reset() != ReturnCode::ok) result = ReturnCode::error;

			if (HeatSensor::reset() != ReturnCode::ok) result = ReturnCode::error;

			if (I2CMultiplexer::setup() != ReturnCode::ok) result = ReturnCode::error;

			if (Bno055::setup() != ReturnCode::ok) result = ReturnCode::error;

			if (ColorSensor::setup() != ReturnCode::ok) result = ReturnCode::error;

			lastReset = millis();

			return result;
		}
	}

	namespace MemWatcher
	{
		namespace
		{
			extern "C" char* sbrk(int i);
			char* ramstart = (char*)0x20070000;
			char* ramend = (char*)0x20088000;
		}	

		uint32_t getDynamicRam()
		{
			struct mallinfo mi = mallinfo();
			return mi.uordblks;
		}

		uint32_t getStackRam()
		{
			register char* stack_ptr asm("sp");
			return ramend - stack_ptr;
		}

		uint32_t getFreeRam()
		{
			char* heapend = sbrk(0);
			struct mallinfo mi = mallinfo();
			register char* stack_ptr asm("sp");
			return stack_ptr - heapend + mi.fordblks;
		}
	}

	namespace Wait
	{
		namespace
		{
			void doWhileWaiting()
			{
				SensorFusion::updateSensors();
				SensorFusion::untimedFusion();
			}
		}

		void delayUnblocking(uint32_t ms)
		{
			auto start = millis();

			while ((millis() - start) < ms)
			{
				RobotLogic::timeBetweenUpdate();
			}
		}

		void waitForFinishedTask()
		{
			while (!SmoothDriving::isTaskFinished()) doWhileWaiting();
		}
	}

	// TODO
	namespace Switch
	{
		namespace
		{
			constexpr auto pin = PinMapping::MappedPins[JAFDSettings::Switch::pin];
		}

		void setup()
		{

		}

		bool getState()
		{

		}
	}

	// TODO
	namespace Buzzer
	{

	}
}