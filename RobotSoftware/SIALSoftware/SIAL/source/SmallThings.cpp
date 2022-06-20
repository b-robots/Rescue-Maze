#include <Arduino.h>

#include "../SIALSettings.h"
#include "../header/SmallThings.h"
#include "../header/DuePinMapping.h"

#include <Wire.h>

namespace SIAL
{
	namespace PowerLEDs
	{
		namespace
		{
			constexpr auto lPWM = PinMapping::MappedPins[SIALSettings::PowerLEDs::Left::pwmPin];	// PWM pin left LED
			constexpr auto rPWM = PinMapping::MappedPins[SIALSettings::PowerLEDs::Right::pwmPin];	// PWM pin right LED

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

			setBrightness(0.0f);

			return ReturnCode::ok;
		}

		void setBrightness(float perc)
		{
			PWM->PWM_CH_NUM[lPWMCh].PWM_CDTYUPD = static_cast<uint16_t>(PWM->PWM_CH_NUM[lPWMCh].PWM_CPRD * perc);
			PWM->PWM_CH_NUM[rPWMCh].PWM_CDTYUPD = static_cast<uint16_t>(PWM->PWM_CH_NUM[rPWMCh].PWM_CPRD * perc);
			PWM->PWM_SCUC = PWM_SCUC_UPDULOCK;
		}

		void setBrightness(bool left, float perc)
		{
			if (left) {
				PWM->PWM_CH_NUM[lPWMCh].PWM_CDTYUPD = static_cast<uint16_t>(PWM->PWM_CH_NUM[lPWMCh].PWM_CPRD * perc);
			}
			else {
				PWM->PWM_CH_NUM[rPWMCh].PWM_CDTYUPD = static_cast<uint16_t>(PWM->PWM_CH_NUM[rPWMCh].PWM_CPRD * perc);
			}

			PWM->PWM_SCUC = PWM_SCUC_UPDULOCK;
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

	namespace Switch
	{
		namespace
		{
			constexpr auto pin = PinMapping::MappedPins[SIALSettings::Switch::pin];
		}

		void setup()
		{
			pin.port->PIO_PER = pin.pin;
			pin.port->PIO_ODR = pin.pin;
			pin.port->PIO_DIFSR = pin.pin;
			pin.port->PIO_IFER = pin.pin;

			pin.port->PIO_SCDR = 0x8ff; // (DIV + 1) * 31us = ~70ms
		}

		bool getState()
		{
			return (pin.port->PIO_PDSR & pin.pin) != 0;
		}
	}
}