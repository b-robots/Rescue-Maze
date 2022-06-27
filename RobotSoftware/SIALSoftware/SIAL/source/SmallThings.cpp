#include <Arduino.h>

#include "../SIALSettings.h"
#include "../header/SmallThings.h"
#include "../header/DuePinMapping.h"
#include "../header/SmoothDriving.h"

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

		void setup()
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

	namespace Bumper {
		constexpr auto leftPin = PinMapping::MappedPins[SIALSettings::Bumper::leftPin];
		constexpr auto rightPin = PinMapping::MappedPins[SIALSettings::Bumper::rightPin];

		volatile bool left;
		volatile bool right;

		void setup() {
			leftPin.port->PIO_PER = leftPin.pin;
			leftPin.port->PIO_ODR = leftPin.pin;
			leftPin.port->PIO_PUER = leftPin.pin;
			leftPin.port->PIO_IER = leftPin.pin;
			leftPin.port->PIO_AIMER = leftPin.pin;
			leftPin.port->PIO_ESR = leftPin.pin;
			leftPin.port->PIO_FELLSR = leftPin.pin;
			leftPin.port->PIO_DIFSR = leftPin.pin;
			leftPin.port->PIO_SCDR = 0x1ff; // 1000ms / (32000hZ / (DIV + 1)) = 16ms
			leftPin.port->PIO_IFER = leftPin.pin;

			rightPin.port->PIO_PER = rightPin.pin;
			rightPin.port->PIO_ODR = rightPin.pin;
			rightPin.port->PIO_PUER = rightPin.pin;
			rightPin.port->PIO_IER = rightPin.pin;
			rightPin.port->PIO_AIMER = rightPin.pin;
			rightPin.port->PIO_ESR = rightPin.pin;
			rightPin.port->PIO_FELLSR = rightPin.pin;
			rightPin.port->PIO_DIFSR = rightPin.pin;
			rightPin.port->PIO_SCDR = 0x1ff; // 1000ms / (32000hZ / (DIV + 1)) = 16ms
			rightPin.port->PIO_IFER = rightPin.pin;
		}

		void interrupt(InterruptSource source, uint32_t isr) {
			if (leftPin.portID == static_cast<uint8_t>(source) && (isr & leftPin.pin)) {
				left = true;
			}

			if (rightPin.portID == static_cast<uint8_t>(source) && (isr & rightPin.pin)) {
				right = true;
			}

			if (left || right) {
				SmoothDriving::stop();
			}
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

			pin.port->PIO_SCDR = 0xfff; // 1000ms / (32000hZ / (DIV + 1)) = 128ms
		}

		bool getState()
		{
			return (pin.port->PIO_PDSR & pin.pin) != 0;
		}
	}
}