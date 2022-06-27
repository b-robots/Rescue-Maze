#include "../header/AllDatatypes.h"
#include "../header/SmallThings.h"

void handleISR(SIAL::InterruptSource interruptSrc, uint32_t isr)
{
	SIAL::Bumper::interrupt(interruptSrc, isr);
}

void PIOA_Handler()
{
	handleISR(SIAL::InterruptSource::pioA, PIOA->PIO_ISR);
}

void PIOB_Handler()
{
	handleISR(SIAL::InterruptSource::pioB, PIOB->PIO_ISR);
}

void PIOC_Handler()
{
	handleISR(SIAL::InterruptSource::pioC, PIOC->PIO_ISR);
}

void PIOD_Handler()
{
	handleISR(SIAL::InterruptSource::pioD, PIOD->PIO_ISR);
}