/*
In this part of the library are all interrupt handler
*/

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "Interrupts_private.h"
#include "MotorControl_private.h"

void PIOA_Handler()
{
	JAFD::MotorControl::encoderInterrupt(JAFD::Interrupts::InterruptSource::pioA, PIOA->PIO_ISR);
}

void PIOB_Handler()
{
	JAFD::MotorControl::encoderInterrupt(JAFD::Interrupts::InterruptSource::pioB, PIOB->PIO_ISR);
}

void PIOC_Handler()
{
	JAFD::MotorControl::encoderInterrupt(JAFD::Interrupts::InterruptSource::pioC, PIOC->PIO_ISR);
}

void PIOD_Handler()
{
	JAFD::MotorControl::encoderInterrupt(JAFD::Interrupts::InterruptSource::pioD, PIOD->PIO_ISR);
}

void TC0_Handler()
{

}

void TC1_Handler()
{

}

void TC2_Handler()
{
	JAFD::MotorControl::calcMotorSpeed();
}