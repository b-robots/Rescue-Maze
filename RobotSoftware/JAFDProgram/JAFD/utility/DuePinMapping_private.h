
/*
This private file of the library is responsible for the access to the SPI EEPROM
*/

#pragma once

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

namespace JAFD
{
	namespace PinMapping
	{
		// PWM Channels
		enum class PWMChannel : uint8_t
		{
			ch0 = 0,
			ch1,
			ch2,
			ch3,
			ch4,
			ch5,
			ch6,
			ch7,
			noPWM
		};

		// ADC Channels
		enum class ADCChannel : uint8_t
		{
			ch0 = 0,
			ch1,
			ch2,
			ch3,
			ch4,
			ch5,
			ch6,
			ch7,
			ch8,
			ch9,
			ch10,
			ch11,
			ch12,
			ch13,
			ch14,
			ch15,
			noADC
		};

		// DAC Channels
		enum class DACChannel : uint8_t
		{
			ch0 = 0,
			ch1,
			noDAC
		};

		// TC Channels
		enum class TCChannel : uint8_t
		{
			tc0ChA0 = 0,
			tc0ChB0,
			tc0ChA1,
			tc0ChB1,
			tc0ChA2,
			tc0ChB2,
			tc1ChA3,
			tc1ChB3,
			tc1ChA4,
			tc1ChB4,
			tc1ChA5,
			tc1ChB5,
			tc2ChA6,
			tc2ChB6,
			tc2ChA7,
			tc2ChB7,
			tc2ChA8,
			tc2ChB8,
			noTC
		};

		// Available Peripherals
		enum class PinPeripherals : uint8_t
		{
			adc,
			dac,
			pwm,
			tc,
			onlyPio
		};

		// Informations about one Pin on the SAM3x8e
		class PinInformation
		{
		private:
			constexpr uint8_t pinDesToPortID(PinDescription pinDes)
			{
				return	(pinDes.ulPeripheralId >= 11 && pinDes.ulPeripheralId <= 16) ?
						static_cast<uint8_t>(pinDes.ulPeripheralId) : 0;
			}

			constexpr PinPeripherals pinDesToPinPeripherals(PinDescription pinDes)
			{
				return	(pinDes.ulPinAttribute & PIN_ATTR_PWM) ? PinPeripherals::pwm :
						((pinDes.ulPinAttribute & PIN_ATTR_TIMER) ? PinPeripherals::tc :
						((pinDes.ulPinAttribute & PIN_ATTR_ANALOG) ?
							((pinDes.ulAnalogChannel == DA0 || pinDes.ulAnalogChannel == DA1) ? PinPeripherals::dac : PinPeripherals::adc) :
						(PinPeripherals::onlyPio)));

			}

			constexpr PWMChannel pinDesToPWMCh(PinDescription pinDes)
			{
				return	(pinDes.ulPWMChannel == NOT_ON_PWM) ? PWMChannel::noPWM :
						((pinDes.ulPWMChannel == PWM_CH0) ? PWMChannel::ch0 :
						((pinDes.ulPWMChannel == PWM_CH1) ? PWMChannel::ch1 :
						((pinDes.ulPWMChannel == PWM_CH2) ? PWMChannel::ch2 :
						((pinDes.ulPWMChannel == PWM_CH3) ? PWMChannel::ch3 :
						((pinDes.ulPWMChannel == PWM_CH4) ? PWMChannel::ch4 :
						((pinDes.ulPWMChannel == PWM_CH5) ? PWMChannel::ch5 :
						((pinDes.ulPWMChannel == PWM_CH6) ? PWMChannel::ch6 :
						((pinDes.ulPWMChannel == PWM_CH7) ? PWMChannel::ch7 :
						(PWMChannel::noPWM)))))))));
			}

			constexpr ADCChannel pinDesToADCCh(PinDescription pinDes)
			{
				return	(pinDes.ulADCChannelNumber == NO_ADC) ? ADCChannel::noADC :
						((pinDes.ulADCChannelNumber == ADC0) ? ADCChannel::ch0 :
						((pinDes.ulADCChannelNumber == ADC1) ? ADCChannel::ch1 :
						((pinDes.ulADCChannelNumber == ADC2) ? ADCChannel::ch2 :
						((pinDes.ulADCChannelNumber == ADC3) ? ADCChannel::ch3 :
						((pinDes.ulADCChannelNumber == ADC4) ? ADCChannel::ch4 :
						((pinDes.ulADCChannelNumber == ADC5) ? ADCChannel::ch5 :
						((pinDes.ulADCChannelNumber == ADC6) ? ADCChannel::ch6 :
						((pinDes.ulADCChannelNumber == ADC7) ? ADCChannel::ch7 :
						((pinDes.ulADCChannelNumber == ADC8) ? ADCChannel::ch8 :
						((pinDes.ulADCChannelNumber == ADC9) ? ADCChannel::ch9 :
						((pinDes.ulADCChannelNumber == ADC10) ? ADCChannel::ch10 :
						((pinDes.ulADCChannelNumber == ADC11) ? ADCChannel::ch11 :
						((pinDes.ulADCChannelNumber == ADC12) ? ADCChannel::ch12 :
						((pinDes.ulADCChannelNumber == ADC13) ? ADCChannel::ch13 :
						((pinDes.ulADCChannelNumber == ADC14) ? ADCChannel::ch14 :
						((pinDes.ulADCChannelNumber == ADC15) ? ADCChannel::ch15 :
						(ADCChannel::noADC)))))))))))))))));
			}

			constexpr DACChannel pinDesToDACCh(PinDescription pinDes)
			{
				return	(pinDes.ulADCChannelNumber == NO_ADC) ? DACChannel::noDAC :
						((pinDes.ulADCChannelNumber == DA0) ? DACChannel::ch0 :
						((pinDes.ulADCChannelNumber == DA1) ? DACChannel::ch1 :
						(DACChannel::noDAC)));
			}

			constexpr TCChannel pinDesToTCCh(PinDescription pinDes)
			{
				return	(pinDes.ulTCChannel == NOT_ON_TIMER) ? TCChannel::noTC :
						((pinDes.ulTCChannel == TC0_CHA0) ? TCChannel::tc0ChA0 :
						((pinDes.ulTCChannel == TC0_CHB0) ? TCChannel::tc0ChB0 :
						((pinDes.ulTCChannel == TC0_CHA1) ? TCChannel::tc0ChA1 :
						((pinDes.ulTCChannel == TC0_CHB1) ? TCChannel::tc0ChB1 :
						((pinDes.ulTCChannel == TC0_CHA2) ? TCChannel::tc0ChA2 :
						((pinDes.ulTCChannel == TC0_CHB2) ? TCChannel::tc0ChB2 :
						((pinDes.ulTCChannel == TC1_CHA3) ? TCChannel::tc1ChA3 :
						((pinDes.ulTCChannel == TC1_CHB3) ? TCChannel::tc1ChB3 :
						((pinDes.ulTCChannel == TC1_CHA4) ? TCChannel::tc1ChA4 :
						((pinDes.ulTCChannel == TC1_CHB4) ? TCChannel::tc1ChB4 :
						((pinDes.ulTCChannel == TC1_CHA5) ? TCChannel::tc1ChA5 :
						((pinDes.ulTCChannel == TC1_CHB5) ? TCChannel::tc1ChB5 :
						((pinDes.ulTCChannel == TC2_CHA6) ? TCChannel::tc2ChA6 :
						((pinDes.ulTCChannel == TC2_CHB6) ? TCChannel::tc2ChB6 :
						((pinDes.ulTCChannel == TC2_CHA7) ? TCChannel::tc2ChA7 :
						((pinDes.ulTCChannel == TC2_CHB7) ? TCChannel::tc2ChB7 :
						((pinDes.ulTCChannel == TC2_CHA8) ? TCChannel::tc2ChA8 :
						((pinDes.ulTCChannel == TC2_CHB8) ? TCChannel::tc2ChB8 :
						(TCChannel::noTC)))))))))))))))))));
			}

		public:
			const uint32_t pin;
			Pio * const port;
			const uint8_t portID;
			const PinPeripherals pinPeripherals;
			const PWMChannel pwmChannel;
			const ADCChannel adcChannel;
			const DACChannel dacChannel;
			const TCChannel tcChannel;

			// Casting constructor to use the already declared mapped pins from variant.cpp
			constexpr PinInformation(PinDescription pinDes) :
				pin(pinDes.ulPin),
				port(pinDes.pPort),
				portID(pinDesToPortID(pinDes)),
				pinPeripherals(pinDesToPinPeripherals(pinDes)),
				pwmChannel(pinDesToPWMCh(pinDes)),
				adcChannel(pinDesToADCCh(pinDes)),
				dacChannel(pinDesToDACCh(pinDes)),
				tcChannel(pinDesToTCCh(pinDes))
			{}

			// Normal constructor
			constexpr PinInformation(uint32_t _pin, Pio * _port, uint8_t _portID, PinPeripherals _pinPeripherals, PWMChannel _pwmChannel, ADCChannel _adcChannel, DACChannel _dacChannel, TCChannel _tcChannel) :
				pin(_pin),
				port(_port),
				portID(_portID),
				pinPeripherals(_pinPeripherals),
				pwmChannel(_pwmChannel),
				adcChannel(_adcChannel),
				dacChannel(_dacChannel),
				tcChannel(_tcChannel)
			{}
		};
		
		// Copied Arduino pin mapping and use constructor of own struct to convert
		constexpr PinDescription MappedPins[] =
		{
			// 0 .. 53 - Digital pins
			// ----------------------
			// 0/1 - UART (Serial)
			PinDescription {PIOA, PIO_PA8A_URXD,     ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT,  (PIN_ATTR_DIGITAL | PIN_ATTR_PWM),	NO_ADC, NO_ADC, PWM_CH0,	NOT_ON_TIMER }, // URXD
			PinDescription {PIOA, PIO_PA9A_UTXD,     ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT,  (PIN_ATTR_DIGITAL | PIN_ATTR_PWM),	NO_ADC, NO_ADC, PWM_CH3,	NOT_ON_TIMER }, // UTXD
			
			// 2
			PinDescription {PIOB, PIO_PB25B_TIOA0,   ID_PIOB, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL | PIN_ATTR_TIMER), NO_ADC, NO_ADC, NOT_ON_PWM,  TC0_CHA0     }, // TIOA0
			PinDescription {PIOC, PIO_PC28B_TIOA7,   ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL | PIN_ATTR_TIMER), NO_ADC, NO_ADC, NOT_ON_PWM,  TC2_CHA7     }, // TIOA7
			PinDescription {PIOC, PIO_PC26B_TIOB6,   ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL | PIN_ATTR_TIMER), NO_ADC, NO_ADC, NOT_ON_PWM,  TC2_CHB6     }, // TIOB6
			
			// 5
			PinDescription {PIOC, PIO_PC25B_TIOA6,   ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL | PIN_ATTR_TIMER), NO_ADC, NO_ADC, NOT_ON_PWM,  TC2_CHA6     }, // TIOA6
			PinDescription {PIOC, PIO_PC24B_PWML7,   ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM),   NO_ADC, NO_ADC, PWM_CH7,     NOT_ON_TIMER }, // PWML7
			PinDescription {PIOC, PIO_PC23B_PWML6,   ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM),   NO_ADC, NO_ADC, PWM_CH6,     NOT_ON_TIMER }, // PWML6
			PinDescription {PIOC, PIO_PC22B_PWML5,   ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM),   NO_ADC, NO_ADC, PWM_CH5,     NOT_ON_TIMER }, // PWML5
			PinDescription {PIOC, PIO_PC21B_PWML4,   ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM),   NO_ADC, NO_ADC, PWM_CH4,     NOT_ON_TIMER }, // PWML4
			// 10
			PinDescription {PIOC, PIO_PC29B_TIOB7,   ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL | PIN_ATTR_TIMER), NO_ADC, NO_ADC, NOT_ON_PWM,  TC2_CHB7     }, // TIOB7
			PinDescription {PIOD, PIO_PD7B_TIOA8,    ID_PIOD, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL | PIN_ATTR_TIMER), NO_ADC, NO_ADC, NOT_ON_PWM,  TC2_CHA8     }, // TIOA8
			PinDescription {PIOD, PIO_PD8B_TIOB8,    ID_PIOD, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL | PIN_ATTR_TIMER), NO_ADC, NO_ADC, NOT_ON_PWM,  TC2_CHB8     }, // TIOB8
			
			// 13 - AMBER LED
			PinDescription {PIOB, PIO_PB27B_TIOB0,   ID_PIOB, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL | PIN_ATTR_TIMER), NO_ADC, NO_ADC, NOT_ON_PWM,  TC0_CHB0     }, // TIOB0
			
			// 14/15 - USART3 (Serial3)
			PinDescription {PIOD, PIO_PD4B_TXD3,     ID_PIOD, PIO_PERIPH_B, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // TXD3
			PinDescription {PIOD, PIO_PD5B_RXD3,     ID_PIOD, PIO_PERIPH_B, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // RXD3
			
			// 16/17 - USART1 (Serial2)
			PinDescription {PIOA, PIO_PA13A_TXD1,    ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM), NO_ADC, NO_ADC, PWM_CH2,  	 NOT_ON_TIMER }, // TXD1
			PinDescription {PIOA, PIO_PA12A_RXD1,    ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM), NO_ADC, NO_ADC, PWM_CH1,	 NOT_ON_TIMER }, // RXD1
			
			// 18/19 - USART0 (Serial1)
			PinDescription {PIOA, PIO_PA11A_TXD0,    ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // TXD0
			PinDescription {PIOA, PIO_PA10A_RXD0,    ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // RXD0
			
			// 20/21 - TWI1
			PinDescription {PIOB, PIO_PB12A_TWD1,    ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM), NO_ADC, NO_ADC, PWM_CH0,	 NOT_ON_TIMER }, // TWD1 - SDA0
			PinDescription {PIOB, PIO_PB13A_TWCK1,   ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM), NO_ADC, NO_ADC, PWM_CH1,     NOT_ON_TIMER }, // TWCK1 - SCL0
			
			// 22
			PinDescription {PIOB, PIO_PB26,          ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 22
			PinDescription {PIOA, PIO_PA14,          ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 23
			PinDescription {PIOA, PIO_PA15,          ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 24
			PinDescription {PIOD, PIO_PD0,           ID_PIOD, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 25
			
			// 26
			PinDescription {PIOD, PIO_PD1,           ID_PIOD, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 26
			PinDescription {PIOD, PIO_PD2,           ID_PIOD, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 27
			PinDescription {PIOD, PIO_PD3,           ID_PIOD, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 28
			PinDescription {PIOD, PIO_PD6,           ID_PIOD, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 29
			
			// 30
			PinDescription {PIOD, PIO_PD9,           ID_PIOD, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 30
			PinDescription {PIOA, PIO_PA7,           ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 31
			PinDescription {PIOD, PIO_PD10,          ID_PIOD, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 32
			PinDescription {PIOC, PIO_PC1,           ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 33
			
			// 34
			PinDescription {PIOC, PIO_PC2,           ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM), NO_ADC, NO_ADC, PWM_CH0,  	 NOT_ON_TIMER }, // PIN 34
			PinDescription {PIOC, PIO_PC3,           ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM), NO_ADC, NO_ADC, PWM_CH0,     NOT_ON_TIMER }, // PIN 35
			PinDescription {PIOC, PIO_PC4,           ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM), NO_ADC, NO_ADC, PWM_CH1,  	 NOT_ON_TIMER }, // PIN 36
			PinDescription {PIOC, PIO_PC5,           ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM), NO_ADC, NO_ADC, PWM_CH1,     NOT_ON_TIMER }, // PIN 37
			
			// 38
			PinDescription {PIOC, PIO_PC6,           ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM), NO_ADC, NO_ADC, PWM_CH2,  	 NOT_ON_TIMER }, // PIN 38
			PinDescription {PIOC, PIO_PC7,           ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM), NO_ADC, NO_ADC, PWM_CH2,  	 NOT_ON_TIMER }, // PIN 39
			PinDescription {PIOC, PIO_PC8,           ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM), NO_ADC, NO_ADC, PWM_CH3,	 NOT_ON_TIMER }, // PIN 40
			PinDescription {PIOC, PIO_PC9,           ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM), NO_ADC, NO_ADC, PWM_CH3,  	 NOT_ON_TIMER }, // PIN 41
			
			// 42
			PinDescription {PIOA, PIO_PA19,          ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM), NO_ADC, NO_ADC, PWM_CH1,     NOT_ON_TIMER }, // PIN 42
			PinDescription {PIOA, PIO_PA20,          ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM), NO_ADC, NO_ADC, PWM_CH2,  	 NOT_ON_TIMER }, // PIN 43
			PinDescription {PIOC, PIO_PC19,          ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM), NO_ADC, NO_ADC, PWM_CH5,  	 NOT_ON_TIMER }, // PIN 44
			PinDescription {PIOC, PIO_PC18,          ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM), NO_ADC, NO_ADC, PWM_CH6,  	 NOT_ON_TIMER }, // PIN 45
			
			// 46
			PinDescription {PIOC, PIO_PC17,          ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 46
			PinDescription {PIOC, PIO_PC16,          ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 47
			PinDescription {PIOC, PIO_PC15,          ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 48
			PinDescription {PIOC, PIO_PC14,          ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 49
			
			// 50
			PinDescription {PIOC, PIO_PC13,          ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 50
			PinDescription {PIOC, PIO_PC12,          ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 51
			PinDescription {PIOB, PIO_PB21,          ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 52
			PinDescription {PIOB, PIO_PB14,          ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM), NO_ADC, NO_ADC, PWM_CH2,     NOT_ON_TIMER }, // PIN 53
			
			
			// 54 .. 65 - Analog pins
			// ----------------------
			PinDescription {PIOA, PIO_PA16X1_AD7,    ID_PIOA, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC0,   ADC7,   NOT_ON_PWM,  NOT_ON_TIMER }, // AD0
			PinDescription {PIOA, PIO_PA24X1_AD6,    ID_PIOA, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC1,   ADC6,   NOT_ON_PWM,  NOT_ON_TIMER }, // AD1
			PinDescription {PIOA, PIO_PA23X1_AD5,    ID_PIOA, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC2,   ADC5,   NOT_ON_PWM,  NOT_ON_TIMER }, // AD2
			PinDescription {PIOA, PIO_PA22X1_AD4,    ID_PIOA, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC3,   ADC4,   NOT_ON_PWM,  NOT_ON_TIMER }, // AD3
			// 58
			PinDescription {PIOA, PIO_PA6X1_AD3,     ID_PIOA, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC4,   ADC3,   NOT_ON_PWM,  TC0_CHB2     }, // AD4
			PinDescription {PIOA, PIO_PA4X1_AD2,     ID_PIOA, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC5,   ADC2,   NOT_ON_PWM,  NOT_ON_TIMER }, // AD5
			PinDescription {PIOA, PIO_PA3X1_AD1,     ID_PIOA, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC6,   ADC1,   NOT_ON_PWM,  TC0_CHB1     }, // AD6
			PinDescription {PIOA, PIO_PA2X1_AD0,     ID_PIOA, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC7,   ADC0,   NOT_ON_PWM,  TC0_CHA1     }, // AD7
			// 62
			PinDescription {PIOB, PIO_PB17X1_AD10,   ID_PIOB, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,					ADC8,   ADC10,  NOT_ON_PWM,  NOT_ON_TIMER }, // AD8
			PinDescription {PIOB, PIO_PB18X1_AD11,   ID_PIOB, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,					ADC9,   ADC11,  NOT_ON_PWM,  NOT_ON_TIMER }, // AD9
			PinDescription {PIOB, PIO_PB19X1_AD12,   ID_PIOB, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,					ADC10,  ADC12,  NOT_ON_PWM,	 NOT_ON_TIMER }, // AD10
			PinDescription {PIOB, PIO_PB20X1_AD13,   ID_PIOB, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC11,  ADC13,  NOT_ON_PWM,  NOT_ON_TIMER }, // AD11
			
			// 66/67 - DAC0/DAC1
			PinDescription {PIOB, PIO_PB15X1_DAC0,   ID_PIOB, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,					ADC12,  DA0,    NOT_ON_PWM,  NOT_ON_TIMER }, // DAC0
			PinDescription {PIOB, PIO_PB16X1_DAC1,   ID_PIOB, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,					ADC13,  DA1,    NOT_ON_PWM,  NOT_ON_TIMER }, // DAC1
			
			// 68/69 - CANRX0/CANTX0
			PinDescription {PIOA, PIO_PA1A_CANRX0,   ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  ADC14,  NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // CANRX
			PinDescription {PIOA, PIO_PA0A_CANTX0,   ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM), ADC15,  NO_ADC, PWM_CH3,	 NOT_ON_TIMER }, // CANTX
			
			// 70/71 - TWI0
			PinDescription {PIOA, PIO_PA17A_TWD0,    ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // TWD0 - SDA1
			PinDescription {PIOA, PIO_PA18A_TWCK0,   ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // TWCK0 - SCL1
			
			// 72/73 - LEDs
			PinDescription {PIOC, PIO_PC30,          ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // LED AMBER RXL
			PinDescription {PIOA, PIO_PA21,          ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM), NO_ADC, NO_ADC, PWM_CH0,  	 NOT_ON_TIMER }, // LED AMBER TXL
			
			// 74/75/76 - SPI
			PinDescription {PIOA, PIO_PA25A_SPI0_MISO,ID_PIOA,PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // MISO
			PinDescription {PIOA, PIO_PA26A_SPI0_MOSI,ID_PIOA,PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // MOSI
			PinDescription {PIOA, PIO_PA27A_SPI0_SPCK,ID_PIOA,PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // SPCK
			
			// 77 - SPI CS0
			PinDescription {PIOA, PIO_PA28A_SPI0_NPCS0,ID_PIOA,PIO_PERIPH_A,PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // NPCS0
			
			// 78 - SPI CS3 (unconnected)
			PinDescription {PIOB, PIO_PB23B_SPI0_NPCS3,ID_PIOB,PIO_PERIPH_B,PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER } // NPCS3
		};

		// Helping functions
		constexpr bool hasPWM(uint8_t pin)
		{
			return MappedPins[pin].pinPeripherals == PinPeripherals::pwm;
		}

		constexpr bool hasADC(uint8_t pin)
		{
			return MappedPins[pin].pinPeripherals == PinPeripherals::adc;
		}

		constexpr bool hasDAC(uint8_t pin)
		{
			return MappedPins[pin].pinPeripherals == PinPeripherals::dac;
		}

		constexpr bool hasTC(uint8_t pin)
		{
			return MappedPins[pin].pinPeripherals == PinPeripherals::tc;
		}

		constexpr uint8_t getPWMChannel(uint8_t pin)
		{
			return static_cast<uint8_t>(MappedPins[pin].pwmChannel);
		}

		constexpr uint8_t getADCChannel(uint8_t pin)
		{
			return static_cast<uint8_t>(MappedPins[pin].adcChannel);
		}
		
		constexpr uint8_t getTCChannel(uint8_t pin)
		{
			return static_cast<uint8_t>(MappedPins[pin].tcChannel) / 6;
		}
		
		constexpr bool toABPeripheral(uint8_t pin)
		{
			return	(hasPWM(pin)) ?
						true :
					((hasTC(pin)) ? 
						(getTCChannel(pin) == 2 ||
						(getTCChannel(pin) == 0 && MappedPins[pin].port == PIOB) ||
						(MappedPins[pin].pin == PIO_PB0 ||
						MappedPins[pin].pin == PIO_PB2 ||
						MappedPins[pin].pin == PIO_PB4 ||
						MappedPins[pin].pin == PIO_PB1 ||
						MappedPins[pin].pin == PIO_PB3 || 
						MappedPins[pin].pin == PIO_PB5)) :
					false);
		}
	}
}