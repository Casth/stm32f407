#include <stdint.h>
#include <string.h>
#include "stm32f407xx.h"

int main(void)
{
    SetSystemCLockTo16MHz();

	// ------------------- GPIO B for SPI2 -------------------------
	// turn on clock for GPIO Port B with RCC_AHB1ENR register
	RCC->AHB1ENR |= (1 << 1);

	// SPI pin mapping:
	// PB15: SPI2_MOSI
	// PB14: SP12_MISO
	// PB13: SPI2_SCK
	// PB12: SPI2_NSS

	// turn on clock of SPI2 with RCC_APB1ENR register
	RCC->APB1ENR |= (1 << 14);

	// set GPIO ports to alternate function mode with GPIO_MODER register
	GPIOB->MODER |= (0b10 << 30); // GPIOB pin 15
	GPIOB->MODER |= (0b10 << 28); // GPIOB pin 14
	GPIOB->MODER |= (0b10 << 26); // GPIOB pin 13
	GPIOB->MODER |= (0b10 << 24); // GPIOB pin 12

	// set GPIO ports to alternate function 5 (AF5) according to
	// alternate function mapping (see STM32_datasheet page 63)
	// with GPIO_AFRH register
	GPIOB->AFRH |= (0b0101 << 28); // GPIOB pin 15
	GPIOB->AFRH |= (0b0101 << 24); // GPIOB pin 14
	GPIOB->AFRH |= (0b0101 << 20); // GPIOB pin 13
	GPIOB->AFRH |= (0b0101 << 16); // GPIOB pin 12

	/**
	 * SPI configuration ---------------------------- code starts
	 */
	// 1. set SPI baud rate
	SPI2->CR1 &= ~(0b111 << 3); // reset BR[2:0] bits (clock frequency = f_PCLK / 2)
	SPI2->CR1 |= (0b001 << 3);	// set BR[2:0] bits to clock frequency = f_PCLK / 4

	// 2. set SPI data frame format
	SPI2->CR1 &= ~(1 << 11); // set DFF bit to 8-bit data frame

	// 3. set clock polarity and clock phase
	SPI2->CR1 &= ~(0b11 << 0); // reset CPHA and CPOL bits
	SPI2->CR1 |= (0b00 << 0);  // set CPHA and CPOL bits to 0 and 0

	// 4. set MSB-/LSB-first option
	SPI2->CR1 &= ~(1 << 7); // reset LSBFIRST bit to MSB-first

	// 5. enables SPI software slave management of NSS with SSM bit, 
	// the NSS pin input is replaced with the value from the SSI bit
	SPI2->CR1 |= (1 << 9);

	// 6. pull NSS to HIGH (and avoid MODF error) with SSI bit by forcing this
	// bit onto the NSS pin
	SPI2->CR1 |= (1 << 8);

	// 7. set SPI to master device with MSTR bit
	SPI2->CR1 |= (1 << 2);

	// 8. enable SPI2 with SPE bit
	SPI2->CR1 |= (1 << 6);
	/**
	 * SPI configuration ---------------------------- code ends
	 */

	// ------------------- SPI2 transmit -------------------------
	char user_data[] = "Hello World!";
	uint32_t size_data = sizeof(user_data);

	/* Loop forever */
	while (1)
	{

		for (uint32_t i = 0; i < size_data - 1; i++)
		{
			// wait until TXE (transmit buffer empty) is set
			while (!(SPI2->SR & (1 << 1)))
				;

			// send data
			SPI2->DR = user_data[i];

			// wait until RXNE (receive buffer not empty) is set
			while (!(SPI2->SR & (1 << 0)))
				;

			// read received data (to clear RXNE flag)
			volatile uint8_t temp = SPI2->DR;

			// wait until BSY flag is reset
			while ((SPI2->SR) & (1 << 7))
				;
		}

		for (volatile int i = 0; i < 200; i++);  // Simple delay loop

	}
}

void SetSystemCLockTo16MHz(void)
{
    // enable HSI clock
    if ((pRCC->CR & (1 << 1)) == 0)
    {
        pRCC->CR |= (1 << 0); // set HSION = 1

        while ((pRCC->CR & (1 << 1)) == 0)
            ; // wait until HSI clock is ready
    }

    pRCC->CFGR &= ~(0x0F << 4);  // set AHB prescaler to 1 (HPRE)
    pRCC->CFGR &= ~(0x07 << 10); // set APB1 prescaler to 1 (PPRE1)
    pRCC->CFGR &= ~(0x07 << 13); // set APB1 prescaler to 1 (PPRE2)
    pRCC->CFGR &= ~(0x03 << 0);  // set HSI as system clock source (SW)

    pFLASH->ACR |= (1 << 9);     // enable instruction cache (ICEN)
    pFLASH->ACR |= (1 << 10);    // enable data cache (DCEN)
    pFLASH->ACR &= ~(0x07 << 0); // reset latency (LATENCY)
    pFLASH->ACR |= (0b011 << 0); // set latency to three wait states (LATENCY)

    pRCC->CR &= ~(1 << 16); // disable HSE clock (HSEON)
}