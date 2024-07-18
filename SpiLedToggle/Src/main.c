#include <stdint.h>
#include <string.h>
#include "stm32f407xx.h"

int main(void)
{
	char user_data[] = "Hello World!";

	// ------------------- GPIO D for LED -------------------------
	// enable AHB1 peripheral clock for GPIO port D for LED
	pRCC->AHB1ENR |= (1 << 3);
	// reset 30th and 31st bit of GPIOD mode register before configure GPIOD pin 15
	pGPIOD->MODER &= ~(0b11 << 30);
	// set GPIOD pin 15 mode to output
	pGPIOD->MODER |= (0b01 << 30);
	pGPIOD->ODR &= ~(1 << 15); // set GPIOD pin 5 output to LOW


	// ------------------- GPIO B for SPI -------------------------
	// turn on clock for GPIO Port B with RCC_AHB1ENR register

	// SPI pin mapping:
	// PB15: SPI2_MOSI
	// PB14: SP12_MISO
	// PB13: SPI2_SCK
	// PB12: SPI2_NSS

	pRCC->AHB1ENR |= (1 << 1);

	// turn on clock of SPI2 with RCC_APB1ENR register
	pRCC->APB1ENR |= (1 << 14);

	// set GPIO ports to analog mode with GPIO_MODER register
	pGPIOB->MODER |= (0b11 << 30);	// GPIOB pin 15
	pGPIOB->MODER |= (0b11 << 28);	// GPIOB pin 14
	pGPIOB->MODER |= (0b11 << 26);	// GPIOB pin 13
	pGPIOB->MODER |= (0b11 << 24);	// GPIOB pin 12

	// set GPIO ports to alternate function 5 (AF5) according to
	// alternate function mapping (see STM32_datasheet page 63)
	// with GPIO_AFRH register
	pGPIOB->AFRH |= (0b0101 << 28);	// GPIOB pin 15
	pGPIOB->AFRH |= (0b0101 << 24);	// GPIOB pin 14
	pGPIOB->AFRH |= (0b0101 << 20);	// GPIOB pin 13
	pGPIOB->AFRH |= (0b0101 << 16);	// GPIOB pin 12

	// set SPI to master device with SPI_CR1 register
	pSPI2->CR1 |= (1 << 2);

	// enables SPI software slave management of NSS with SPI_CR1
	// register, which replace the NSS pin input with the value
	// from the SSI bit
	pSPI2->CR1 |= (1 << 9);

	// pull NSS to HIGH (and avoid MODF error) with SPI_CR1 register
	// by forcing this bit onto the NSS pin
	pSPI2->CR1 |= (1 << 8);

	// enable SPI2 with SPI_CR1 register
	pSPI2->CR1 |= (1 << 6);

	uint32_t len = strlen(user_data); 	// length of user data
	uint8_t *p_tx_buffer = (uint8_t*) user_data;

	while (len > 0) {
		// wait until TXE (transmit buffer empty) is set with
		// SPI_SR register
		while(!(pSPI2->SR & (1 << 1)));

		// check DFF (data frame format) is 16-bit or 8-bit with
		// SPI_CR1 register
		if ((pSPI2->CR1 & (1 << 11))) {
			// load data into data register
			pSPI2->DR = *((uint16_t*) p_tx_buffer);
			len = len - 2;
			(uint16_t*) p_tx_buffer++;
		} else {
			pSPI2->DR = *p_tx_buffer;
			len--;
			p_tx_buffer++;
		}
	}

	// check busy flag of SPI with SPI_SR register
	while( ((pSPI2->SR) & (1 << 7)) );

	// disable SPI2 with SPI_CR1 register
	pSPI2->CR1 &= ~(1 << 6);

	pGPIOD->ODR ^= (1 << 15); // toggle GPIOD pin 5 output

    /* Loop forever */
	while(1);
}
