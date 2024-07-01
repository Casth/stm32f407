#include <stdint.h>
#include "stm32f407xx.h"

void SetSystemCLockTo16MHz(void);

int countercapture = 0;

int main(void)
{
    SetSystemCLockTo16MHz();

    pRCC->AHB1ENR |= (1 << 3); // turn on peripheral clock for GPIO port D

    /**
     * Set GPIO PD15 to alternative function mode
     */
    pGPIOD->MODER |= (0b10 << 30);  // set GPIO PD15 in alternate function mode
    pGPIOD->AFRH |= (0b0010 << 28); // set GPIO PD15 to alternate function mode to act as TIM4_CH4 (AF2) according to alternate function mapping in STM32 datasheet

    /**
     * Set GPIO PD12 to alternative function mode
     */
    pGPIOD->MODER |= (0b10 << 24);  // set GPIO PD12 in alternate function mode
    pGPIOD->AFRH |= (0b0010 << 16); // set GPIO PD12 to alternate function mode to act as TIM4_CH1 (AF2) according to alternate function mapping in STM32 datasheet

    /**
     * Set timer TIM4 CH4 (at PD15) to output compare mode with output toggle
     */
    pRCC->APB1ENR |= (1 << 2);     // enable clock access to TIM4, which is on APB1 bus
    pTIM4->PSC = 8000 - 1;         // set prescaler value
    pTIM4->ARR = 1000 - 1;         // set auto-reload value
    pTIM4->CCMR2 &= ~(0b11 << 8);  // select CC4 channel as output with CC4S
    pTIM4->CCMR2 |= (0b011 << 12); // set output compare to toggle mode with TIM4_CCMR2 OC4M register
    pTIM4->CCER |= (1 << 12);      // enable TIM4 CH4 output in compare mode with TIM4_CCER CC4E register
    pTIM4->CNT = 0;                // clear the counter
    // pTIM4->CR1 |= (1 << 0);        // enable timer 4

    /**
     * Set timer TIM4 CH1 (at PD12) to input capture mode
     */
    pTIM4->CCMR1 = (0b01 << 0); // set TIM4 CC1 channel to input mode
    pTIM4->CCER = (1 << 0);     // enable TIM4 CC1 channel
    pTIM4->CR1 = (1 << 0);      // enable TIM4 timer
    
    /**
     * Main loop
    */
    while(1) {
    	// wait until the edge is captured
    	while(!((pTIM4->SR) & (1 << 1)));

        // read the captured value
        countercapture = pTIM4->CCR1;
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
