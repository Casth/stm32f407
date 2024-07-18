#include <stdint.h>
#include "stm32f407xx.h"

void SetSystemCLockTo16MHz(void);

int countercapture = 0;

int main(void)
{
    SetSystemCLockTo16MHz();

    /**
     * Set GPIO PD15 to alternative function mode
     */
    pRCC->AHB1ENR |= (1 << 3);      // turn on peripheral clock for GPIO port D
    pGPIOD->MODER |= (0b10 << 30);  // set GPIO PD15 in alternate function mode
    pGPIOD->AFRH |= (0b0010 << 28); // set GPIO PD15 to alternate function mode to act as TIM4_CH4 (AF2) according to alternate function mapping in STM32 datasheet

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
    pTIM4->CR1 = (1 << 0);         // enable TIM4 timer

    /**
     * Set GPIO PC6 to alternative function mode
     */
    pRCC->AHB1ENR |= (1 << 2);      // turn on peripheral clock for GPIO port C
    pGPIOC->MODER |= (0b10 << 12);  // set GPIO PC6 in alternate function mode
    pGPIOC->AFRL |= (0b0010 << 24); // set GPIO PC6 to alternate function mode to act as TIM3_CH1 (AF2) according to alternate function mapping in STM32 datasheet

    /**
     * Set timer TIM3 CH1 (at PC6) to input capture mode
     */
    pRCC->APB1ENR |= (1 << 1); // enable clock access to TIM3, which is on APB1 bus
    pTIM3->PSC = 800 - 1;      // set prescaler value
    // pTIM3->ARR = 1000 - 1;         // set auto-reload value
    pTIM3->CCMR1 &= ~(0b11 << 0);   // reset CC1S flag (least significant two bits)
    pTIM3->CCMR1 |= (0b01 << 0);    // set TIM3 CC1 channel to input mode
    pTIM3->CCMR1 &= ~(0b1111 << 4); // set TIM3 IC1F (input capture filter) to none filter
    pTIM3->CCER &= ~(0b11 << 1);    // set TIM3 CC1NP and CC1P to capture rising edge
    pTIM3->CCER |= (1 << 0);        // enable TIM3 CC1 channel
    pTIM3->CR1 |= (1 << 0);         // enable TIM3 timer

    /**
     * Main loop
     */
    while (1)
    {
        // wait until the edge is captured
        while (!((pTIM3->SR) & (1 << 1)))
            ;

        // read the captured value
        countercapture = pTIM3->CCR1;
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
