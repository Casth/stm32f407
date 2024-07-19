#include <stdint.h>
#include "stm32f407xx.h"

void SetSystemCLockTo16MHz(void);

int main() {
    SetSystemCLockTo16MHz();

    pRCC->AHB1ENR |= (1 << 3); // turn on peripheral clock for GPIO port D

    pGPIOD->MODER |= (0b10 << 30); // set GPIO PD15 in alternate function mode
    pGPIOD->AFRH |= (0b0010 << 28); // set GPIO PD15 to alternate function mode to act as TIM4_CH4 (AF2) according to alternate function mapping in STM32 datasheet

    pRCC->APB1ENR |= (1 << 2); // enable clock access to TIM4, which is on APB1 bus
    pTIM4->PSC = 16000 - 1; // set prescaler value
    pTIM4->ARR = 1000 - 1; // set auto-reload value

	pTIM4->CCMR2 &= ~(0b11 << 8);   // select CC4 channel as output with CC4S
    pTIM4->CCMR2 |= (0b110 << 12); // set output compare to PWM mode 1 with TIM4_CCMR2 OC4M register
    pTIM4->CCER |= (1 << 12); // enable TIM4 CH4 output in compare mode with TIM4_CCER CC4E register
    pTIM4->CNT = 0; // clear the counter
    pTIM4->CCR4 = 200;	// set CCR1 in ratio to AAR, which corresponds to the duty cycle
    pTIM4->CR1 |= (1 << 0); // enable timer 4

	return 0;
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
