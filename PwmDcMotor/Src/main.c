#include <stdint.h>
#include "stm32f407xx.h"

void Timer4Init(void);
void delay(int ms);
void SetSystemCLockTo16MHz(void);

int main(void)
{
    SetSystemCLockTo16MHz();
    Timer4Init();

    TIM4->CCR3 = 0;   // reset CCR for CH3
    TIM4->CCR4 = 0;   // reset CCR for CH4

    while (1) {
        for (int i=0; i<255; i++) {
            TIM4->CCR3 = i;   // increase CH3 (on PD14)
            delay(10);
        }
        for (int i=255; i>0; i--) {
            TIM4->CCR3 = i;   // decrease CH3 (on PD14)
            delay(10);
        }

        delay(20);    // pause
        
        for (int i=0; i<255; i++) {
            TIM4->CCR4 = i;   // increase CH4 (on PD15)
            delay(10);
        }
        for (int i=255; i>0; i--) {
            TIM4->CCR4 = i;   // decrease CH4 (on PD15)
            delay(10);
        }

        delay(20);    // pause
    }
}

/**
 * Configure PD14 and PD15 to PWM output
*/
void Timer4Init(void)
{
    RCC->AHB1ENR |= (1 << 3); // turn on peripheral clock for GPIO port D

    GPIOD->MODER |= (0b10 << 28 | 0b10 << 30); // set GPIO PD14 and PD15 in alternate function mode
    GPIOD->AFRH |= (0b0010 << 24 | 0b0010 << 28); // set GPIO PD14 and PD15 to AF2 to act as TIM4_CH3 and CH4

    RCC->APB1ENR |= (1 << 2); // enable clock access to TIM4, which is on APB1 bus
    TIM4->PSC = 0; // set prescaler value
    TIM4->ARR = 255; // set auto-reload value

	TIM4->CCMR2 &= ~(0b11 << 1 | 0b11 << 8);   // select CC3 and CC4 channel as output
    TIM4->CCMR2 |= (0b110 << 4 | 0b110 << 12); // set output compare to PWM mode 1
    TIM4->CCER |= (1 << 8 | 1 << 12); // enable TIM4 CH4 output in compare mode
    TIM4->CNT = 0; // clear the counter
    TIM4->CR1 |= (1 << 0); // enable timer 4
}

void delay(int ms) {
    int i;
    for (; ms>0; ms--) {
        for (i=0; i<3195; i++);
    }
}

void SetSystemCLockTo16MHz(void)
{
    // enable HSI clock
    if ((RCC->CR & (1 << 1)) == 0)
    {
        RCC->CR |= (1 << 0); // set HSION = 1

        while ((RCC->CR & (1 << 1)) == 0)
            ; // wait until HSI clock is ready
    }

    RCC->CFGR &= ~(0x0F << 4);  // set AHB prescaler to 1 (HPRE)
    RCC->CFGR &= ~(0x07 << 10); // set APB1 prescaler to 1 (PPRE1)
    RCC->CFGR &= ~(0x07 << 13); // set APB1 prescaler to 1 (PPRE2)
    RCC->CFGR &= ~(0x03 << 0);  // set HSI as system clock source (SW)

    FLASH->ACR |= (1 << 9);     // enable instruction cache (ICEN)
    FLASH->ACR |= (1 << 10);    // enable data cache (DCEN)
    FLASH->ACR &= ~(0x07 << 0); // reset latency (LATENCY)
    FLASH->ACR |= (0b011 << 0); // set latency to three wait states (LATENCY)

    RCC->CR &= ~(1 << 16); // disable HSE clock (HSEON)
}
