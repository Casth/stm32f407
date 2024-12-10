/*
 * stm32f407xx.h
 *
 * Created on: 2024.05.03
 * Author: Lin Li
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

#define PERIPH_BASEADDR 0x40000000U                  // peripheral base address
#define APB1PERIPH_BASEADDR PERIPH_BASEADDR          // APB1 base address
#define APB2PERIPH_BASEADDR 0x40010000U              // APB2 base address
#define AHB1PERIPH_BASEADDR 0x40020000U              // AHB1 base address
#define AHB2PERIPH_BASEADDR 0x50000000U              // AHB2 base address
#define EXTI_BASEADDR 0x40013C00U                    // EXTI register base address (on APB2)
#define SYSCFG_BASEADDR 0x40013800U                  // SYSCFG base address (on APB2)
#define RCC_BASEADDR 0x40023800U                     // RCC base address (on AHB1)
#define CRC_BASEADDR 0x40023000U                     // CRC base address (on AHB1)
#define FLASH_BASEADDR 0x40023C00U                   // Flash base address (on AHB1)
#define TIM2_BASEADDR (APB1PERIPH_BASEADDR + 0x0000) // TIM2 base address
#define TIM3_BASEADDR (APB1PERIPH_BASEADDR + 0x0400) // TIM3 base address
#define TIM4_BASEADDR (APB1PERIPH_BASEADDR + 0x0800) // TIM4 base address
#define TIM5_BASEADDR (APB1PERIPH_BASEADDR + 0x0C00) // TIM5 base address
#define CAN1_BASEADDR (APB1PERIPH_BASEADDR + 0x6400) // bxCAN1 base address
#define CAN2_BASEADDR (APB1PERIPH_BASEADDR + 0x6800) // bxCAN2 base address

/*
 * EXTI Register
 * source: https://www.learningaboutelectronics.com/Articles/C-structure-code-of-the-EXTI-register-map-STM32F407xx.php
 */
typedef struct
{
    volatile uint32_t IMR;
    volatile uint32_t EMR;
    volatile uint32_t RTSR;
    volatile uint32_t FTSR;
    volatile uint32_t SWIER;
    volatile uint32_t PR;
} EXTI_RegDef_t;
#define EXTI ((EXTI_RegDef_t *)EXTI_BASEADDR)
EXTI_RegDef_t *pEXTI = ((EXTI_RegDef_t *)EXTI_BASEADDR);

/*
 * SPI Register
 * source: https://www.learningaboutelectronics.com/Articles/C-structure-code-of-the-SPI-register-map-STM32F407xx.php
 */
#define SPI2_BASEADDR (APB1PERIPH_BASEADDR + 0x3800) // SPI2 base address (based on APB1)
#define SPI3_BASEADDR (APB1PERIPH_BASEADDR + 0x3C00) // SPI3 base address (based on APB1)
#define SPI1_BASEADDR (APB2PERIPH_BASEADDR + 0x3000) // SPI1 base address (based on APB2)
#define SPI4_BASEADDR (APB2PERIPH_BASEADDR + 0x3400) // SPI4 base address (based on APB2)
#define SPI5_BASEADDR (APB2PERIPH_BASEADDR + 0x5000) // SPI5 base address (based on APB2)
#define SPI6_BASEADDR (APB2PERIPH_BASEADDR + 0x5400) // SPI6 base address (based on APB2)
typedef struct
{
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t CRCPR;
    volatile uint32_t RXCRCR;
    volatile uint32_t TXCRCR;
    volatile uint32_t I2SCFGR;
    volatile uint32_t I2SPR;
} SPI_RegDef_t;
#define SPI1 ((SPI_RegDef_t *)SPI1_BASEADDR)
#define SPI2 ((SPI_RegDef_t *)SPI2_BASEADDR)
#define SPI3 ((SPI_RegDef_t *)SPI3_BASEADDR)
#define SPI4 ((SPI_RegDef_t *)SPI4_BASEADDR)
#define SPI5 ((SPI_RegDef_t *)SPI5_BASEADDR)
#define SPI6 ((SPI_RegDef_t *)SPI6_BASEADDR)
SPI_RegDef_t *pSPI1 = ((SPI_RegDef_t *)SPI1_BASEADDR);
SPI_RegDef_t *pSPI2 = ((SPI_RegDef_t *)SPI2_BASEADDR);
SPI_RegDef_t *pSPI3 = ((SPI_RegDef_t *)SPI3_BASEADDR);
SPI_RegDef_t *pSPI4 = ((SPI_RegDef_t *)SPI4_BASEADDR);
SPI_RegDef_t *pSPI5 = ((SPI_RegDef_t *)SPI5_BASEADDR);
SPI_RegDef_t *pSPI6 = ((SPI_RegDef_t *)SPI6_BASEADDR);

/*
 * I2C Register
 * source: https://www.learningaboutelectronics.com/Articles/C-structure-code-of-the-I2C-register-map-STM32F407xx.php
 */
#define I2C1_BASEADDR (APB1PERIPH_BASEADDR + 0x5400) // I2C1 base address (base on APB1)
#define I2C2_BASEADDR (APB1PERIPH_BASEADDR + 0x5800) // I2C2 base address (base on APB1)
#define I2C3_BASEADDR (APB1PERIPH_BASEADDR + 0x5C00) // I2C3 base address (base on APB1)
typedef struct
{
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t OAR1;
    volatile uint32_t OAR2;
    volatile uint32_t DR;
    volatile uint32_t SR1;
    volatile uint32_t SR2;
    volatile uint32_t CCR;
    volatile uint32_t TRISE;
    volatile uint32_t FLTR;
} I2C_RegDef_t;
#define I2C1 ((I2C_RegDef_t *)I2C1_BASEADDR)
#define I2C2 ((I2C_RegDef_t *)I2C2_BASEADDR)
#define I2C3 ((I2C_RegDef_t *)I2C3_BASEADDR)
I2C_RegDef_t *pI2C1 = ((I2C_RegDef_t *)I2C1_BASEADDR);
I2C_RegDef_t *pI2C2 = ((I2C_RegDef_t *)I2C2_BASEADDR);
I2C_RegDef_t *pI2C3 = ((I2C_RegDef_t *)I2C3_BASEADDR);

/*
 * USART Register
 * source: https://www.learningaboutelectronics.com/Articles/C-structure-code-of-the-USART-register-map-STM32F407xx.php
 */
#define USART2_BASEADDR (APB1PERIPH_BASEADDR + 0x4400) // USART2 base address (based on APB1)
#define USART3_BASEADDR (APB1PERIPH_BASEADDR + 0x4800) // USART3 base address (based on APB1)
#define USART4_BASEADDR (APB1PERIPH_BASEADDR + 0x4C00) // USART4 base address (based on APB1)
#define USART5_BASEADDR (APB1PERIPH_BASEADDR + 0x5000) // USART5 base address (based on APB1)
#define USART1_BASEADDR (APB2PERIPH_BASEADDR + 0x1000) // USART1 base address (based on APB2)
#define USART6_BASEADDR (APB2PERIPH_BASEADDR + 0x1400) // USART6 base address (based on APB2)
typedef struct
{
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t BRR;
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t CR3;
    volatile uint32_t GTPR;
} USART_RegDef_t;
#define USART1 ((USART_RegDef_t *)USART1_BASEADDR)
#define USART2 ((USART_RegDef_t *)USART2_BASEADDR)
#define USART3 ((USART_RegDef_t *)USART3_BASEADDR)
#define USART4 ((USART_RegDef_t *)USART4_BASEADDR)
#define USART5 ((USART_RegDef_t *)USART5_BASEADDR)
#define USART6 ((USART_RegDef_t *)USART6_BASEADDR)
USART_RegDef_t *pUSART1 = ((USART_RegDef_t *)USART1_BASEADDR);
USART_RegDef_t *pUSART2 = ((USART_RegDef_t *)USART2_BASEADDR);
USART_RegDef_t *pUSART3 = ((USART_RegDef_t *)USART3_BASEADDR);
USART_RegDef_t *pUSART4 = ((USART_RegDef_t *)USART4_BASEADDR);
USART_RegDef_t *pUSART5 = ((USART_RegDef_t *)USART5_BASEADDR);
USART_RegDef_t *pUSART6 = ((USART_RegDef_t *)USART6_BASEADDR);

/*
 * SYSCFG Register
 * source: https://www.learningaboutelectronics.com/Articles/C-structure-code-of-the-SYSCFG-register-map-STM32F407xx.php
 */
typedef struct
{
    volatile uint32_t MEMRMP;
    volatile uint32_t PMC;
    volatile uint32_t EXTICR1;
    volatile uint32_t EXTICR2;
    volatile uint32_t EXTICR3;
    volatile uint32_t EXTICR4;
    uint32_t RESERVED1[2];
    volatile uint32_t CMPCR;
} SYSCFG_RegDef_t;
#define SYSCFG ((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)
SYSCFG_RegDef_t *pSYSCFG = ((SYSCFG_RegDef_t *)SYSCFG_BASEADDR);

/*
 * GPIO Register
 * source: https://www.learningaboutelectronics.com/Articles/C-structure-code-of-the-GPIO-register-map-STM32F407xx.php
 */
#define GPIOA_BASEADDR (AHB1PERIPH_BASEADDR + 0x0000) // GPIOA base address (based on AHB1)
#define GPIOB_BASEADDR (AHB1PERIPH_BASEADDR + 0x0400) // GPIOB base address (based on AHB1)
#define GPIOC_BASEADDR (AHB1PERIPH_BASEADDR + 0x0800) // GPIOC base address (based on AHB1)
#define GPIOD_BASEADDR (AHB1PERIPH_BASEADDR + 0x0C00) // GPIOD base address (based on AHB1)
#define GPIOE_BASEADDR (AHB1PERIPH_BASEADDR + 0x1000) // GPIOE base address (based on AHB1)
#define GPIOF_BASEADDR (AHB1PERIPH_BASEADDR + 0x1400) // GPIOF base address (based on AHB1)
#define GPIOG_BASEADDR (AHB1PERIPH_BASEADDR + 0x1800) // GPIOG base address (based on AHB1)
#define GPIOH_BASEADDR (AHB1PERIPH_BASEADDR + 0x1C00) // GPIOH base address (based on AHB1)
#define GPIOI_BASEADDR (AHB1PERIPH_BASEADDR + 0x2000) // GPIOI base address (based on AHB1)
#define GPIOJ_BASEADDR (AHB1PERIPH_BASEADDR + 0x2400) // GPIOJ base address (based on AHB1)
#define GPIOK_BASEADDR (AHB1PERIPH_BASEADDR + 0x2800) // GPIOK base address (based on AHB1)
typedef struct
{
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDR;
    volatile uint32_t PUPDR;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t LCKR;
    volatile uint32_t AFRL;
    volatile uint32_t AFRH;
} GPIO_RegDef_t;
#define GPIOA ((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOF ((GPIO_RegDef_t *)GPIOF_BASEADDR)
#define GPIOG ((GPIO_RegDef_t *)GPIOG_BASEADDR)
#define GPIOH ((GPIO_RegDef_t *)GPIOH_BASEADDR)
#define GPIOI ((GPIO_RegDef_t *)GPIOI_BASEADDR)
#define GPIOJ ((GPIO_RegDef_t *)GPIOJ_BASEADDR)
#define GPIOK ((GPIO_RegDef_t *)GPIOK_BASEADDR)
GPIO_RegDef_t *pGPIOA = GPIOA;
GPIO_RegDef_t *pGPIOB = GPIOB;
GPIO_RegDef_t *pGPIOC = GPIOC;
GPIO_RegDef_t *pGPIOD = GPIOD;
GPIO_RegDef_t *pGPIOE = GPIOE;
GPIO_RegDef_t *pGPIOF = GPIOF;
GPIO_RegDef_t *pGPIOG = GPIOG;
GPIO_RegDef_t *pGPIOH = GPIOH;
GPIO_RegDef_t *pGPIOI = GPIOI;
GPIO_RegDef_t *pGPIOJ = GPIOJ;
GPIO_RegDef_t *pGPIOK = GPIOK;

/*
 * RCC Register
 * source: https://www.learningaboutelectronics.com/Articles/C-structure-code-of-the-RCC-register-map-STM32F407xx.php
 */
typedef struct
{
    volatile uint32_t CR;
    volatile uint32_t PLLCFGR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t AHB1RSTR;
    volatile uint32_t AHB2RSTR;
    volatile uint32_t AHB3RSTR;
    uint32_t RESERVED0;
    volatile uint32_t APB1RSTR;
    volatile uint32_t APB2RSTR;
    uint32_t RESERVED1[2];
    volatile uint32_t AHB1ENR;
    volatile uint32_t AHB2ENR;
    volatile uint32_t AHB3ENR;
    uint32_t RESERVED2;
    volatile uint32_t APB1ENR;
    volatile uint32_t APB2ENR;
    uint32_t RESERVED3[2];
    volatile uint32_t AHB1LPENR;
    volatile uint32_t AHB2LPENR;
    volatile uint32_t AHB3LPENR;
    uint32_t RESERVED4;
    volatile uint32_t APB1LPENR;
    volatile uint32_t APB2LPENR;
    uint32_t RESERVED5[2];
    volatile uint32_t BDCR;
    volatile uint32_t CSR;
    uint32_t RESERVED6[2];
    volatile uint32_t SSCGR;
    volatile uint32_t PLLI2SCFGR;
    volatile uint32_t PLLSAICFGR;
    volatile uint32_t DCKCFGR;
    volatile uint32_t CKGATENR;
    volatile uint32_t DCKCFGR2;
} RCC_RegDef_t;
#define RCC ((RCC_RegDef_t *)RCC_BASEADDR)
RCC_RegDef_t *pRCC = ((RCC_RegDef_t *)RCC_BASEADDR);

/*
 * Flash Register
 */
typedef struct
{
    volatile uint32_t ACR;
    volatile uint32_t KEYR;
    volatile uint32_t OPTKEYR;
    volatile uint32_t SR;
    volatile uint32_t CR;
    volatile uint32_t OPTCR;
} FLASH_RegDef_t;
#define FLASH ((FLASH_RegDef_t *)FLASH_BASEADDR)
FLASH_RegDef_t *pFLASH = ((FLASH_RegDef_t *)FLASH_BASEADDR);

/*
 * TIM Register (for TIM2-5)
 * source: http://www.learningaboutelectronics.com/Articles/How-to-create-a-delay-using-a-general-purpose-timer-STM32F446-C.php
 */
typedef struct
{
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SMCR;
    volatile uint32_t DIER;
    volatile uint32_t SR;
    volatile uint32_t EGR;
    volatile uint32_t CCMR1;
    volatile uint32_t CCMR2;
    volatile uint32_t CCER;
    volatile uint32_t CNT;
    volatile uint32_t PSC;
    volatile uint32_t ARR;
    uint32_t RESERVED0;
    volatile uint32_t CCR1;
    volatile uint32_t CCR2;
    volatile uint32_t CCR3;
    volatile uint32_t CCR4;
    uint32_t RESERVED1;
    volatile uint32_t DCR;
    volatile uint32_t DMAR;
    volatile uint32_t OR;
} TIM_RegDef_t;
#define TIM2 ((TIM_RegDef_t *)TIM2_BASEADDR)
#define TIM3 ((TIM_RegDef_t *)TIM3_BASEADDR)
#define TIM4 ((TIM_RegDef_t *)TIM4_BASEADDR)
#define TIM5 ((TIM_RegDef_t *)TIM5_BASEADDR)
TIM_RegDef_t *pTIM2 = ((TIM_RegDef_t *)TIM2_BASEADDR);
TIM_RegDef_t *pTIM3 = ((TIM_RegDef_t *)TIM3_BASEADDR);
TIM_RegDef_t *pTIM4 = ((TIM_RegDef_t *)TIM4_BASEADDR);
TIM_RegDef_t *pTIM5 = ((TIM_RegDef_t *)TIM5_BASEADDR);


/**
 * CAN Register
 * source: STM32F407 reference manual
*/
typedef struct {
    volatile uint32_t FR1;
    volatile uint32_t FR2;
} CAN_Filter_t;
typedef struct {
    volatile uint32_t TIR;
    volatile uint32_t TDTR;
    volatile uint32_t TDLR;
    volatile uint32_t TDHR;
} CAN_TxMailbox_t;
typedef struct {
    volatile uint32_t RIR;
    volatile uint32_t RDTR;
    volatile uint32_t RDLR;
    volatile uint32_t RDHR;
} CAN_RxMailbox_t;
typedef struct 
{
    volatile uint32_t MCR;
    volatile uint32_t MSR;
    volatile uint32_t TSR;
    volatile uint32_t RF0R;
    volatile uint32_t RF1R;
    volatile uint32_t IER;
    volatile uint32_t ESR;
    volatile uint32_t BTR;
    uint32_t RESERVED0[88];
    CAN_TxMailbox_t TxMailbox[3];
    // volatile uint32_t TI0R;
    // volatile uint32_t TDT0R;
    // volatile uint32_t TDL0R;
    // volatile uint32_t TDH0R;
    // volatile uint32_t TI1R;
    // volatile uint32_t TDT1R;
    // volatile uint32_t TDL1R;
    // volatile uint32_t TDH1R;
    // volatile uint32_t TI2R;
    // volatile uint32_t TDT2R;
    // volatile uint32_t TDL2R;
    // volatile uint32_t TDH2R;
    CAN_RxMailbox_t RxMailbox[2];
    // volatile uint32_t RI0R;
    // volatile uint32_t RDT0R;
    // volatile uint32_t RDL0R;
    // volatile uint32_t RDH0R;
    // volatile uint32_t RI1R;
    // volatile uint32_t RDT1R;
    // volatile uint32_t RDL1R;
    // volatile uint32_t RDH1R;
    uint32_t RESERVED1[12];
    volatile uint32_t FMR;
    volatile uint32_t FM1R;
    uint32_t RESERVED2;
    volatile uint32_t FS1R;
    uint32_t RESERVED3;
    volatile uint32_t FFA1R;
    uint32_t RESERVED4;
    volatile uint32_t FA1R;
    uint32_t RESERVED5;
    uint32_t RESERVED6[7];
    CAN_Filter_t Filter[28];
    // volatile uint32_t F0R1;
    // volatile uint32_t F0R2;
    // volatile uint32_t F1R1;
    // volatile uint32_t F1R2;
    // volatile uint32_t F2R1;
    // volatile uint32_t F2R2;
    // volatile uint32_t F3R1;
    // volatile uint32_t F3R2;
    // volatile uint32_t F4R1;
    // volatile uint32_t F4R2;
    // volatile uint32_t F5R1;
    // volatile uint32_t F5R2;
    // volatile uint32_t F6R1;
    // volatile uint32_t F6R2;
    // volatile uint32_t F7R1;
    // volatile uint32_t F7R2;
    // volatile uint32_t F8R1;
    // volatile uint32_t F8R2;
    // volatile uint32_t F9R1;
    // volatile uint32_t F9R2;
    // volatile uint32_t F10R1;
    // volatile uint32_t F10R2;
    // volatile uint32_t F11R1;
    // volatile uint32_t F11R2;
    // volatile uint32_t F12R1;
    // volatile uint32_t F12R2;
    // volatile uint32_t F13R1;
    // volatile uint32_t F13R2;
    // volatile uint32_t F14R1;
    // volatile uint32_t F14R2;
    // volatile uint32_t F15R1;
    // volatile uint32_t F15R2;
    // volatile uint32_t F16R1;
    // volatile uint32_t F16R2;
    // volatile uint32_t F17R1;
    // volatile uint32_t F17R2;
    // volatile uint32_t F18R1;
    // volatile uint32_t F18R2;
    // volatile uint32_t F19R1;
    // volatile uint32_t F19R2;
    // volatile uint32_t F20R1;
    // volatile uint32_t F20R2;
    // volatile uint32_t F21R1;
    // volatile uint32_t F21R2;
    // volatile uint32_t F22R1;
    // volatile uint32_t F22R2;
    // volatile uint32_t F23R1;
    // volatile uint32_t F23R2;
    // volatile uint32_t F24R1;
    // volatile uint32_t F24R2;
    // volatile uint32_t F25R1;
    // volatile uint32_t F25R2;
    // volatile uint32_t F26R1;
    // volatile uint32_t F26R2;
    // volatile uint32_t F27R1;
    // volatile uint32_t F27R2;
} CAN_RegDef_t;
#define CAN1 ((CAN_RegDef_t *)CAN1_BASEADDR)
#define CAN2 ((CAN_RegDef_t *)CAN2_BASEADDR)
CAN_RegDef_t *pCAN1 = ((CAN_RegDef_t *)CAN1_BASEADDR);
CAN_RegDef_t *pCAN2 = ((CAN_RegDef_t *)CAN2_BASEADDR);

typedef struct CanBusTxStdFrame {
    uint32_t identifier;
    uint8_t length;
    uint8_t data[8];
} CAN_Tx_StdFrame_t;

typedef struct CanBusRxStdFrame {
    uint32_t identifier;
    uint8_t length;
    uint8_t data[8];
} CAN_Rx_StdFrame_t;

/*
 * CRC Register
 */
typedef struct
{
    volatile uint32_t DR;
    volatile uint32_t IDR;
    volatile uint32_t CR;
} CRC_RegDef_t;
#define CRC ((CRC_RegDef_t *)CRC_BASEADDR)
CRC_RegDef_t *pCRC = ((CRC_RegDef_t *)CRC_BASEADDR);

#endif /* INC_STM32F407XX_H_ */