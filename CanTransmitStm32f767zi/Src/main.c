#include <stdint.h>
#include <string.h>
#include "stdlib.h"
#include "stm32f76xxx.h"

#define FLAG_LOOPBACK 0
#define ID_CAN_TX 0x108
#define ID_CAN_RX 0x220

void EnableCan1Pins(void);
void EnableLedPins(void);
void InitCan1(void);
void ConfigCan1Filter(void);
void SendCan1Message(CAN_Tx_StdFrame_t *tx_frame);
uint16_t ReceiveCan1Message(CAN_Rx_StdFrame_t *rx_frame);
void StartCan1(void);
void SetSystemCLockTo16MHz(void);

int main(void)
{
    SetSystemCLockTo16MHz();

    /**
     * Pin usage:
     * PB9: CAN1_TX (AF9)
     * PB8: CAN1_RX (AF9)
     *
     *
     * Bus:
     * bxCAN1 is located on APB1 (AHB1)
     *
     * Bit timing:
     * CAN baud rate: 500 kBit/s -> t_bit = 0.002 ms = 2µs
     * STM32 clock frequency: 16 MHz -> t_clk = 0.0000625 ms = 0.0625 µs
     * -> duration of 1 bit = 32 clock cycles
     * set prescaler (CAN_BTR > BRP bits) to 2
     * -> 1 TQ = 2 clock cycles
     * SyncSeg:    1 TQ
     * Segment 1: 11 TQ
     * Segment 2:  4 TQ
     * SJW:        2 TQ
     *
     * Filter bank:
     * Use bank 0 for CAN1
     * Rx Filter accepts message with identifier 0x220
     *
     * Transmit:
     * Tx sends message with identifier 0x108
     */

    EnableCan1Pins(); // enable clock for CAN peripheral
    EnableLedPins();
    InitCan1();         // CAN initialization mode
    ConfigCan1Filter(); // filter configuration
    StartCan1();        // start CAN1 by leaving initialization mode

    // declare Tx and Rx message
    CAN_Tx_StdFrame_t tx_frame;
    tx_frame.identifier = ID_CAN_TX;
    tx_frame.length = 8;
    for (int i = 0; i < 8; i++)
    {
        tx_frame.data[i] = (uint8_t)i;
    }
    CAN_Rx_StdFrame_t rx_frame;

    /* Loop forever */
    uint16_t sum_received = 0; // cumulated value of received messages
    while (1)
    {
        // set Tx data frame
        // for (int i = 0; i < 8; i++)
        // {
        //     tx_frame.data[i] = (uint8_t)(rand() % 255);
        // }

        // transmit message
        SendCan1Message(&tx_frame);

        // receive message
        sum_received += ReceiveCan1Message(&rx_frame);

        if (sum_received >= 60000)
        {
            sum_received = 0; // reset to prevent overflow
            GPIOB->ODR ^= (1 << 7);
        }
    }
}

void EnableCan1Pins(void)
{
    // AHB1 activation
    RCC->AHB1ENR |= (1 << 1); // enable GPIOB on AHB1 bus

    // PB9 configuration to AF9
    GPIOB->MODER &= ~(0b11 << 18); // reset PB9 mode
    GPIOB->MODER |= (0b10 << 18);  // set PB9 to alternate mode
    GPIOB->AFRH &= ~(0b1111 << 4); // reset PB9 alternative function
    GPIOB->AFRH |= (0b1001 << 4);  // set PB9 to AF9

    // PB8 configuration to AF9
    GPIOB->MODER &= ~(0b11 << 16); // reset PB8 mode
    GPIOB->MODER |= (0b10 << 16);  // set PB8 to alternate mode
    GPIOB->AFRH &= ~(0b1111 << 0); // reset PB8 alternative function
    GPIOB->AFRH |= (0b1001 << 0);  // set PB8 to AF9
}

void EnableLedPins(void)
{
    // AHB1 activation
    RCC->AHB1ENR |= (1 << 1); // enable GPIOB on AHB1 bus

    // PB7: blue LED
    GPIOB->MODER &= ~(0b11 << 14); // reset PB7 mode
    GPIOB->MODER |= (0b01 << 14);  // set PB7 to output mode
}

void InitCan1(void)
{
    // APB1 activation
    RCC->APB1ENR |= (1 << 25); // enable CAN1 on APB1 bus

    RCC->APB1RSTR |= (1 << 25);  // reset CAN1
    RCC->APB1RSTR &= ~(1 << 25); // release CAN1 from reset

    // CAN1 initialization
    CAN1->MCR |= (1 << 0); // set INRQ bit to trigger initialization mode
    while (((CAN1->MSR) & (1 << 0)) == 0U)
        ; // wait until INAK bit to be set
          // (0: not in init mode, 1: init mode)

    CAN1->MCR &= ~(1 << 1); // reset SLEEP bit to explicitly exit sleep mode
    while (((CAN1->MSR) & (1 << 1)) != 0U)
        ; // wait until SLAK bit to be reset
          // (0: not in sleep mode, 1: sleep mode)

    // >>>>> CAN bit timing <<<<<
    CAN1->BTR &= ~(0x3FFFFFF); // Clear all fields of BTR: BRP, TS1, TS2, SJW
    CAN1->BTR |= (1 << 0);     // set BRP[9:0] = 1, and therefore t_q = 2 * t_pclk
    CAN1->BTR |= (10 << 16);   // set TS1[3:0] = 10 (time segment 1), and therefore t_bs1 = 11 * t_q
    CAN1->BTR |= (3 << 20);    // set TS2[2:0] = 3 (time segment 2), and therefore t_bs2 = 4 * t_q
    CAN1->BTR |= (1 << 24);    // set SJW[1:0] = 1 (synchronization jump width), and therefore t_rjw = 2 * t_q

    // debug: enable loop back mode
    if (FLAG_LOOPBACK)
    {
        CAN1->BTR |= (1 << 30); // set LBKM bit to enable loop back mode
    }
}

void ConfigCan1Filter(void)
{
    CAN1->FMR |= (1 << 0);        // set FINIT to enter filters init mode
    CAN1->FMR &= ~(0b11111 << 8); // reset CAN2SB[5:0]
    CAN1->FMR |= (0b01111 << 8);  // set CAN2SB[5:0] = 15 to
                                  // assign filter bank 15-28 to CAN2
                                  // and assign bank 1-14 to CAN1
    CAN1->FA1R &= ~(1 << 0);      // deactivate filter bank 0
    CAN1->FS1R |= (1 << 0);       // set filter bank 0 to 32-bit scale
    CAN1->FM1R &= ~(1 << 0);      // set filter bank 0 to identifier mask mode
    // CAN1->Filter[0].FR1 = (ID_CAN_RX << 5) << 16; // set identifier ID to Tx message ID
    // CAN1->Filter[0].FR2 = (ID_CAN_RX << 5) << 16; // set identifier mask to Tx message ID
    CAN1->Filter[0].FR1 = (((ID_CAN_TX) & (ID_CAN_RX)) << 5) << 16; // set mask to allow Tx and Rx messages
    CAN1->Filter[0].FR2 = (((ID_CAN_TX) & (ID_CAN_RX)) << 5) << 16; // set mask to allow Tx and Rx messages
    // CAN1->Filter[0].FR1 = ~(0xFFFFFFFF);                            // set mask to allow all messages
    // CAN1->Filter[0].FR2 = ~(0xFFFFFFFF);                            // set mask to allow all messages
    CAN1->FFA1R &= ~(1 << 0);                                       // assign filter bank to FIFO 0
    CAN1->FA1R |= (1 << 0);                                         // activate filter bank 0
    CAN1->FMR &= ~(1 << 0);                                         // reset FINIT to exit filter init mode
}

void SendCan1Message(CAN_Tx_StdFrame_t *tx_frame)
{
    while (((CAN1->TSR) & (1 << 26)) == 0U)
        ; // wait until TME0 bit (Tx mailbox 0) to be set

    // set identifier
    CAN1->TxMailbox[0].TIR &= ~(0x7FF << 21);               // reset STID[10:0]
    CAN1->TxMailbox[0].TIR |= (tx_frame->identifier << 21); // set standard identifier to 0x108

    // set data length
    CAN1->TxMailbox[0].TDTR &= ~(0b1111 << 0);          // reset DLC[3:0]
    CAN1->TxMailbox[0].TDTR |= (tx_frame->length << 0); // set DLC[3:0] to 8

    // set data
    CAN1->TxMailbox[0].TDLR = (tx_frame->data[3] << 24 | tx_frame->data[2] << 16 | tx_frame->data[1] << 8 | tx_frame->data[0] << 0); // set DATA3/2/1/0[7:0]
    CAN1->TxMailbox[0].TDHR = (tx_frame->data[7] << 24 | tx_frame->data[6] << 16 | tx_frame->data[5] << 8 | tx_frame->data[4] << 0); // set DATA7/6/5/4[7:0]

    CAN1->TxMailbox[0].TIR |= (1 << 0); // set TXRQ bit to request transmission in mailbox
}

uint16_t ReceiveCan1Message(CAN_Rx_StdFrame_t *rx_frame)
{
    uint16_t sum = 0;

    if ((CAN1->RF0R & (0b11 << 0)) != 0U) // check FMP0[1:0] whether FIFO 0 message is pending
    {
        rx_frame->identifier = (CAN1->RxMailbox[0].RIR >> 21) & 0x7FF; // get message identifier
        rx_frame->length = (CAN1->RxMailbox[0].RDTR & 0b1111);         // get data length
        for (int i = 0; i < 8; i++)
        {
            rx_frame->data[i] = 0;
        }
        rx_frame->data[0] = (CAN1->RxMailbox[0].RDLR >> 0) & 0xFF;
        rx_frame->data[1] = (CAN1->RxMailbox[0].RDLR >> 8) & 0xFF;
        rx_frame->data[2] = (CAN1->RxMailbox[0].RDLR >> 16) & 0xFF;
        rx_frame->data[3] = (CAN1->RxMailbox[0].RDLR >> 24) & 0xFF;
        rx_frame->data[4] = (CAN1->RxMailbox[0].RDHR >> 0) & 0xFF;
        rx_frame->data[5] = (CAN1->RxMailbox[0].RDHR >> 8) & 0xFF;
        rx_frame->data[6] = (CAN1->RxMailbox[0].RDHR >> 16) & 0xFF;
        rx_frame->data[7] = (CAN1->RxMailbox[0].RDHR >> 24) & 0xFF;

        for (int i = 0; i < 8; i++)
        {
            sum += (uint16_t)(rx_frame->data[i]);
        }

        CAN1->RF0R |= (1 << 5); // set RFOM0 to release FIFO 0 mailbox
    }

    return sum;
}

void StartCan1(void)
{
    CAN1->MCR &= ~(1 << 0); // reset INRQ bit to exit initialization mode
    while (((CAN1->MSR) & (1 << 0)) == 1U)
        ; // wait until INAK bit to be reset
}

void SetSystemCLockTo16MHz(void)
{
    // enable HSI clock
    if ((RCC->CR & (1 << 1)) == 0U)
    {
        RCC->CR |= (1 << 0); // set HSION = 1

        while ((RCC->CR & (1 << 1)) == 0U)
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
