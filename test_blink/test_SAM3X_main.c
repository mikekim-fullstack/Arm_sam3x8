/* ========================================================================== */
/*                                                                            */
/*   main.c                                                                   */
/*   (c) 2020.08.16 MIKE KIM                                                   */
/*                                                                            */
/*   Description                                                              */
/*                                                                            */
/*   Minimal blink for SAM3X (Arduino Due)                                    */
/* ========================================================================== */

#include <stdint.h>
#include "stdio.h"
#include "sam.h"
#include "mkZoeRobotics_serial.h"
 //#include "wiring_constants.h"
 /*
#define F_CPU 84000000L
#define __SAM3X8E__
#include "sam.h"

// Pin 13 mask
uint32_t led = (1u << 27);

void setup();
void loop();
void wait();

int main() {
    setup();
    while (1)
        loop();

    return 0;
}

void setup()
{
    // Output Enable Register
    REG_PIOB_OER = led;
}

// the loop routine runs over and over again forever:

void loop()
{
    REG_PIOB_SODR = led; // Set Output Data Register, turns LED on
    wait();
    REG_PIOB_CODR = led; // Clear Output Data Register, turns LED off
    wait();
}

void wait() {
    int n;
    for (n = 0; n < F_CPU/40; n++)
        asm volatile ("nop\n\t");
}
*/
// #include <usart.h>
extern MKSerial mkSerial;
volatile uint8_t data;
void serialEvent() __attribute__((weak));
void serialEvent() { }

static int uart_putchar(const uint8_t c);
void UART_Handler (void) __attribute__ ((weak));
void UART_Handler(void) {
 
  if (UART->UART_SR & UART_SR_RXRDY) {

    data = UART->UART_RHR;
    uart_putchar(data);
  }
}

void setup() {

  uint32_t ul_sr;

    // ==> Pin configuration
    // Disable interrupts on Rx and Tx
    PIOA->PIO_IDR = PIO_PA8A_URXD | PIO_PA9A_UTXD;

    // Disable the PIO of the Rx and Tx pins so that the peripheral controller can use them
    PIOA->PIO_PDR = PIO_PA8A_URXD | PIO_PA9A_UTXD;

    // Read current peripheral AB select register and set the Rx and Tx pins to 0 (Peripheral A function)
    // ul_sr = PIOA->PIO_ABSR;
    // PIOA->PIO_ABSR &= ~(PIO_PA8A_URXD | PIO_PA9A_UTXD) & ul_sr;

    // // Enable the pull up on the Rx and Tx pin
    // PIOA->PIO_PUER = PIO_PA8A_URXD | PIO_PA9A_UTXD;

    // ==> Actual uart configuration
    // Enable the peripheral uart controller
    PMC->PMC_PCER0 = 1 << ID_UART;

    // Reset and disable receiver and transmitter
    UART->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX | UART_CR_RXDIS | UART_CR_TXDIS;

    // Set the baudrate
    UART->UART_BRGR = 21; // 84000000 / 16 * x = BaudRate (write x into UART_BRGR)

    // No Parity
    UART->UART_MR = UART_MR_PAR_NO | UART_MR_CHMODE_NORMAL;

    // Disable PDC channel
    UART->UART_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;

    // Configure interrupts
    UART->UART_IDR = 0xFFFFFFFF;
    UART->UART_IER = UART_IER_RXRDY | UART_IER_OVRE | UART_IER_FRAME;

    // Enable UART interrupt in NVIC
    NVIC_EnableIRQ((IRQn_Type) ID_UART);

    // Enable receiver and transmitter
    UART->UART_CR = UART_CR_RXEN | UART_CR_TXEN;
}
static int uart_putchar(const uint8_t c)
{
    // Check if the transmitter is ready
    if((UART->UART_SR & UART_SR_TXRDY) != UART_SR_TXRDY)
    return 1;

    // Send the character
    UART->UART_THR = c;
    while(!((UART->UART_SR) & UART_SR_TXEMPTY)); // Wait for the charactere to be send
    return 0;
}

int main()
{
  SystemInit();
  //setup();
  mkSerial.begin(250000);
  uart_putchar('Z');
  while(1){

  }
  // while(1)
  // {
  //   while ((UART->UART_SR & UART_SR_TXRDY ) == 0);
  //   UART->UART_THR = data;
  //   //delay(500);
  // }
}


/*

#define USE_PIO_INTERRUPT
// LED on B27 (Arduino Due)
#define LED_PIN 27

// Output on B14 (Duet)
//#define LED_PIN 14
void ISR1() {
  
 
  static uint32_t Count;
 
  if (Count++ > 210000) {  // MCK/2/TC_RC
    Count = 0;
    PIOB->PIO_ODSR ^= PIO_ODSR_P27;   // Toggle LED with a 1 Hz frequency

  }
}

Usart usart;
int main()
{
//USART_Configure(&usart, )
 noInterrupts(); // disable interrupts
 // some code
 interrupts(); // enable interrupts
 
 
  PMC->PMC_PCER0 |= PMC_PCER0_PID12;  // PIOB power ON
  PIOB->PIO_OER |= PIO_OER_P27;
  //PIOB->PIO_OWER |= PIO_OWER_P27;

  PMC->PMC_PCER0 |= PMC_PCER0_PID11;  // PIOA power ON

  // *************  Timer Counter 0 Channel 0 to generate PWM pulses thru TIOA0  ***********
  PMC->PMC_PCER0 |= PMC_PCER0_PID27;                      // TC0 power ON - Timer Counter 0 channel 0 IS TC0

  PIOB->PIO_PDR |= PIO_PDR_P25;
  PIOB->PIO_ABSR |= PIO_ABSR_P25;

  TC0->TC_CHANNEL[0].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1  // MCK/2, clk on rising edge
                              | TC_CMR_WAVE               // Waveform mode
                              | TC_CMR_WAVSEL_UP_RC        // UP mode with automatic trigger on RC Compare
                              | TC_CMR_ACPA_CLEAR          // Clear TIOA0 on RA compare match
                              | TC_CMR_ACPC_SET;           // Set TIOA0 on RC compare match

  TC0->TC_CHANNEL[0].TC_RC = 200;  //<*********************  Frequency = (Mck/128)/TC_RC  Hz
  TC0->TC_CHANNEL[0].TC_RA = 10;  //<********************   Duty cycle = (TC_RA/TC_RC) * 100  %

  TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_SWTRG | TC_CCR_CLKEN; // Software trigger TC0 counter and enable

  //attachInterrupt(24, ISR1, RISING);
#ifndef USE_PIO_INTERRUPT
  NVIC_DisableIRQ(PIOA_IRQn);
#endif


while(1) {
  #ifndef USE_PIO_INTERRUPT
    if (PIOA->PIO_ISR & PIO_ISR_P15) ISR1();
  #endif

}

  return 0;
}
*/
////////////////////////////////////////////////////////////////////////

// // #pragma import(__use_no_semihosting_swi)
// // struct __FILE { int handle; };
// // FILE __stdout;
// // LED on B27 (Arduino Due)
// #define LED_PIN 27

// // Output on B14 (Duet)
// //#define LED_PIN 14

// // PMC definitions
// #define PMC_PCER0 *(volatile uint32_t *)0x400E0610

// #define ID_PIOA 11
// #define ID_PIOB 12
// #define ID_PIOC 13
// #define ID_PIOD 14
// #define ID_PIOE 15
// #define ID_PIOF 16

// #define PMC_WPMR *(volatile uint32_t *)0x400E06E4

// #define PMC_WPKEY 0x504D43

// // PIO definitions

// struct gpio {
//   // + 0x00
//   volatile uint32_t PIO_PER;
//   volatile uint32_t PIO_PDR;
//   volatile uint32_t PIO_PSR;
//   volatile uint32_t res1;
//   // + 0x10
//   volatile uint32_t PIO_OER;
//   volatile uint32_t PIO_ODR;
//   volatile uint32_t PIO_OSR;
//   volatile uint32_t res2;
//   // + 0x20
//   volatile uint32_t PIO_IFER;
//   volatile uint32_t PIO_IFDR;
//   volatile uint32_t PIO_IFSR;
//   volatile uint32_t res3;
//   // + 0x30
//   volatile uint32_t PIO_SODR;
//   volatile uint32_t PIO_CODR;
//   volatile uint32_t PIO_ODSR;
//   volatile uint32_t PIO_PDSR;
//   // + 0x40
//   volatile uint32_t PIO_IER;
//   volatile uint32_t PIO_IDR;
//   volatile uint32_t PIO_IMR;
//   volatile uint32_t PIO_ISR;
//   // + 0x50
//   volatile uint32_t PIO_MDER;
//   volatile uint32_t PIO_MDDR;
//   volatile uint32_t PIO_MDSR;
//   volatile uint32_t res4;
//   // + 0x60
//   volatile uint32_t PIO_PUDR;
//   volatile uint32_t PIO_PUER;
//   volatile uint32_t PIO_PUSR;
//   volatile uint32_t res5;
//   // + 0x70
//   volatile uint32_t PIO_ABSR;
//   volatile uint32_t res6[3];
//   // + 0x80
//   volatile uint32_t PIO_SCIFSR;
//   volatile uint32_t PIO_DIFSR;
//   volatile uint32_t PIO_IFDGSR;
//   volatile uint32_t PIO_SCDR;
//   // + 0x90
//   volatile uint32_t res7[4];
//   // + 0xA0
//   volatile uint32_t PIO_OWER;
//   volatile uint32_t PIO_OWDR;
//   volatile uint32_t PIO_OWSR;
//   volatile uint32_t res8;
//   // ...
// };

// #define PIOA ((struct gpio *)0x400E0E00)
// #define PIOB ((struct gpio *)0x400E1000)
// #define PIOC ((struct gpio *)0x400E1200)
// #define PIOD ((struct gpio *)0x400E1400)
// #define PIOE ((struct gpio *)0x400E1600)
// #define PIOF ((struct gpio *)0x400E1800)

// #define PIOA_WPMR *(volatile uint32_t *)0x400E0EE4
// #define PIOB_WPMR *(volatile uint32_t *)0x400E10E4
// #define PIOC_WPMR *(volatile uint32_t *)0x400E12E4
// #define PIOD_WPMR *(volatile uint32_t *)0x400E14E4
// #define PIOE_WPMR *(volatile uint32_t *)0x400E16E4

// #define PIO_WPKEY 0x50494F


// #define CR 0x0D // Carriage return
// #define LF 0x0A // Linefeed
// void Uart0Init(void);
// void SetClockFreq(void);
// int sendchar(int ch);
// // Comment out the following line to use 6MHz clock
// #define CLOCK50MHZ
// // Register addresses
// #define SYSCTRL_RCC *((volatile unsigned long *)(0x400FE060))
// #define SYSCTRL_RIS *((volatile unsigned long *)(0x400FE050))
// #define SYSCTRL_RCGC1 *((volatile unsigned long *)(0x400FE104))
// #define SYSCTRL_RCGC2 *((volatile unsigned long *)(0x400FE108))
// #define GPIOPA_AFSEL *((volatile unsigned long *)(0x40004420))
// #define UART0_DATA *((volatile unsigned long *)(0x4000C000))
// #define UART0_FLAG *((volatile unsigned long *)(0x4000C018))
// #define UART0_IBRD *((volatile unsigned long *)(0x4000C024))
// #define UART0_FBRD *((volatile unsigned long *)(0x4000C028))
// #define UART0_LCRH *((volatile unsigned long *)(0x4000C02C))
// #define UART0_CTRL *((volatile unsigned long *)(0x4000C030))
// #define UART0_RIS *((volatile unsigned long *)(0x4000C03C))
// #define NVIC_CCR *((volatile unsigned long *)(0xE000ED14))


// // #define interrupts() __enable_irq()
// // #define noInterrupts() __disable_irq()

// void HardwareInit (void)
// {
//   // noInterrupts();
//   // enable peripheral clock
// 	//  PMC_WPMR  = PMC_WPKEY << 8 | 0;
//   PMC_PCER0 = 1 << ID_PIOB;

//   // PIOC
// 	//  PIOC_WPMR  = PIO_WPKEY << 8 | 0;

//   PIOB->PIO_PER = 1 << LED_PIN;
//   PIOB->PIO_OER = 1 << LED_PIN;
//   PIOB->PIO_OWER = 1 << LED_PIN;
// }

// void delay (volatile uint32_t time)
// {
//   while (time--)
//     __asm ("nop");
// }

// int main()
// {
//   HardwareInit ();
//   // Simple code to output hello world message
//   NVIC_CCR = NVIC_CCR | 0x200; // Set STKALIGN
//   SetClockFreq(); // Setup clock setting (50MHz/6MHz)
//   Uart0Init(); // Initialize Uart0
//   printf ("Hello world!\n");
//   while (1)
//   {
//     PIOB->PIO_SODR = 1 << LED_PIN;
//     delay (1000000);
//     PIOB->PIO_CODR = 1 << LED_PIN;
//     delay (1000000);
//   }
//   return 0;
// }
// void SetClockFreq(void)
// {
//   #ifdef CLOCK50MHZ
//   // Set BYPASS, clear USRSYSDIV and SYSDIV
//   SYSCTRL_RCC = (SYSCTRL_RCC & 0xF83FFFFF) | 0x800 ;
//   // Clr OSCSRC, PWRDN and OEN
//   SYSCTRL_RCC = (SYSCTRL_RCC & 0xFFFFCFCF);
//   // Change SYSDIV, set USRSYSDIV and Crystal value
//   SYSCTRL_RCC = (SYSCTRL_RCC & 0xF87FFC3F) | 0x01C002C0;
//   // Wait until PLLLRIS is set
//   while ((SYSCTRL_RIS & 0x40)==0); // wait until PLLLRIS is set
//   // Clear bypass
//   SYSCTRL_RCC = (SYSCTRL_RCC & 0xFFFFF7FF) ;
//   #else
//   // Set BYPASS, clear USRSYSDIV and SYSDIV
//   SYSCTRL_RCC = (SYSCTRL_RCC & 0xF83FFFFF) | 0x800 ;
//   #endif
//   return;
// }
// void Uart0Init(void)
// {
//   SYSCTRL_RCGC1 = SYSCTRL_RCGC1 | 0x0003; // Enable UART0 & UART1 clock
//   SYSCTRL_RCGC2 = SYSCTRL_RCGC2 | 0x0001; // Enable PORTA clock
//   UART0_CTRL = 0; // Disable UART
//   #ifdef CLOCK50MHZ
//   UART0_IBRD = 27; // Program baud rate for 50MHz clock
//   UART0_FBRD = 9;
//   #else
//   UART0_IBRD = 3; // Program baud rate for 6MHz clock
//   UART0_FBRD = 17;
//   #endif
//   UART0_LCRH = 0x60; // 8 bit, no parity
//   UART0_CTRL = 0x301; // Enable TX and RX, and UART enable
//   GPIOPA_AFSEL = GPIOPA_AFSEL | 0x3; // Use GPIO pins as UART0
//   return;
// }

// /* Output a character to UART0 (used by printf function to output data) */
// int sendchar (int ch) 
// {
//   if (ch == '\n') {
//   while ((UART0_FLAG & 0x8)); // Wait if it is busy
//   UART0_DATA = CR; // output extra CR to get correct
//   } // display on HyperTerminal
//   while ((UART0_FLAG & 0x8)); // Wait if it is busy
//   return (UART0_DATA = ch); // output data
// }
// /* Retargetting code for text output */
// // int fputc(int ch, FILE *f) 
// // {
// //   return (sendchar(ch));
// // }

// void _sys_exit(int return_code) 
// {
//   /* dummy exit */
//   label: goto label; /* endless loop */
// }

