/*
 * Serial.c
 *
 * Created: 28/09/2016 09:51:56
 * Author : Xx_YoloSw4Ger420BlazzzSnoop_xX (my real name, no fake)
 */ 

#include "sam.h"

///////////////////////////////
//////////Configuration////////
///////////////////////////////
static void configure_uart(void)
{
    if ((PMC->PMC_PCSR0 & (1u << ID_UART)) != (1u << ID_UART)) {
			PMC->PMC_PCER0 = 1 << ID_UART;
	}
    uint32_t ul_sr;

    // ==> Pin configuration
    // Disable interrupts on Rx and Tx
    PIOA->PIO_IDR = PIO_PA8A_URXD | PIO_PA9A_UTXD;

    // Disable the PIO of the Rx and Tx pins so that the peripheral controller can use them
    PIOA->PIO_PDR = PIO_PA8A_URXD | PIO_PA9A_UTXD;

    // Read current peripheral AB select register and set the Rx and Tx pins to 0 (Peripheral A function)
    ul_sr = PIOA->PIO_ABSR;
    PIOA->PIO_ABSR &= ~(PIO_PA8A_URXD | PIO_PA9A_UTXD) & ul_sr;

    // Enable the pull up on the Rx and Tx pin
    PIOA->PIO_PUER = PIO_PA8A_URXD | PIO_PA9A_UTXD;

    // ==> Actual uart configuration
    // Enable the peripheral uart controller
    PMC->PMC_PCER0 = PMC_PCER0_PID8;//1 << ID_UART;

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
///////////////////////////////
//////////Functions////////////
///////////////////////////////
static int uart_getchar(uint8_t *c)
{
    // Check if the receiver is ready
    if((UART->UART_SR & UART_SR_RXRDY) == 0) {
        //uart_putchar('$');
        return 1;
    }

    // Read the character
    *c = (uint8_t) UART->UART_RHR;
    //while(!((UART->UART_SR) & UART_SR_RXRDY));
    //while(!((UART->UART_SR) & UART_SR_ENDRX));
    //while(!((UART->UART_SR) & UART_SR_RXBUFF));
    return 0;
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

static void uart_getString(uint8_t *c, int length)
{
    int i = 0;
    for(i=0; i<length; i++) {
        while(!((UART->UART_SR) & UART_SR_RXRDY));
        uart_getchar(&c[i]);
    }
}

static void uart_putString(uint8_t *c, int length)
{
    int i = 0;
    for(i=0; i<length; i++) {
        uart_putchar(c[i]);
    }
}

int main(void)
{
    /* Initialize the SAM system */
    SystemInit();
    configure_uart();
    uart_putchar('a'); // Send 'a' just one time...

    /* Replace with your application code */
    while (1)
    {

    }
}