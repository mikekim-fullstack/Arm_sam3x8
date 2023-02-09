/**
  MKSerial.cpp 
  Copyright (c) 2020 Mike Kim (ZOE Robotics).  All right reserved.
  Modified 5 August 2020 by Mike Kim
*/
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "mkZoeRobotics_serial.h"
//#define BAUD 115200//250000// //9600

// ring_buffer rx_buffer  =  { { 0 }, 0, 0 };
// ring_buffer tx_buffer  =  { { 0 }, 0, 0 };

MKSerial mkSerial;




void UART_Handler (void) __attribute__ ((weak));
void UART_Handler(void) 
{
  mkSerial.IrqHandler();
}

// Constructors ////////////////////////////////////////////////////////////////

MKSerial::MKSerial()
{
  alpha[0]='1';
}

// Public Methods //////////////////////////////////////////////////////////////

void MKSerial::begin(long baud)
{
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
    PMC->PMC_PCER0 = 1 << ID_UART;
  
    // Reset and disable receiver and transmitter
    UART->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX | UART_CR_RXDIS | UART_CR_TXDIS;

    // Set the baudrate
    UART->UART_BRGR = F_CPU/ 16 /baud;// = BaudRate (write x into UART_BRGR)

    // No Parity
    UART->UART_MR = UART_MR_PAR_NO | UART_MR_CHMODE_NORMAL;

    // Disable PDC channel
    UART->UART_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;

    // Configure interrupts
    UART->UART_IDR = 0xFFFFFFFF;
    UART->UART_IER = UART_IER_RXRDY | UART_IER_OVRE | UART_IER_FRAME;
    UART->UART_IDR = UART_IDR_TXRDY;

    // Enable UART interrupt in NVIC
    NVIC_EnableIRQ((IRQn_Type) UART_IRQn);

    // Enable receiver and transmitter
    UART->UART_CR = UART_CR_RXEN | UART_CR_TXEN;
}
void MKSerial::IrqHandler(void)
{
  uint32_t status = UART->UART_SR;

  // while (!((status & UART_SR_RXRDY) == UART_SR_RXRDY)){}
  // uint8_t uData = UART->UART_RHR;
  // store_char(uData);


  if ((status & UART_SR_RXRDY) == UART_SR_RXRDY)
   store_char(UART->UART_RHR);

 // Do we need to keep sending data?
  // if ((status & UART_SR_TXRDY) == UART_SR_TXRDY) 
  // {
  //   if (tx_buffer.tail != tx_buffer.head) {
  //     UART->UART_THR = tx_buffer.buffer[tx_buffer.tail];
  //     tx_buffer.tail = (unsigned int)(tx_buffer.tail + 1) % UART_BUFFER_SIZE;
  //   }
  //   else
  //   {
  //     // Mask off transmit interrupt so we don't get it anymore
  //     UART->UART_IDR = UART_IDR_TXRDY;
  //   }
  // }

  // Acknowledge errors
  if ((status & UART_SR_OVRE) == UART_SR_OVRE || (status & UART_SR_FRAME) == UART_SR_FRAME)
  {
    // TODO: error reporting outside ISR
    UART->UART_CR |= UART_CR_RSTSTA;
  }

}
void MKSerial::end()
{
  // Clear any received data
  rx_buffer.head = rx_buffer.tail;

  // Wait for any outstanding data to be sent
  flush();

  // Disable UART interrupt in NVIC
  NVIC_DisableIRQ( (IRQn_Type) UART_IRQn );
}



int MKSerial::peek(void)
{
  if (rx_buffer.head == rx_buffer.tail) {
    return -1;
  } else {
    return rx_buffer.buffer[rx_buffer.tail];
  }
}

int MKSerial::read(void)
{
  // if the head isn't ahead of the tail, we don't have any characters
  if (rx_buffer.head == rx_buffer.tail) {
    return -1;
  } else {
    uint8_t  c = rx_buffer.buffer[rx_buffer.tail];
    rx_buffer.tail = (unsigned int)(rx_buffer.tail + 1) % UART_BUFFER_SIZE;
    return c;
  }
}

void MKSerial::flush()
{
  rx_buffer.head = rx_buffer.tail;
  // while (tx_buffer.head != tx_buffer.tail); //wait for transmit data to be sent
  // // Wait for transmission to complete
  // while ((UART->UART_SR & UART_SR_TXEMPTY) != UART_SR_TXEMPTY)
   ;
}
// size_t MKSerial::write(const uint8_t *buffer, size_t size)
// {
//   size_t n = 0;
//   while (size--) {
//     if (write(*buffer++)) n++;
//     else break;
//   }
//   return n;
// }



// void MKSerial::print(const String &s)
// {
//   write(s.c_str(), s.length());
// }
void MKSerial::print(char c, int base)
{
  print((long) c, base);
}

void MKSerial::print(unsigned char b, int base)
{
  print((unsigned long) b, base);
}

void MKSerial::print(int n, int base)
{
  print((long) n, base);
}

void MKSerial::print(unsigned int n, int base)
{
  print((unsigned long) n, base);
}

void MKSerial::print(long n, int base)
{
  if (base == 0) {
    write(n);
  } else if (base == 10) {
    if (n < 0) {
      print('-');
      n = -n;
    }
    printNumber(n, 10);
  } else {
    printNumber(n, base);
  }
}

void MKSerial::print(unsigned long n, int base)
{
  if (base == 0) write(n);
  else printNumber(n, base);
}

void MKSerial::print(double n, int digits)
{
  printFloat(n, digits);
}

void MKSerial::println(void)
{
  print('\n');  
}

// void MKSerial::println(const String &s)
// {
//   print(s);
//   println();
// }
// void MKSerial::println(const String &s)
// {
//   size_t n = print(s);
//   n += println();
//   //return n;
// }

void MKSerial::println(const char c[])
{
  print(c);
  println();
}

void MKSerial::println(char c, int base)
{
  print(c, base);
  println();
}

void MKSerial::println(unsigned char b, int base)
{
  print(b, base);
  println();
}

void MKSerial::println(int n, int base)
{
  print(n, base);
  println();
}

void MKSerial::println(unsigned int n, int base)
{
  print(n, base);
  println();
}

void MKSerial::println(long n, int base)
{
  print(n, base);
  println();
}

void MKSerial::println(unsigned long n, int base)
{
  print(n, base);
  println();
}

void MKSerial::println(double n, int digits)
{
  print(n, digits);
  println();
}

// Private Methods /////////////////////////////////////////////////////////////

void MKSerial::printNumber(unsigned long n, uint8_t base)
{
  unsigned char buf[8 * sizeof(long)]; // Assumes 8-bit chars. 
  unsigned long i = 0;

  if (n == 0) {
    print('0');
    return;
  } 

  while (n > 0) {
    buf[i++] = n % base;
    n /= base;
  }

  for (; i > 0; i--)
    print((char) (buf[i - 1] < 10 ?
      '0' + buf[i - 1] :
      'A' + buf[i - 1] - 10));
}

void MKSerial::printFloat(double number, uint8_t digits) 
{ 
  // Handle negative numbers
  if (number < 0.0)
  {
     print('-');
     number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;
  
  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    print("."); 

  // Extract digits from the remainder one at a time
  while (digits-- > 0)
  {
    remainder *= 10.0;
    int toPrint = int(remainder);
    print(toPrint);
    remainder -= toPrint; 
  } 
}






