/**
  mkSerial.h 
  Copyright (c) 2020 Mike Kim (ZOE Robotics).  All right reserved.
  Modified 20 August 2020 by Mike Kim
*/

#ifndef _MKSERIAL_H
#define _MKSERIAL_H



#ifndef F_CPU
#define F_CPU 84000000UL // or whatever may be your frequency
#endif

#ifndef FORCE_INLINE
#define  FORCE_INLINE __attribute__((always_inline)) inline
#endif

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "sam.h"
#include "mkZoeRobotics_message.h"


#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2
#define BYTE 0



// Define constants and variables for buffering incoming serial data.  We're
// using a ring buffer (I think), in which rx_buffer_head is the index of the
// location to which to write the next incoming character and rx_buffer_tail
// is the index of the location from which to read.
#define UART_BUFFER_SIZE 128

typedef struct _ring_buffer
{
  volatile uint8_t buffer[UART_BUFFER_SIZE];
  volatile int head;
  volatile int tail;
}ring_buffer;


// extern ring_buffer rx_buffer;
// extern ring_buffer tx_buffer;


class MKSerial //: public Stream
{

  public:
    ring_buffer rx_buffer  =  { { 0 }, 0, 0 };
    uint8_t alpha[12]={'1' ,'2','3','4','5','6','7','8','9','A','B','c'};
    int cntt=0;
  public:
    MKSerial();
    void begin(long);
    void end();
    int peek(void);
    int read(void);
    void flush(void);
    FORCE_INLINE void store_char(unsigned char c)
    {
      int i = (unsigned int)(rx_buffer.head + 1) % UART_BUFFER_SIZE;

      // if we should be storing the received character into the location
      // just before the tail (meaning that the head would advance to the
      // current location of the tail), we're about to overflow the buffer
      // and so we don't write the character or advance the head.
      if (i != rx_buffer.tail) {
        rx_buffer.buffer[rx_buffer.head] = c;
        rx_buffer.head = i;
      }
    }
    
     int available(void)
    {
      return (uint32_t)(UART_BUFFER_SIZE + rx_buffer.head - rx_buffer.tail) % UART_BUFFER_SIZE;
    }
    
    int write( uint8_t uc_data)
    {
       
      // Check if the transmitter is ready
      if((UART->UART_SR & UART_SR_TXRDY) != UART_SR_TXRDY)
      return 1;

      // Send the character
      //UART->UART_THR = uc_data;
      //uc_data = alpha[cntt++];

        UART->UART_THR = uc_data;
      while(!((UART->UART_SR) & UART_SR_TXEMPTY)); // Wait for the charactere to be send
    
      //while(!((UART->UART_SR) & UART_SR_TXEMPTY)); // Wait for the charactere to be send
      return 0;


      // // Check if the transmitter is ready
      // if(((UART->UART_SR & UART_SR_TXRDY) != UART_SR_TXRDY) |
      //     (tx_buffer.tail != tx_buffer.head))
      // {
      //   // If busy we buffer
      //   int nextWrite = (tx_buffer.head + 1) % UART_BUFFER_SIZE;
      //   while (tx_buffer.tail == nextWrite)
      //     ; // Spin locks if we're about to overwrite the buffer. This continues once the data is sent
      //   tx_buffer.buffer[tx_buffer.head] = uc_data;
      //   tx_buffer.head = nextWrite;
      //   // Make sure TX interrupt is enabled
      //   UART->UART_IER = UART_IER_TXRDY;
      // }
      // else
      // {
      //   // Send the character
      //   UART->UART_THR = uc_data;
      // }
    
    }
    
    
    // FORCE_INLINE void checkRx(void)
    // {
    //   if((M_UCSRxA & (1<<M_RXCx)) != 0) {
    //     unsigned char c  =  M_UDRx;
    //     int i = (unsigned int)(rx_buffer.head + 1) % UART_BUFFER_SIZE;

    //     // if we should be storing the received character into the location
    //     // just before the tail (meaning that the head would advance to the
    //     // current location of the tail), we're about to overflow the buffer
    //     // and so we don't write the character or advance the head.
    //     if (i != rx_buffer.tail) {
    //       rx_buffer.buffer[rx_buffer.head] = c;
    //       rx_buffer.head = i;
    //     }
    //   }
    // }
    
    
    private:
    void printNumber(unsigned long, uint8_t);
    void printFloat(double, uint8_t);
    
    
  public:
    
     void write(const char *str)
    {
     // write('-');
      while (*str)
        write(*str++);
      
    }


     void write(const uint8_t *buffer, size_t size)
    {
      while (size--)
        write(*buffer++);
    }

    // FORCE_INLINE void print(const String &s)
    // {
    //   for (int i = 0; i < (int)s.length(); i++) {
    //     write(s[i]);
    //   }
    // }
    
     void print(const char *str)
    {
      write(str);
    }
    //size_t print(const String &s);
    void print(char, int = BYTE);
    void print(unsigned char, int = BYTE);
    void print(int, int = DEC);
    void print(unsigned int, int = DEC);
    void print(long, int = DEC);
    void print(unsigned long, int = DEC);
    void print(double, int = 2);

    //void println(const String &s);
    void println(const char[]);
    void println(char, int = BYTE);
    void println(unsigned char, int = BYTE);
    void println(int, int = DEC);
    void println(unsigned int, int = DEC);
    void println(long, int = DEC);
    void println(unsigned long, int = DEC);
    void println(double, int = 2);
    void println(void);
    void IrqHandler(void);
};


extern MKSerial mkSerial;


#endif

