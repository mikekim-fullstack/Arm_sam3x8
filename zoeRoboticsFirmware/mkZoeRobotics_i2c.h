/**
  mkZoeRobotics_I2C.h 
  Copyright (c) 2020 Mike Kim (ZOE Robotics).  All right reserved.
  4 SEPT 2020 by Mike Kim
*/

#ifndef _MKZOEROBOTICS_I2C_H
#define _MKZOEROBOTICS_I2C_H
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "../include/due_sam3x.h"
#include "mkZoeRobotics_globalDataStruct.h"

#define WIRE_INTERFACES_COUNT 2

#define WIRE_INTERFACE       TWI1
#define WIRE_INTERFACE_ID    ID_TWI1
#define WIRE_ISR_HANDLER     TWI1_Handler
#define WIRE_ISR_ID          TWI1_IRQn

#define WIRE1_INTERFACE      TWI0
#define WIRE1_INTERFACE_ID   ID_TWI0
#define WIRE1_ISR_HANDLER    TWI0_Handler
#define WIRE1_ISR_ID         TWI0_IRQn
#define I2C_MASTER 0
#define I2C_SLAVE 1

#define BUFFER_LENGTH 32

 // WIRE_HAS_END means Wire has end()
#define WIRE_HAS_END 1
#define I2C_BUFFER_SIZE 280 //28*10

typedef struct 
{
  uint8_t buff[I2C_BUFFER_SIZE];
  volatile int head;
  volatile int tail;
}I2Cring_buffer;

class MKI2C {
public:
   MKI2C(Twi *twi, void(*end_cb)(void));
	MKI2C(Twi *twi);
  
   void begin(int mode, uint8_t address);
   // void begin();
   // void begin(int);
   void end();
	void startInterrupt(uint32_t InterrConfig=TWI_IER_SVACC) ;
   void setClock(uint32_t);
   void beginTransmission(uint8_t);
   void beginTransmission(int);
   uint8_t endTransmission(void);
   uint8_t endTransmission(uint8_t);
   uint8_t requestFrom(uint8_t, uint8_t);
   uint8_t requestFrom(uint8_t, uint8_t, uint8_t);
   uint8_t requestFrom(uint8_t, uint8_t, uint32_t, uint8_t, uint8_t);
   uint8_t requestFrom(int, int);
   uint8_t requestFrom(int, int, int);
   virtual size_t write(uint8_t);
   virtual size_t write(const uint8_t *, size_t);
	
   virtual int available(void);
   virtual int read(void);
   virtual int peek(void);
   virtual void flush(void);
   void onReceive(void(*)(int));
   void onRequest(void(*)(void));

   inline size_t write(unsigned long n) { return write((uint8_t)n); }
   inline size_t write(long n) { return write((uint8_t)n); }
   inline size_t write(unsigned int n) { return write((uint8_t)n); }
   inline size_t write(int n) { return write((uint8_t)n); }


   void onService(void);

private:
	//I2Cring_buffer ringBuffer  =  { { 0 }, 0, 0 };
	// RX Buffer
	uint8_t rxBuffer[BUFFER_LENGTH];
	uint8_t rxBufferIndex;
	uint8_t rxBufferLength;

	// TX Buffer
	uint8_t txAddress;
	uint8_t txBuffer[BUFFER_LENGTH];
	uint8_t txBufferLength;

	// Service buffer
	uint8_t srvBuffer[BUFFER_LENGTH];
	uint8_t srvBufferIndex;
	uint8_t srvBufferLength;

	// Callback user functions
	void (*onRequestCallback)(void);
	void (*onReceiveCallback)(int);

	// Called before initialization
	void (*onBeginCallback)(void);

	// Called after deinitialization
	void (*onEndCallback)(void);

	// TWI instance
	Twi *twi;
	uint8_t MK_WIRE_INTERFACE;

	// TWI state
	enum MKI2CStatus {
		UNINITIALIZED,
		MASTER_IDLE,
		MASTER_SEND,
		MASTER_RECV,
		SLAVE_IDLE,
		SLAVE_RECV,
		SLAVE_SEND
	};
	MKI2CStatus status;

	// TWI clock frequency
	static const uint32_t TWI_CLOCK = 100000;
	uint32_t twiClock;

	// Timeouts (
	static const uint32_t RECV_TIMEOUT = 100000;
	static const uint32_t XMIT_TIMEOUT = 100000;
};

#if WIRE_INTERFACES_COUNT > 0
extern MKI2C mkI2C;
#endif
#if WIRE_INTERFACES_COUNT > 1
extern MKI2C mkI2C1;
#endif

#endif