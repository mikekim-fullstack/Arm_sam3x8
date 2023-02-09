/**
  mkZoeRobotics_I2C.h 
  Copyright (c) 2020 Mike Kim (ZOE Robotics).  All right reserved.
  4 SEPT 2020 by Mike Kim
*/
#include <string.h>
#include "mkZoeRobotics_define.h"
#include "mkZoeRobotics_i2c.h"
#include "assert.h"




static inline bool TWI_FailedAcknowledge(Twi *pTwi) {
	return pTwi->TWI_SR & TWI_SR_NACK;
}

static inline bool TWI_WaitTransferComplete(Twi *_twi, uint32_t _timeout) {
	uint32_t _status_reg = 0;
	while ((_status_reg & TWI_SR_TXCOMP) != TWI_SR_TXCOMP) {
		_status_reg = TWI_GetStatus(_twi);

		if (_status_reg & TWI_SR_NACK)
			return false;

		if (--_timeout == 0)
			return false;
	}
	return true;
}

static inline bool TWI_WaitByteSent(Twi *_twi, uint32_t _timeout) {
	uint32_t _status_reg = 0;
	while ((_status_reg & TWI_SR_TXRDY) != TWI_SR_TXRDY) {
		_status_reg = TWI_GetStatus(_twi);

		if (_status_reg & TWI_SR_NACK)
			return false;

		if (--_timeout == 0)
			return false;
	}

	return true;
}

static inline bool TWI_WaitByteReceived(Twi *_twi, uint32_t _timeout) {
	uint32_t _status_reg = 0;
	while ((_status_reg & TWI_SR_RXRDY) != TWI_SR_RXRDY) {
		_status_reg = TWI_GetStatus(_twi);

		if (_status_reg & TWI_SR_NACK)
			return false;

		if (--_timeout == 0)
			return false;
	}

	return true;
}

static inline bool TWI_STATUS_SVREAD(uint32_t status) {
	return (status & TWI_SR_SVREAD) == TWI_SR_SVREAD;
}

static inline bool TWI_STATUS_SVACC(uint32_t status) {
	return (status & TWI_SR_SVACC) == TWI_SR_SVACC;
}

static inline bool TWI_STATUS_GACC(uint32_t status) {
	return (status & TWI_SR_GACC) == TWI_SR_GACC;
}

static inline bool TWI_STATUS_EOSACC(uint32_t status) {
	return (status & TWI_SR_EOSACC) == TWI_SR_EOSACC;
}

static inline bool TWI_STATUS_NACK(uint32_t status) {
	return (status & TWI_SR_NACK) == TWI_SR_NACK;
}

MKI2C::MKI2C(Twi *_twi,  void(*_endCb)(void)) :
	twi(_twi), rxBufferIndex(0), rxBufferLength(0), txAddress(0),
			txBufferLength(0), srvBufferIndex(0), srvBufferLength(0), status(
					UNINITIALIZED), 
						onEndCallback(_endCb), twiClock(TWI_CLOCK) {
							//if(twi==TWI0) MK_WIRE_INTERFACE = TWI
}
MKI2C::MKI2C(Twi *_twi) : twi(_twi){
							//if(twi==TWI0) MK_WIRE_INTERFACE = TWI
}
// void MKI2C::begin(void) {
// 	if (onBeginCallback)
// 		onBeginCallback();

// 	// Disable PDC channel
// 	twi->TWI_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;

// 	TWI_ConfigureMaster(twi, twiClock, VARIANT_MCK);
// 	status = MASTER_IDLE;
// }

void MKI2C::begin(int mode, uint8_t address) {
	twi=TWI1;
	mode = I2C_SLAVE;
  twi->TWI_CR = 0;
  twi->TWI_CR = TWI_CR_SVDIS | TWI_CR_MSDIS; // Disable Master and Slave modes
  //pTwi->TWI_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;
  if (twi == TWI0) {// TWI0
    PMC->PMC_PCER0 |= PMC_PCER0_PID22;      // TWI0 power ON
    PIOA->PIO_PDR |= PIO_PDR_P17            // Enable peripheral control
                     | PIO_PDR_P18;
    PIOA->PIO_ABSR &= ~(PIO_PA17A_TWD0      // TWD0 & TWCK0 Peripherals type A
                        | PIO_PA18A_TWCK0);
  }
  else {
    PMC->PMC_PCER0 |= PMC_PCER0_PID23;      // TWI1 power ON
    PIOB->PIO_PDR |= PIO_PDR_P13            // Enable peripheral control
                     | PIO_PDR_P12;
    PIOB->PIO_ABSR &= ~(PIO_PB12A_TWD1      // TWD1 & TWCK1 Peripherals type A
                        | PIO_PB13A_TWCK1);
      // Define  INPUT 
    PIOB->PIO_ODR |=PIO_PDR_P13 ;

    // Define  OUTPUT 
   // PIOB->PIO_OER |=PIO_PDR_P12 ;

  }

  
  // I2C lines are Open drain by hardware, no need to program PIO_MDER

  twi->TWI_CR |= TWI_CR_SWRST;  // TWIn software reset
  twi->TWI_RHR;                // Flush reception buffer

  //Enable master mode
  if (mode == I2C_MASTER) {
    //enter slave address
    twi->TWI_MMR |= TWI_MMR_DADR(address);

    twi->TWI_CR |= TWI_CR_MSEN ;  // Master mode enable
    
    //clockwave from 100khz to 400khz
    setClock( 400000); // from 100000 to 400000
	 status = MASTER_IDLE;
  }
  else { // Enable Slave mode
    twi->TWI_SMR = 0;
    twi->TWI_SMR = TWI_SMR_SADR(address);
    twi->TWI_CR |= TWI_CR_SVEN | TWI_CR_MSDIS;     // Slave mode enable
    for (int i=0; i < 1000000; i++);
    assert( (twi->TWI_CR & TWI_CR_SVDIS)!= TWI_CR_SVDIS ) ;
	 status = MASTER_IDLE;
  }
  //mkI2C.startInterrupt();
/*
  //Enable master mode
  if (mode  == I2C_MASTER) {// I2C_MASTER==0
  		if(twi == TWI1) {
			NVIC_DisableIRQ(WIRE1_ISR_ID);
			NVIC_ClearPendingIRQ(WIRE1_ISR_ID);
			NVIC_SetPriority(WIRE1_ISR_ID, 1);
			//enter slave address
			twi->TWI_MMR |= TWI_MMR_DADR(address);

			twi->TWI_CR |= TWI_CR_MSEN ;  // Master mode enable

			//clockwave from 100khz to 400khz
			setClock(400000); // from 100000 to 400000
			status = MASTER_IDLE;
			twi->TWI_IER = TWI_IER_SVACC;
			NVIC_EnableIRQ(WIRE1_ISR_ID);
		}
		else {
			NVIC_DisableIRQ(WIRE_ISR_ID);
			NVIC_ClearPendingIRQ(WIRE_ISR_ID);
			NVIC_SetPriority(WIRE_ISR_ID, 1);
			//enter slave address
			twi->TWI_MMR |= TWI_MMR_DADR(address);

			twi->TWI_CR |= TWI_CR_MSEN ;  // Master mode enable

			//clockwave from 100khz to 400khz
			setClock(400000); // from 100000 to 400000
			status = MASTER_IDLE;
			twi->TWI_IER = TWI_IER_SVACC;
			NVIC_EnableIRQ(WIRE_ISR_ID);
		}
      
  }
  else { // Enable Slave mode
  		if(twi == TWI1) {
			NVIC_DisableIRQ(WIRE1_ISR_ID);
			NVIC_ClearPendingIRQ(WIRE1_ISR_ID);
			NVIC_SetPriority(WIRE1_ISR_ID, 1);
			twi->TWI_SMR = 0;
			twi->TWI_SMR = TWI_SMR_SADR(address);
			twi->TWI_CR |= TWI_CR_SVEN | TWI_CR_MSDIS;     // Slave mode enable
			for (int i=0; i < 1000000; i++);
			assert( (twi->TWI_CR & TWI_CR_SVDIS)!= TWI_CR_SVDIS ) ;
			status = SLAVE_IDLE;
			twi->TWI_IER = TWI_IER_SVACC;
			NVIC_EnableIRQ(WIRE1_ISR_ID);
		}
		else {
			NVIC_DisableIRQ(WIRE_ISR_ID);
			NVIC_ClearPendingIRQ(WIRE_ISR_ID);
			NVIC_SetPriority(WIRE_ISR_ID, 1);
			twi->TWI_SMR = 0;
			twi->TWI_SMR = TWI_SMR_SADR(address);
			twi->TWI_CR |= TWI_CR_SVEN | TWI_CR_MSDIS;     // Slave mode enable
			for (int i=0; i < 1000000; i++);
			assert( (twi->TWI_CR & TWI_CR_SVDIS)!= TWI_CR_SVDIS ) ;
			status = SLAVE_IDLE;
			twi->TWI_IER = TWI_IER_SVACC;
			NVIC_EnableIRQ(WIRE_ISR_ID);
		}
    
  }
  */  
	// if (onBeginCallback)
	// 	onBeginCallback();

	// // Disable PDC channel
	// twi->TWI_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;

	// TWI_ConfigureSlave(twi, address);
	// status = SLAVE_IDLE;
	// TWI_EnableIt(twi, TWI_IER_SVACC);
	// //| TWI_IER_RXRDY | TWI_IER_TXRDY	| TWI_IER_TXCOMP);
}

// void MKI2C::begin(int address) {
// 	begin((uint8_t) address);
// }
void MKI2C::startInterrupt(uint32_t InterrConfig) {
	status = SLAVE_IDLE;
	twi->TWI_CR |= TWI_CR_START;
	twi->TWI_IER = InterrConfig;
	
  if (twi == TWI0) {
		NVIC_SetPriority(TWI0_IRQn, 2);
    	NVIC_EnableIRQ(TWI0_IRQn);
  }
  else {
		NVIC_SetPriority(TWI1_IRQn, 2);
    	NVIC_EnableIRQ(TWI1_IRQn);
  }
  
}
void MKI2C::end(void) {
	TWI_Disable(twi);

	// Enable PDC channel
	twi->TWI_PTCR &= ~(UART_PTCR_RXTDIS | UART_PTCR_TXTDIS);

	if (onEndCallback)
		onEndCallback();
}

void MKI2C::setClock(uint32_t frequency) {
	twiClock = frequency;
	TWI_SetClock(twi, twiClock, F_CPU);
}

uint8_t MKI2C::requestFrom(uint8_t address, uint8_t quantity, uint32_t iaddress, uint8_t isize, uint8_t sendStop) {
	if (quantity > BUFFER_LENGTH)
		quantity = BUFFER_LENGTH;

	// perform blocking read into buffer
	int readed = 0;
	TWI_StartRead(twi, address, iaddress, isize);
	do {
		// Stop condition must be set during the reception of last byte
		if (readed + 1 == quantity)
			TWI_SendSTOPCondition( twi);

		if (TWI_WaitByteReceived(twi, RECV_TIMEOUT))
			rxBuffer[readed++] = TWI_ReadByte(twi);
		else
			break;
	} while (readed < quantity);
	TWI_WaitTransferComplete(twi, RECV_TIMEOUT);

	// set rx buffer iterator vars
	rxBufferIndex = 0;
	rxBufferLength = readed;

	return readed;
}

uint8_t MKI2C::requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop) {
	return requestFrom((uint8_t) address, (uint8_t) quantity, (uint32_t) 0, (uint8_t) 0, (uint8_t) sendStop);
}

uint8_t MKI2C::requestFrom(uint8_t address, uint8_t quantity) {
	return requestFrom((uint8_t) address, (uint8_t) quantity, (uint8_t) true);
}

uint8_t MKI2C::requestFrom(int address, int quantity) {
	return requestFrom((uint8_t) address, (uint8_t) quantity, (uint8_t) true);
}

uint8_t MKI2C::requestFrom(int address, int quantity, int sendStop) {
	return requestFrom((uint8_t) address, (uint8_t) quantity, (uint8_t) sendStop);
}

void MKI2C::beginTransmission(uint8_t address) {
	status = MASTER_SEND;

	// save address of target and empty buffer
	txAddress = address;
	txBufferLength = 0;
}

void MKI2C::beginTransmission(int address) {
	beginTransmission((uint8_t) address);
}

//
//	Originally, 'endTransmission' was an f(void) function.
//	It has been modified to take one parameter indicating
//	whether or not a STOP should be performed on the bus.
//	Calling endTransmission(false) allows a sketch to
//	perform a repeated start.
//
//	WARNING: Nothing in the library keeps track of whether
//	the bus tenure has been properly ended with a STOP. It
//	is very possible to leave the bus in a hung state if
//	no call to endTransmission(true) is made. Some I2C
//	devices will behave oddly if they do not see a STOP.
//
uint8_t MKI2C::endTransmission(uint8_t sendStop) {
	uint8_t error = 0;
	// transmit buffer (blocking)
	TWI_StartWrite(twi, txAddress, 0, 0, txBuffer[0]);
	if (!TWI_WaitByteSent(twi, XMIT_TIMEOUT))
		error = 2;	// error, got NACK on address transmit
	
	if (error == 0) {
		uint16_t sent = 1;
		while (sent < txBufferLength) {
			TWI_WriteByte(twi, txBuffer[sent++]);
			if (!TWI_WaitByteSent(twi, XMIT_TIMEOUT))
				error = 3;	// error, got NACK during data transmmit
		}
	}
	
	if (error == 0) {
		TWI_Stop(twi);
		if (!TWI_WaitTransferComplete(twi, XMIT_TIMEOUT))
			error = 4;	// error, finishing up
	}

	txBufferLength = 0;		// empty buffer
	status = MASTER_IDLE;
	return error;
}

//	This provides backwards compatibility with the original
//	definition, and expected behaviour, of endTransmission
//
uint8_t MKI2C::endTransmission(void)
{
	return endTransmission(true);
}

size_t MKI2C::write(uint8_t data) {
	if (status == MASTER_SEND) {
		if (txBufferLength >= BUFFER_LENGTH)
			return 0;
		txBuffer[txBufferLength++] = data;
		return 1;
	} else {
		if (srvBufferLength >= BUFFER_LENGTH)
			return 0;
		srvBuffer[srvBufferLength++] = data;
		return 1;
	}
}

size_t MKI2C::write(const uint8_t *data, size_t quantity) {
	if (status == MASTER_SEND) {
		for (size_t i = 0; i < quantity; ++i) {
			if (txBufferLength >= BUFFER_LENGTH)
				return i;
			txBuffer[txBufferLength++] = data[i];
		}
	} else {
		for (size_t i = 0; i < quantity; ++i) {
			if (srvBufferLength >= BUFFER_LENGTH)
				return i;
			srvBuffer[srvBufferLength++] = data[i];
		}
	}
	return quantity;
}
//  size_t MKI2C::write(SENDData &Data)
//  {
// 	int len = sizeof(Data);
// 	 uint8_t data8bit[len];
// 	 memcpy(data8bit, &Data,len);
// 	 write(data8bit, len);

//  }
int MKI2C::available(void) {
	return rxBufferLength - rxBufferIndex;
}

int MKI2C::read(void) {
	if (rxBufferIndex < rxBufferLength)
		return rxBuffer[rxBufferIndex++];
	return -1;
}

int MKI2C::peek(void) {
	if (rxBufferIndex < rxBufferLength)
		return rxBuffer[rxBufferIndex];
	return -1;
}

void MKI2C::flush(void) {
	// Do nothing, use endTransmission(..) to force
	// data transfer.
}

void MKI2C::onReceive(void(*function)(int)) {
	onReceiveCallback = function;
}

void MKI2C::onRequest(void(*function)(void)) {
	onRequestCallback = function;
}

void MKI2C::onService(void) {
	// Retrieve interrupt status
	
	uint32_t sr = TWI_GetStatus(twi);

	if (status == SLAVE_IDLE && TWI_STATUS_SVACC(sr)) {
		TWI_DisableIt(twi, TWI_IDR_SVACC);
		TWI_EnableIt(twi, TWI_IER_RXRDY | TWI_IER_GACC | TWI_IER_NACK
				| TWI_IER_EOSACC | TWI_IER_SCL_WS | TWI_IER_TXCOMP);

		srvBufferLength = 0;
		srvBufferIndex = 0;

		// Detect if we should go into RECV or SEND status
		// SVREAD==1 means *master* reading -> SLAVE_SEND
		if (!TWI_STATUS_SVREAD(sr)) {
			status = SLAVE_RECV;
		} else {
			status = SLAVE_SEND;

			// Alert calling program to generate a response ASAP
			if (onRequestCallback)
				onRequestCallback();
			else
				// create a default 1-byte response
				write((uint8_t) 0);
		}
	}

	if (status != SLAVE_IDLE && TWI_STATUS_EOSACC(sr)) {
		if (status == SLAVE_RECV && onReceiveCallback) {
			// Copy data into rxBuffer
			// (allows to receive another packet while the
			// user program reads actual data)
			for (uint8_t i = 0; i < srvBufferLength; ++i)
				rxBuffer[i] = srvBuffer[i];
			rxBufferIndex = 0;
			rxBufferLength = srvBufferLength;

			// Alert calling program
			onReceiveCallback( rxBufferLength);
		}

		// Transfer completed
		TWI_EnableIt(twi, TWI_SR_SVACC);
		TWI_DisableIt(twi, TWI_IDR_RXRDY | TWI_IDR_GACC | TWI_IDR_NACK
				| TWI_IDR_EOSACC | TWI_IDR_SCL_WS | TWI_IER_TXCOMP);
		status = SLAVE_IDLE;
	}

	if (status == SLAVE_RECV) {
		if (TWI_STATUS_RXRDY(sr)) {
			if (srvBufferLength < BUFFER_LENGTH)
				srvBuffer[srvBufferLength++] = TWI_ReadByte(twi);
		}
	}

	if (status == SLAVE_SEND) {
		if (TWI_STATUS_TXRDY(sr) && !TWI_STATUS_NACK(sr)) {
			uint8_t c = 'x';
			if (srvBufferIndex < srvBufferLength)
				c = srvBuffer[srvBufferIndex++];
			TWI_WriteByte(twi, c);
		}
	}
}

#if WIRE_INTERFACES_COUNT > 0
// static void Wire_Init(void) {
// 	pmc_enable_periph_clk(WIRE_INTERFACE_ID);
// 	PIO_Configure(
// 			g_APinDescription[PIN_WIRE_SDA].pPort,
// 			g_APinDescription[PIN_WIRE_SDA].ulPinType,
// 			g_APinDescription[PIN_WIRE_SDA].ulPin,
// 			g_APinDescription[PIN_WIRE_SDA].ulPinConfiguration);
// 	PIO_Configure(
// 			g_APinDescription[PIN_WIRE_SCL].pPort,
// 			g_APinDescription[PIN_WIRE_SCL].ulPinType,
// 			g_APinDescription[PIN_WIRE_SCL].ulPin,
// 			g_APinDescription[PIN_WIRE_SCL].ulPinConfiguration);

// 	NVIC_DisableIRQ(WIRE_ISR_ID);
// 	NVIC_ClearPendingIRQ(WIRE_ISR_ID);
// 	NVIC_SetPriority(WIRE_ISR_ID, 0);
// 	NVIC_EnableIRQ(WIRE_ISR_ID);
// }

static void Wire_Deinit(void) {
	NVIC_DisableIRQ(WIRE_ISR_ID);
	NVIC_ClearPendingIRQ(WIRE_ISR_ID);

	pmc_disable_periph_clk(WIRE_INTERFACE_ID);

	// no need to undo PIO_Configure, 
	// as Peripheral A was enable by default before,
	// and pullups were not enabled
}

//MKI2C mkI2C = MKI2C(TWI1);
MKI2C mkI2C = MKI2C(TWI1, Wire_Deinit);//
void WIRE_ISR_HANDLER(void) {
	mkI2C.onService();
}
#endif

#if WIRE_INTERFACES_COUNT > 1
// static void Wire1_Init(void) {
// 	pmc_enable_periph_clk(WIRE1_INTERFACE_ID);
// 	PIO_Configure(
// 			g_APinDescription[PIN_WIRE1_SDA].pPort,
// 			g_APinDescription[PIN_WIRE1_SDA].ulPinType,
// 			g_APinDescription[PIN_WIRE1_SDA].ulPin,
// 			g_APinDescription[//PIN_WIRE1_SDA].ulPinConfiguration);
// 	PIO_Configure(
// 			g_APinDescription[PIN_WIRE1_SCL].pPort,
// 			g_APinDescription[PIN_WIRE1_SCL].ulPinType,
// 			g_APinDescription[PIN_WIRE1_SCL].ulPin,
// 			g_APinDescription[PIN_WIRE1_SCL].ulPinConfiguration);

// 	NVIC_DisableIRQ(WIRE1_ISR_ID);
// 	NVIC_ClearPendingIRQ(WIRE1_ISR_ID);
// 	NVIC_SetPriority(WIRE1_ISR_ID, 0);
// 	NVIC_EnableIRQ(WIRE1_ISR_ID);
// }

static void Wire1_Deinit(void) {
	NVIC_DisableIRQ(WIRE1_ISR_ID);
	NVIC_ClearPendingIRQ(WIRE1_ISR_ID);

	pmc_disable_periph_clk(WIRE1_INTERFACE_ID);

	// no need to undo PIO_Configure, 
	// as Peripheral A was enable by default before,
	// and pullups were not enabled
}

MKI2C mkI2C1 = MKI2C(TWI0, Wire1_Deinit);

void WIRE1_ISR_HANDLER(void) {
	mkI2C1.onService();
}
#endif

