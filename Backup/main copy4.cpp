#include <stdio.h>
#include <math.h>
#include <string.h>
#include <assert.h>
#include "mkZoeRobotics_define.h"
#include "../include/due_sam3x.init.h"

#include "timetick.h"
#include "mkZoeRobotics_serial.h"
#include "mkZoeRobotics_command.h"
#include "mkZoeRobotics_velProfile.h"
#include "twi.h"
#include "mkZoeRobotics_spi.h"
#include "mkZoeRobotics_i2c.h"
//////////////////////////////////////////////////////////////


speedRampData speedData[MAX_MOTOR_NUM]={0};
portIOPair   IOMap[MAX_MOTOR_NUM][4]={0};
struct GLOBAL_FLAGS g_status;
unsigned long  elapsedTime[MAX_MOTOR_NUM]={0};
volatile  uint32_t gIRQ_step_count[MAX_MOTOR_NUM]={0};
volatile  uint16_t gIRQ_TC_FLAG_DONE[MAX_MOTOR_NUM]={0};
static int nTimer = TC_0;

extern MKSerial mkSerial;
extern MKCommand mkCommand;
extern MKVelProfile mkVelProfile;
extern MKSPI mkSPI;
extern MKI2C mkI2C1;

///////////////////////////////////////////////////////////////
// Speed ramp states
#define STOP  0
#define ACCEL 1
#define DECEL 2
#define RUN   3
////////////////////////////////////////////////////////////////////////
inline static  void writePluse(Pio *portIO, uint32_t pin)
{
   portIO->PIO_ODSR ^= pin;
   portIO->PIO_ODSR ^= pin;
}
//////////////////////////////////////////////////////////
void TC0_Handler()
{
  TC_GetStatus(TC0, 0);
  if(speedData[nTimer].activated)
  {
      // ...GIVE AN IMPULSE TO MOTOR...
      writePluse(IOMap[nTimer][ST_PULSE].pIO, IOMap[nTimer][ST_PULSE].pin);

      // ...CLEAR TC STATUS REGISTER...
      TC_SetRC(TC0, 0, speedData[nTimer].Cn);
      ++speedData[nTimer].step_count;
      // ... ACCELERATION AREA ....
      if(speedData[nTimer].step_count<speedData[nTimer].Na)
      {
            if(speedData[nTimer].step_count==0)
            {
                speedData[nTimer].Cn = speedData[nTimer].Cn_acc0;
                speedData[nTimer].rest = 0;
            }
            else
            {
              //%%% SQRT approximation of speedData[num].Cn = speedData[num].Cn_acc0*(sqrt(step_count+1)-sqrt(step_count));
              speedData[nTimer].C =  speedData[nTimer].Cn*(1- 2.0/(4 * speedData[nTimer].step_count + 1))+speedData[nTimer].rest ;
              speedData[nTimer].Cn = floor(speedData[nTimer].C);
              speedData[nTimer].rest =  speedData[nTimer].C-speedData[nTimer].Cn;
            }
      }
      // ... DECELERATION AREA ...
      else if( speedData[nTimer].step_count>=speedData[nTimer].Nac && speedData[nTimer].step_count<speedData[nTimer].steps)
      {
        
          //%%% SQRT approximation
          if(speedData[nTimer].step_count==speedData[nTimer].Nac)
          {
              speedData[nTimer].rest=0;
          }
          else {
            speedData[nTimer].C =  speedData[nTimer].Cn*(1 + 2.0/(4 * speedData[nTimer].NNb - 1))+speedData[nTimer].rest ;
            speedData[nTimer].Cn = floor(speedData[nTimer].C);
            speedData[nTimer].rest =  speedData[nTimer].C-speedData[nTimer].Cn;
          }
          speedData[nTimer].NNb--;
      }
      // ... FINISHED SPEED PROFILE AND NOTIFY IN MAIN LOOP...
      else if(speedData[nTimer].step_count>=speedData[nTimer].steps)
      {
          gIRQ_TC_FLAG_DONE[nTimer]=1;
      }
  }// ---------end of if(speedData[nTimer].activated) 
}
// -------------------- END OF TC0_Handler ---------------------------

void TC1_Handler()
{

 ////////////////// EXAMPLE FOR OTHER TIMER ////////////////////////////
  TC_GetStatus(TC0, 1);
   //////////////////////////////////////////////
  NVIC_DisableIRQ(TC1_IRQn);
  TC_SetRC(TC0, 1, speedData[1].Cn);
  NVIC_EnableIRQ(TC1_IRQn);
  //////////////////////////////////////////////
  writePluse(IOMap[TC_1][ST_PULSE].pIO, IOMap[TC_1][ST_PULSE].pin);
  //pulse_stepper_motor(0,speedData[num].dir);
  ++gIRQ_step_count[1];
  gIRQ_TC_FLAG_DONE[1]=1;
}
/////////////////////////////////////////////////////////////////////
inline static  void setupIO_Output(Pio *portIO, uint32_t pin)
{
  portIO->PIO_IDR = pin; // disable interrupt
  portIO->PIO_MDDR = pin;// Disable to connect multi devices 
  portIO->PIO_OWER = pin;// Enable writing on ODSR

  portIO->PIO_PER = pin; // PIO ENABLE
  portIO->PIO_OER = pin; // OUTPUT ENALBE
  portIO->PIO_PUDR = pin;// PULL-UP DISALBLE
}
inline static  void setupIO_Input(Pio *portIO, uint32_t pin)
{
  portIO->PIO_IDR = pin; // disable interrupt
  portIO->PIO_MDDR = pin;// Disable to connect multi devices 
  portIO->PIO_OWER = pin;// Enable writing on ODSR

  portIO->PIO_PER = pin; // PIO ENABLE
  portIO->PIO_ODR = pin; // OUTPUT DISABLE (INPUT ENABLED)
  //portIO->PIO_IFER = pin;// INPUTFILTER ENABLE
  //portIO->PIO_PUDR = pin;// PULL-UP DISALBLE
}
////////////////////////////////////////////////////////
 static  void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency=350) 
 {
    pmc_set_writeprotect(false);
    pmc_enable_periph_clk((uint32_t)irq);
    //TC_Configure(tc, channel,  TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK2 | TC_CMR_ACPC_CLEAR | TC_CMR_ASWTRG_CLEAR);//  | TC_CMR_CPCSTOP);
    TC_Configure(tc, channel,  TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK1);
    uint32_t rc = F_CPU/8/frequency; //128 because we selected TIMER_CLOCK4 above
    TC_SetRA(tc, channel, rc); 
    TC_SetRC(tc, channel, rc);
    TC_Start(tc, channel);
    tc->TC_CHANNEL[channel].TC_IER=TC_IER_CPCS;
    tc->TC_CHANNEL[channel].TC_IDR=~TC_IER_CPCS;
    NVIC_ClearPendingIRQ(irq);
    NVIC_SetPriority(irq, 0);
    NVIC_EnableIRQ(irq);
}

volatile static uint16_t gIRQ_SPI_FLAG_RX=0, gIRQ_I2C_FLAG_DONE=0;
static uint8_t spi_databuf[24];
volatile static int spi_cnt=0;
 typedef struct {
    // ...OUTPUT...
    uint32_t steps; // total steps(pluse)
    uint32_t Na ; // pulse counts for acceleration area
    uint32_t Nac ; // pulse counts for acceleration and constant areas
    uint32_t NNb ; // pulse counts for deceleration area
    uint32_t Cn_acc0; // fisrt period time for 1st pulse of acceleration on Timer.
    uint8_t dir; // step motor direction 1:CCW, 0:CC
} speedProfile;
static speedProfile revData;
//uint8_t sendData[24]={0,};
//volatile uint8_t spi_cnt_tx=0;
static uint8_t DataReceived=0;
void SPI0_Handler() 
{
  uint32_t status = SPI0->SPI_SR;
  //

  if ((status & SPI_SR_RDRF) == SPI_SR_RDRF) 
  {
    /* --------- working version ----
      spi_databuf[spi_cnt] = SPI0->SPI_RDR ;
      ++spi_cnt;
      if(spi_cnt==1)
      {
        gIRQ_SPI_FLAG_RX=0;
      }
      else if(spi_cnt>=24)
      { 
          memcpy(&revData, spi_databuf, 24);
          // mkSerial.print("Steps: ");
          // mkSerial.print(revData.steps);
          // mkSerial.print(", Cn_acc0: ");
          // mkSerial.println(revData.Cn_acc0);
          spi_cnt=0;
          gIRQ_SPI_FLAG_RX=1;
      }   
    */
      spi_databuf[spi_cnt] = SPI0->SPI_RDR ;
      ++spi_cnt;
      if(spi_cnt==1)
      {
        gIRQ_SPI_FLAG_RX=0;
      }
      else if(spi_cnt>=26)
      { 
        //mkSerial.println(" ending in spi");
          // +++ 0x3A: WRITE +++ //
          if(spi_databuf[0]==0x3A && spi_databuf[25]==0x3B)
          {
            
            memcpy(&revData, &spi_databuf[1], 24);
            mkVelProfile.set_speed_profile(revData.steps,
                                        revData.Na,
                                        revData.Nac,
                                        revData.NNb,
                                        revData.Cn_acc0,
                                        revData.dir
                                        );
            gIRQ_SPI_FLAG_RX=3;
            // mkSerial.println(" ...ASK: WRITING... ");
            // mkSerial.print("Steps: ");
            // mkSerial.print(revData.steps);
            // mkSerial.print(", Cn_acc0: ");
            // mkSerial.println(revData.Cn_acc0);
          }
          // +++ 0x3C: READ +++ //
          else if(spi_databuf[0]==0x3C && spi_databuf[25]==0x3B)
          {
              //mkSerial.println(" ...ASK: READING... ");
          }
          else {
              gIRQ_SPI_FLAG_RX=0;
          }
          
          spi_cnt=0;
          
      }   
   /*
      DataReceived = SPI0->SPI_RDR ;//& SPI_RDR_RD_Msk;
      if(DataReceived==0x3A) 
      {
        mkSerial.println(DataReceived);
        spi_cnt =0;
        gIRQ_SPI_FLAG_RX=1;
      }
      else if(DataReceived==0x3B)
      {   mkSerial.println(spi_cnt);
          gIRQ_SPI_FLAG_RX=2;
          spi_cnt =0;
      }
      else if(gIRQ_SPI_FLAG_RX==1)
      {
        spi_databuf[spi_cnt]=DataReceived;
        ++spi_cnt;
      }
      */
   
    
      // if(DataReceived==0x3A) { 
      //   mkSerial.println("START: SPI GOT 0x3A");
      //   gIRQ_SPI_FLAG_RX=1;// start...
      //   spi_cnt=0;
      //  // return;
      // }
      // else if(DataReceived==0x3B) { 
      //   mkSerial.print(spi_cnt);
      //   mkSerial.println(", END: SPI GOT 0x3B");
      //   gIRQ_SPI_FLAG_RX=3;// start...
      //   spi_cnt=0;
      //  // return;
      // }
      // // else if(gIRQ_SPI_FLAG_RX==1)
      // // {
      //   spi_databuf[spi_cnt] = DataReceived;
       //  spi_cnt++;
         
      // }
  }
    // else if(gIRQ_SPI_FLAG_RX==1)
    // {      
    //   spi_databuf[spi_cnt++] = DataReceived;//SPI0->SPI_RDR;
    //   if(spi_cnt==24)
    //   {
    //     // mkSPI.startWrite();
    //     //   mkSPI.write(0x3A);
    //     //   mkSPI.write(0xEE);
    //     //   mkSPI.write(0xFE);
    //     // mkSPI.endWrite();

    //     spi_cnt =0; 
    //     gIRQ_SPI_FLAG_RX=0;// finished
        
          
        
    //     mkSerial.println("SPI received...");
    //     // while ((SPI0->SPI_SR & SPI_SR_TXEMPTY) == 0);
    //     // //load the Transmit Data Register with the value to transmit
    //     // SPI0->SPI_TDR = 0XAA;	
    //     // //Wait for data to be transferred to serializer
    //     // while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0);

    //     //spi_databuf[spi_cnt]='\0';
    //   }
  //   }
   

  if ((status & SPI_SR_TDRE)==SPI_SR_TDRE) {
    if(gIRQ_SPI_FLAG_RX==3) {
		  SPI0->SPI_TDR = 0x3A;//(uint16_t)(0x3A<<8)|(0xEE);
      gIRQ_SPI_FLAG_RX=0;
    }
	 }
  
   
}
/////////////////////////////////////////////////////
 uint8_t i2c_buff[84];
volatile int i2c_cnt=0;

#if 0
// #define read  1
// #define write 0
#define I2C_ADDRESS      0x5B
#define BUFFER_LENGTH 32
// TWI state
	enum TwoWireStatus_ {
		UNINITIALIZED,
		MASTER_IDLE,
		MASTER_SEND,
		MASTER_RECV,
		SLAVE_IDLE,
		SLAVE_RECV,
		SLAVE_SEND
	};
	TwoWireStatus_ status;
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

/**********************    Configure clock    *********************/

void SetClock(Twi* pTWI , uint32_t frequency) {
  uint32_t CLDIV = 0;
  uint32_t CKDIV = 0;
  uint8_t readyByte = 0;

  while (!readyByte) {
    CLDIV = ((F_CPU / (2 * frequency)) - 4) / (1 << CKDIV) ;
    if ( CLDIV <= 255 ) {
      readyByte = 1 ;
    }
    else {
      CKDIV++ ;
    }
  }
  pTWI->TWI_CWGR = (CKDIV << 16) | (CLDIV << 8) | CLDIV;
}
void I2c_Disable(Twi *pTwi)
{
    //assert( pTwi ) ;

    uint32_t i;

    /* TWI software reset */
    pTwi->TWI_CR = TWI_CR_SWRST;
    pTwi->TWI_RHR;

    /* Wait at least 10 ms */
    for (i=0; i < 1000000; i++);

    /* TWI Slave Mode Disabled, TWI Master Mode Disabled*/
    pTwi->TWI_CR = TWI_CR_SVDIS | TWI_CR_MSDIS;
}

/*********************     Init TWIn      ************************/

void I2c_Init(Twi* pTWI, bool Master) {
  assert( pTWI ) ;
  pTWI->TWI_CR = 0;
  pTWI->TWI_CR = TWI_CR_SVDIS | TWI_CR_MSDIS; // Disable Master and Slave modes
  //pTWI->TWI_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;
  if (pTWI == TWI0) {
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

  pTWI->TWI_CR |= TWI_CR_SWRST;  // TWIn software reset
  pTWI->TWI_RHR;                // Flush reception buffer

  

  //Enable master mode
  if (Master == true) {
    //enter slave address
    pTWI->TWI_MMR |= TWI_MMR_DADR(I2C_ADDRESS);

    pTWI->TWI_CR |= TWI_CR_MSEN ;  // Master mode enable
    
    //clockwave from 100khz to 400khz
    SetClock(pTWI, 400000); // from 100000 to 400000
  }
  else { // Enable Slave mode
    pTWI->TWI_SMR = 0;
    pTWI->TWI_SMR = TWI_SMR_SADR(I2C_ADDRESS);
    pTWI->TWI_CR |= TWI_CR_SVEN | TWI_CR_MSDIS;     // Slave mode enable
    for (int i=0; i < 1000000; i++);
    assert( (pTWI->TWI_CR & TWI_CR_SVDIS)!= TWI_CR_SVDIS ) ;
  }
  //mkSerial.println((pTWI->TWI_CR & TWI_CR_SVDIS)!= TWI_CR_SVDIS);

// pTWI->TWI_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;
//   PMC->PMC_PCER0 |= PMC_PCER0_PID23;      // TWI1 power ON
//     PIOB->PIO_PDR |= PIO_PDR_P13            // Enable peripheral control
//                      | PIO_PDR_P12;
//     PIOB->PIO_ABSR &= ~(PIO_PB12A_TWD1      // TWD1 & TWCK1 Peripherals type A
//                         | PIO_PB13A_TWCK1);
//   pTWI->TWI_CR = TWI_CR_SWRST;  // TWIn software reset
//   pTWI->TWI_RHR;                // Flush reception buffer

//   pTWI->TWI_CR = TWI_CR_SVDIS | TWI_CR_MSDIS; // Disable Master and Slave modes
//   pTWI->TWI_SMR = 0;
//     pTWI->TWI_SMR = TWI_SMR_SADR(I2C_ADDRESS);
//     pTWI->TWI_CR = TWI_CR_SVEN;     // Slave mode enable
//     for (int i=0; i < 1000000; i++);
//     assert( (pTWI->TWI_CR & TWI_CR_SVDIS)!= TWI_CR_SVDIS ) ;

}

/*****************   Set Interrupt Configuration   *********************/
size_t write_(uint8_t data) {
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
int I2C_available(void) {
	return rxBufferLength - rxBufferIndex;
}

int I2C_read(void) {
	if (rxBufferIndex < rxBufferLength)
		return rxBuffer[rxBufferIndex++];
	return -1;
}

int I2C_peek(void) {
	if (rxBufferIndex < rxBufferLength)
		return rxBuffer[rxBufferIndex];
	return -1;
}

void I2C_flush(void) {
	// Do nothing, use endTransmission(..) to force
	// data transfer.
}
	// Callback user functions
	void onRequestCallback(void)
  {
    
    if(gIRQ_I2C_FLAG_DONE==1)
    {
      gIRQ_I2C_FLAG_DONE=0;
      write_('z');
      
    }
    else {
      write_('a');
    }
    // if(i2c_cnt>=24)
    // {
    //    mkSerial.println("onRequestCallback");
    //   i2c_cnt=0;
    //   for(int i=0; i<4; i++)
    //     mkSerial.print(i2c_buff[i]);
    //   mkSerial.println(" << end");



    //  // TWI_WriteByte(TWI1, 10);
    //   //write_('z');
    //   // mkSerial.print("srvBufferLength: ");
    //   // mkSerial.println((int)srvBufferLength);
    //   // mkSerial.print("srvBufferIndex: ");
    //   // mkSerial.println((int)srvBufferIndex);
      

      
    // }
    // mkSerial.println("onRequestCallback");
  }
  //////////////////////////////
	void onReceiveCallback(int len){
    //mkSerial.println("onReceiveCallback");
    while(I2C_available())
    {
      // mkSerial.print("Rev: ");
      // mkSerial.println(I2C_read());
      i2c_buff[i2c_cnt] = I2C_read();
      //  mkSerial.print(i2c_cnt);
      //  mkSerial.print(", ");
      // mkSerial.println(i2c_buff[i2c_cnt]);
      i2c_cnt++;
      if(i2c_cnt>=24)
      {
        memcpy(&revData, &i2c_buff[0], 24);
        mkVelProfile.updateVelocityProfile(revData.steps,
                                    revData.Na,
                                    revData.Nac,
                                    revData.NNb,
                                    revData.Cn_acc0,
                                    revData.dir
                                    );
        i2c_cnt=0;
      }
    }
  }

	// Called before initialization
	void onBeginCallback(void){
     mkSerial.println("onBeginCallback");
  }

	// Called after deinitialization
	void (*onEndCallback)(void);
void I2c_Interrupt(Twi* pTWI, uint32_t InterrConfig) {
  status = SLAVE_IDLE;
  TWI1->TWI_CR |= TWI_CR_START;
  pTWI->TWI_IER = InterrConfig;
  if (pTWI == TWI0) {
    NVIC_SetPriority(TWI1_IRQn, 2);
    NVIC_EnableIRQ(TWI0_IRQn);
  }
  else {
    NVIC_SetPriority(TWI1_IRQn, 2);
    NVIC_EnableIRQ(TWI1_IRQn);
  }
}

/*******************        TWI1 Handler    ***********************/


void TWI1_Handler(void) {
  // uint32_t status = TWI0->TWI_SR;
  // if (status & TWI_SR_SVREAD) {
  //   uint8_t c = TWI0->TWI_RHR;
  //   mkSerial.println(c);

  // }
  // Retrieve interrupt status
	uint32_t sr = TWI_GetStatus(TWI1);

	if (status == SLAVE_IDLE && TWI_STATUS_SVACC(sr)) {
		TWI_DisableIt(TWI1, TWI_IDR_SVACC);
		TWI_EnableIt(TWI1, TWI_IER_RXRDY | TWI_IER_GACC | TWI_IER_NACK
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
				write_((uint8_t) 0);
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
		TWI_EnableIt(TWI1, TWI_SR_SVACC);
		TWI_DisableIt(TWI1, TWI_IDR_RXRDY | TWI_IDR_GACC | TWI_IDR_NACK
				| TWI_IDR_EOSACC | TWI_IDR_SCL_WS | TWI_IER_TXCOMP);
		status = SLAVE_IDLE;
	}

	if (status == SLAVE_RECV) {
		if (TWI_STATUS_RXRDY(sr)) {
			if (srvBufferLength < BUFFER_LENGTH)
				srvBuffer[srvBufferLength++] = TWI_ReadByte(TWI1);
		}
	}

	if (status == SLAVE_SEND) {
		if (TWI_STATUS_TXRDY(sr) && !TWI_STATUS_NACK(sr)) {
			uint8_t c = 'x';
			if (srvBufferIndex < srvBufferLength)
      {
        c = srvBuffer[srvBufferIndex++];
        TWI_WriteByte(TWI1, c);
      }
     
		}
	}
}

/************************     Write 1 byte     *******************/

void I2c_WriteByte(Twi* pTWI, uint8_t data) {
  pTWI->TWI_THR |= data;
  //wait for ack
  while (!(pTWI->TWI_SR & TWI_SR_TXRDY));
}
/****************************     Start    ****************************/

// void I2c_Start(Twi* pTWI, uint8_t slave_address, uint8_t mread) { //read=1, write=0
//   //set slave address
//   pTWI->TWI_MMR = (pTWI->TWI_MMR & ~TWI_MMR_DADR_Msk)
//                   | TWI_MMR_DADR(slave_address);
//   //set read/write direction
//   if (mread == write) { //write
//     pTWI->TWI_MMR &= ~TWI_MMR_MREAD;
//   }
//   else if (mread == read) { //read
//     pTWI->TWI_MMR |= TWI_MMR_MREAD;
//   }
//   //send start
//   pTWI->TWI_CR |= TWI_CR_START;

//   //wait for ack
//   while (!(pTWI->TWI_SR & TWI_SR_TXRDY));
// }

/***************************   Stop    ****************************/

void I2c_Stop(Twi* pTWI) {
  pTWI->TWI_CR |= TWI_CR_STOP;
}
/**********************     Read 1 byte   **************************/

uint8_t I2c_ReadByte(Twi* pTWI) {
  uint8_t receivedByte;
  //If the stop bit in the control register is not set,
  //Sam3x will automatically ACK after reading TWIn_RHR register
  //RXRDY will be set when data arrives in TWIn_RHR register

  while (!(pTWI->TWI_SR & TWI_SR_RXRDY));
  //reading data will clear RXRDY bit in the status register
  receivedByte = pTWI->TWI_RHR;
  return receivedByte;
}
/***********************    Write last byte    *******************/

void I2c_WriteLastByte(Twi* pTWI, uint8_t data) {
  pTWI->TWI_THR |= data;
  I2c_Stop(pTWI);
  //wait for ack
  while (!(pTWI->TWI_SR & TWI_SR_TXRDY));
  while (!(pTWI->TWI_SR & TWI_SR_TXCOMP));
}

/*****************     Read the last byte   ***********************/

uint8_t I2c_ReadLastByte(Twi* pTWI) {
  uint8_t receivedByte;
  //Sam3x requires stop bit to be set before data is set on the TWIn_RHR
  //when stop bit is set, Sam3x will send a NACK instead of an ACK automatically
  I2c_Stop(pTWI);
  //When data arrives in the TWIn_RHR register RXRDY is set in the Status Register
  while (!(pTWI->TWI_SR & TWI_SR_RXRDY));
  //reading data will clear RXRDY bit in the status register
  receivedByte = pTWI->TWI_RHR;
  while (!(pTWI->TWI_SR & TWI_SR_TXCOMP));
  return receivedByte;
}
/////////////////////////////////////////////////////////////
#endif


// Callback user functions
	void I2C_RequestCallback(void)
  {
    
    if(gIRQ_I2C_FLAG_DONE==1)
    {
      gIRQ_I2C_FLAG_DONE=0;
      mkI2C.write('z');
      
    }
    else {
      mkI2C.write('a');
    }
    
  }
  //////////////////////////////
	void I2C_ReceiveCallback(int len){
    //mkSerial.println("onReceiveCallback");
    while(mkI2C.available())
    {
      // mkSerial.print("Rev: ");
      // mkSerial.println(I2C_read());
      i2c_buff[i2c_cnt] = mkI2C.read();
      //  mkSerial.print(i2c_cnt);
      //  mkSerial.print(", ");
      // mkSerial.println(i2c_buff[i2c_cnt]);
      i2c_cnt++;
      if(i2c_cnt>=24)
      {
        memcpy(&revData, &i2c_buff[0], 24);
        mkVelProfile.set_speed_profile(revData.steps,
                                    revData.Na,
                                    revData.Nac,
                                    revData.NNb,
                                    revData.Cn_acc0,
                                    revData.dir
                                    );
        i2c_cnt=0;
      }
    }
  }

int main(void)
{
    ////////////////////////+++ SPI +++///////////////////////////////////
  char databuf[64];
  int recordlen[10];
  int count = 0;
  int record = 0;

  int modefault = 0;
  int overrun = 0;
  uint32_t StatusReg;
  ////////////////////////--- SPI ---/////////////////////////////////// 
  char buffer[128];



  init_controller();
  //TWI_ConfigureSlave(TWI0, 0x5b);
  // TWI1 twi1 = TWI1;
  // TWI1->TWI_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;
  // TWI_ConfigureSlave(TWI1, 0x5b);
	// //status = SLAVE_IDLE;
	// TWI_EnableIt(TWI1, TWI_IER_SVACC);
  mkSerial.begin(115200);
  mkSerial.println("FROM SAM3X8 CONTROLLER!");
  
  // I2c_Init(TWI1, false);  // TWI0 Slave
  // I2c_Interrupt(TWI1, TWI_IER_SVACC);// | TWI_IER_NACK | TWI_IER_RXRDY | TWI_IER_TXCOMP| TWI_IER_RXBUFF | TWI_IER_TXBUFE); //TWI_SR_SVREAD


  mkI2C.begin(I2C_SLAVE, 0x5B);
  
  
  mkI2C.onRequest(I2C_RequestCallback);
  mkI2C.onReceive(I2C_ReceiveCallback);
  mkI2C.startInterrupt(TWI_IER_SVACC);
  //mkI2C.write(0);


  ////////////////////////+++ SPI +++///////////////////////////////////
  // setupIO_Input(PIOA,PIO_IDR_P26);// MOSI A26
  // setupIO_Input(PIOA,PIO_IDR_P27);//CLK A27
  // setupIO_Output(PIOA,PIO_IDR_P25);// MISO A25
  //SPI_Configure(SPI0, ID_SPI0, SPI_MR_MSTR | SPI_MR_PS | SPI_MR_MODFDIS);// for master
  // SPI_Configure(SPI0, ID_SPI0,  SPI_MR_PS | SPI_MR_MODFDIS);
  // SPI_ConfigureNPCS(SPI0, 0, SPI_CSR_NCPHA);
  // SPI_Enable(SPI0);
  ////////////////////////--- SPI ---/////////////////////////////////// 
  // // SET IO MAP FOR MOTOR #1 ON TIMER 0
  IOMap[TC_0][ST_PULSE].pIO = PIOB; IOMap[TC_0][ST_PULSE].pin = PIO_OER_P26; // PULSE FOR STEPPER MOTOR ON PIN 22
  IOMap[TC_0][ST_DIR].pIO =   PIOA; IOMap[TC_0][ST_DIR].pin = PIO_OER_P14;     // DIRECTION FOR STEPPER MOTOR ON PIN 23
  IOMap[TC_0][HOMESTOP].pIO = PIOA; IOMap[TC_0][HOMESTOP].pin = PIO_OER_P15; // HOME STOP FOR STEPPER MOTOR ON PIN 24
  IOMap[TC_0][ENDSTOP].pIO = PIOD;  IOMap[TC_0][ENDSTOP].pin = PIO_OER_P0;  // END STOP FOR STEPPER MOTOR ON PIN 24

  //ENABLE IO PIN AS AN OUTPUT
  for(int timer=0; timer<MAX_MOTOR_NUM; timer++)
  {
    for (int map=0; map<4; map++)
      setupIO_Output(IOMap[timer][map].pIO, IOMap[timer][map].pin);
  }

  mkCommand.setCallBack_gen_speed_profile(mkVelProfile.gen_speed_profile);
  mkCommand.setCallBack_set_speed_profile(mkVelProfile.set_speed_profile);
  mkCommand.setCallBack_update_speed_only(mkVelProfile.update_speed_only);
  // mkSerial.begin(115200);
  // mkSerial.println("FROM SAM3X8 CONTROLLER!");

  //TC0 channel 0, the IRQ0 for that channel and the desired frequency
  startTimer(TC0, 0, TC0_IRQn); 
  ///////////////////////////////////////////////////

  gIRQ_SPI_FLAG_RX=0;
  /* Main loop */
  while(1) {
    if(mkCommand.buflen < (BUFSIZE-1)){
        mkCommand.getCommand();
    }
    if(mkCommand.buflen) {
      mkSerial.println(mkCommand.buflen);
      mkCommand.process_commands();
      mkCommand.buflen = (mkCommand.buflen-1);
      mkCommand.bufindr = (mkCommand.bufindr + 1)%BUFSIZE;
    }
    ////////////////////////+++ SPI +++/////////////////////////////////// 
    /*
    StatusReg = SPI0->SPI_SR;
    
    // Count mode faults and overruns
    if (StatusReg & SPI_SR_MODF){
        modefault++;
        mkSerial.println("I got SPI modefault Error!");
        }
    
    if (StatusReg & SPI_SR_OVRES)
        overrun++;
    
    // Rising edge on slave select (e.g. new message!)
    if (StatusReg & SPI_SR_NSSR)
    {
        // recordlen[record] = count;

        count = 0;
        record++;
        //mkSerial.println("I got SPI message.");
    }
    
      // Receive data is available. Got it.
      if (StatusReg & SPI_SR_RDRF)
      {
          uint8_t readval = SPI0->SPI_RDR ;
          databuf[count] = readval;
          count++;
      }
      if(count>=19)
      {
        databuf[count]='\0';
        mkSerial.print(databuf);
        mkSerial.println("I got SPI message.");
        mkSPI.write(0xEE);
        count =0; 
      }
*/
    // if(gIRQ_SPI_FLAG_RX==1)
    //  {
    //    // mkSPI.startWrite();
    //    mkSPI.write(0x3A);
    //    // mkSPI.write(0xEE);
    //    // mkSPI.write(0xFE);
    //    // mkSPI.endWrite();

    // //     memcpy(&revData, spi_databuf, 24);
    //     mkSerial.print("Steps: ");
    //     mkSerial.print(revData.steps);
    //     mkSerial.print(", Cn_acc0: ");
    //     mkSerial.println(revData.Cn_acc0);

    //      gIRQ_SPI_FLAG_RX=0;
    //  } 
    
      ////////////////////////--- SPI ---/////////////////////////////////// 
    if(gIRQ_TC_FLAG_DONE[nTimer]==1)
    {
      // NVIC_DisableIRQ(TC0_IRQn);
      // elapsedTime[nTimer] = millis() - elapsedTime[nTimer];
      // float current = elapsedTime[nTimer]*0.001;
      // float speed_mmPsec = speedData[nTimer].step_count*0.025/current;
      
      // mkSerial.print("time[sec]: ");
      // mkSerial.println(current);
      // mkSerial.print("speed[mm/sec]: ");
      // mkSerial.println(speed_mmPsec);
      // mkSerial.println("----------------END OF MOTOR #1  -------------");
      speedData[nTimer].activated=false;
      speedData[nTimer].step_count = 0;
      gIRQ_TC_FLAG_DONE[nTimer]=0;
      gIRQ_I2C_FLAG_DONE = 1;
      // ...NOTIFY TO HOST...
      //mkSerial.println("READY"); //for serial notification
    } 
    
    
  }
  return 0;
}
/////////////////////////////////////////////

