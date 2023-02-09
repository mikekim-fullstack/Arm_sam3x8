/**
  mkMain.cpp
  Copyright (c) 2020 Mike Kim (ZOE Robotics).  All right reserved.
  4 Sept 2020 by Mike Kim
*/
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

// #include "twi.h"
// #include "mkZoeRobotics_spi.h"
// #include "mkZoeRobotics_i2c.h"

#include "DueFlashStorage.h"


#define ADDR_FLASH_INIT 0
#define ADDR_FLASH_STEPCNT 4

//////////////////////////////////////////////////////////////
volatile static uint16_t gIRQ_SPI_FLAG_RX = 0;
volatile static uint16_t gIRQ_I2C_FLAG_DONE = 0, gIRQ_I2C_FLAG_STARTED = 0, gIRQ_I2C_FLAG_CANCELED = 0;
static uint8_t spi_databuf[24];
volatile static int spi_cnt = 0;

static uint8_t DataReceived = 0;
//volatile uint32_t step_absPos=0;// absolute step counter to track current position
volatile int8_t activatedEE = -1;
static CIRCLEProfile revDataCircle; // from I2C input
static SPEEDProfile revData;        // from I2C input
POSData posData;
SPEEDRampData speedData[MAX_MOTOR_NUM] = {0};
//portIOPair IOMap[5] = {0};
MOTORCHANEL motorCh[3]={0};
struct GLOBAL_FLAGS g_status;
unsigned long elapsedTime[MAX_MOTOR_NUM] = {0};
volatile uint32_t gIRQ_step_count[MAX_MOTOR_NUM] = {0};
volatile uint16_t gIRQ_TC_FLAG_DONE[MAX_MOTOR_NUM] = {0};
int motorID = 0;

//MKStepperMotorCtrl mkSTMotorCtrl;
extern MKSerial mkSerial;
extern MKCommand mkCommand;
extern MKVelProfile mkVelProfile;
// extern MKSPI mkSPI;
// extern MKI2C mkI2C1;
OP_MODE OperationMode;
volatile uint8_t statusHoming[3] = {0, 0, 0};
volatile char reportSteps[128];

volatile char activateTimer1=0;
char tmpBuffer[64]={0};
///////////////////////////////////////////////////////////////

// Speed ramp states
// #define STOP 0
// #define ACCEL 1
// #define DECEL 2
// #define RUN 3
void stopNow();
void shutdownNow();
uint8_t getHomeSWStatus();
uint8_t getStatusHoming();
void setStatusHoming(uint8_t val);
inline void reportStatus();
void reportStatus(int codeVaule);

inline static uint8_t pioReadInput(Pio *pPio, uint32_t pin);
void homeMode()
{
}
////////////////////////////////////////////////////////
// Handling Velocity Profile
void init_interrupt()
{
  pmc_enable_periph_clk(ID_PIOA);
  NVIC_DisableIRQ(PIOA_IRQn);
  NVIC_ClearPendingIRQ(PIOA_IRQn);
  NVIC_SetPriority(PIOA_IRQn, 0);
  NVIC_EnableIRQ(PIOA_IRQn);

  pmc_enable_periph_clk(ID_PIOB);
  NVIC_DisableIRQ(PIOB_IRQn);
  NVIC_ClearPendingIRQ(PIOB_IRQn);
  NVIC_SetPriority(PIOB_IRQn, 0);
  NVIC_EnableIRQ(PIOB_IRQn);

  pmc_enable_periph_clk(ID_PIOC);
  NVIC_DisableIRQ(PIOC_IRQn);
  NVIC_ClearPendingIRQ(PIOC_IRQn);
  NVIC_SetPriority(PIOC_IRQn, 0);
  NVIC_EnableIRQ(PIOC_IRQn);

  pmc_enable_periph_clk(ID_PIOD);
  NVIC_DisableIRQ(PIOD_IRQn);
  NVIC_ClearPendingIRQ(PIOD_IRQn);
  NVIC_SetPriority(PIOD_IRQn, 0);
  NVIC_EnableIRQ(PIOD_IRQn);
}
static void startTimer0(uint32_t frequency = 350)
{
  Tc *tc = TC0;
  uint32_t channel = 0;
  IRQn_Type irq = TC0_IRQn;
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk((uint32_t)irq);
  //TC_Configure(tc, channel,  TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK2 | TC_CMR_ACPC_CLEAR | TC_CMR_ASWTRG_CLEAR);//  | TC_CMR_CPCSTOP);
  TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK2);
  uint32_t rc = F_CPU / 8 / frequency; //2 because we selected TIMER_CLOCK1 above
  TC_SetRA(tc, channel, rc);
  TC_SetRC(tc, channel, rc);

  tc->TC_CHANNEL[channel].TC_IER = TC_IER_CPCS;
  tc->TC_CHANNEL[channel].TC_IDR = ~TC_IER_CPCS;
  NVIC_ClearPendingIRQ(irq);
  NVIC_SetPriority(irq, 0);
  NVIC_EnableIRQ(irq);
  TC_Start(tc, channel);
}
////////////////////////////////////////////////////////
// Handling Pin 22 for 10microseconds(100kHz) duration pulse
static void startTimer1(uint32_t frequency = 10000)
{
  Tc *tc = TC0;
  uint32_t channel = 1;
  IRQn_Type irq = TC1_IRQn;
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk((uint32_t)irq);
  //TC_Configure(tc, channel,  TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK2 | TC_CMR_ACPC_CLEAR | TC_CMR_ASWTRG_CLEAR);//  | TC_CMR_CPCSTOP);
  TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK1);
  uint32_t rc = F_CPU / 2 / frequency; //1288 because we selected TIMER_CLOCK4 above
  TC_SetRA(tc, channel, rc);
  TC_SetRC(tc, channel, rc);

  tc->TC_CHANNEL[channel].TC_IER = TC_IER_CPCS;
  tc->TC_CHANNEL[channel].TC_IDR = ~TC_IER_CPCS;
  NVIC_ClearPendingIRQ(irq);
  NVIC_SetPriority(irq, 1);
  NVIC_EnableIRQ(irq);
  TC_Start(tc, channel);
}
////////////////////////////////////////////////////////
// Reporting steps every 100mseconds(10Hz) duration pulse
static void startTimer2(uint32_t frequency = 10)
{
  Tc *tc = TC0;
  uint32_t channel = 2;
  IRQn_Type irq = TC2_IRQn;
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk((uint32_t)irq);
  //TC_Configure(tc, channel,  TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK2 | TC_CMR_ACPC_CLEAR | TC_CMR_ASWTRG_CLEAR);//  | TC_CMR_CPCSTOP);
  TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);
  uint32_t rc = F_CPU / 128 / frequency; //128 because we selected TIMER_CLOCK4 above
  TC_SetRA(tc, channel, rc);
  TC_SetRC(tc, channel, rc);

  tc->TC_CHANNEL[channel].TC_IER = TC_IER_CPCS;
  tc->TC_CHANNEL[channel].TC_IDR = ~TC_IER_CPCS;
  NVIC_ClearPendingIRQ(irq);
  NVIC_SetPriority(irq, 3);
  NVIC_EnableIRQ(irq);
  TC_Start(tc, channel);
}
 void stopTimer0()
{
  Tc *tc = TC0;
  uint32_t channel = 0;
  IRQn_Type irq = TC0_IRQn;
  NVIC_DisableIRQ(irq);
  TC_Stop(tc, channel);
}
 void stopTimer1()
{
  Tc *tc = TC0;
  uint32_t channel = 1;
  IRQn_Type irq = TC1_IRQn;
  NVIC_DisableIRQ(irq);
  TC_Stop(tc, channel);
}
 void stopTimer2()
{
  Tc *tc = TC0;
  uint32_t channel = 2;
  IRQn_Type irq = TC2_IRQn;
  NVIC_DisableIRQ(irq);
  TC_Stop(tc, channel);
}
////////////////////////////////////////////////////////////////////////
inline static void pinModeAsOutput(Pio *portIO, uint32_t pin)
{
  portIO->PIO_IDR = pin;  // disable interrupt
  portIO->PIO_MDDR = pin; // Disable to connect multi devices
  portIO->PIO_OWER = pin; // Enable writing on ODSR

  portIO->PIO_PER = pin;  // PIO ENABLE
  portIO->PIO_OER = pin;  // OUTPUT ENALBE
  portIO->PIO_PUDR = pin; // PULL-UP DISALBLE
  //portIO->PIO_PUER = pin; // PULL-UP ENALBLE
}
inline static void pinModeAsInput(Pio *portIO, uint32_t pin)
{
  // pmc_enable_periph_clk(PIOB_IRQn);
  // NVIC_DisableIRQ(PIOB_IRQn);
  // NVIC_ClearPendingIRQ(PIOB_IRQn);
  // NVIC_SetPriority(PIOB_IRQn, 1);
  // NVIC_EnableIRQ(PIOB_IRQn);

  portIO->PIO_PER = pin;  // PIO ENABLE
  portIO->PIO_ODR = pin;  // OUTPUT DISABLE (INPUT ENABLED)
  portIO->PIO_IDR = pin;  // disable interrupt
  portIO->PIO_PUDR = pin; // PULL-UP  DISABLE
  // portIO->PIO_PUER = pin; // PULL-UP  ENABLE
  //portIO->PIO_IFER = pin; // INPUTFILTER ENABLE
  portIO->PIO_IFDR = pin;

  //portIO->PIO_MDDR = pin; // Disable to connect multi devices
  //portIO->PIO_OWER = pin; // Enable writing on ODSR

  //

  // portIO->PIO_AIMDR = pin; //any change

  // portIO->PIO_ESR = pin;    // "Edge" Select Register
  // portIO->PIO_FELLSR = pin; // "Falling Edge / Low Level" Select Register

  // portIO->PIO_LSR = pin;    // "Level" Select Register
  // portIO->PIO_FELLSR = pin; // "Falling Edge / Low Level" Select Register

  // portIO->PIO_IER = pin; // Enable interrupt

  // portIO->PIO_LSR = pin;    // "Level" Select Register
  // 		portIO->PIO_REHLSR = pin; // "Rising Edge / High Level" Select Register
}

inline static void pioWriteOutput(Pio *pPio, uint32_t pin, uint32_t val)
{
  // Disable Interrupt...
  // pPio->PIO_IDR = pin; //PIO_OER_P26 ;
  // // setup PULLUP (NOMALLY LOW)
  // pPio->PIO_PUER = pin;
  /* Set default value */
  if (val)
  {
    pPio->PIO_SODR = pin;
  }
  else
  {
    pPio->PIO_CODR = pin;
  }
  /* Configure pin(s) as output(s) */
  // pPio->PIO_OER = pin;
  // pPio->PIO_PER = pin;
}
uint8_t getStatusHoming()
{
  if(motorID<0) return 0;
  return statusHoming[motorID];
}
void setStatusHoming(uint8_t val)
{
  if(motorID<0) return;
   statusHoming[motorID]=val;
}
uint8_t getHomeSWStatus()
{
  if(motorID<0) return 0;
  return pioReadInput(motorCh[motorID].swH.pIO, motorCh[motorID].swH.pin);
  //return pioReadInput(IOMap[HOMESTOP].pIO, IOMap[HOMESTOP].pin);
}
inline static uint8_t pioReadInput(Pio *pPio, uint32_t pin)
{
  if ((pPio->PIO_PDSR & pin) == 0)
    return 0;
  else
    return 1;
}
inline static void setMotorDirection(int ch, int8_t dir)
{

  if (dir == 1)
  { // CCW
  
   pioWriteOutput(motorCh[ch].dir.pIO, motorCh[ch].dir.pin, HIGH);
  }
  else
  { // CW
    pioWriteOutput(motorCh[ch].dir.pIO, motorCh[ch].dir.pin, LOW);
  }
}

inline static void writePluse(Pio *portIO, uint32_t pin)
{
  portIO->PIO_ODSR ^= pin;
  portIO->PIO_ODSR ^= pin;
}
inline static void writePluse(Pio *portIO, uint32_t pin, uint32_t value)
{
  if (value)
  {
    portIO->PIO_SODR = pin; // Set Output Data Register
  }
  else
  {
    portIO->PIO_CODR = pin; // Clear Output Data Register
  }
}

//////////////////////////////////////////////////////////
volatile static uint8_t irqBBusy = 0;
void PIOB_Handler()
{
  if (!irqBBusy)
  {
    uint32_t isr = PIOB->PIO_ISR;
    if ((isr & motorCh[motorID].swH.pin)) // & IOMap[HOMESTOP].pin)
    {
      irqBBusy = 1;
      mkSerial.println("PIN B 21 Interrupted");
      irqBBusy = 0;
    }
  }
  //   uint32_t isr = PIOC->PIO_ISR;
  // uint8_t leading_zeros;
  // while((leading_zeros=__CLZ(isr))<32)
  // {
  //   uint8_t pin=32-leading_zeros-1;
  //   if(pin==21)
  //     mkSerial.println("PIN B 21 Interrupted");
  //   isr=isr&(~(1<<pin));
  // }
  // mkSerial.println("PIN B Interrupted");
  // if (pioReadInput(IOMap[HOMESTOP].pIO, IOMap[HOMESTOP].pin))
  //   mkSerial.println("PIN B 21 Interrupted");
}
//////////////////////////////////////////////////////////
volatile static int nP = 0;
volatile static uint32_t circle_step = 0, totalsteps = 0;
void TC0_Handler()
{
  TC_GetStatus(TC0, 0);
  // Test
  /*
  ////////////////////////////////
  // mkSerial.println("++++++++++++++++++ Timer0");
    
  writePluse(IOMap[ST_PULSE].pIO, IOMap[ST_PULSE].pin, HIGH);
  writePluse(IOMap[4].pIO, IOMap[4].pin, HIGH);
  startTimer1(150000); // make 10microseconds period (100KHz) for pulse...
    // if(activateTimer1==0) 
    // {
    //   writePluse(IOMap[ST_PULSE].pIO, IOMap[ST_PULSE].pin, HIGH);
    //   writePluse(IOMap[4].pIO, IOMap[4].pin, HIGH);
    //   activateTimer1=1;
    // }
   // 
*/
  /////////////////////////////////////////////////////////////
  // +++ EndEffector Acr Motion +++ //
/*
  
  if (activatedEE == 1)
  {
    // ...GIVE AN IMPULSE TO MOTOR...
    //writePluse(IOMap[nTimer][ST_PULSE].pIO, IOMap[nTimer][ST_PULSE].pin);
    pioWriteOutput(IOMap[ST_PULSE].pIO, IOMap[ST_PULSE].pin, HIGH);
    startTimer1(); // make 10microseconds period (100KHz) for pulse...

    // ...CLEAR TC STATUS REGISTER...
    TC_SetRC(TC0, 0, mkVelProfile.motionData[nP].Cn);

    ++circle_step;
    sendData.step_pos += mkVelProfile.motionData[nP].dir;

    if (circle_step >= mkVelProfile.motionData[nP].steps)
    {
      circle_step = 0;
      ++nP;

      setMotorDirection(mkVelProfile.motionData[nP].dir);
    }
    // pioWriteOutput(PIOB, PIO_OER_P26, LOW);
    // +++ end of profile +++ //
    if (nP >= mkVelProfile.nM) // nM: total number of steps
    {
      stopTimer1();
      circle_step = 0;
      nP = 0;
      setMotorDirection(mkVelProfile.motionData[nP].dir);
      // float eltime = (millis() - elapsedTime[0]) * 0.001;
      // mkSerial.println(totalsteps);
      // mkSerial.println(eltime);
      // elapsedTime[0] = millis();
    }
    //writePluse(IOMap[nTimer][ST_PULSE].pIO, IOMap[nTimer][ST_PULSE].pin, LOW);
  } /////////////////////////////////////////////////////////////
  // +++ Linear Motion +++ //
  */
 //mkSerial.println(speedData[nTimer].activated);
  
  if (speedData[motorID].activated)
  {
   // mkSerial.println("++++++++++++++++++ Timer0");
    writePluse(motorCh[0].pulse.pIO, motorCh[0].pulse.pin, HIGH);
    writePluse(motorCh[1].pulse.pIO, motorCh[1].pulse.pin, HIGH);
    writePluse(motorCh[2].pulse.pIO, motorCh[2].pulse.pin, HIGH);
    startTimer1(50000); // make 10microseconds period (100KHz) for pulse...
    // ...CLEAR TC STATUS REGISTER...
    TC_SetRC(TC0, 0, speedData[motorID].Cn);
     
    // ++speedData[nTimer].step_count;
    // posData.abs_step_pos += mkVelProfile.motionData[nP].dir;
    // if (speedData[nTimer].dir > 0)
    //   ++sendData.step_pos;
    // else
    //   --sendData.step_pos;
    // ... ACCELERATION AREA ....
    
    if (speedData[motorID].step_count < speedData[motorID].Na)
    {
      if (speedData[motorID].step_count == 0)
      {
        elapsedTime[motorID] = millis();
        speedData[motorID].Cn = speedData[motorID].Cn_acc0;
        speedData[motorID].rest = 0;
      }
      else
      {
        //%%% SQRT approximation of speedData[num].Cn = speedData[num].Cn_acc0*(sqrt(step_count+1)-sqrt(step_count));
        speedData[motorID].C = speedData[motorID].Cn * (1 - 2.0 / (4 * speedData[motorID].step_count + 1)) + speedData[motorID].rest;
        speedData[motorID].Cn = floor(speedData[motorID].C);
        speedData[motorID].rest = speedData[motorID].C - speedData[motorID].Cn;
      }
    }
    // ... DECELERATION AREA ...
    else if (speedData[motorID].step_count >= speedData[motorID].Nac && speedData[motorID].step_count < speedData[motorID].steps)
    {

      //%%% SQRT approximation
      if (speedData[motorID].step_count == speedData[motorID].Nac)
      {
        speedData[motorID].rest = 0;
      }
      else
      {
        speedData[motorID].C = speedData[motorID].Cn * (1 + 2.0 / (4 * speedData[motorID].NNb - 1)) + speedData[motorID].rest;
        speedData[motorID].Cn = floor(speedData[motorID].C);
        speedData[motorID].rest = speedData[motorID].C - speedData[motorID].Cn;
      }
      speedData[motorID].NNb--;
    }
    // ... FINISHED SPEED PROFILE AND NOTIFY IN MAIN LOOP...
    else if (speedData[motorID].step_count >= speedData[motorID].steps)
    {
      stopTimer1();
      gIRQ_TC_FLAG_DONE[motorID] = 1;
    }
    
  } // ---------end of if(speedData[nTimer].activated)
  
}
// -------------------- END OF TC0_Handler ---------------------------

//////////////////////////////////////////////////////////
/*
void TC0_Handler_origin()
{
  TC_GetStatus(TC0, 0);
  if (speedData[nTimer].activated)
  {
    // ...GIVE AN IMPULSE TO MOTOR...
    writePluse(IOMap[nTimer][ST_PULSE].pIO, IOMap[nTimer][ST_PULSE].pin);

    // ...CLEAR TC STATUS REGISTER...
    TC_SetRC(TC0, 0, speedData[nTimer].Cn);
    ++speedData[nTimer].step_count;
    if (speedData[nTimer].dir > 0)
      ++sendData.step_pos;
    else
      --sendData.step_pos;
    // ... ACCELERATION AREA ....
    if (speedData[nTimer].step_count < speedData[nTimer].Na)
    {
      if (speedData[nTimer].step_count == 0)
      {
        speedData[nTimer].Cn = speedData[nTimer].Cn_acc0;
        speedData[nTimer].rest = 0;
      }
      else
      {
        //%%% SQRT approximation of speedData[num].Cn = speedData[num].Cn_acc0*(sqrt(step_count+1)-sqrt(step_count));
        speedData[nTimer].C = speedData[nTimer].Cn * (1 - 2.0 / (4 * speedData[nTimer].step_count + 1)) + speedData[nTimer].rest;
        speedData[nTimer].Cn = floor(speedData[nTimer].C);
        speedData[nTimer].rest = speedData[nTimer].C - speedData[nTimer].Cn;
      }
    }
    // ... DECELERATION AREA ...
    else if (speedData[nTimer].step_count >= speedData[nTimer].Nac && speedData[nTimer].step_count < speedData[nTimer].steps)
    {

      //%%% SQRT approximation
      if (speedData[nTimer].step_count == speedData[nTimer].Nac)
      {
        speedData[nTimer].rest = 0;
      }
      else
      {
        speedData[nTimer].C = speedData[nTimer].Cn * (1 + 2.0 / (4 * speedData[nTimer].NNb - 1)) + speedData[nTimer].rest;
        speedData[nTimer].Cn = floor(speedData[nTimer].C);
        speedData[nTimer].rest = speedData[nTimer].C - speedData[nTimer].Cn;
      }
      speedData[nTimer].NNb--;
    }
    // ... FINISHED SPEED PROFILE AND NOTIFY IN MAIN LOOP...
    else if (speedData[nTimer].step_count >= speedData[nTimer].steps)
    {
      gIRQ_TC_FLAG_DONE[nTimer] = 1;
    }
  } // ---------end of if(speedData[nTimer].activated)
}
// -------------------- END OF TC0_Handler ---------------------------
*/
void TC1_Handler()
{
  TC_GetStatus(TC0, 1);
  if (speedData[motorID].step_count < speedData[motorID].steps){
    posData.abs_step_pos += speedData[motorID].dir;
    ++speedData[motorID].step_count;
    writePluse(motorCh[0].pulse.pIO, motorCh[0].pulse.pin, LOW);
    writePluse(motorCh[1].pulse.pIO, motorCh[1].pulse.pin, LOW);
    writePluse(motorCh[2].pulse.pIO, motorCh[2].pulse.pin, LOW);

   // writePluse(IOMap[4].pIO, IOMap[4].pin, LOW);
  }
  stopTimer1();
}

void TC2_Handler()
{
  TC_GetStatus(TC0, 2);
  //sprintf((char *)reportSteps, "S%d", (int32_t)sendData.step_pos);
  //mkSerial.println((char *)reportSteps);
  mkSerial.print("S:");
  mkSerial.println((int32_t)posData.abs_step_pos);
}

///////////////////////////////////////////////////////////////////
void stopNow()
{
  speedData[motorID].activated=false;
  stopTimer1();
  sprintf(tmpBuffer, "R4 P%d", (int32_t)posData.abs_step_pos);
  mkSerial.println(tmpBuffer);
}
void shutdownNow()
{
  speedData[motorID].activated=false;
  stopTimer0();
  stopTimer1();
  sprintf(tmpBuffer, "R5 P%d", (int32_t)posData.abs_step_pos);
  mkSerial.println(tmpBuffer);
}
void resume()
{

}
inline void reportStatus()
{
   sprintf(tmpBuffer, "R%d M%d P%d O%d S%d H%d",posData.CMDCode, motorID, (int32_t)posData.abs_step_pos, 
       OperationMode, statusHoming[motorID], pioReadInput(motorCh[motorID].swH.pIO, motorCh[motorID].swH.pin));
      mkSerial.println(tmpBuffer); //for serial notification
}
void reportStatus(int codeValue)
{
   sprintf(tmpBuffer, "R%d M%d P%d O%d S%d H%d",codeValue,  motorID, (int32_t)posData.abs_step_pos,
       OperationMode, statusHoming[motorID], pioReadInput(motorCh[motorID].swH.pIO, motorCh[motorID].swH.pin));
      mkSerial.println(tmpBuffer); //for serial notification
}
/////////////////////////////////////////////////////////////////////
#include <stdbool.h>
//uint32_t DIST2STEP = MICROSTEPPING/X_DIST2STEP_20T5MM;
int main(void)
{
  /*
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
  init_interrupt();

  mkSerial.begin(250000);
    // mkSerial.begin(115200);
  mkSerial.println("FROM SAM3X8 CONTROLLER!");

  // TWI1 twi1 = TWI1;
  // TWI1->TWI_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;
  // TWI_ConfigureSlave(TWI1, 0x5b);
  // //status = SLAVE_IDLE;
  // TWI_EnableIt(TWI1, TWI_IER_SVACC);

  // mkSerial.println(sizeof(uint8_t));
  // mkSerial.println(sizeof(uint32_t));
  // mkSerial.println(sizeof(SENDData));
  // mkSerial.println(sizeof(SPEEDProfile));
  // I2c_Init(TWI1, false);  // TWI0 Slave
  // I2c_Interrupt(TWI1, TWI_IER_SVACC);// | TWI_IER_NACK | TWI_IER_RXRDY | TWI_IER_TXCOMP| TWI_IER_RXBUFF | TWI_IER_TXBUFE); //TWI_SR_SVREAD

  //mkI2C.write(0);

  ////////////////////////+++ SPI +++///////////////////////////////////
  // pinModeAsInput(PIOA,PIO_IDR_P26);// MOSI A26
  // pinModeAsInput(PIOA,PIO_IDR_P27);//CLK A27
  // pinModeAsOutput(PIOA,PIO_IDR_P25);// MISO A25
  //SPI_Configure(SPI0, ID_SPI0, SPI_MR_MSTR | SPI_MR_PS | SPI_MR_MODFDIS);// for master
  // SPI_Configure(SPI0, ID_SPI0,  SPI_MR_PS | SPI_MR_MODFDIS);
  // SPI_ConfigureNPCS(SPI0, 0, SPI_CSR_NCPHA);
  // SPI_Enable(SPI0);
  ////////////////////////--- SPI ---///////////////////////////////////
  // // SET IO MAP FOR MOTOR #1 ON TIMER 0
  // IOMap[TC_0][ST_PULSE].pIO = PIOB;
  // IOMap[TC_0][ST_PULSE].pin = PIO_OER_P26; // PULSE FOR STEPPER MOTOR ON PIN 22
  // IOMap[TC_0][ST_DIR].pIO = PIOA;
  // IOMap[TC_0][ST_DIR].pin = PIO_OER_P14; // DIRECTION FOR STEPPER MOTOR ON PIN 23
  // IOMap[TC_0][HOMESTOP].pIO = PIOA;
  // IOMap[TC_0][HOMESTOP].pin = PIO_OER_P15; // HOME STOP FOR STEPPER MOTOR ON PIN 24
  // IOMap[TC_0][ENDSTOP].pIO = PIOD;
  // IOMap[TC_0][ENDSTOP].pin = PIO_OER_P0; // END STOP FOR STEPPER MOTOR ON PIN 24

  // OUTPUT

  // mkSTMotorCtrl.setPulseMap( PIOC, PIO_OER_P13);
  // mkSTMotorCtrl.setDirectionMap( PIOC,PIO_OER_P12);
 // mkSerial.println("Start!");

  IOMap[ST_PULSE].pIO = PIOC;
  IOMap[ST_PULSE].pin = PIO_OER_P13; // PULSE FOR STEPPER MOTOR ON PIN 50
  IOMap[ST_DIR].pIO = PIOC;
  IOMap[ST_DIR].pin = PIO_OER_P12; // DIRECTION FOR STEPPER MOTOR ON PIN 51

  // INPUT (USING INTERRUPT)
  IOMap[HOMESTOP].pIO = PIOB;
  IOMap[HOMESTOP].pin = PIO_OER_P21; // HOME STOP FOR STEPPER MOTOR ON PIN 52
  IOMap[ENDSTOP].pIO = PIOB;
  IOMap[ENDSTOP].pin = PIO_OER_P14; // END STOP FOR STEPPER MOTOR ON PIN 53
                                    //ENABLE IO PIN AS AN OUTPUT

  for (int map = 0; map < 2; map++)
    pinModeAsOutput(IOMap[map].pIO, IOMap[map].pin);
  for (int map = 2; map < 4; map++)
    pinModeAsInput(IOMap[map].pIO, IOMap[map].pin);


  // +++ Test +++
  // int pin15pullup = 55;
  // pin15pullup = (PIOA->PIO_PUSR & PIO_OER_P15) >> 15;
  // mkSerial.print("pin A15 pullup(0): ");
  // mkSerial.println(pin15pullup);
  // +++ I2C +++ // Not so stable in noise level from motor side 
  // I'll avoid it
  // mkI2C.begin(I2C_SLAVE, 0x5b); //0x5B, 0x5C, 0x5D, 0x5E
  // mkI2C.onRequest(I2C_RequestCallback);
  // mkI2C.onReceive(I2C_ReceiveCallback);
  // mkI2C.startInterrupt(TWI_IER_SVACC);
  // -------------------------------------------

  mkCommand.setCallBackFuncGO(mkVelProfile.gen_speed_profile);
  mkCommand.setCallBackFuncGO_STEP(mkVelProfile.updateVelocityProfile);
  mkCommand.setCallBackFuncSET(mkVelProfile.modify_speed_profile);
  // mkSerial.begin(115200);
  // mkSerial.println("FROM SAM3X8 CONTROLLER!");
  /////////////// ++ Circle Motion ++ ///////////////////////
  unsigned long time1 = millis();
  float center[2] = {1.0, -0.25};
  //mkVelProfile.createLinearMotion(2000, 150, 300, 300);
  //mkVelProfile.createArcMotion(1.0 / (180), 0.05, center, -90 * DEG2RAD, 360);


  // char msg[128];
  // sprintf(msg, "totalsteps=%d, abs step sum=%d, step_sum=%d\n", totalSteps, abs_sum_steps, sum_steps);
  // mkSerial.println(msg);

  // time1 = millis() - time1;
  // mkSerial.println(time1);
  // mkSerial.println(mkVelProfile.nM);
  //setMotorDirection(mkVelProfile.motionData[0].dir);

  ///////////////////////////////////////////////////////////
  //TC0 channel 0, the IRQ0 for that channel and the desired frequency
  //startTimer(TC0, 0, TC0_IRQn);         // Handling velocity Profile ...
  startTimer0(); // Handling velocity Profile ...
  //startTimer2();// Report Steps every 100 msec...

  activatedEE = 1;
  elapsedTime[nTimer] = millis();
  ///////////////////////////////////////////////////

  gIRQ_SPI_FLAG_RX = 0;
  */
  //////////////////////////////////////////////////////
char buffer[128];

  init_controller();
  init_interrupt();
  mkSerial.begin(250000);
  
////////////////////////////////////
// +++++ First time setup for flash storage +++++

  /*
  bool isFirstTime = dueFlashStorage.write(ADDR_FLASH_INIT,1);

   mkSerial.print("isFirstTime=");
  mkSerial.println(isFirstTime);

  uint32_t *stepCnt1= new uint32_t;
  *stepCnt1=100;
  if(1) 
  {

    // uint32_t stepCnt1 = 0;
    uint8_t buff[sizeof(uint32_t)]={0};
    memcpy(buff, stepCnt1, sizeof(uint32_t));
    dueFlashStorage.write(ADDR_FLASH_STEPCNT, buff, sizeof(uint32_t));
    // write 0 to address 0 to indicate that it is not the first time running anymore
    dueFlashStorage.write(ADDR_FLASH_INIT, 0);
  }
  //else 
  {
    //////////////////////////////
    // read the flash memory
    uint8_t *pBuff = dueFlashStorage.readAddress(ADDR_FLASH_STEPCNT);
    memcpy(stepCnt1, pBuff, sizeof(uint32_t));
    stepCnt1++;

    ////////////////////////////////////
    // write it into flashmemory
    uint8_t buff[sizeof(uint32_t)]={0};
    memcpy(buff, stepCnt1, sizeof(uint32_t));
    dueFlashStorage.write(ADDR_FLASH_STEPCNT, buff, sizeof(uint32_t));

  }

  mkSerial.print("Flash: stepper count=");
  mkSerial.println(*stepCnt1);
  delete[] stepCnt1;
  */
////////////////////////////////////////////////////////

    // mkSerial.begin(115200);
  mkSerial.println("FROM SAM3X8 CONTROLLER!");
  
//////////////////////////////////////////////
// Motor Chanel #0
  motorCh[0].pulse.pIO = PIOC;
  motorCh[0].pulse.pin = PIO_OER_P13;

  motorCh[0].dir.pIO = PIOC;
  motorCh[0].dir.pin = PIO_OER_P12;

  motorCh[0].swH.pIO = PIOB;
  motorCh[0].swH.pin = PIO_OER_P21;

  motorCh[0].swE.pIO = PIOB;
  motorCh[0].swE.pin = PIO_OER_P14;

//////////////////////////////////////////////
// Motor Chanel #1
  motorCh[1].swE.pIO = PIOC;
  motorCh[1].swE.pin = PIO_OER_P14;

  motorCh[1].swH.pIO = PIOC;
  motorCh[1].swH.pin = PIO_OER_P15;

  motorCh[1].dir.pIO = PIOC;
  motorCh[1].dir.pin = PIO_OER_P16;

  motorCh[1].pulse.pIO = PIOC;
  motorCh[1].pulse.pin = PIO_OER_P17;

//////////////////////////////////////////////
// Motor Chanel #2
  motorCh[2].swE.pIO = PIOC;
  motorCh[2].swE.pin = PIO_OER_P18;

  motorCh[2].swH.pIO = PIOC;
  motorCh[2].swH.pin = PIO_OER_P19;

  motorCh[2].dir.pIO = PIOA;
  motorCh[2].dir.pin = PIO_OER_P20;

  motorCh[2].pulse.pIO = PIOA;
  motorCh[2].pulse.pin = PIO_OER_P19;

///////////////////////////////////////////////
// Define Pin as Input or Output...
  for (int ch = 0; ch < 3; ch++)
  {
    pinModeAsOutput(motorCh[ch].pulse.pIO, motorCh[ch].pulse.pin);
    pinModeAsOutput(motorCh[ch].dir.pIO, motorCh[ch].dir.pin);

    pinModeAsInput(motorCh[ch].swE.pIO, motorCh[ch].swE.pin);
    pinModeAsInput(motorCh[ch].swH.pIO, motorCh[ch].swH.pin);
  }
    

  ////////////////////////////////////////////////////

  // IOMap[ST_PULSE].pIO = PIOC;
  // IOMap[ST_PULSE].pin = PIO_OER_P13; // PULSE FOR STEPPER MOTOR ON PIN 50

  // IOMap[4].pIO = PIOC;
  // IOMap[4].pin = PIO_OER_P15; // PULSE FOR STEPPER MOTOR ON PIN 48 :duplicate for debug for pluse

  // IOMap[ST_DIR].pIO = PIOC;
  // IOMap[ST_DIR].pin = PIO_OER_P12; // DIRECTION FOR STEPPER MOTOR ON PIN 51

  // // INPUT (USING INTERRUPT)
  // IOMap[HOMESTOP].pIO = PIOB;
  // IOMap[HOMESTOP].pin = PIO_OER_P21; // HOME STOP FOR STEPPER MOTOR ON PIN 52

  // IOMap[ENDSTOP].pIO = PIOB;
  // IOMap[ENDSTOP].pin = PIO_OER_P14; // END STOP FOR STEPPER MOTOR ON PIN 53
  //                                   //ENABLE IO PIN AS AN OUTPUT


  // for (int map = 0; map < 2; map++)
  //   pinModeAsOutput(IOMap[map].pIO, IOMap[map].pin);
  // pinModeAsOutput(IOMap[4].pIO, IOMap[4].pin);
  // for (int map = 2; map < 4; map++)
  //   pinModeAsInput(IOMap[map].pIO, IOMap[map].pin);

  mkCommand.setCallBack_gen_speed_profile(mkVelProfile.gen_speed_profile);
  mkCommand.setCallBack_set_speed_profile(mkVelProfile.set_speed_profile);
  mkCommand.setCallBack_update_speed_only(mkVelProfile.update_speed_only);
   // startTimer1(100000);// for turnoff pulse 
    // startTimer0(30000); // Handling velocity Profile ...
    startTimer0(350); // Handling velocity Profile ...
    
  //startTimer2();// Report Steps every 100 msec...

  activatedEE = -1;
  elapsedTime[motorID] = millis();
  // mkSerial.begin(115200);
  // mkSerial.println("FROM SAM3X8 CONTROLLER!");
  /////////////// ++ Circle Motion ++ ///////////////////////
  ////////////////////////////////////////////
  /* Main loop */
  while (1)
  {
    if (mkCommand.buflen < (BUFSIZE - 1))
    {
      mkCommand.getCommand();
    }
    if (mkCommand.buflen)
    {
     // mkSerial.println(mkCommand.buflen);
      mkCommand.process_commands();
      mkCommand.buflen = (mkCommand.buflen - 1);
      mkCommand.bufindr = (mkCommand.bufindr + 1) % BUFSIZE;
    }

    if (gIRQ_TC_FLAG_DONE[motorID] == 1)
    {
      // NVIC_DisableIRQ(TC0_IRQn);
      
      elapsedTime[motorID] = millis() - elapsedTime[motorID];
      float current = elapsedTime[motorID] * 0.001;
      // float speed_mmPsec = speedData[nTimer].step_count*0.025/current;

      mkSerial.print("time[sec]: ");
      mkSerial.println(current);
      mkSerial.print("CNT:");
      mkSerial.println(speedData[motorID].step_count); 

      mkSerial.print("AbsPos:");
      mkSerial.println((int32_t)posData.abs_step_pos); 
      // mkSerial.println(speed_mmPsec);
      // mkSerial.println("----------------END OF MOTOR #1  -------------");
      speedData[motorID].activated = false;
      speedData[motorID].step_count = 0;
      gIRQ_TC_FLAG_DONE[motorID] = 0;
      gIRQ_I2C_FLAG_DONE = 1;
      // ...NOTIFY TO HOST...
      if (OperationMode == HOMING && statusHoming[motorID]==2)
      { // Homing is Done successfully...
        posData.CMDCode=28;
        posData.abs_step_pos=0;
        OperationMode=JOB_DONE;
        statusHoming[motorID]=3;
        reportStatus();
        statusHoming[motorID]=0;
      }
      else  reportStatus();
      // sprintf(tmpBuffer, "R%d P%d O%d S%d H%d",posData.CMDCode, OperationMode, 
      // (int32_t)posData.abs_step_pos, endstops[0], pioReadInput(IOMap[HOMESTOP].pIO, IOMap[HOMESTOP].pin));
      // mkSerial.println(tmpBuffer); //for serial notification
    // stopTimer1();
    }
    
    
    if (OperationMode == HOMING)
    {
      // +++ HIT HOME POSITION +++ //
      uint8_t bHomeSW = pioReadInput(motorCh[motorID].swH.pIO, motorCh[motorID].swH.pin); 
      if (bHomeSW)
      {
        // 1. Heading to home end stop
        if(statusHoming[motorID] == 0 ){
            statusHoming[motorID] = 1;
            
            speedData[motorID].activated = false;
            stopTimer1();
            gIRQ_TC_FLAG_DONE[motorID]=0;
           // OperationMode = JOB_DONE;
            
            activatedEE = 0;
            speedData[motorID].step_count = 0;
            
            sprintf(tmpBuffer, "R28 M%d, P%d O%d S%d H%d", motorID, posData.abs_step_pos, OperationMode, 1, bHomeSW);
            mkSerial.println(tmpBuffer);
          }
        
        // else if (endstops[0] == 1)// 2. Find Starting home position
        // {
        //    mkVelProfile.gen_speed_profile(0, 5, 20, 50, 50);
        //    endstops[0]  = 2;
        // }
      }
      else 
      {
        if(statusHoming[motorID] == 1){
            //mkSerial.println("SW 21 WORKING IN POLLING");
            //OperationMode=STOPPED;
            statusHoming[motorID]=2;
            gIRQ_TC_FLAG_DONE[motorID]=0;
            activatedEE = 0;
            speedData[motorID].activated = false;
            stopTimer1();
           // OperationMode = JOB_DONE;
            posData.abs_step_pos=0;
            //speedData[nTimer].step_count = 0;
            sprintf(tmpBuffer, "R28 M%d P%d O%d S%d H%d", motorID, posData.abs_step_pos, OperationMode, 2, bHomeSW);
            mkSerial.println(tmpBuffer);
            
          }
          
      }
    }// End of Homing process

    if (!pioReadInput(motorCh[motorID].swE.pIO, motorCh[motorID].swE.pin))
    {
      speedData[motorID].activated = false;
      stopTimer1();
      mkSerial.println("SW 14 WORKING IN POLLING");
    }
      
  }
  return 0;
}
/////////////////////////////////////////////
