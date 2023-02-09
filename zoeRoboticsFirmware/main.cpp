/**
mkMain.cpp
Copyright (c) 2020 Mike Kim (ZOE Robotics).  All right reserved.
4 Sept 2020 by Mike Kim
*/
#include <stdio.h>
#include <math.h>
#include <string.h>
// #include <iostream>
// #include <sstream>
#include <assert.h>
#include "mkZoeRobotics_define.h"
#include "../include/due_sam3x.init.h"

#include "timetick.h"

#include "mkZoeRobotics_globalDataStruct.h"
#include "mkZoeRobotics_serial.h"
#include "mkZoeRobotics_command.h"
#include "mkZoeRobotics_velProfile.h"


// #include "twi.h"
// #include "mkZoeRobotics_spi.h"
// #include "mkZoeRobotics_i2c.h"

// #include "DueFlashStorage.h"


/////////////////////////////////
#include "mkCANEncoder.h"
//#include "can.h"



////////////////////////////
#define ADDR_FLASH_INIT 0
#define ADDR_FLASH_STEPCNT 4

//////////////////////////////////////////////////////////////
volatile static uint16_t gIRQ_SPI_FLAG_RX = 0;
volatile static uint16_t gIRQ_I2C_FLAG_DONE = 0, gIRQ_I2C_FLAG_STARTED = 0, gIRQ_I2C_FLAG_CANCELED = 0;
static uint8_t spi_databuf[24];
volatile static int spi_cnt = 0;
volatile int8_t isAnyMotion=0;
//uint8_t numJobSequence=0;
JOBSTATUS jobStatus;

//static uint8_t DataReceived = 0;
//volatile uint32_t step_absPos=0;// absolute step counter to track current position
//volatile int8_t activatedEE = -1;
//static CIRCLEProfile revDataCircle; // from I2C input
//static SPEEDProfile revData;        // from I2C input
POSData posData[MAX_MOTOR_NUM];// [X, R1, R2, Z]
SPEEDRampData speedData[MAX_MOTOR_NUM];
KIN_DATA kinData[MAX_MOTOR_NUM];
MOTORCHANEL motorCh[MAX_MOTOR_NUM];
struct GLOBAL_FLAGS g_status;
unsigned long elapsedTime[MAX_MOTOR_NUM] = {0};
volatile uint32_t gIRQ_step_count[MAX_MOTOR_NUM] = {0};
//volatile uint16_t gIRQ_TC_FLAG_DONE[MAX_MOTOR_NUM] = {0};
int motorID = 0;

//MKStepperMotorCtrl mkSTMotorCtrl;
extern MKSerial mkSerial;
extern MKCommand mkCommand;
extern   mkCANEncoder mkCAN;

// extern MKSPI mkSPI;
// extern MKI2C mkI2C1;
// OP_MODE OperationMode;
volatile uint8_t statusHomeSW[MAX_MOTOR_NUM] = {0};
volatile uint8_t statusHoming[MAX_MOTOR_NUM] = {0};
volatile char reportSteps[128];

volatile char activateTimer1=0;
//char tmpBuffer[96]={0};
SERIAL_BUFFER_DATA serialSendBuf;
int ich=0;
extern MKVelProfile mkVelProfile;
static portIOPair SSR_POWER;
static portIOPair SSR_Z_BRAKE;
static portIOPair SSR_CUP_PWR;
static portIOPair SW_CUP_CYCLE;
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
void reportACK(int codeValue, int mID, int errorCode=0);
void reportStatus();
void reportStatus(int codeValue, int motorID);
void reportAllPosStatus(int respCode, int codeValue);
void reportGenKinDataStatus(int RC, int codeValue, int rev);
void startTimer(int n, int prescale, uint32_t frequency=350);
void stopTimer(int n);
void stopMotionAll();
void stopMotion(int id);
void rebootTimers();
void resetElapsedTime();
void processFinishingMove(int nTimer);
void controlPowerLine(bool bPowerOn);
void controlZBrake(bool bBrakeOn);
void controlDropCup(int delayTime);
inline static void writePluse(Pio *portIO, uint32_t pin, uint32_t value);
void delay( unsigned long ms );

static volatile bool bCupDropSignal=false;
static int cupSWDelayTime=25;
void controlDropCup(int delayTime) {
  cupSWDelayTime= delayTime;
  writePluse(SSR_CUP_PWR.pIO, SSR_CUP_PWR.pin, LOW);// ON
  bCupDropSignal=true;
}
void delay( unsigned long ms )
{
    if (ms == 0)
        return;
    uint32_t start = GetTickCount();
    do {
        yield();
    } while (GetTickCount() - start < ms);
}
void resetElapsedTime()
{
    for(int i=0; i<4; i++)
    {
        elapsedTime[i] = millis();
    }
}
void resetElapsedTime(int ch)
{
    elapsedTime[ch] = millis();
}
inline static bool pioReadInput(Pio *pPio, uint32_t pin, bool bPullUp=true);
void homeMode()
{
}
////////////////////////////////////////////////////////
// Handling Velocity Profile
void init_interrupt()
{
    pmc_enable_periph_clk(ID_PIOA);
    NVIC_DisableIRQ(PIOA_IRQn);
    // NVIC_ClearPendingIRQ(PIOA_IRQn);
    // NVIC_SetPriority(PIOA_IRQn, 0);
    // NVIC_EnableIRQ(PIOA_IRQn);
    
    pmc_enable_periph_clk(ID_PIOB);
    NVIC_DisableIRQ(PIOB_IRQn);
    // NVIC_ClearPendingIRQ(PIOB_IRQn);
    // NVIC_SetPriority(PIOB_IRQn, 0);
    // NVIC_EnableIRQ(PIOB_IRQn);
    
    pmc_enable_periph_clk(ID_PIOC);
    NVIC_DisableIRQ(PIOC_IRQn);
    // NVIC_ClearPendingIRQ(PIOC_IRQn);
    // NVIC_SetPriority(PIOC_IRQn, 0);
    // NVIC_EnableIRQ(PIOC_IRQn);
    
    pmc_enable_periph_clk(ID_PIOD);
    NVIC_DisableIRQ(PIOD_IRQn);
    // NVIC_ClearPendingIRQ(PIOD_IRQn);
    // NVIC_SetPriority(PIOD_IRQn, 0);
    // NVIC_EnableIRQ(PIOD_IRQn);
}
void startTimer(int n, int prescale, uint32_t frequency)
{
    
    Tc *tc = NULL;
    uint32_t channel = 0;
    IRQn_Type irq = TC0_IRQn;
    stopTimer(n);
    // speedData[n].reset();
    // posData[n].reset();
    switch(n)
    {
    case 0:
        tc=TC0; channel=0; irq = TC0_IRQn;
        break;
    case 1:
        tc=TC0; channel=1; irq = TC1_IRQn;
        break;
    case 2:
        tc=TC0; channel=2; irq = TC2_IRQn;
        break;
    case 3:
        tc=TC1; channel=0; irq = TC3_IRQn;
        break;
    case 4:
        tc=TC1; channel=1; irq = TC4_IRQn;
        break;
    case 5:
        tc=TC1; channel=2; irq = TC5_IRQn;
        break;
    case 6:
        tc=TC2; channel=0; irq = TC6_IRQn;
        break;
    case 7:
        tc=TC2; channel=1; irq = TC7_IRQn;
        break;
    default:
        return;
    }
    pmc_set_writeprotect(false);
    pmc_enable_periph_clk((uint32_t)irq);
    //TC_Configure(tc, channel,  TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK2 | TC_CMR_ACPC_CLEAR | TC_CMR_ASWTRG_CLEAR);//  | TC_CMR_CPCSTOP);
    if(prescale==2)
        TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK1);
    else if(prescale==8)
        TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK2);
    else if(prescale==32)
        TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK3);
    else if(prescale==128)
        TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);
    
    uint32_t rc = F_CPU / prescale / frequency; //2 because we selected TIMER_CLOCK1 above
    //TC_SetRA(tc, channel, rc);
    TC_SetRC(tc, channel, rc);
    
    tc->TC_CHANNEL[channel].TC_IER = TC_IER_CPCS;
    tc->TC_CHANNEL[channel].TC_IDR = ~TC_IER_CPCS;
    NVIC_ClearPendingIRQ(irq);
    NVIC_SetPriority(irq, 2);
    NVIC_EnableIRQ(irq);
    TC_Start(tc, channel);
} 

void stopTimer(int n)
{
    Tc *tc = TC0;
    uint32_t channel = 0;
    IRQn_Type irq = TC0_IRQn;
    switch(n)
    {
    case 0:
        tc=TC0; channel=0; irq = TC0_IRQn;
        break;
    case 1:
        tc=TC0; channel=1; irq = TC1_IRQn;
        break;
    case 2:
        tc=TC0; channel=2; irq = TC2_IRQn;
        break;
    case 3:
        tc=TC1; channel=0; irq = TC3_IRQn;
        break;
    case 4:
        tc=TC1; channel=1; irq = TC4_IRQn;
        break;
    case 5:
        tc=TC1; channel=2; irq = TC5_IRQn;
        break;
    case 6:
        tc=TC2; channel=0; irq = TC6_IRQn;
        break;
    case 7:
        tc=TC2; channel=1; irq = TC7_IRQn;
        break;
    default:
        return;
    }
    NVIC_DisableIRQ(irq);
    TC_Stop(tc, channel);
    // if(n>=0 && n<4) reportStatus(n);
    
}
void rebootTimers()
{
    for(int i=0; i<4; i++){
        speedData[i].reset();
        posData[i].reset();
    }
    startTimer(0,TICK_PRESCALE,340); //Z
    startTimer(1,TICK_PRESCALE,340); //X
    startTimer(2,TICK_PRESCALE,340); //Q
    startTimer(3,TICK_PRESCALE,340); //R
    //startTimer(4,2,30000); //pulse
    // startTimer(4,2,100000); //pulse
}
void controlPowerLine(bool bPowerOn)
{
    if(bPowerOn){
        writePluse(SSR_POWER.pIO, SSR_POWER.pin, HIGH);
        Sleep(1000);
        writePluse(SSR_Z_BRAKE.pIO, SSR_Z_BRAKE.pin, HIGH);
        reportACK(SC_POWER,0);
    }
    else {
        stopMotionAll();
        writePluse(SSR_Z_BRAKE.pIO, SSR_Z_BRAKE.pin, LOW);
        //reportACK(SC_POWER,0);
        Sleep(1500);
        writePluse(SSR_POWER.pIO, SSR_POWER.pin, LOW);
        // writePluse(SSR_POWER.pIO, SSR_POWER.pin, LOW);
        // reportACK(SC_Z_BRAKE,0);
        // Sleep(100);
        
    }
    
}
void controlZBrake(bool bBrakeOn) 
{
    if(bBrakeOn) writePluse(SSR_Z_BRAKE.pIO, SSR_Z_BRAKE.pin, HIGH);
    else writePluse(SSR_Z_BRAKE.pIO, SSR_Z_BRAKE.pin, LOW);
}
void stopMotionAll()
{
    for(int i=0; i<MAX_MOTOR_NUM; i++) 
    {
        stopTimer(i);
        speedData[i].reset();
        kinData[i].reset();
    }
}
void stopMotion(int id)
{
    speedData[id].reset();
    kinData[id].reset();
}
/////////////////////////////////////////////
//  void stopTimer0()
// {
//   Tc *tc = TC0;
//   uint32_t channel = 0;
//   IRQn_Type irq = TC0_IRQn;
//   NVIC_DisableIRQ(irq);
//   TC_Stop(tc, channel);
// }
//  void stopTimer1()
// {
//   Tc *tc = TC0;
//   uint32_t channel = 1;
//   IRQn_Type irq = TC1_IRQn;
//   NVIC_DisableIRQ(irq);
//   TC_Stop(tc, channel);
// }
//  void stopTimer2()
// {
//   Tc *tc = TC0;
//   uint32_t channel = 2;
//   IRQn_Type irq = TC2_IRQn;
//   NVIC_DisableIRQ(irq);
//   TC_Stop(tc, channel);
// }
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
void stopTimer3()
{
    Tc *tc = TC1;
    uint32_t channel = 0;
    IRQn_Type irq = TC3_IRQn;
    NVIC_DisableIRQ(irq);
    TC_Stop(tc, channel);
}
void stopTimer4()
{
    Tc *tc = TC1;
    uint32_t channel = 1;
    IRQn_Type irq = TC4_IRQn;
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
inline static void pinModeAsInput(Pio *portIO, uint32_t pin, bool selPULLUP=true)
{
    
    portIO->PIO_PER = pin;  // PIO ENABLE
    portIO->PIO_ODR = pin;  // OUTPUT DISABLE (INPUT ENABLED)
    portIO->PIO_IDR = pin;  // disable interrupt
    portIO->PIO_PUDR = pin; // PULL-UP  DISABLE
    if(selPULLUP) portIO->PIO_PUER = pin; // PULL-UP  ENABLE
    //portIO->PIO_IFER = pin; // INPUTFILTER ENABLE
    portIO->PIO_IFDR = pin;
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
uint8_t getHomeSWStatus(int ch)
{
    if(ch<0) return 0;
    return pioReadInput(motorCh[ch].swH.pIO, motorCh[ch].swH.pin);
}
inline static bool pioReadInput(Pio *pPio, uint32_t pin, bool bPullUp)
{
    bool level=0;
    
    if ((pPio->PIO_PDSR & pin) == 0)
        level = 0 ;
    else
        level= 1;
    if(bPullUp) level = !level;
    return level;
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
volatile static int nP[4] = {0}, np=0;
volatile static uint32_t totalsteps[4] = {0}, plusWidth[4]={0};
//volatile static bool pulseTick[4]={false};

void TC0_Handler_test()
{
    TC_GetStatus(TC0, 0);
    
    if(nP[0]==0){
        //if(totalsteps==speedData[motorID].Cn)
        
        writePluse(motorCh[0].pulse.pIO, motorCh[0].pulse.pin, HIGH);
        writePluse(motorCh[1].pulse.pIO, motorCh[1].pulse.pin, HIGH);
        writePluse(motorCh[2].pulse.pIO, motorCh[2].pulse.pin, HIGH);
        nP[0]=1;
        //serialSendBuf.write("TC0_Handler");
        // totalsteps[0]=0;
        // plusWidth[0]=1;
        
        /////////////////////////////////////////////////////////
        
        // speedData[motorID].C = speedData[motorID].Cn * (1 - 2.0 / (4 * speedData[motorID].step_count + 1)) + speedData[motorID].rest;
        //   speedData[motorID].Cn = floor(speedData[motorID].C);
        //   speedData[motorID].rest = speedData[motorID].C - speedData[motorID].Cn;
        //////////////////////////////////////////////////////
        
        
    }
    else if(nP[0]==1 ) {
        // posData[motorID].abs_step_pos += speedData[motorID].dir;
        // ++speedData[motorID].step_count;
        writePluse(motorCh[0].pulse.pIO, motorCh[0].pulse.pin,  LOW);
        writePluse(motorCh[1].pulse.pIO, motorCh[1].pulse.pin, LOW);
        writePluse(motorCh[2].pulse.pIO, motorCh[2].pulse.pin, LOW);
        nP[0]=0;
        // plusWidth[0]=0;
    }
    //  totalsteps[0]++;
    // plusWidth[0]++;
}
/////////////////////////////////////////
void TC0_Handler()// 0: X-Axis
{
    ////////////////////////////////////////////////
    TC_GetStatus(TC0, 0);
    // processFinishingMove(0);
    // return;//test
    int mID=0;
    
    /////////////////////////////////////////////////////////////
    // ** Kinematic Cartesian Motion ** //
    if (kinData[mID].activated){
        if(kinData[mID].getMotionCn()>0) {
            TC_SetRC(TC0, 0, kinData[mID].getMotionCn());
        }
        else {
            kinData[mID].step_count = 0;
            kinData[mID].nextMotionData();
            if(kinData[mID].isMotionDone())
            {
                kinData[mID].motionDone();
                processFinishingMove(mID);
            }
            return;
        }
        //////////////////////////////////////
        // -- When the step is zero, only takes time by Cn...
        if(kinData[mID].getMotionSteps()==0) {
            kinData[mID].nextMotionData();
        }
        // -- When the step exits generate pulse for a motor...
        else  {
            kinData[mID].pulseTick=true;
            setMotorDirection(mID, kinData[mID].getMotionDir());
            writePluse(motorCh[mID].pulse.pIO, motorCh[mID].pulse.pin, HIGH);
            kinData[mID].step_count++;// should uncomment
            startTimer(mID+4,TICK_PRESCALE,100000);
            ////////////////////////////////////////////////////////////
            if(kinData[mID].step_count == kinData[mID].getMotionSteps())
            {
                kinData[mID].step_count = 0;
                kinData[mID].nextMotionData();
            }
        }
        // -- Check if all motion is done...
        if(kinData[mID].isMotionDone())
        {
            kinData[mID].motionDone();
            processFinishingMove(mID);
        }
    } 
    /////////////////////////////////////////////////////////////
    // ** Joint Motion ** //
    else if (speedData[mID].activated)
    {
        // ...CLEAR TC STATUS REGISTER...
        TC_SetRC(TC0, 0, speedData[mID].Cn);
        
        if(speedData[mID].step_count==speedData[mID].totalSteps) {
            // ... FINISHED SPEED PROFILE AND NOTIFY IN MAIN LOOP...
            speedData[mID].step_count = 0;
            processFinishingMove(mID);
        }
        else {
            speedData[mID].pulseTick=true;
            setMotorDirection(mID, speedData[mID].dir);
            writePluse(motorCh[mID].pulse.pIO, motorCh[mID].pulse.pin, HIGH);
            startTimer(mID+4,TICK_PRESCALE,100000);
            if (speedData[mID].step_count < speedData[mID].Na)
            {
                if (speedData[mID].step_count == 0)
                {
                    speedData[mID].Cn = speedData[mID].Cn_acc0;
                    speedData[mID].rest = 0;
                }
                else
                {
                    //%%% SQRT approximation of speedData[num].Cn = speedData[num].Cn_acc0*(sqrt(step_count+1)-sqrt(step_count));
                    //speedData[mID].Cn = speedData[mID].Cn_acc0*(sqrt(speedData[mID].step_count+1)-sqrt(speedData[mID].step_count));
                    speedData[mID].C = speedData[mID].Cn * (1 - 2.0 / float(4 * speedData[mID].step_count + 1)) + speedData[mID].rest;
                    speedData[mID].Cn = floor(speedData[mID].C+0.5);
                    speedData[mID].rest = speedData[mID].C - speedData[mID].Cn;
                }
            }
            // ... DECELERATION AREA ...
            else if (speedData[mID].step_count >= speedData[mID].Nac && speedData[mID].step_count < speedData[mID].totalSteps)
            {
                if (speedData[mID].step_count == speedData[mID].Nac)
                {
                    speedData[mID].rest = 0;
                }
                else
                {
                    // speedData[mID].Cn = speedData[mID].Cn_acc0*(sqrt(speedData[mID].step_count+1)-sqrt(speedData[mID].step_count));
                    speedData[mID].C = speedData[mID].Cn * float(1 + 2.0 / (4 * speedData[mID].NNb - 1)) + speedData[mID].rest;
                    speedData[mID].Cn = floor(speedData[mID].C+0.5);
                    speedData[mID].rest = speedData[mID].C - speedData[mID].Cn;
                }
                speedData[mID].NNb--;
            }
            speedData[mID].step_count++;
        }////////////////////////////////////////////
    } // ---------end of if(speedData[nTimer].activated)
    
}

void TC1_Handler()// 1: R1 revolute Joint-Axis
{
    TC_GetStatus(TC0, 1);
    int mID=1;
    /////////////////////////////////////////////////////////////
    // ** Kinematic Cartesian Motion ** //
    if (kinData[mID].activated)
    {
        if(kinData[mID].getMotionCn()>0) {
            TC_SetRC(TC0, 1, kinData[mID].getMotionCn());
        }
        else {
            kinData[mID].step_count = 0;
            kinData[mID].nextMotionData();
            if(kinData[mID].isMotionDone())
            {
                kinData[mID].motionDone();
                processFinishingMove(mID);
            }
            return;
        }
        //////////////////////////////////////
        // -- When the step is zero, only takes time by Cn...
        if(kinData[mID].getMotionSteps()==0) {
            kinData[mID].nextMotionData();
        }
        // -- When the step exits generate pulse for a motor...
        else  {
            kinData[mID].pulseTick=true;
            setMotorDirection(mID, kinData[mID].getMotionDir());
            writePluse(motorCh[mID].pulse.pIO, motorCh[mID].pulse.pin, HIGH);
            kinData[mID].step_count++;// should uncomment
            startTimer(mID+4,TICK_PRESCALE,100000);
            ////////////////////////////////////////////////////////////
            if(kinData[mID].step_count == kinData[mID].getMotionSteps())
            {
                kinData[mID].step_count = 0;
                kinData[mID].nextMotionData();
            }
        }
        // -- Check if all motion is done...
        if(kinData[mID].isMotionDone())
        {
            kinData[mID].motionDone();
            processFinishingMove(mID);
        }
    } 
    /////////////////////////////////////////////////////////////
    // ** Joint Motion ** //
    else if (speedData[mID].activated)
    {
        // ...CLEAR TC STATUS REGISTER...
        TC_SetRC(TC0, 1, speedData[mID].Cn);
        if(speedData[mID].step_count==speedData[mID].totalSteps) {
            // ... FINISHED SPEED PROFILE AND NOTIFY IN MAIN LOOP...
            speedData[mID].step_count = 0;
            processFinishingMove(mID);
        }
        else {
            speedData[mID].pulseTick=true;
            setMotorDirection(mID, speedData[mID].dir);
            writePluse(motorCh[mID].pulse.pIO, motorCh[mID].pulse.pin, HIGH);
            startTimer(mID+4,TICK_PRESCALE,100000);
            if (speedData[mID].step_count < speedData[mID].Na)
            {
                if (speedData[mID].step_count == 0)
                {
                    speedData[mID].Cn = speedData[mID].Cn_acc0;
                    speedData[mID].rest = 0;
                }
                else
                {
                    //%%% SQRT approximation of speedData[num].Cn = speedData[num].Cn_acc0*(sqrt(step_count+1)-sqrt(step_count));
                    //speedData[mID].Cn = speedData[mID].Cn_acc0*(sqrt(speedData[mID].step_count+1)-sqrt(speedData[mID].step_count));
                    speedData[mID].C = speedData[mID].Cn * (1 - 2.0 / float(4 * speedData[mID].step_count + 1)) + speedData[mID].rest;
                    speedData[mID].Cn = floor(speedData[mID].C+0.5);
                    speedData[mID].rest = speedData[mID].C - speedData[mID].Cn;
                }
            }
            // ... DECELERATION AREA ...
            else if (speedData[mID].step_count >= speedData[mID].Nac && speedData[mID].step_count < speedData[mID].totalSteps)
            {
                if (speedData[mID].step_count == speedData[mID].Nac)
                {
                    speedData[mID].rest = 0;
                }
                else
                {
                    // speedData[mID].Cn = speedData[mID].Cn_acc0*(sqrt(speedData[mID].step_count+1)-sqrt(speedData[mID].step_count));
                    speedData[mID].C = speedData[mID].Cn * float(1 + 2.0 / (4 * speedData[mID].NNb - 1)) + speedData[mID].rest;
                    speedData[mID].Cn = floor(speedData[mID].C+0.5);
                    speedData[mID].rest = speedData[mID].C - speedData[mID].Cn;
                }
                speedData[mID].NNb--;
            }
            speedData[mID].step_count++;
        }////////////////////////////////////////////
    } // ---------end of if(speedData[nTimer].activated)
}

void TC2_Handler()// R2: Revolute Joint #2
{
    TC_GetStatus(TC0, 2);
    int mID=2;
    /////////////////////////////////////////////////////////////
    // ** Kinematic Cartesian Motion ** //
    if (kinData[mID].activated)
    {
        if(kinData[mID].getMotionCn()>0) {
            TC_SetRC(TC0, 2, kinData[mID].getMotionCn());
        }
        else {
            kinData[mID].step_count = 0;
            kinData[mID].nextMotionData();
            if(kinData[mID].isMotionDone())
            {
                kinData[mID].motionDone();
                processFinishingMove(mID);
            }
            return;
        }
        //////////////////////////////////////
        // -- When the step is zero, only takes time by Cn...
        if(kinData[mID].getMotionSteps()==0) {
            kinData[mID].nextMotionData();
        }
        // -- When the step exits generate pulse for a motor...
        else  {
            kinData[mID].pulseTick=true;
            setMotorDirection(mID, kinData[mID].getMotionDir());
            writePluse(motorCh[mID].pulse.pIO, motorCh[mID].pulse.pin, HIGH);
            kinData[mID].step_count++;// should uncomment
            startTimer(mID+4,TICK_PRESCALE,100000);
            ////////////////////////////////////////////////////////////
            if(kinData[mID].step_count == kinData[mID].getMotionSteps())
            {
                kinData[mID].step_count = 0;
                kinData[mID].nextMotionData();
            }
        }
        // -- Check if all motion is done...
        if(kinData[mID].isMotionDone())
        {
            kinData[mID].motionDone();
            processFinishingMove(mID);
        }
    } 
    /////////////////////////////////////////////////////////////
    // ** Joint Motion ** //
    else if (speedData[mID].activated)
    {
        // ...CLEAR TC STATUS REGISTER...
        TC_SetRC(TC0, 2 , speedData[mID].Cn);
        if(speedData[mID].step_count==speedData[mID].totalSteps) {
            // ... FINISHED SPEED PROFILE AND NOTIFY IN MAIN LOOP...
            speedData[mID].step_count = 0;
            processFinishingMove(mID);
        }
        else {
            speedData[mID].pulseTick=true;
            setMotorDirection(mID, speedData[mID].dir);
            writePluse(motorCh[mID].pulse.pIO, motorCh[mID].pulse.pin, HIGH);
            startTimer(mID+4,TICK_PRESCALE,100000);
            if (speedData[mID].step_count < speedData[mID].Na)
            {
                if (speedData[mID].step_count == 0)
                {
                    speedData[mID].Cn = speedData[mID].Cn_acc0;
                    speedData[mID].rest = 0;
                }
                else
                {
                    //%%% SQRT approximation of speedData[num].Cn = speedData[num].Cn_acc0*(sqrt(step_count+1)-sqrt(step_count));
                    //speedData[mID].Cn = speedData[mID].Cn_acc0*(sqrt(speedData[mID].step_count+1)-sqrt(speedData[mID].step_count));
                    speedData[mID].C = speedData[mID].Cn * (1 - 2.0 / float(4 * speedData[mID].step_count + 1)) + speedData[mID].rest;
                    speedData[mID].Cn = floor(speedData[mID].C+0.5);
                    speedData[mID].rest = speedData[mID].C - speedData[mID].Cn;
                }
            }
            // ... DECELERATION AREA ...
            else if (speedData[mID].step_count >= speedData[mID].Nac && speedData[mID].step_count < speedData[mID].totalSteps)
            {
                if (speedData[mID].step_count == speedData[mID].Nac)
                {
                    speedData[mID].rest = 0;
                }
                else
                {
                    // speedData[mID].Cn = speedData[mID].Cn_acc0*(sqrt(speedData[mID].step_count+1)-sqrt(speedData[mID].step_count));
                    speedData[mID].C = speedData[mID].Cn * float(1 + 2.0 / (4 * speedData[mID].NNb - 1)) + speedData[mID].rest;
                    speedData[mID].Cn = floor(speedData[mID].C+0.5);
                    speedData[mID].rest = speedData[mID].C - speedData[mID].Cn;
                }
                speedData[mID].NNb--;
            }
            speedData[mID].step_count++;
        }////////////////////////////////////////////
    }// ---------end of if(speedData[nTimer].activated)
}

void TC3_Handler()// 3: Z-Axis 
{
    TC_GetStatus(TC1, 0);
    int mID=3;
    /////////////////////////////////////////////////////////////
    // ** Kinematic Cartesian Motion ** //
    if (kinData[mID].activated)
    {
        if(kinData[mID].getMotionCn()>0) {
            TC_SetRC(TC1, 0, kinData[mID].getMotionCn());
        }
        else {
            kinData[mID].step_count = 0;
            kinData[mID].nextMotionData();
            if(kinData[mID].isMotionDone())
            {
                kinData[mID].motionDone();
                processFinishingMove(mID);
            }
            return;
        }
        //////////////////////////////////////
        // -- When the step is zero, only takes time by Cn...
        if(kinData[mID].getMotionSteps()==0) {
            kinData[mID].nextMotionData();
        }
        // -- When the step exits generate pulse for a motor...
        else  {
            kinData[mID].pulseTick=true;
            setMotorDirection(mID, kinData[mID].getMotionDir());
            writePluse(motorCh[mID].pulse.pIO, motorCh[mID].pulse.pin, HIGH);
            kinData[mID].step_count++;// should uncomment
            startTimer(mID+4,TICK_PRESCALE,100000);
            ////////////////////////////////////////////////////////////
            if(kinData[mID].step_count == kinData[mID].getMotionSteps())
            {
                kinData[mID].step_count = 0;
                kinData[mID].nextMotionData();
            }
        }
        // -- Check if all motion is done...
        if(kinData[mID].isMotionDone())
        {
            kinData[mID].motionDone();
            processFinishingMove(mID);
        }
    } 
    /////////////////////////////////////////////////////////////
    // ** Joint Motion ** //
    else if (speedData[mID].activated)
    {
        // ...CLEAR TC STATUS REGISTER...
        TC_SetRC(TC1, 0, speedData[mID].Cn);
        if(speedData[mID].step_count==speedData[mID].totalSteps) {
            // ... FINISHED SPEED PROFILE AND NOTIFY IN MAIN LOOP...
            speedData[mID].step_count = 0;
            processFinishingMove(mID);
        }
        else {
            speedData[mID].pulseTick=true;
            setMotorDirection(mID, speedData[mID].dir);
            writePluse(motorCh[mID].pulse.pIO, motorCh[mID].pulse.pin, HIGH);
            startTimer(mID+4,TICK_PRESCALE,100000);
            if (speedData[mID].step_count < speedData[mID].Na)
            {
                if (speedData[mID].step_count == 0)
                {
                    speedData[mID].Cn = speedData[mID].Cn_acc0;
                    speedData[mID].rest = 0;
                }
                else
                {
                    //%%% SQRT approximation of speedData[num].Cn = speedData[num].Cn_acc0*(sqrt(step_count+1)-sqrt(step_count));
                    //speedData[mID].Cn = speedData[mID].Cn_acc0*(sqrt(speedData[mID].step_count+1)-sqrt(speedData[mID].step_count));
                    speedData[mID].C = speedData[mID].Cn * (1 - 2.0 / float(4 * speedData[mID].step_count + 1)) + speedData[mID].rest;
                    speedData[mID].Cn = floor(speedData[mID].C+0.5);
                    speedData[mID].rest = speedData[mID].C - speedData[mID].Cn;
                }
            }
            // ... DECELERATION AREA ...
            else if (speedData[mID].step_count >= speedData[mID].Nac && speedData[mID].step_count < speedData[mID].totalSteps)
            {
                if (speedData[mID].step_count == speedData[mID].Nac)
                {
                    speedData[mID].rest = 0;
                }
                else
                {
                    // speedData[mID].Cn = speedData[mID].Cn_acc0*(sqrt(speedData[mID].step_count+1)-sqrt(speedData[mID].step_count));
                    speedData[mID].C = speedData[mID].Cn * float(1 + 2.0 / (4 * speedData[mID].NNb - 1)) + speedData[mID].rest;
                    speedData[mID].Cn = floor(speedData[mID].C+0.5);
                    speedData[mID].rest = speedData[mID].C - speedData[mID].Cn;
                }
                speedData[mID].NNb--;
            }
            speedData[mID].step_count++;
        }////////////////////////////////////////////
    }
    
}
//volatile static bool pulseDown[4]={false};
/*
void TC4_Handler()// Pulse and direction Timer
{
TC_GetStatus(TC1, 1);
/////////////////////////////////////////////////////////////
// ** Joint Motion ** //
for(int mID=0; mID<4; mID++)
{
if(speedData[mID].activated == true)
{
if( speedData[mID].pulseTick==true){
speedData[mID].pulseTick=false;
speedData[mID].pulseDown=true;
setMotorDirection(mID, speedData[mID].dir);
writePluse(motorCh[mID].pulse.pIO, motorCh[mID].pulse.pin, HIGH);
}
else if(speedData[mID].pulseDown==true){
speedData[mID].pulseDown=false;
if (speedData[mID].step_count < speedData[mID].totalSteps){
writePluse(motorCh[mID].pulse.pIO, motorCh[mID].pulse.pin, LOW);
posData[mID].abs_step_pos += speedData[mID].dir;
++speedData[mID].step_count;// should uncomment
}
}
}
/////////////////////////////////////////////////////////////
// ** Kinematic Cartesian Motion ** //
else if(kinData[mID].activated == true) {
if( kinData[mID].pulseTick==true){
kinData[mID].pulseTick=false;
kinData[mID].pulseDown=true;
setMotorDirection(mID, kinData[mID].getMotionDir());
writePluse(motorCh[mID].pulse.pIO, motorCh[mID].pulse.pin, HIGH);
}
else if(kinData[mID].pulseDown==true){
kinData[mID].pulseDown=false;
//if (kinData[mID].step_count < kinData[mID].steps){
writePluse(motorCh[mID].pulse.pIO, motorCh[mID].pulse.pin, LOW);
posData[mID].abs_step_pos += kinData[mID].getMotionDir();
++kinData[mID].step_count;// should uncomment
//}
}
}
}

}
*/
void TC4_Handler()// Pulse and direction Timer
{
    TC_GetStatus(TC1, 1);
    int mID=0;
    if(speedData[mID].activated == true || speedData[mID].pulseTick) {
        speedData[mID].pulseTick=false;
        writePluse(motorCh[mID].pulse.pIO, motorCh[mID].pulse.pin, LOW);
        posData[mID].abs_step_pos += speedData[mID].dir;
        
    }
    else if(kinData[mID].activated == true || kinData[mID].pulseTick) {
        kinData[mID].pulseTick=false;
        writePluse(motorCh[mID].pulse.pIO, motorCh[mID].pulse.pin, LOW);
        posData[mID].abs_step_pos += kinData[mID].getMotionDir();
    }
    stopTimer(4);
}
//////////////////////////////////////////////////////////////////////////////////
void TC5_Handler()// original
{
    TC_GetStatus(TC1, 2);
    int mID=1;
    if(speedData[mID].activated == true || speedData[mID].pulseTick) {
        speedData[mID].pulseTick=false;
        writePluse(motorCh[mID].pulse.pIO, motorCh[mID].pulse.pin, LOW);
        posData[mID].abs_step_pos += speedData[mID].dir;
        
    }
    else if(kinData[mID].activated == true || kinData[mID].pulseTick) {
        kinData[mID].pulseTick=false;
        writePluse(motorCh[mID].pulse.pIO, motorCh[mID].pulse.pin, LOW);
        posData[mID].abs_step_pos += kinData[mID].getMotionDir();
    }
    stopTimer(5);
}
void TC6_Handler()// original
{
    TC_GetStatus(TC2, 0);
    int mID=2;
    if(speedData[mID].activated == true || speedData[mID].pulseTick) {
        speedData[mID].pulseTick=false;
        writePluse(motorCh[mID].pulse.pIO, motorCh[mID].pulse.pin, LOW);
        posData[mID].abs_step_pos += speedData[mID].dir;
        
    }
    else if(kinData[mID].activated == true || kinData[mID].pulseTick) {
        kinData[mID].pulseTick=false;
        writePluse(motorCh[mID].pulse.pIO, motorCh[mID].pulse.pin, LOW);
        posData[mID].abs_step_pos += kinData[mID].getMotionDir();
    }
    stopTimer(6);
}
void TC7_Handler()// original
{
    TC_GetStatus(TC2, 1);
    int mID=3;
    if(speedData[mID].activated == true || speedData[mID].pulseTick) {
        speedData[mID].pulseTick=false;
        writePluse(motorCh[mID].pulse.pIO, motorCh[mID].pulse.pin, LOW);
        posData[mID].abs_step_pos += speedData[mID].dir;
        
    }
    else if(kinData[mID].activated == true || kinData[mID].pulseTick) {
        kinData[mID].pulseTick=false;
        writePluse(motorCh[mID].pulse.pIO, motorCh[mID].pulse.pin, LOW);
        posData[mID].abs_step_pos += kinData[mID].getMotionDir();
    }
    stopTimer(7);
}

///////////////////////////////////////////////////////////////////

void resume()
{
    
} 
// void reportElapseTime(int idM)
// {
//   //uint32_t t = GetTickCount();
//  // double currTime = ((millis() - elapsedTime[idM]))*0.001f;
//   sprintf(tmpBuffer, "eTime[%d]=%d",idM, (millis() - elapsedTime[idM]));
//   serialSendBuf.write(tmpBuffer);
// }
void reportACK(int codeValue, int mID, int errorCode) 
{
    char tmpBuffer[96]={0};
    sprintf(tmpBuffer, "R%d G%d M%d J%d N%d E%d",
            RC_ACK,  codeValue, mID, jobStatus.jobID, jobStatus.nSequence, errorCode);
    serialSendBuf.write(tmpBuffer);
}
void reportStatus()
{
    char tmpBuffer[96]={0};
    sprintf(tmpBuffer, "R%d G%d M%d P%d O%d S%d H%d T%d J%d N%d",
            RC_STATUS,
            posData[motorID].CMDCode, 
            motorID, 
            (int32_t)posData[motorID].abs_step_pos, 
            posData[motorID].OperationMode, 
            statusHoming[motorID], 
            getHomeSWStatus(motorID), 
            0,//(millis() - elapsedTime[motorID]),
            jobStatus.jobID,
            jobStatus.nSequence);
    serialSendBuf.write(tmpBuffer);
}
void reportStatus(int codeValue, int mID)
{
    char tmpBuffer[96]={0};
    sprintf(tmpBuffer, "R%d G%d M%d P%d O%d S%d H%d T%d J%d N%d",
            RC_STATUS,
            codeValue,  
            mID, 
            (int32_t)posData[mID].abs_step_pos,
            posData[motorID].OperationMode, 
            statusHoming[mID], 
            getHomeSWStatus(motorID), 
            0,//(millis() - elapsedTime[mID]),
            jobStatus.jobID, jobStatus.nSequence);
    
    serialSendBuf.write(tmpBuffer);
    //mkSerial.println(tmpBuffer); //for serial notification
}
void reportEncoderValue(int codeValue, int mID, int32_t encoderValue)
{
    char tmpBuffer[96]={0};
    sprintf(tmpBuffer, "R%d G%d M%d P%d J%d N%d",
            RC_ENCODER_VALUE,
            codeValue,  
            mID, 
            encoderValue,
            jobStatus.jobID, jobStatus.nSequence);
    
    serialSendBuf.write(tmpBuffer);
    //mkSerial.println(tmpBuffer); //for serial notification
}
// * Report All current absolute position of stepper motors
void reportAllPosStatus(int respCode, int codeValue)
{
    char tmpBuffer[96]={0};
    sprintf(tmpBuffer, "R%d G%d A%d B%d C%d D%d J%d N%d",
            respCode,
            codeValue,  
            (int32_t)posData[0].abs_step_pos,
            (int32_t)posData[1].abs_step_pos,
            (int32_t)posData[2].abs_step_pos,
            (int32_t)posData[3].abs_step_pos,
            jobStatus.jobID, jobStatus.nSequence
            );
    
    serialSendBuf.write(tmpBuffer);
    //mkSerial.println(tmpBuffer); //for serial notification
}

void reportGenKinDataStatus(int RC, int codeValue, int errorCode)
{
    char tmpBuffer[96]={0};
    sprintf(tmpBuffer, "R%d G%d A%d B%d C%d D%d E%d F%d H%d J%d N%d",
            RC,
            codeValue, 
            kinData[0].dataSize, kinData[0].totalSteps,
            kinData[1].dataSize, kinData[1].totalSteps,
            kinData[2].dataSize, kinData[2].totalSteps,
            errorCode,
            jobStatus.jobID, jobStatus.nSequence
            );
    
    serialSendBuf.write(tmpBuffer);
    //mkSerial.println(tmpBuffer); //for serial notification
}
/////////////////////////////////////////////////////////////////////
void processFinishingMove(int nAxis) 
{
    speedData[nAxis].reset();
    kinData[nAxis].reset();
    
    if (posData[nAxis].OperationMode == MOVING)
    {
        //--isAnyMotion;
        posData[nAxis].OperationMode=JOB_DONE;
        reportStatus(posData[nAxis].CMDCode, nAxis);   
    } 
    else if (posData[nAxis].OperationMode == HOMING && statusHoming[nAxis]==2)
    { // Homing is Done successfully...
        posData[nAxis].CMDCode=SC_HOMING;
        posData[nAxis].abs_step_pos=0;// Set absolute position zeros(reset position)... 
        posData[nAxis].OperationMode=JOB_DONE;
        statusHoming[nAxis]=3;
        reportStatus();
        statusHoming[nAxis]=0;
        //--isAnyMotion;
    }
}
///////////////////////////////////////////////////////////////////////
/*
#define TEST1_CAN_COMM_MB_IDX    0
#define TEST1_CAN_TRANSFER_ID    0x01
#define TEST1_CAN0_TX_PRIO       15
// CAN frame max data length
#define MAX_CAN_FRAME_DATA_LEN   8

// Message variable to be send
uint32_t CAN_MSG_1 = 0;
static CAN_FRAME output;
void initCAN() 
{
Can0.begin(CAN_BPS_500K);
Can1.begin(CAN_BPS_500K);
// mkSerial.println("Start\0");
//The default is to allow nothing through if nothing is specified
Can0.watchFor();
Can1.watchFor();
}
void readEncoderData(int sel) 
{
// Prepare transmit ID, data and data length in CAN0 mailbox 0
output.id = TEST1_CAN_TRANSFER_ID;
output.length = MAX_CAN_FRAME_DATA_LEN;
output.extended = false;
output.rtr = false;



if(sel==0) {
output.data.byte[0] = 0x04;// Data Length
output.data.byte[1] = 0x01;// Device ID(Encoder)
output.data.byte[2] = 0x01;// CMD(1: read encoder value)
output.data.byte[3] = 0x00;// Data
Can0.sendFrame(output);
}
else if(sel==1) {
output.data.byte[0] = 0x04;// Data Length
output.data.byte[1] = 0x01;// Device ID(Encoder)
output.data.byte[2] = 0x01;// CMD(1: read encoder value)
output.data.byte[3] = 0x00;// Data
Can1.sendFrame(output);
}
}
void setZeroPos()
{
output.data.byte[0] = 0x04;// Data Length
output.data.byte[1] = 0x01;// Device ID(Encoder)
output.data.byte[2] = 0x06;// CMD(1: read encoder value)
output.data.byte[3] = 0x00;// Data
Can1.sendFrame(output);
}
void loopCAN() {
// put your main code here, to run repeatedly:
static unsigned long lastTime = 0;
static bool sel=true;

// if (mkSerial.available() > 0) {
//  // CAN_MSG_1 = mkSerial.parseInt();
//   if (mkSerial.read() == 's') {
//     setZeroPos();
//     mkSerial.print("Set zero position\n");
//   }
// }
if(Can0.available()!=0) {
CAN_FRAME incoming;
Can0.read(incoming);

// mkSerial.print("CAN0 message received= ");
// mkSerial.print(incoming.data.high, HEX);
// mkSerial.println(incoming.data.low, HEX);
// mkSerial.print("\nEnd of test");
}

if(Can1.available()!=0) {
CAN_FRAME incoming;
Can1.read(incoming);
int len = incoming.data.byte[0];
int dataLen=len-4;
int encoderValue=0;
for(int i=1; i<=len-3; i++){
encoderValue += (incoming.data.byte[len-i]<<(8*(dataLen+1-i)));
}


mkSerial.print("CAN1 encoderValue=");
mkSerial.println(encoderValue);
mkSerial.print("CAN1 AbsoluteAng=");
float val = encoderValue/4096.0*360.0;
mkSerial.println(val);

for(int i=0; i<len; i++) {
mkSerial.print(i);
mkSerial.print("=");
mkSerial.println(incoming.data.byte[i], HEX);
}
mkSerial.print("\nEnd of test\n");
}

if ((millis() - lastTime) > 1000) 
{
mkSerial.println(sel);
lastTime = millis();
readEncoderData(sel*0+1);
sel = !sel;    

}
}
*/
////////////////////////////////////////////////////////////////////////
#include <stdbool.h>
int main(void)
{
    //////////////////////////////////////////////////////
    //char buffer[128];
    
    init_controller();
    //init();
#if 0
    ich=2;
    CIRCLEProfile  circleProfile;
    circleProfile.speed=360.0;
    circleProfile.radius=50;
    circleProfile.cenPosX=1000;
    circleProfile.cenPosY=-250;
    circleProfile.EETheta=-90;
    circleProfile.arcAng=360;
    int rev = mkVelProfile.gen_circl_profile(circleProfile);//A
#endif
    
    init_interrupt();
    
    mkSerial.begin(115200);
    serialSendBuf.reset();
    #if 1
    // mkSerial.begin(250000);
     //   mkSerial.println("FROM ZOEROBOTICS CONTROLLER!");// test...
    rebootTimers();
    mkCAN.initCAN();
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
    
    
    //////////////////////////////////////////////
    // Motor Chanel #0  CH0=>D:C16, P:C17, S:C19
    // X-Axis
    uint8_t ch=0;
    
    motorCh[ch].dir.pIO = PIOC;
    motorCh[ch].dir.pin = PIO_OER_P16;
    
    motorCh[ch].pulse.pIO = PIOC;
    motorCh[ch].pulse.pin = PIO_OER_P17;
    
    motorCh[ch].swH.pIO = PIOC;
    motorCh[ch].swH.pin = PIO_OER_P18;
    
    //////////////////////////////////////////////
    // Motor Chanel #1 CH1=>D:C9, P:C8, S:C6
    // R1-Axis (Rotation)
    ch = 1;
    motorCh[ch].dir.pIO = PIOC;
    motorCh[ch].dir.pin = PIO_OER_P9;
    
    motorCh[ch].pulse.pIO = PIOC;
    motorCh[ch].pulse.pin = PIO_OER_P8;
    
    motorCh[ch].swH.pIO = PIOC;
    motorCh[ch].swH.pin = PIO_OER_P7;
    
    //////////////////////////////////////////////
    // Motor Chanel #2 CH2=>D:C3, P:C2, S:D10
    // R2-Axis (Rotation)
    ch = 2;
    motorCh[ch].dir.pIO = PIOC;
    motorCh[ch].dir.pin = PIO_OER_P3;
    
    motorCh[ch].pulse.pIO = PIOC;
    motorCh[ch].pulse.pin = PIO_OER_P2;
    
    motorCh[ch].swH.pIO = PIOC;
    motorCh[ch].swH.pin = PIO_OER_P1;
    
    
    //////////////////////////////////////////////
    // Motor Chanel #3 CH3=>D:D6, P:D3, S:D1
    // Z-Axis (Up-Down)...
    ch = 3;
    motorCh[ch].dir.pIO = PIOD;
    motorCh[ch].dir.pin = PIO_OER_P6;
    
    motorCh[ch].pulse.pIO = PIOD;
    motorCh[ch].pulse.pin = PIO_OER_P3;
    
    motorCh[ch].swH.pIO = PIOD;
    motorCh[ch].swH.pin = PIO_OER_P2;
    // ch = 3;
    // motorCh[ch].pulse.pIO = PIOB;
    // motorCh[ch].pulse.pin = PIO_OER_P18;
    // motorCh[ch].dir.pIO = PIOB;
    // motorCh[ch].dir.pin = PIO_OER_P19;
    // motorCh[ch].swH.pIO = PIOB;
    // motorCh[ch].swH.pin = PIO_OER_P20;
    ///////////////////////////////////////////////
    // Define Pin as Input or Output...
    for (int ch = 0; ch < MAX_MOTOR_NUM; ch++)
    {
        pinModeAsOutput(motorCh[ch].pulse.pIO, motorCh[ch].pulse.pin);
        pinModeAsOutput(motorCh[ch].dir.pIO, motorCh[ch].dir.pin);
        
        // pinModeAsInput(motorCh[ch].swE.pIO, motorCh[ch].swE.pin,PULLUP_INPUT);
        pinModeAsInput(motorCh[ch].swH.pIO, motorCh[ch].swH.pin,PULLUP_INPUT);
    }
    //////////////////////////////////
    // On/Off SSR (Soild State Relay) for Power
    // PINOUT: 13
    SSR_POWER.pIO = PIOB;
    SSR_POWER.pin = PIO_OER_P27;
    pinModeAsOutput(SSR_POWER.pIO, SSR_POWER.pin);
    writePluse(SSR_POWER.pIO, SSR_POWER.pin, LOW);// By default Power is turned off...
    //////////////////////////////////
    // ON/OFF SSR FOR Z-AXIS BRAKE (ON: BRAKE OFF, OFF: BRAKE ON)
    // // PINOUT: A8
    SSR_Z_BRAKE.pIO = PIOB;
    SSR_Z_BRAKE.pin = PIO_OER_P17;
    pinModeAsOutput(SSR_Z_BRAKE.pIO, SSR_Z_BRAKE.pin);
    writePluse(SSR_Z_BRAKE.pIO, SSR_Z_BRAKE.pin, LOW);
    
    ////////////////////////////////////////////////////
    // CupDrop Motor Power Control...
    // SSR: Normally Open
    // PINOUT: A10
    SSR_CUP_PWR.pIO = PIOB;
    SSR_CUP_PWR.pin = PIO_OER_P19;
    pinModeAsOutput(SSR_CUP_PWR.pIO, SSR_CUP_PWR.pin);
    

    ////////////////////////////////////////////////////
    // CupDrop Switch for Cup Stop Cycle...
    // PINOUT: A9
    SW_CUP_CYCLE.pIO = PIOB;
    SW_CUP_CYCLE.pin = PIO_OER_P18;
    pinModeAsInput(SW_CUP_CYCLE.pIO, SW_CUP_CYCLE.pin,PULLUP_INPUT);
    writePluse(SSR_CUP_PWR.pIO, SSR_CUP_PWR.pin, HIGH);// OFF
    ////////////////////////////////////
    mkCommand.setCallBack_gen_linear_prfile(mkVelProfile.gen_linear_profile);
    mkCommand.setCallback_gen_EErotation_profile(mkVelProfile.gen_EErotation_profile);
    mkCommand.setCallBack_gen_circle_prfile(mkVelProfile.gen_circl_profile);
    mkCommand.setCallBack_gen_spiral_prfile(mkVelProfile.gen_spiral_profile);
    mkCommand.setCallBack_gen_speed_profile(mkVelProfile.gen_speed_profile);
    mkCommand.setCallBack_set_speed_profile(mkVelProfile.set_speed_profile);
    mkCommand.setCallBack_update_speed_only(mkVelProfile.update_speed_only);
    
    
    
    #endif
    
// 
    serialSendBuf.write("FROM ZOEROBOTICS CONTROLLER!");//test
    // mkSerial.println("FROM ZOEROBOTICS CONTROLLER!");
    ////////////////////////////////////////////
    Sleep(1000);
    
    
#if 0
    char temp[128];
    sprintf(temp,"-- Last(rev=%d) Cn=%d, dataSize=%d, totalS=%d",
            rev,
            kinData[ich].motionData[kinData[ich].dataSize-1].Cn, 
            kinData[ich].dataSize, 
            kinData[ich].totalSteps
            );
    serialSendBuf.write(temp);
#endif
    int16_t sHomeSW=0;
    
    // speedData[0].activated=true;
    // kinData[mID].activated=true;
    
    
    ///////////////////////////////////////
    /*
CAN_FRAME output;
CANRaw Can0(CAN0, 255);
CANRaw Can1(CAN1, 255);
// Initialize CAN0 and CAN1, baudrate is 250kb/s
Can0.begin(CAN_BPS_500K);
Can1.begin(CAN_BPS_500K);

//The default is to allow nothing through if nothing is specified

//only allow this one frame ID through. 
Can1.watchFor(TEST1_CAN_TRANSFER_ID);

// Prepare transmit ID, data and data length in CAN0 mailbox 0
output.id = TEST1_CAN_TRANSFER_ID;
output.length = MAX_CAN_FRAME_DATA_LEN;
//Set first four bytes (32 bits) all at once
output.data.low = CAN_MSG_DUMMY_DATA;//CAN_MSG_1;
//Set last four bytes (32 bits) all at once
output.data.high = CAN_MSG_DUMMY_DATA;
//Send out the frame on whichever mailbox is free or queue it for
//sending when there is an opening.

output.data.byte[0] = 0xED;
output.data.byte[1] = 0xFE;
output.data.byte[2] = 0xDD;
output.data.byte[3] = 0x55;

output.data.byte[4] = 0xEF;
output.data.byte[5] = 0xBE;
output.data.byte[6] = 0xAD;
output.data.byte[7] = 0xDE;

Can0.sendFrame(output);
*/
    //////////////////////////////
    
    // bool isFirst=true;
    // can_mailbox_write(CAN0, &can0_mailbox);
    
    /* Main loop */
    ///////////////////////////////////////////////////////////////
    while (1)
    {
        #if 1
        // continue;
        //   if(nP[0]==0){
        
        //     writePluse(motorCh[0].pulse.pIO, motorCh[0].pulse.pin, HIGH);
        //     writePluse(motorCh[1].pulse.pIO, motorCh[1].pulse.pin, HIGH);
        //     writePluse(motorCh[2].pulse.pIO, motorCh[2].pulse.pin, HIGH);
        //     nP[0]=1;
        //     totalsteps[0]=0;
        //     plusWidth[0]=1;
        // }
        // else if(nP[0]>=10000) {
        //   // posData[motorID].abs_step_pos += speedData[motorID].dir;
        //   // ++speedData[motorID].step_count;
        //   writePluse(motorCh[0].pulse.pIO, motorCh[0].pulse.pin,  LOW);
        //   writePluse(motorCh[1].pulse.pIO, motorCh[1].pulse.pin, LOW);
        //   writePluse(motorCh[2].pulse.pIO, motorCh[2].pulse.pin, LOW);
        //   nP[0]=-1;
        // }
        // nP[0]++;
        
        ////////////////////////////////////////
        // 1. Processing Message Packet from PC...
        if (mkCommand.buflen < (BUFSIZE - 1))
        {
            mkCommand.getCommand();
        }
        if (mkCommand.buflen)
        {
            mkCommand.process_commands();
            mkCommand.buflen = (mkCommand.buflen - 1);
            mkCommand.bufindr = (mkCommand.bufindr + 1) % BUFSIZE;
        }
        //////////////////////////////////////////    
        // Read Encoder Value[degree]
        if(mkCAN.bReadSignal[0]) mkCAN.setReadEncoderData(0);// Encoder ID:0
        if(mkCAN.readEncoderData(0)) reportEncoderValue(SC_GET_ENCODER,1,mkCAN.encoderValue[0]);
        
        if(mkCAN.bReadSignal[1]) mkCAN.setReadEncoderData(1);// Encoder ID:1
        if(mkCAN.readEncoderData(1)) reportEncoderValue(SC_GET_ENCODER,2,mkCAN.encoderValue[1]);
        // mkCAN.test_loopCAN();// test
        
        ///////////////////////////////////////////////
        // 2. Processing the End of Job...
        /*
for(int index=0; index<MAX_MOTOR_NUM; index++)
{
if(gIRQ_TC_FLAG_DONE[index] == 1) {
gIRQ_TC_FLAG_DONE[index]=0;


speedData[index].reset();
if (posData[index].OperationMode == MOVING)
{
posData[index].OperationMode=JOB_DONE;
reportStatus(0, index);   
} 
else if (posData[index].OperationMode == HOMING && statusHoming[index]==2)
{ // Homing is Done successfully...
posData[index].CMDCode=SC_HOMING;
posData[index].abs_step_pos=0;
posData[index].OperationMode=JOB_DONE;
statusHoming[index]=3;
reportStatus();
posData[index].CMDCode=10;
statusHoming[index]=0;
}
}
}
*/
        ///////////////////////////////////////////////
        // 3. Processing Homing Sequences
        if (posData[motorID].OperationMode == HOMING)
        {
            // +++ HIT HOME POSITION +++ //
            bool bHomeSW = getHomeSWStatus();
            if(bHomeSW) ++sHomeSW;// Ignore false triger from electrical surges...
            else sHomeSW=0;
            
            // When the homeS/W is contacted...
            if (bHomeSW && sHomeSW>5 && statusHoming[motorID] == 0 )
            {
                sHomeSW=0;
                // A. When the robot approched to the homeS/W and contacted,
                //    stop the robot and report the status and then wait for next action 
                //if(posData[motorID].jobID==(SC_HOMING+2) ){
                if(jobStatus.nSequence==2) {
                    statusHoming[motorID] = 1;
                    speedData[motorID].activated = false;// stop motor...
                    speedData[motorID].step_count = 0;
                    reportStatus();
                }
                // B. When the homeSW is already contacted at the begining and moving away from homeSW
                //    Stop the robot and report and then wait for next action...
                //else if(posData[motorID].jobID==(SC_HOMING+3)){
                else if(jobStatus.nSequence==3){
                    statusHoming[motorID]=1;
                }
            }
            else if (statusHoming[motorID] == 1 && bHomeSW==false )
            {
                statusHoming[motorID]=2;
                //gIRQ_TC_FLAG_DONE[motorID]=0;
                speedData[motorID].activated = false;// stop motor...
                posData[motorID].abs_step_pos=0;
                reportStatus();
            }
        }// End of Homing process
        
        ///////////////////////////////////////////////////
        if(bCupDropSignal && pioReadInput(SW_CUP_CYCLE.pIO, SW_CUP_CYCLE.pin)){
          bCupDropSignal=false;
          Wait(cupSWDelayTime);// give small time to pass S/W ...
          writePluse(SSR_CUP_PWR.pIO, SSR_CUP_PWR.pin, HIGH);// OFF
          Wait(500);
          reportACK(SC_DROP_CUP, 0);
        }

        ///////////////////////////////////////////////////
        // 4. Processing sending message packets back to PC 
        if (serialSendBuf.buflen)
        {
            if(mkSerial.isEmpty())
            {
                mkSerial.println(serialSendBuf.read());
            }
            
        }// -------------- End of ALL Processes-------------
    #endif
    // mkSerial.println("FROM ZOEROBOTICS CONTROLLER!");
    // Sleep(1000);
        ////////////////////////////////////////////////////////
        
    }// ---------- End of While(1) --------------
    return 0;
}
/////////////////////////////////////////////
