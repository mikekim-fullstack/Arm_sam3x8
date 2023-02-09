/**
  mkMain.cpp
  Copyright (c) 2020 Mike Kim (ZOE Robotics).  All right reserved.
  4 Sept 2020 by Mike Kim
*/
// 
// 
//
#if 0
#include "../include/due_sam3x.init.h"
#define ARDUINO_MAIN
//#include "Arduino.h"
#include "variant.h"
#include "due_can.h"
#include "mkZoeRobotics_serial.h"

#define TEST1_CAN_COMM_MB_IDX    0
#define TEST1_CAN_TRANSFER_ID    0x01//0x07
#define TEST1_CAN0_TX_PRIO       15
#define CAN_MSG_DUMMY_DATA       0x55AAEE22

// CAN frame max data length
#define MAX_CAN_FRAME_DATA_LEN   8

// Message variable to be send
uint32_t CAN_MSG_1 = 0;

//Leave defined if you use native port, comment if using programming port
//#define mkSerial SerialUSB

// CANRaw Can0(CAN0, 255);
// CANRaw Can1(CAN1, 255);
/*
 * Cortex-M3 Systick IT handler
 */
/*
extern void SysTick_Handler( void )
{
  // Increment tick count each ms
  TimeTick_Increment() ;
}
*/

// Weak empty variant initialization function.
// May be redefined by variant files.
void initVariant() __attribute__((weak));
void initVariant() { }

static bool sel=true;
CAN_FRAME output;

void setup()
{
 // CAN_FRAME output;
  PIO_Configure(
    g_APinDescription[PINS_CAN0].pPort,
    g_APinDescription[PINS_CAN0].ulPinType,
    g_APinDescription[PINS_CAN0].ulPin,
    g_APinDescription[PINS_CAN0].ulPinConfiguration);
  PIO_Configure(
    g_APinDescription[PINS_CAN1].pPort,
    g_APinDescription[PINS_CAN1].ulPinType,
    g_APinDescription[PINS_CAN1].ulPin,
    g_APinDescription[PINS_CAN1].ulPinConfiguration);
  // start serial port at 9600 bps:
  mkSerial.begin(115200);
   // Initialize CAN0 and CAN1, baudrate is 250kb/s
  Can0.begin(CAN_BPS_500K);
  Can1.begin(CAN_BPS_500K);
  mkSerial.println("Start");
  //The default is to allow nothing through if nothing is specified
  
  //only allow this one frame ID through. 
  Can0.watchFor();
  Can1.watchFor();
}
void setZeroPos()
{
    output.data.byte[0] = 0x04;// Data Length
    output.data.byte[1] = 0x01;// Device ID(Encoder)
    output.data.byte[2] = 0x06;// CMD(1: read encoder value)
    output.data.byte[3] = 0x00;// Data
    Can1.sendFrame(output);
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


void loop() {
  // put your main code here, to run repeatedly:
  CAN_FRAME output;
  static unsigned long lastTime = 0;
  


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
int main( void )
{
	// Initialize watchdog
//	watchdogSetup();

	
  init();

	initVariant();

//	delay(1);

#if defined(USBCON)
	USBDevice.attach();
#endif

	setup();
 
	for (;;)
	{
		loop();
		// if (serialEventRun) serialEventRun();
	}

	return 0;
}
#endif

#include "../include/due_sam3x.init.h"
#include "variant.h"//
#include "pio.h"
#include "mkZoeRobotics_serial.h"

 #include "due_can.h"
// #include "mkZoeRobotics_serial.h"



#define TEST1_CAN_COMM_MB_IDX    0
#define TEST1_CAN_TRANSFER_ID    0x01
#define TEST1_CAN0_TX_PRIO       6
#define CAN_MSG_DUMMY_DATA       0x55AAEE22

// CAN frame max data length
#define MAX_CAN_FRAME_DATA_LEN   8

// Message variable to be send
uint32_t CAN_MSG_1 = 0;
////////////////////////////
#define ADDR_FLASH_INIT 0
#define ADDR_FLASH_STEPCNT 4

#include <stdbool.h>

//////////////////////////////////////////////////////////////
char tmpBuffer[96]={0};
SERIAL_BUFFER_DATA serialSendBuf;
static bool sel=true;
CAN_FRAME output;
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

void loop() {
  // put your main code here, to run repeatedly:
  CAN_FRAME output;
  static unsigned long lastTime = 0;
  


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
int main()
{
  // init_controller();
  init();
  // CAN_FRAME output;

 init_interrupt();
  mkSerial.begin(115200);


  /*
    CAN_FRAME output;
//   CANRaw Can0(CAN0, 255);
// CANRaw Can1(CAN1, 255);
// Initialize CAN0 and CAN1, baudrate is 250kb/s
  Can0.begin(CAN_BPS_1000K);
  Can1.begin(CAN_BPS_1000K);

  //The default is to allow nothing through if nothing is specified
  
  //only allow this one frame ID through. 
  Can1.watchFor(TEST1_CAN_TRANSFER_ID);

  // Prepare transmit ID, data and data length in CAN0 mailbox 0
  output.id = TEST1_CAN_TRANSFER_ID;

 
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
  //mkSerial.println("FROM ZOEROBOTICS CONTROLLER!");
   //sprintf(tmpBuffer, "FROM ZOEROBOTICS CONTROLLER!\0");
    // serialSendBuf.write(tmpBuffer,strlen(tmpBuffer));
    //serialSendBuf.write(tmpBuffer);
  Can0.begin(CAN_BPS_500K);
  Can1.begin(CAN_BPS_500K);
  mkSerial.println("Start\0");
  //The default is to allow nothing through if nothing is specified
  
  //only allow this one frame ID through. 
  Can0.watchFor();
  Can1.watchFor();

  while(1) {

    loop();

    // if (serialSendBuf.buflen)
    // {
    //   if(mkSerial.isEmpty())
    //   {
    //     mkSerial.println(serialSendBuf.read());
    //   }
          
    // }
  }
  return 0;
}

#if 0
///////////////////////////////////
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <assert.h>
// #include "../include/due_sam3x.init.h"
#include "pio.h"
// #include "mkZoeRobotics_serial.h"
// #include "timetick.h"
// #include "Arduino.h"
// #include "variant.h"



/////////////////////////////////
// CAN TEST
#include "due_can.h"



#define TEST1_CAN_COMM_MB_IDX    0
#define TEST1_CAN_TRANSFER_ID    0x07
#define TEST1_CAN0_TX_PRIO       15
#define CAN_MSG_DUMMY_DATA       0x55AAEE22

// CAN frame max data length
#define MAX_CAN_FRAME_DATA_LEN   8

// Message variable to be send
uint32_t CAN_MSG_1 = 0;
////////////////////////////
#define ADDR_FLASH_INIT 0
#define ADDR_FLASH_STEPCNT 4

//////////////////////////////////////////////////////////////
char tmpBuffer[96]={0};
SERIAL_BUFFER_DATA serialSendBuf;
////////////////////////////////////////////////////////////////////////


// #define NODETX_CAN_ID    0x15555555
// #define NODERX_CAN_ID    0x0AAAAAAA
// #define MAX_CAN_FRAME_DATA_LEN   8

// uint32_t sentFrames, receivedFrames;

// CAN_Frame frameTX, frameRX, incoming;
int main(void)
{
//////////////////////////////////////////////////////
//char buffer[128];

  init_controller();
  init_interrupt();

  mkSerial.begin(115200);
  //mkSerial.begin(250000);
  
  
///////////////////////////////
// CAN initialization...

// typedef struct {
// 	uint32_t ul_mb_idx;
// 	uint8_t uc_obj_type;  //! Mailbox object type, one of the six different objects.
// 	uint8_t uc_id_ver;    //! 0 stands for standard frame, 1 stands for extended frame.
// 	uint8_t uc_length;    //! Received data length or transmitted data length.
// 	uint8_t uc_tx_prio;   //! Mailbox priority, no effect in receive mode.
// 	uint32_t ul_status;   //! Mailbox status register value.
// 	uint32_t ul_id_msk;   //! No effect in transmit mode.
// 	uint32_t ul_id;       //! Received frame ID or the frame ID to be transmitted.
// 	uint32_t ul_fid;      //! Family ID.
// 	uint32_t ul_datal;
// 	uint32_t ul_datah;
// } can_mb_conf_t;
/*
can_mb_conf_t can0_mailbox;
can_mb_conf_t can1_mailbox;

PIO_Configure(PIOA, PIO_PERIPH_A, PIO_PA1A_CANRX0 | PIO_PA0A_CANTX0, PIO_DEFAULT);
PIO_Configure(PIOB, PIO_PERIPH_A, PIO_PB15A_CANRX1 | PIO_PB14A_CANTX1, PIO_DEFAULT);

pmc_enable_periph_clk(ID_CAN0);
pmc_enable_periph_clk(ID_CAN1);
int res0 = can_init(CAN0, SystemCoreClock, CAN_BPS_500K);
int res1 = can_init(CAN1, SystemCoreClock, CAN_BPS_500K);
mkSerial.println(res0);
mkSerial.println(res1);
mkSerial.println(SystemCoreClock);
can_reset_all_mailbox(CAN0);
can_reset_all_mailbox(CAN1);
 
can1_mailbox.ul_mb_idx = 0;
can1_mailbox.uc_obj_type = CAN_MB_RX_MODE;
can1_mailbox.ul_id_msk = CAN_MAM_MIDvA_Msk | CAN_MAM_MIDvB_Msk;
can1_mailbox.ul_id = CAN_MID_MIDvA(0x07);
can_mailbox_init(CAN1, &can1_mailbox);
 
can0_mailbox.ul_mb_idx = 0;
can0_mailbox.uc_obj_type = CAN_MB_TX_MODE;
can0_mailbox.uc_tx_prio = 15;
can0_mailbox.uc_id_ver = 0;//! 0 stands for standard frame, 1 stands for extended frame.
can0_mailbox.ul_id_msk = 0;
can_mailbox_init(CAN0, &can0_mailbox);
 
can0_mailbox.ul_id = CAN_MID_MIDvA(0x07);
can0_mailbox.ul_datal = 0x0A0B0C0D;
can0_mailbox.ul_datah = 0x0A0B0C0D;
can0_mailbox.uc_length = 8;
*/
// CAN frame max data length
/*
CAN_SAM3X CAN_0(0);
CAN_SAM3X CAN_1(1);
CAN_0.begin(CAN_BPS_500K);
CAN_1.begin(CAN_BPS_500K);

sentFrames = 1;
receivedFrames = 1;

//Initialize the definitions for the frames we'll be sending.
//This can be done here because the frame never changes
frameTX.id = NODETX_CAN_ID;
frameTX.length = MAX_CAN_FRAME_DATA_LEN;
frameTX.extended = 0;
frameTX.rtr = 0;

frameRX.id = NODERX_CAN_ID;
frameRX.length = MAX_CAN_FRAME_DATA_LEN;
frameRX.extended = 0;
frameRX.rtr = 0;

//We are using extended frames so mark that here. Otherwise it will just use
//the first 11 bits of the ID set

// Process
frameTX.data[3] = sentFrames;
frameTX.data[2] = sentFrames >> 8;
frameTX.data[1] = sentFrames >> 16;
frameTX.data[0] = sentFrames >> 24;

CAN_0.write(frameTX);
*/
//////////////////////////////////////////////
// Motor Chanel #0  CH0=>D:C16, P:C17, S:C19




  ///////////////////////////////////////
  
  CAN_FRAME output;
//   CANRaw Can0(CAN0, 255);
// CANRaw Can1(CAN1, 255);
// Initialize CAN0 and CAN1, baudrate is 250kb/s
  Can0.begin(CAN_BPS_1000K);
  Can1.begin(CAN_BPS_1000K);

  //The default is to allow nothing through if nothing is specified
  
  //only allow this one frame ID through. 
  Can1.watchFor(TEST1_CAN_TRANSFER_ID);

  // Prepare transmit ID, data and data length in CAN0 mailbox 0
  output.id = TEST1_CAN_TRANSFER_ID;

 
  output.data.byte[0] = 0xED;
  output.data.byte[1] = 0xFE;
  output.data.byte[2] = 0xDD;
  output.data.byte[3] = 0x55;

  output.data.byte[4] = 0xEF;
  output.data.byte[5] = 0xBE;
  output.data.byte[6] = 0xAD;
  output.data.byte[7] = 0xDE;

  Can0.sendFrame(output);
  
  //////////////////////////////
  serialSendBuf.write("FROM ZOEROBOTICS CONTROLLER!");
  bool isFirst=true;
  // can_mailbox_write(CAN0, &can0_mailbox);
  
                        /* Main loop */
  ///////////////////////////////////////////////////////////////
  while (1)
  {
    // if (CAN_1.available())
    // {
    //   frameRX = CAN_1.read();
    //   receivedFrames++;
    //   serialSendBuf.write("I Got it");
    //   // Process

    //   //CAN_0.write(frameTX);
    // }
    ////////////////////////////////////////
    // 1. Processing Message Packet from PC...
    // if (mkCommand.buflen < (BUFSIZE - 1))
    // {
    //   mkCommand.getCommand();
    // }
    // if (mkCommand.buflen)
    // {
    //   mkCommand.process_commands();
    //   mkCommand.buflen = (mkCommand.buflen - 1);
    //   mkCommand.bufindr = (mkCommand.bufindr + 1) % BUFSIZE;
    // }

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
    // if (posData[motorID].OperationMode == HOMING)
    // {
    //   // +++ HIT HOME POSITION +++ //
    //  bool bHomeSW = getHomeSWStatus();
    //  if(bHomeSW) ++sHomeSW;// Ignore false triger from electrical surges...
    //  else sHomeSW=0;
 
    //   // When the homeS/W is contacted...
    //   if (bHomeSW && sHomeSW>5 && statusHoming[motorID] == 0 )
    //   {
    //     sHomeSW=0;
    //     // A. When the robot approched to the homeS/W and contacted,
    //     //    stop the robot and report the status and then wait for next action 
    //     //if(posData[motorID].jobID==(SC_HOMING+2) ){
    //       if(jobStatus.nSequence==2) {
    //         statusHoming[motorID] = 1;
    //         speedData[motorID].activated = false;// stop motor...
    //         speedData[motorID].step_count = 0;
    //         reportStatus();
    //      }
    //       // B. When the homeSW is already contacted at the begining and moving away from homeSW
    //       //    Stop the robot and report and then wait for next action...
    //       //else if(posData[motorID].jobID==(SC_HOMING+3)){
    //       else if(jobStatus.nSequence==3){
    //         statusHoming[motorID]=1;
    //       }
    //   }
    //   else if (statusHoming[motorID] == 1 && bHomeSW==false )
    //   {
    //         statusHoming[motorID]=2;
    //         //gIRQ_TC_FLAG_DONE[motorID]=0;
    //         speedData[motorID].activated = false;// stop motor...
    //         posData[motorID].abs_step_pos=0;
    //         reportStatus();
    //   }
    // }// End of Homing process
    ///////////////////////////////////////////////////
    //if(isAnyMotion>0) reportAllPosStatus(RC_STATUS_MOTION,0,0);


      // uint32_t ul_status;
      // can_global_send_transfer_cmd(CAN0, CAN_TCR_MB0);
        
      //   // if ((can_mailbox_get_status(CAN1, 0) & CAN_MSR_MRDY)) 
      //    {

      //        can1_mailbox.ul_mb_idx = 0;
      //        can1_mailbox.ul_status = ul_status;
      //        can_mailbox_read(CAN1, &can1_mailbox);
      //        //g_ul_recv_status = 1;
         
      //     isFirst = false;
      //     //can_mailbox_read(CAN1, &can1_mailbox);
      //     sprintf(tmpBuffer, "CAN received=L:%X, H=%X",can1_mailbox.ul_datal, can1_mailbox.ul_datah);
      //     serialSendBuf.write(tmpBuffer);
      //   }

    ///////////////////////////////////////////////////
    // 4. Processing sending message packets back to PC 
    
    if (serialSendBuf.buflen)
    {
      if(mkSerial.isEmpty())
      {
        mkSerial.println(serialSendBuf.read());
      }
          
    }// -------------- End of ALL Processes-------------
    
  
   //if(Can1.available()!=0) 
   {
      CAN_FRAME incoming;
      Can1.read(incoming);
      
      // sprintf(tmpBuffer, "CAN received=L:%X, H=%X",incoming.data.low, incoming.data.high);
      //     serialSendBuf.write(tmpBuffer);
        // Disable CAN0 Controller
//      Can0.disable();
//    
//      // Disable CAN1 Controller
//      Can1.disable();

    }
    ////////////////////////////////////////////////////////
    //if(  isFirst) 
    // {
    //   uint32_t ul_status;
    //   can_global_send_transfer_cmd(CAN0, CAN_TCR_MB0);
    //      ul_status = can_mailbox_get_status(CAN1, 0);
    //      if ((can_mailbox_get_status(CAN1, 0) & CAN_MSR_MRDY)) {
    //          can1_mailbox.ul_mb_idx = 0;
    //          can1_mailbox.ul_status = ul_status;
    //          can_mailbox_read(CAN1, &can1_mailbox);
    //          //g_ul_recv_status = 1;
         
    //       isFirst = false;
    //       //can_mailbox_read(CAN1, &can1_mailbox);
    //       sprintf(tmpBuffer, "CAN received=L:%X, H=%X",can1_mailbox.ul_datal, can1_mailbox.ul_datah);
    //       serialSendBuf.write(tmpBuffer);
    //     }
      // CAN_FRAME incoming;
      // Can1.read(incoming);
      
      
      // sprintf(tmpBuffer, "CAN received=L:%X, H=%X",incoming.data.low, incoming.data.high);
      // serialSendBuf.write(tmpBuffer);
      // Can0.disable();
      // Can1.disable();

 
    //}
 }// ---------- End of While(1) --------------
  return 0;
}
/////////////////////////////////////////////
#endif