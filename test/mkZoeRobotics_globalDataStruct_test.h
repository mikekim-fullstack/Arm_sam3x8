/**
  mkZoeRobotics_globalVariabls.h 
  Copyright (c) 2020 Mike Kim (ZOE Robotics).  All right reserved.
  Originated at 28 August 2020 by Mike Kim
*/
#ifndef _MKZOEROBOTICS_GLOBALVARIABLES_TEST_H_
#define _MKZOEROBOTICS_GLOBALVARIABLES_TEST_H_
// #include "../include/due_sam3x.h"
#include <stdint.h>
#include <string.h>
#include "mkZoeRobotics_define_test.h"
#ifndef BUFSIZE
  #define BUFSIZE 24
#endif
#ifndef MAX_CMD_SIZE
  #define MAX_CMD_SIZE 96
#endif
// OPERATION MODE
enum OP_MODE {ERROR=999, STOPPED=1, JOB_DONE, ASK_MOVE, MOVING, SET_SPEED,GEN_EEMOTION,GEN_CIRCLE,GEN_SPIRAL, ASK_HOME, HOMING };
enum SEND_CMD { SC_MOVE  =0, SC_SET_SPEED, SC_GEN_EEMOTION, SC_GEN_CIRCLE, SC_GEN_SPIRAL,
                SC_SETPOS=7, SC_STOP,
                SC_HOMING=9,  SC_STATUS, SC_STATUS_ALL_POS,
                SC_ASK_MOVE = 50, SC_ASK_CIRCLE, SC_ASK_HOMING,
                SC_REBOOT=100};
// enum OP_MODE { OM_MOVE  =0, OM_SET_SPEED, 
//                OM_SETPOS=3, OM_STOP, 
//                OM_HOMING=9, OM_STATUS, OM_REBOOT};
struct GLOBAL_FLAGS
{
  //! True when stepper motor is running.
  unsigned char running : 1;
  //! True when uart has received a string (ended with '/r').
  unsigned char cmd : 1;
  //! Dummy bits to fill up a byte.
  unsigned char dummy : 6;
};
// typedef struct
// {
//   Pio *pIO; // Direction Pin
//   uint32_t pin;
// } portIOPair;
// typedef struct 
// {
//   /* data */
//   portIOPair pulse;
//   portIOPair dir;
//   portIOPair swH;
//   portIOPair swE;
// }MOTORCHANEL;

typedef struct _speedRampData
{
  volatile bool activated=false;
  volatile bool pulseTick=false;
  volatile bool pulseDown=false;
  float rest=0;
  float Ta=0, Tb=0, Tc=0, Ttotal=0;
  uint32_t Na=0;             // Step count for acceleration Zone
  uint32_t Nb=0;             // Step count for deceleration Zone
  uint32_t Nc=0;             // Step count for conatant speed Zone
  uint32_t Nac=0;            //Nac=Na+Nc
  uint32_t NNb=0;            // Same as Nb for deceleration
  uint32_t Cn_const_speed=0; //time delay(OCR1A) for constant speed zone
  uint32_t Cn_acc0=0;        // time delay at the starting acceleration.
  uint32_t Cn_dec0;        // time delay at the starting deceleration.

  volatile uint32_t Cn=0;    // time delay by N step
  volatile uint32_t totalSteps=0; // total steps
  volatile uint32_t step_count=0;
  float C=0;

  //! What part of the speed ramp we are in.
  unsigned char run_state;
  //! Direction stepper motor should move.
  volatile int8_t dir=0; // 1:CCW, -1:CW
  
  void reset()
  {
    activated=false;
    pulseTick=false;
    pulseDown=false;
    rest=0;
    Ta=0, Tb=0, Tc=0, Ttotal=0;
    Na=0;             // Step count for acceleration Zone
    Nb=0;             // Step count for deceleration Zone
    Nc=0;             // Step count for conatant speed Zone
    Nac=0;            //Nac=Na+Nc
    NNb=0;            // Same as Nb for deceleration
    Cn_const_speed=0; //time delay(OCR1A) for constant speed zone
    Cn_acc0=0;        // time delay at the starting acceleration.
    Cn_dec0;        // time delay at the starting deceleration.

    Cn=0;    // time delay by N step
    totalSteps=0; // total steps
    step_count=0;
    C=0;
  }
} SPEEDRampData;

typedef struct _KINEMATICS_DATA_
{
  typedef struct
  {
    uint32_t Cn=0; // pluse frequency (one counter time)
    uint8_t steps=0; // how many pulse should be counted
    int8_t dir=0; // stepper motor direction(1: CCW, -1:CW)
  } MOTIONDATA;

  MOTIONDATA motionData[MAX_MOTIONDATA_SIZE]={};
  volatile uint16_t indexMotionData=0;
  volatile bool activated=false;
  volatile bool pulseTick=false;
  volatile bool pulseDown=false;
  volatile uint16_t step_count=0;
  
  uint16_t dataSize=0;
  uint32_t totalSteps=0;
  uint32_t getMotionCn()
  {
    return motionData[indexMotionData].Cn;
  }
  uint8_t getMotionSteps()
  {
    return motionData[indexMotionData].steps;
  }
  int8_t getMotionDir() {
    return motionData[indexMotionData].dir;
  }
  void nextMotionData()
  {
    ++indexMotionData;
  }
  bool isMotionDone()
  {
    if (indexMotionData >= dataSize) return true;
    return false;
  }
  void motionDone()
  {
    indexMotionData=0;
    activated=false;
    pulseTick=false;
    pulseDown=false;
    step_count=0;
  }
  void reset()
  {
    indexMotionData=0;
    activated=false;
    pulseTick=false;
    pulseDown=false;
    step_count=0;
    memset(motionData,0,sizeof(MOTIONDATA)*MAX_MOTIONDATA_SIZE);
    dataSize=0;
    totalSteps=0;
    
  }
} KIN_DATA;

///////////////////////////////////////////////////////////////
typedef struct _SERIAL_BUFFER_DATA
{
  // const int BUFSIZE = 24;
  // const int MAX_CMD_SIZE=96;
  char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];
  char inputString[MAX_CMD_SIZE];
  char cmdString[MAX_CMD_SIZE];
  int buflen = 0;

  uint8_t serial_char;
  int serial_count = 0;
  bool comment_mode = false;
  int bufindr = 0;
  int bufindw = 0;
  char *strchr_pointer=0; // just a pointer to find chars in the command string
  ///////////////////////
  void write()
  {
    serial_count = strlen(inputString);
    strcpy(cmdbuffer[bufindw], inputString);
    bufindw = (bufindw + 1)%BUFSIZE;
    buflen += 1;
  }//////////////////////
  void write(char *input)
  {
    serial_count = strlen(input);
    strcpy(cmdbuffer[bufindw], input);
    bufindw = (bufindw + 1)%BUFSIZE;
    buflen += 1;
  }//////////////////////
  char* read()
  {
    strcpy(cmdString, cmdbuffer[bufindr]);
    buflen = (buflen - 1);
    bufindr = (bufindr + 1) % BUFSIZE;
    return cmdString;
  }/////////////////////////

} SERIAL_BUFFER_DATA;

//////////////////////////////////////////////////////////////

typedef struct
{
    double EEx[2];//[START, END]
    double EEy[2];//[START, END]
    double EETheta;
    double Vel;
}LINEARProfile;// 28bytes
typedef struct
{
    double EEx;
    double EEy;
    double EETheta[2];//[START, END]
    double Vel;
}EEROTATIONProfile;// 28bytes
typedef struct
{
    double speed;
    double radius;
    double cenPosX, cenPosY;
    double EETheta;
    double arcAng;
    double posZ=0;
    double heightZ=0;
}SPIRALProfile;// 28bytes
typedef struct
{
    uint8_t select;
    double speed;
    double radius;
    double cenPosX, cenPosY;
    double EETheta;
    double arcAng;
}CIRCLEProfile;// 28bytes
// ... storing I.K motion for timer

typedef struct
{
  // ...OUTPUT...
  uint8_t select;
  uint32_t steps;   // total steps(pluse)
  uint32_t Na;      // pulse counts for acceleration area
  uint32_t Nac;     // pulse counts for acceleration and constant areas
  uint32_t NNb;     // pulse counts for deceleration area
  uint32_t Cn_acc0; // fisrt period time for 1st pulse of acceleration on Timer.
  int8_t dir;       // step motor direction 1:CCW, -1:CC
} SPEEDProfile;// 28bytes

typedef struct
{
  uint8_t ack=0;
  int CMDCode=0;
  volatile int32_t abs_step_pos=0;// Step counts
  volatile float   abs_mm_pos=0;// milimeters
  volatile OP_MODE OperationMode=STOPPED;
  public:
  void reset(){
    CMDCode = 0;
    //memset(CMDCode,0,sizeof(char)*5);
    abs_step_pos=0;
    abs_mm_pos=0;
    OP_MODE OperationMode=STOPPED;
  }
} POSData;

typedef struct _KIN_PARAM_
{
  double x, y;  // x, y position of End-Effector
  double theta; // tip angle of End-Effector

  double t1, t2; // joing angle
  double L;      // x position of robot
} KIN_PARAM;

enum TIMERSEL
{
  TC_0 = 0,
  TC_1,
  TC_2,
  TC_3,
  TC_4,
  TC_5,
  TC_6,
  TC_7
};
enum PINMAP
{
  ST_PULSE = 0,
  ST_DIR,
  HOMESTOP,
  ENDSTOP
};

#endif //_MKZOEROBOTICS_GLOBALVARIABLES_TEST_H_