/**
  mkZoeRobotics_command.cpp 
  Copyright (c) 2020 Mike Kim (ZOE Robotics).  All right reserved.
  Modified 5 August 2020 by Mike Kim
*/

#include <string.h>
#include "mkZoeRobotics_define.h"
#include "mkZoeRobotics_command.h"
#include "mkZoeRobotics_serial.h"
#include "mkCANEncoder.h"
#include "timetick.h"
#ifndef min
#define min(a, b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef max
#define max(a, b) (((a) > (b)) ? (a) : (b))
#endif

MKCommand mkCommand;
extern int motorID;
extern MKSerial MKSerial;
extern   mkCANEncoder mkCAN;
extern OP_MODE OperationMode;
extern POSData posData[4];
extern SPEEDRampData speedData[MAX_MOTOR_NUM];
extern KIN_DATA kinData[3];
//extern  uint16_t gIRQ_TC_FLAG_DONE[MAX_MOTOR_NUM] ;
extern SERIAL_BUFFER_DATA serialSendBuf;
extern unsigned long elapsedTime[MAX_MOTOR_NUM];
//extern  int8_t activatedEE;
extern int8_t isAnyMotion;
extern JOBSTATUS jobStatus;

extern uint8_t getHomeSWStatus();
extern uint8_t getStatusHoming();
extern void setStatusHoming(uint8_t val);
extern void reportACK(int codeValue, int mID, int errorCode = 0);
extern void reportStatus(int codeValue, int mID);
extern void reportAllPosStatus(int respCode, int codeValue);
extern void reportGenKinDataStatus(int RC, int codeValue, int rev);
extern void startTimer(int n, int prescale, uint32_t frequency);
extern void rebootTimers();
extern void stopTimer(int n);
extern void stopMotionAll();
extern void stopMotion(int id);
extern void resetElapsedTime();
extern void resetElapsedTime(int ch);
extern void controlPowerLine(bool bPowerOn);
extern void controlZBrake(bool bBrakeOn);
extern void delay( unsigned long ms );
extern void controlDropCup(int delayTime);
void MKCommand::getCommand()
{
  while (mkSerial.available() > 0 && buflen < BUFSIZE)
  {
    serial_char = mkSerial.read();
    if (serial_char == '\n' ||
        serial_char == '\r' ||
        (serial_char == ';' && comment_mode == false) ||
        serial_count >= (MAX_CMD_SIZE - 1))
    {

      if (!serial_count)
      {                       //if empty line
        comment_mode = false; //for new command
        return;
      }
      //SERIAL_PROTOCOLLNPGM(MSG_OK);
      cmdbuffer[bufindw][serial_count] = 0; //terminate string
                                            //if(!comment_mode)
                                            // {
      comment_mode = false;                 //for new command
                                            // mkSerial.print("I got this: ");
                                            // mkSerial.println(cmdbuffer[bufindw]);
      bufindw = (bufindw + 1) % BUFSIZE;
      buflen += 1;
      //}
      serial_count = 0; //clear buffer
    }
    else
    {
      //if(serial_char == ';') comment_mode = true;
      //if(!comment_mode)
      cmdbuffer[bufindw][serial_count++] = serial_char;
      //  mkSerial.print(serial_char);
      //  mkSerial.print('\n');
    }
  }
}
/////////////////////////////////////////////////////
void MKCommand::getStartMove(int axis_sel)
{
  bool bTimer[4] = {false};
  //isAnyMotion = 0;
  ////////////////////////////////////
  // Kinematics move
  if (axis_sel & (1 << SM_KIN))
  {
    for (int i = 0; i < 3; i++)
    {
      posData[i].OperationMode = MOVING;
      posData[i].CMDCode = 0;
      kinData[i].activated = true;
      //isAnyMotion++;
      resetElapsedTime(i);
    }
    if (axis_sel & (1 << SM_Z)) //  Start Z-axis
    {
      posData[3].OperationMode = MOVING;
      posData[3].CMDCode = 0;
      kinData[3].activated = true;
      //isAnyMotion++;
      resetElapsedTime(3);
    }
  }
  ////////////////////////////////////
  // Joint move
  else
  {
    if (axis_sel & (1 << SM_X)) // Start X-axis
    {
      bTimer[0] = true;
      isAnyMotion++;
    }
    if (axis_sel & (1 << SM_R1)) // Start R1-axis
    {
      bTimer[1] = true;
      //isAnyMotion++;
    }
    if (axis_sel & (1 << SM_R2)) // Start R2-axis(Revolute-Joint1)
    {
      bTimer[2] = true;
      //isAnyMotion++;
    }
    if (axis_sel & (1 << SM_Z)) //  Start Z-axis(Revolute-Joint2)
    {
      bTimer[3] = true;
      //isAnyMotion++;
    }
    ///////////////////////////////////////

    for (int i = 0; i < 4; i++)
    {
      if (bTimer[i])
      {
        posData[i].OperationMode = MOVING;
        posData[i].CMDCode = 0;
        speedData[i].activated = true;
        resetElapsedTime(i);
      }
    }
  } // -----else {
}
////////////////////////////////////////////
void MKCommand::process_commands()
{
  //mkSerial.println("process_commands---G1");
  unsigned long codenum; //throw away variable
  char *starpos = NULL;
  if (code_seen('J')) {
    jobStatus.jobID = (int)code_value();
  }
  if (code_seen('N')){
    jobStatus.nSequence = (int)code_value();
  }
  if (code_seen('G')){
    int codeValue = (int)code_value();
    int selectedAxis = -1;
    switch (codeValue)
    {
    case SC_MOVE: // G0: Start Move
      if (code_seen('A'))
      {
        selectedAxis = (int)code_value();
        getStartMove(selectedAxis);
      }
      
      reportACK(codeValue, selectedAxis);
      return;
    case SC_SET_SPEED: // G1: Set Speed Profile

      if (code_seen('M'))
      {
        motorID = (uint32_t)code_value(); //+ (axis_relative_modes[i] || relative_mode)*current_position[i];
      }
      if (code_seen('J'))
      {
        jobStatus.jobID = (int)code_value();
      }
      resetElapsedTime(motorID);
      // if(posData[motorID].OperationMode !=JOB_DONE)
      //   return reportACK(codeValue, motorID, jobID, MOVING);//reportStatus(codeValue, motorID);
      posData[motorID].OperationMode = SET_SPEED;
      posData[motorID].CMDCode = codeValue;
      if (!get_speed_profile())
      {
        posData[motorID].OperationMode = ERROR + SET_SPEED;
        //mkSerial.println("format should be: G17 S[steps] A[Na] C[Nac] D[Nd] T[time0] O[dir]");
      }
      reportACK(codeValue, motorID,  posData[motorID].OperationMode);

      return;
    case SC_GEN_EEMOTION:
    {
      int rev = get_gen_linear_motion_profile();
      reportGenKinDataStatus(RC_STATUS_LINEAR, SC_GEN_EEMOTION, rev);
      return;
    }
    case SC_GEN_EEROTATION:
    {
      int rev = get_gen_EErotation_motion_profile();
      reportGenKinDataStatus(RC_STATUS_EEROTATION, SC_GEN_EEROTATION, rev);
      
      return;
    }
    case SC_GEN_CIRCLE:
    {
      int rev = get_gen_circular_motion_profile();
      reportGenKinDataStatus(RC_STATUS_CIRCLE, SC_GEN_CIRCLE, rev);
      return;
    }
    case SC_GEN_SPIRAL:
    {
      int rev = get_gen_spiral_motion_profile();
      reportGenKinDataStatus(RC_STATUS_SPRIAL, SC_GEN_SPIRAL, rev);
      return;
    }
    case SC_SETPOS: // G3: Set Position
      if (code_seen('A'))
      {
        posData[0].abs_step_pos = (int32_t)code_value();
      }
      if (code_seen('B'))
      {
        posData[1].abs_step_pos = (int32_t)code_value();
      }
      if (code_seen('C'))
      {
        posData[2].abs_step_pos = (int32_t)code_value();
      }
      if (code_seen('D'))
      {
        posData[3].abs_step_pos = (int32_t)code_value();
      }
      reportACK(codeValue, 0);
      return;
    case SC_STOP: // G4: STOP immediatelty
      stopMotionAll();
      rebootTimers();
      posData[motorID].OperationMode = JOB_DONE;
      reportACK(codeValue, 0);
      return;
    case SC_TIME_DELAY_MC:
      if (code_seen('M'))
      {
        int ms = (uint32_t)code_value();
        Sleep(ms);
        reportACK(codeValue, 0);
        return;
      }
      else {
        reportACK(codeValue, motorID, ERROR_MISSING_M+SC_TIME_DELAY_MC);
      }
      
      
      return;
    case SC_SET_ZERO_ENCODER:// Rotational Axis....
      if (code_seen('M'))
      {
        motorID = (uint32_t)code_value();
        if(motorID==1) { 
          mkCAN.setZeroPos(0);
          posData[1].abs_step_pos=0;
        }
        else if(motorID==2) { 
          mkCAN.setZeroPos(1);
          posData[2].abs_step_pos=0;
        }
        reportACK(codeValue, 0);
      }
      else {
        reportACK(codeValue, motorID, ERROR_MISSING_M+SC_SET_ZERO_ENCODER);
      }

    return;
    case SC_GET_ENCODER:
      if (code_seen('M'))
      {
        motorID = (uint32_t)code_value();
        if(motorID==1) mkCAN.bReadSignal[0]=true;
        else if(motorID==2) mkCAN.bReadSignal[1]=true;
      }
      else {
        reportACK(codeValue, motorID, ERROR_MISSING_M+SC_GET_ENCODER);
      }
    return;
    case SC_HOMING: // G9: Homing Process
      if (code_seen('M')) motorID = (uint32_t)code_value();
      else {
        reportACK(codeValue, motorID, ERROR_MISSING_M+SC_HOMING);
        return;
      }
      posData[motorID].CMDCode = codeValue;
      if (get_speed_profile())
      {
        posData[motorID].OperationMode = HOMING;
        speedData[motorID].activated = true;
        resetElapsedTime(motorID);
      }
      else
      {
        reportACK(codeValue, motorID, ERROR + SC_HOMING);
      }
      return;

    case SC_STATUS: // G10: get Current Status
      resetElapsedTime(motorID);
      if (code_seen('M'))
      {
        motorID = (int)code_value();
      }
      else {
        reportACK(codeValue, motorID, ERROR_MISSING_M+SC_STATUS);
        return;
      }
      reportStatus(codeValue, motorID);
      return;
    case SC_STATUS_ALL_POS:
      reportAllPosStatus(RC_STATUS_ALL_POS, codeValue);
      return;
    case SC_UPDATE_MOTION:
      reportAllPosStatus(RC_UPDATE_MOTION, codeValue);
      return;
    case SC_REBOOT: // Restart
      rebootTimers();
      reportACK(codeValue, 0);
      return;
    
    case SC_POWER: // CONTROL POWER ON/OFF
      if(code_seen('M')) {
        controlPowerLine((bool)code_value());
      }
      else {
        reportACK(codeValue, 0, ERROR_MISSING_M+SC_POWER);
        return;
      }
      
      //reportACK(codeValue);
      return;
    // case SC_Z_BRAKE: // CONTROL Brake ON/OFF
    //   if(code_seen('M')) {
    //     controlZBrake((bool)code_value());
    //   }
    //   else {
    //     reportACK(codeValue, 0, ERROR+SC_Z_BRAKE);
    //     return;
    //   }
      
      // reportACK(codeValue,0);
      case SC_DROP_CUP:
        
        if(code_seen('M')) {
          controlDropCup(code_value());
        }
        else {
          reportACK(codeValue, 0, ERROR_MISSING_M+SC_DROP_CUP);
          return;
        }
      return;
      default:
      return;
    }
  }
}
int MKCommand::get_gen_linear_motion_profile()
{
  bool seen = false;
  if (code_seen('X'))
  {
    linearProfile.EEx[0] = code_value();
    // linearProfile.EEx[0]=1193;
    seen = true;
  }
  else
    seen = false;

  if (code_seen('Y'))
  {
    linearProfile.EEx[1] = code_value();
    // linearProfile.EEx[1]= 1193;
    seen = true;
  }
  else
    seen = false;

  if (code_seen('Z'))
  {
    linearProfile.EEy[0] = code_value();
    // linearProfile.EEy[0]= -148;
    seen = true;
  }
  else
    seen = false;

  if (code_seen('T'))
  {
    linearProfile.EEy[1] = code_value();
    // linearProfile.EEy[1]= -401;
    seen = true;
  }
  else
    seen = false;

  if (code_seen('V'))
  {
    linearProfile.EETheta = code_value();
    seen = true;
  }
  else
    seen = false;

  if (code_seen('A'))
  {
    linearProfile.Vel = code_value();
    // linearProfile.Vel = 100;
    seen = true;
  }
  else
    seen = false;

  if (code_seen('J'))
  {
    jobStatus.jobID = (int)code_value();
  }

  if (code_seen('N'))
  {
    jobStatus.nSequence = (uint8_t)code_value();
  }

  if (seen)
  {
    return callback_gen_linear_profile(linearProfile);
    
  }

  return ERROR;
}
int MKCommand::get_gen_EErotation_motion_profile()
{
  bool seen = false;
  if (code_seen('X'))
  {
    eeRotationProfile.EEx = code_value();
    // linearProfile.EEx[0]=1193;
    seen = true;
  }
  else
    seen = false;

  if (code_seen('Y'))
  {
    eeRotationProfile.EEy = code_value();
    // linearProfile.EEx[1]= 1193;
    seen = true;
  }
  else
    seen = false;

  if (code_seen('Z'))
  {
    eeRotationProfile.EETheta[0] = code_value();
    // linearProfile.EEy[0]= -148;
    seen = true;
  }
  else
    seen = false;

  if (code_seen('T'))
  {
    eeRotationProfile.EETheta[1] = code_value();
    // linearProfile.EEy[1]= -401;
    seen = true;
  }
  else
    seen = false;

  if (code_seen('V'))
  {
    eeRotationProfile.Vel = code_value();
    seen = true;
  }
  else
    seen = false;
  if (code_seen('J'))
  {
    jobStatus.jobID = (int)code_value();
  }

  if (code_seen('N'))
  {
    jobStatus.nSequence = (uint8_t)code_value();
  }

  if (seen)
  {
    return callback_gen_EErotation_profile(eeRotationProfile);
    
  }

  return ERROR;
}
int MKCommand::get_gen_circular_motion_profile()
{
  bool seen = false;
  if (code_seen('S'))
  {
    circleProfile.speed = code_value();
    seen = true;
  }
  else
    seen = false;

  if (code_seen('R'))
  {
    circleProfile.radius = code_value();
    seen = true;
  }
  else
    seen = false;

  if (code_seen('X'))
  {
    circleProfile.cenPosX = code_value();
    seen = true;
  }
  else
    seen = false;

  if (code_seen('Y'))
  {
    circleProfile.cenPosY = code_value();
    seen = true;
  }
  else
    seen = false;

  if (code_seen('T'))
  {
    circleProfile.EETheta = code_value();
    seen = true;
  }
  else
    seen = false;

  if (code_seen('A'))
  {
    circleProfile.arcAng = code_value();
    seen = true;
  }
  else
    seen = false;

  if (code_seen('J'))
  {
    jobStatus.jobID = (int)code_value();
  }

  if (code_seen('N'))
  {
    jobStatus.nSequence = (uint8_t)code_value();
  }

  if (seen)
  {
    return callback_gen_circle_profile(circleProfile);
    
  }

  return ERROR;
  
}
int MKCommand::get_gen_spiral_motion_profile()
{
  bool seen = false;
  if (code_seen('S'))
  {
    spiralProfile.speed = code_value();
    seen = true;
  }
  else
    seen = false;

  if (code_seen('R'))
  {
    spiralProfile.radius = code_value();
    seen = true;
  }
  else
    seen = false;

  if (code_seen('X'))
  {
    spiralProfile.cenPosX = code_value();
    seen = true;
  }
  else
    seen = false;

  if (code_seen('Y'))
  {
    spiralProfile.cenPosY = code_value();
    seen = true;
  }
  else
    seen = false;

  if (code_seen('T'))
  {
    spiralProfile.EETheta = code_value();
    seen = true;
  }
  else
    seen = false;

  if (code_seen('A'))
  {
    spiralProfile.arcAng = code_value();
    seen = true;
  }
  if (code_seen('B'))
  {
    spiralProfile.posZ = code_value();
    seen = true;
  }
  if (code_seen('C'))
  {
    spiralProfile.heightZ = code_value();
    seen = true;
  }
  else
    seen = false;

  if (code_seen('J'))
  {
    jobStatus.jobID = (int)code_value();
  }

  if (code_seen('N'))
  {
    jobStatus.nSequence = (uint8_t)code_value();
  }

  if (seen)
  {
    return callback_gen_spiral_profile(spiralProfile);
    
  }

  return ERROR;
  
}

bool MKCommand::get_speed_profile()
{
  bool seen = false;

  if (code_seen('S'))
  {
    speedProfile.steps = (uint32_t)code_value(); //+ (axis_relative_modes[i] || relative_mode)*current_position[i];
    seen = true;
  }
  else
    seen = false;

  if (code_seen('A'))
  {
    speedProfile.Na = (uint32_t)code_value();
  }
  else
    seen = false;

  if (code_seen('C'))
  {
    speedProfile.Nac = (uint32_t)code_value();
  }
  else
    seen = false;

  if (code_seen('D'))
  {
    speedProfile.NNb = (uint32_t)code_value();
  }
  else
    seen = false;

  if (code_seen('T'))
  {
    speedProfile.Cn_acc0 = (uint32_t)code_value();
  }
  else
    seen = false;

  if (code_seen('O'))
  {
    speedProfile.dir = (int8_t)code_value();
  }
  else
    seen = false;
  if (code_seen('J'))
  {
    jobStatus.jobID = (int)code_value();
  }

  if (code_seen('N'))
  {
    jobStatus.nSequence = (uint8_t)code_value();
  }

  if (seen)
  {
    callback_set_speed_profile(speedProfile);
    return true;
  }
  else
  {
    return false;
    //mkSerial.println("format should be: G17 S[steps] A[Na] C[Nac] D[Nd] T[time0] O[dir]");
  }
  return false;
}

bool MKCommand::code_seen(char code)
{
  strchr_pointer = strchr(cmdbuffer[bufindr], code);
  return (strchr_pointer != NULL); //Return True if a character was found
}

float MKCommand::code_value()
{
  return (strtod(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL));
}

long MKCommand::code_value_long()
{
  return (strtol(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL, 10));
}
