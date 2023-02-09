/**
  mkZoeRobotics_velProfile.c 
  Copyright (c) 2020 Mike Kim (ZOE Robotics).  All right reserved.
  Originated at 28 August 2020 by Mike Kim
*/
//  g++ -Wall -g   main_test_velocity.c mkZoeRobotics_velProfile_test.cpp  -o main 
#include <stdio.h>
#include <math.h>
#include "mkZoeRobotics_velProfile_test.h"
// #include "mkZoeRobotics_serial.h"
// #include "mkZoeRobotics_define.h"
//#include "mkMotorCtrl.h"
//void setMotorDirection(int8_t dir);
MKVelProfile mkVelProfile;
extern KIN_DATA kinData[3];
//extern MKStepperMotorCtrl mkSTMotorCtrl;
int8_t activatedEE;
// extern MKSerial mkSerial;
 SPEEDRampData speedData[MAX_MOTOR_NUM];
#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })
#define getDir(a) \
    ( { __typeof__(a) _a = (a);      \
          (_a>0) ? 1:-1;  \
    }  )
// extern MOTORCHANEL motorCh[3];
// extern unsigned long elapsedTime[MAX_MOTOR_NUM];
// extern volatile uint32_t gIRQ_step_count[MAX_MOTOR_NUM];
// extern volatile uint16_t gIRQ_flag[MAX_MOTOR_NUM];

// extern  void stopTimer(int n);
// inline void setMotorDir(int8_t direction)
// {
//   mkSerial.print(direction);
//   mkSerial.println(": setMOTORDIR()");
// }
//extern uint32_t DIST2STEP;
// extern POSData posData[4];
int motorID=0;
  //#define SPR 1600 // (200*8); //% step per revolution
// static  float alpha = (2.0 * M_PI / MICROSTEPPING); //  % [rad] angle per step
// static  float two_alpha = 2.0 * alpha;       // alpha*2
///////////////////////////////////////////////////
// Initializing static member variable: motorParams for static member functions... 
 MOTOR_PARAMS MKVelProfile::motorParams[2]={
              // Motor#0: X-Axis 
              {X_MICROSTEPPING, X_MICROSTEPPING/X_DIST2STEP_20T5MM, 
              double(X_DIST2STEP_20T5MM)/double(X_MICROSTEPPING), 
              2.0*M_PI/X_MICROSTEPPING, 4.0*M_PI/X_MICROSTEPPING},
              // Motor#1: Z-Axis
              {Z_MICROSTEPPING, Z_MICROSTEPPING/Z_DIST2STEP_20D5MM, 
              double(Z_DIST2STEP_20D5MM)/double(Z_MICROSTEPPING), 
              2.0*M_PI/Z_MICROSTEPPING, 4.0*M_PI/Z_MICROSTEPPING}
        };
  // ++ deltaT_Lx is duration when EE rotate 1deg ++
  double DIST2STEP[4]={16.0, 1527.89, 763.944,80.0}; //[X, R1, R2, Z]
  double STEP2DIST[4]={1.0/16.0, 1.0/1527.89, 1.0/763.944, 1.0/80.0}; //[X, R1, R2, Z]
    //  motorParams[0].MICROSTEPPING = X_MICROSTEPPING;
    // motorParams[0].DIST2STEP = X_MICROSTEPPING/X_DIST2STEP_20T5MM;
    // MOTOR_PARAMS MKVelProfile::motorParams[0].STEP2DIST = double(X_DIST2STEP_20T5MM)/double(X_MICROSTEPPING);
    // MOTOR_PARAMS MKVelProfile::motorParams[0].alpha = 2.0*M_PI/MKVelProfile::motorParams[0].MICROSTEPPING;
    // MOTOR_PARAMS MKVelProfile::motorParams[0].two_alpha = MKVelProfile::motorParams[0].alpha*2.0;

    // MKVelProfile::motorParams[1].MICROSTEPPING = Z_MICROSTEPPING;
    // MKVelProfile::motorParams[1].DIST2STEP = Z_MICROSTEPPING/Z_DIST2STEP_20D5MM;
    // MKVelProfile::motorParams[1].STEP2DIST = double(Z_DIST2STEP_20D5MM)/double(Z_MICROSTEPPING);
    // MKVelProfile::motorParams[1].alpha = 2.0*M_PI/MKVelProfile::motorParams[1].MICROSTEPPING;
    // MKVelProfile::motorParams[1].two_alpha = MKVelProfile::motorParams[1].alpha*2.0;
/*
void MKVelProfile::set_speed_profile( uint32_t steps, uint32_t Na, uint32_t Nac, uint32_t Nd, uint32_t start_time0, int8_t dir)
{
  //int num = 0;

  speedData[motorID].activated = false;
  stopTimer(motorID);
  // mkSerial.print(motorID);
  // mkSerial.println(", I'm in updateVelocityProfile!");

  activatedEE = -1;
  
  speedData[motorID].steps = steps;
  speedData[motorID].Na = Na;
  speedData[motorID].Nac = Nac;
  speedData[motorID].NNb = Nd;
  speedData[motorID].Cn_acc0 = start_time0;
  speedData[motorID].dir = dir;

  if (dir == 1)
  { // CCW
    
      motorCh[motorID].dir.pIO->PIO_ODSR = motorCh[motorID].dir.pin;
      motorCh[motorID].dir.pIO->PIO_CODR = motorCh[motorID].dir.pin;
 

  }
  else
  { // CW
      motorCh[motorID].dir.pIO->PIO_ODSR ^= motorCh[motorID].dir.pin;
      motorCh[motorID].dir.pIO->PIO_SODR = motorCh[motorID].dir.pin;
  }

  speedData[motorID].Cn = speedData[motorID].Cn_acc0;
  elapsedTime[motorID] = GetTickCount();
  speedData[motorID].step_count = 0;

  //speedData[motorID].activated = true;
}
*/
#if 0
void MKVelProfile::set_speed_profile( uint32_t steps, uint32_t Na, uint32_t Nac, uint32_t Nd, uint32_t start_time0, int8_t dir)
{
  int num = motorID;
  // // mkSerial.print(motorID);
  //   // mkSerial.println(", I'm in updateVelocityProfile!");
  //for(int num=0; num<4; num++)
  //{
    speedData[num].activated = false;
   //stopTimer(num);
    

    activatedEE = -1;
    
    speedData[num].totalSteps = steps;
    speedData[num].Na = Na;
    speedData[num].Nac = Nac;
    speedData[num].NNb = Nd;
    speedData[num].Cn_acc0 = start_time0;
    speedData[num].dir = dir;

    if (dir == 1)
    { // CCW
        motorCh[num].dir.pIO->PIO_ODSR = motorCh[num].dir.pin;
        motorCh[num].dir.pIO->PIO_CODR = motorCh[num].dir.pin;
    }
    else
    { // CW
        motorCh[num].dir.pIO->PIO_ODSR ^= motorCh[num].dir.pin;
        motorCh[num].dir.pIO->PIO_SODR = motorCh[num].dir.pin;
    }

    speedData[num].Cn = speedData[num].Cn_acc0;
    speedData[num].step_count = 0;
//}
  //speedData[motorID].activated = true;
}
#endif
void MKVelProfile::gen_speed_profile(uint16_t num, double distance, double speed, double accel, double decel)
{

 // double distance = 0.025 * steps; // [mm]
   activatedEE = -1;
  int dir = SIGN(distance);
  uint32_t steps = lround(abs(distance)*motorParams[num].DIST2STEP);
  // mkSerial.print("gen_speed_profile::Moving Distance[mm]=");
  // mkSerial.println(distance);

  //if(speed>=MAX_SPEED_STEP) speed=MAX_SPEED_STEP; // Max speed...

  // if(sel_sqrt_cal==SQRT_APPRO){
  //   TICK_FREQ_SQRT_ERROR_COM *= 0.676;//0.69367;//initial error: 0.69367 at n=1, ((sqrt(2)-sqrt(1)))/((1.0-2.0/(4.0+1.0)))
  // }



  // % 1. Calcalate Time
  // % Ta = speed/acceleration
  speedData[num].dir = dir;
  speedData[num].Ta = speed / accel; //%[sec]
  speedData[num].Tb = speed / decel; //%[sec]
  speedData[num].Tc = (steps * motorParams[num].alpha - 0.5 * accel * speedData[num].Ta * speedData[num].Ta - 0.5 * decel * speedData[num].Tb * speedData[num].Tb) / speed;
  if (speedData[num].Tc > 0)
  {
    speedData[num].Ttotal = speedData[num].Ta + speedData[num].Tc + speedData[num].Tb;
  }
  else
  {
    speedData[num].Ttotal = speedData[num].Ta + speedData[num].Tb;
  }

  // % 2. Calculate Step
  // % convert Time Zone to Stepper Motor Count
  // % Acceleration Zone
  // % n = speed^2/(2*alpha*accel)
  speedData[num].totalSteps = steps;
  speedData[num].Na = floor(speed * speed / (motorParams[num].two_alpha * accel));
  uint32_t Nacc = floor(steps * decel / (accel + decel));

  if (speedData[num].Na < Nacc)
  {
    //%Nb = floor(speed^2/(2*alpha*decel));
    speedData[num].Nb = accel / decel * speedData[num].Na;
    speedData[num].Nc = steps - (speedData[num].Na + speedData[num].Nb);
  }
  else
  {
    speedData[num].Na = Nacc;
    speedData[num].Nb = steps - Nacc;
    speedData[num].Nc = 0;
  }
  speedData[num].Nac = speedData[num].Na + speedData[num].Nc;
  speedData[num].NNb = speedData[num].Nb;
  speedData[num].Cn_const_speed = uint32_t(TICK_FREQ_SQRT_ERROR_COM * sqrt(motorParams[num].two_alpha / accel) * (sqrt(speedData[num].Na + 1) - sqrt(speedData[num].Na)));
  speedData[num].Cn_acc0 = uint32_t(TICK_FREQ_SQRT_ERROR_COM * sqrt(motorParams[num].two_alpha / accel));
  speedData[num].Cn_dec0 = uint32_t(TICK_FREQ_SQRT_ERROR_COM * sqrt(motorParams[num].two_alpha / decel));

  speedData[num].Cn = speedData[num].Cn_acc0;

  speedData[num].step_count = 0;
  // if (dir == 1)
  // { // CCW
  //   IOMap[num][ST_DIR].pIO->PIO_ODSR = IOMap[num][ST_DIR].pin;
  //   IOMap[num][ST_DIR].pIO->PIO_CODR = IOMap[num][ST_DIR].pin;
  // }
  // else
  // { // CW
  //   IOMap[num][ST_DIR].pIO->PIO_ODSR ^= IOMap[num][ST_DIR].pin;
  //   IOMap[num][ST_DIR].pIO->PIO_SODR = IOMap[num][ST_DIR].pin;
  // }
  //mkSTMotorCtrl.writeMotorDirection(dir);

  // mkSerial.println("----------------start -------------");
  // mkSerial.print("speedData[num].Cn=");
  // mkSerial.println(speedData[num].Cn);
  // mkSerial.print("Total time[");
  // mkSerial.print(num);
  // mkSerial.print("]=:");
  // mkSerial.println(speedData[num].Ttotal, 4);
  // mkSerial.println(steps);
  // mkSerial.println(speedData[num].Cn_acc0);
  // mkSerial.println(speedData[num].Na);
  // mkSerial.println(speedData[num].Nac);
  // mkSerial.println(speedData[num].NNb);

  //speedData[num].activated = true;
}

void MKVelProfile::update_speed_only(uint16_t num, uint32_t steps)
{
  speedData[num].Cn = steps;
  //// mkSerial.println("modify_speed_profile");
}

void MKVelProfile::forKin(double L, double t1, double t2)
{
  kinParam.x = L + LINK_2 * cos(t1 + t2) + LINK_1 * cos(t1);
  kinParam.y = LINK_2 * sin(t1 + t2) + LINK_1 * sin(t1);
  kinParam.theta = t1 + t2;
}
bool MKVelProfile::invKin(double x, double y, double theta)
{
  double ratio = (y - LINK_2 * sin(theta)) / (LINK_1);
  if (abs(ratio) > 1.0)
    return false;
  kinParam.t1 = asin(ratio);
  kinParam.t2 = theta - kinParam.t1;
  kinParam.L = x - (LINK_2 * cos(kinParam.t1 + kinParam.t2) + LINK_1 * cos(kinParam.t1));
  return true;
}
bool MKVelProfile::invKin(KIN_PARAM &input)
{
  double ratio = (input.y - LINK_2 * sin(input.theta)) / (LINK_1);
  if (abs(ratio) > 1.0)
    return false;
  kinParam.t1 = asin(ratio);
  kinParam.t2 = input.theta - kinParam.t1;
  kinParam.L = input.x - (LINK_2 * cos(kinParam.t1 + kinParam.t2) + LINK_1 * cos(kinParam.t1));
  return true;
}
bool MKVelProfile::arcMotionIK(double arcR, double cenPosX, double cenPosY, double EndEffectorAng, double rotAng)
{
  //KIN_PARAM forwardInput;
  double x = cenPosX + arcR * cos(rotAng);
  double y = cenPosY + arcR * sin(rotAng);
  double theta = EndEffectorAng;

  return invKin(x, y, theta);
}

int MKVelProfile::gen_linear_profile(LINEARProfile & linearProfile)
{

  ///////////////////////////////////
  kinData[0].reset();
  kinData[1].reset();
  kinData[2].reset();
  double aCoEE[2][4], curEEPos[2];
  // bool isSameTrajectory=true;
  

  double initialEEPos[2]={linearProfile.EEx[0],linearProfile.EEy[0]};
  double finalEEPos[2]  ={linearProfile.EEx[1],linearProfile.EEy[1]};

  // double maxPos=0.0;
  // for(int i=0; i<2; i++){
  //   double deltaPos = finalEEPos[i] -  initialEEPos[i];
  //   if(maxPos<=abs(deltaPos))
  //   {
  //       maxPos=abs(deltaPos);
  //   }
  // }
  // if(maxPos<=200) linearProfile.Vel *=0.75;
  // double Tf=maxPos/linearProfile.Vel;
  // double deltaT= maxPos/10000.0;// maintain 750 data set

  double diffDist[2];
  diffDist[0]= finalEEPos[0] -  initialEEPos[0];
  diffDist[1]= finalEEPos[1] -  initialEEPos[1];
  double distEE = sqrt(diffDist[0]*diffDist[0] + diffDist[1]*diffDist[1]);
  if(distEE<1.0) return ERROR+6;// too small movement...
  if(distEE<=150) linearProfile.Vel *=0.65;
  double Tf=distEE/linearProfile.Vel;
  double deltaT=4.0/1000.0;// 4 milisec
  if(distEE/deltaT<50) {
    deltaT= distEE/5000.0;
  }
// double deltaT= distEE/50000.0;// maintain 750 data set
 if(Tf<=1.0) {
   Tf=1.0;
  //  deltaT=3/1000.0;
 }
  for(int i=0; i<2; i++) {
            aCoEE[i][0] = initialEEPos[i];
            aCoEE[i][1] = 0.0;
            aCoEE[i][2] = 3.0*(finalEEPos[i]-initialEEPos[i])/(Tf*Tf);
            aCoEE[i][3] = -2.0*(finalEEPos[i]-initialEEPos[i])/(Tf*Tf*Tf);
            // if(abs(aCoEE[i][2])>1e-10) {
            //   isSameTrajectory=false;
            // }
        }
  

  double currT=0.0;
  double currEEPos[3];

  int totalN =  ceil(Tf/deltaT);
  int zeroCN = int(tick_freq * deltaT+0.5);
  int totalCN[3]={0};
  printf("Tf=%f, deltalT=%f, totalN=%3.2f, %d\n", Tf, deltaT, Tf/deltaT, totalN);
  ///////////////////////////////////
  // double dx=abs(linearProfile.EEx[1]-linearProfile.EEx[0]);
  // double dy=-abs(linearProfile.EEy[1]-linearProfile.EEy[0]);

  // double sx = (linearProfile.EEx[0]<linearProfile.EEx[1]) ? 1 : -1;
  // double sy = (linearProfile.EEy[0]<linearProfile.EEy[1]) ? 1 : -1;
  // double err = dx+dy; 
  // int x = round(linearProfile.EEx[0]);
  // int y = round(linearProfile.EEy[0]);
  // double dist = sqrt(dx*dx + dy*dy);
////////////////////////////////////////////////////////
  // ++ deltaT_Lx is duration when EE move 1mm ++
  double deltaPos[3]={0}, deltaPosPrev[3]={0};//, residue[3]={0}, prevResidue[3]={0};
  double sumdL[3]={0}, sumDist[3]={0};
  double deltaPos_residue[3]={0};
  double prevPos[3]={0}, steps[3]={0};
  double abs_sum_steps[3]={0};
  double totalDistance[3]={0};
  //double dist[nAng+1] ;
  int cnt_zeros[3]={0};
  int index_step[3]={0};

  // double deltaT_per_1deg = 1.0/circleProfile.speed;
  double abs_step[3]={0};
  // double abs_step_1[3]={0},abs_step_residue [3]={0};
  // int acc_area = round(dist*0.1); // accelleration area from start (x%)
  // int acc_dec_area = acc_area;
  // double deltaT = 1.0/linearProfile.Vel;
  // int acc_area[3] = {10};
  bool bCheckSum=true;
  // bool bDebug=0;
  double sum_deltaPos[3]={0}, sum_stepPos[3]={0};
  double diffSteps=0;
 ////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////
// Linear for index 0

  if(!mkVelProfile.invKin(linearProfile.EEx[0], linearProfile.EEy[0], linearProfile.EETheta))
  {
    //if(bDebug) printf("!!!! NO IK SOLUTION !!!!\n");
    return 100+1;
  }
   prevPos[0] = mkVelProfile.kinParam.L;// Calculated by arcMotionIK (initial deltaPosue)
   prevPos[1] = mkVelProfile.kinParam.t1;
   prevPos[2] = mkVelProfile.kinParam.t2;
  //////////////////////////////////////////////////////////////////////
  // -- Start to calcute the number of pulses, delay time counts (Cn) and direction of motors
  int n=0;
  while(1)
  {
    currT +=deltaT;
    

    for(int i=0; i<2; i++){
        curEEPos[i] =
                aCoEE[i][3]*pow(currT,3) +
                aCoEE[i][2]*pow(currT,2)  +
                aCoEE[i][1]*    currT  +
                aCoEE[i][0];
    }

    //////////////////////////////////////////////////////////
    //  the I.K.
    if(!mkVelProfile.invKin(curEEPos[0], curEEPos[1], linearProfile.EETheta))
    {
      // if(bDebug) printf("!!!! NO IK SOLUTION !!!!\n");
      return 100+2;
    }

    ////////////////////////////////////////////////
    // Handing 3 motors...
    for(int CH=0; CH<3; CH++)
    {

      /////////////////////////////////////////
      // Calculate Delta Position
      // 0: X - Linear Motion (mm)
      if(CH==0) {
        deltaPos[CH] = (mkVelProfile.kinParam.L - prevPos[CH]); // [mm]
        prevPos[CH] = mkVelProfile.kinParam.L;
      }
      /////////////////////////////////////////
      // 1: R1 - Rotation Motion (rad)
      else if(CH==1) {
        deltaPos[CH] = (mkVelProfile.kinParam.t1 - prevPos[CH]); // [mm]
        prevPos[CH] = mkVelProfile.kinParam.t1;
      }
      /////////////////////////////////////////
      // 2: R2 - Rotation Motion (rad)
      else if(CH==2) {
        deltaPos[CH] = (mkVelProfile.kinParam.t2 - prevPos[CH]); // [mm]
        prevPos[CH] = mkVelProfile.kinParam.t2;
      }
      // "sum_deltaPos[CH]"" is for checking if there is difference between real step count and calculate count...
      sum_deltaPos[CH] += deltaPos[CH];
      totalDistance[CH] += abs(deltaPos[CH]);
    //%---------------------------------------------------------------------//
    //% Only accept more than one step if not, calculate the residue and pass
    //% it to next step...
      if(n==0) {
          deltaPos_residue[CH] =  deltaPos[CH];
      } else {
          sumdL[CH] = sumdL[CH] + abs(deltaPosPrev[CH]);
          deltaPos_residue[CH] =  deltaPos[CH]+SIGN(deltaPos[CH])*(sumdL[CH] - sumDist[CH]);
      }
      deltaPosPrev[CH]= deltaPos[CH];
      abs_step[CH] = abs(DIST2STEP[CH]*deltaPos_residue[CH]);
      if(abs_step[CH]>=1.0) {
        steps[CH] = SIGN(deltaPos[CH])*int(abs_step[CH]+0.5);;
      }
      else {
        steps[CH]=0;
      }
      abs_sum_steps[CH] += abs(steps[CH]);
      //--------------------------------------------------------------------//
      // Based on number of steps, calcuate Cn(count for delay) and direction
      // of motor and number of steps for this duration...
      
      if (steps[CH] == 0) {
        // When Step==0, handling time delays...
        kinData[CH].motionData[index_step[CH]].steps=0;
        if(n==0) {
          kinData[CH].motionData[index_step[CH]].Cn = zeroCN;//8000; // [mm]
          kinData[CH].motionData[index_step[CH]].dir = SIGN(deltaPos[CH]);
        } else {
          // Delay time is critical to syncronize 3 axis when step is zeros..
          // Cn value is function of deltaT...
          kinData[CH].motionData[index_step[CH]].Cn = zeroCN;//kinData[CH].motionData[index_step[CH]-1].Cn+100;//8200;//kinData[CH].motionData[index_step[CH]-1].Cn+200*0;
          kinData[CH].motionData[index_step[CH]].dir = kinData[CH].motionData[index_step[CH]-1].dir;
        }
      }
      else
      {
        kinData[CH].motionData[index_step[CH]].Cn = int(tick_freq * deltaT / abs(steps[CH])+0.5);
        kinData[CH].motionData[index_step[CH]].steps = int(abs(steps[CH])+0.5);
        kinData[CH].motionData[index_step[CH]].dir = SIGN(deltaPos[CH]); // stepper motor direction...
      }
      totalCN[CH] +=kinData[CH].motionData[index_step[CH]].Cn;
      if(CH==1) printf( "Cn1[%d]=%d\n", n, kinData[CH].motionData[index_step[CH]].Cn);
      sumDist[CH] = sumDist[CH] + kinData[CH].motionData[index_step[CH]].steps*STEP2DIST[CH];
      index_step[CH]++;
      if(index_step[CH]>=MAX_MOTIONDATA_SIZE-1) return ERROR+5;
    }// --------- for(CH)
    if(currT>=Tf) {
      for(int CH=0; CH<3; CH++) {
        kinData[CH].totalSteps = int(totalDistance[CH] * DIST2STEP[CH]+0.5);
        kinData[CH].dataSize = index_step[CH];
        // printf("Pos(%d)=%3.2f\n", CH+1, initialEEPos[CH]+sum_deltaPos[CH]);
        printf("totalCn(%d)=%d, totalSteps=%d\n", CH+1, totalCN[CH], kinData[CH].totalSteps );
        if(kinData[CH].totalSteps!=abs_sum_steps[CH]) bCheckSum=false;
      }
      break;
   }
   n++;
  }
  // kinData[0].totalSteps=1000;
  if(bCheckSum){
    return 1; 
  }// Calculation is right
  else {
     return 1;//ERROR+3; 
  }// total steps and sum of step is different (Calculation is wrong)...
}
/////////////////////////////////////////////////////////////////////////////////
//int createArcMotion() ==>  Return: 
// -1: no IK Solution
//  0: step count is not matching from length of move 
//  1: Everything is fine...
#if 0
int MKVelProfile::createArcMotion(double vel, double radius, double *cenPos, double EndEffectorAng , int nAng)
{
    double DIST2STEP[4]={16.0, 1527.89, 763.944,80.0}; //[X, R1, R2, Z]
  double STEP2DIST[4]={1.0/16.0, 1.0/1527.89, 1.0/763.944, 1.0/80.0}; //[X, R1, R2, Z]
  
  int8_t rotDir = SIGN(nAng);
  double deltaPos[3]={0}, residue[3]={0}, prevResidue[3]={0};
  double deltaPos_residue[3]={0};
  double prevPos[3]={0}, steps[3]={0};
  double abs_sum_steps[3]={0};
  double totalDistance[3]={0};
  //double dist[nAng+1] ;
  int cnt_zeros[3]={0};
  int index_step[3]={0};

  //double deltaT_per_1deg = 1.0/vel;
  double abs_step[3]={0};
  double abs_step_1[3]={0},abs_step_residue [3]={0};
  // int acc_dec_area = 16;//round(fabs(nAng)*5.0/100.0); // accelleration area from start (x%)
  double deltaT = 1.0/vel;//deltaT_per_1deg;
  // int acc_area=acc_dec_area;
  bool bCheckSum=true;
  // bool bDebug=0;

  /////////////////////////////////////////////////////////////////////////
// Calculation for index 0
  if(!arcMotionIK(radius, cenPos, EndEffectorAng*DEG2RAD, 0))
  {
    //if(bDebug) printf("!!!! NO IK SOLUTION !!!!\n");
    return -1;
  }
   prevPos[0] = kinParam.L;// Calculated by arcMotionIK (initial deltaPosue)
   prevPos[1] = kinParam.t1;
   prevPos[2] = kinParam.t2;

  double sum_deltaPos[3]={0}, sum_stepPos[3]={0};
  double EndEffectorRad = EndEffectorAng*DEG2RAD;
  for (int i = 0; i < nAng-6; i++)
  {
    //////////////////////////////////////////////////////////
    // Calculate the I.K.
    if(!arcMotionIK(radius, cenPos, EndEffectorRad ,  (i+1)*DEG2RAD * rotDir))
    {
      // if(bDebug) printf("!!!! NO IK SOLUTION !!!!\n");
      return -1;
    }
    for(int CH=0; CH<3; CH++)
    {
      motionData[CH][index_step[CH]].Cn = i*100;
        motionData[CH][index_step[CH]].steps = i;
        motionData[CH][index_step[CH]].dir = 1; // stepper motor direction...
        index_step[CH]++;
        totalSteps[CH]=i;
        motionDataSize[CH]=i;
    }
  }
  return 1;
}
#endif
#if 1 
int MKVelProfile::gen_circl_profile( CIRCLEProfile & circleProfile)
{
  
  
  int8_t rotDir = SIGN(circleProfile.arcAng);
  double deltaPos[3]={0}, residue[3]={0}, prevResidue[3]={0};
  double deltaPos_residue[3]={0};
  double prevPos[3]={0}, steps[3]={0};
  double abs_sum_steps[3]={0};
  double totalDistance[3]={0};
  //double dist[nAng+1] ;
  int cnt_zeros[3]={0};
  int index_step[3]={0};

  double deltaT_per_1deg = 1.0/circleProfile.speed;
  double abs_step[3]={0};
  double abs_step_1[3]={0},abs_step_residue [3]={0};
  //int acc_dec_area[3] = {10};//round(fabs(nAng)*5.0/100.0); // accelleration area from start (x%)
  double deltaT = deltaT_per_1deg;
  int acc_area[3] = {10};
  bool bCheckSum=true;
  // bool bDebug=0;
  double sum_deltaPos[3]={0}, sum_stepPos[3]={0};
  double diffSteps=0;
/////////////////////////////////////////////////////////////////////////
// Calculation for index 0
  if(!arcMotionIK(circleProfile.radius, circleProfile.cenPosX, circleProfile.cenPosY, circleProfile.EETheta*DEG2RAD, 0))
  {
    //if(bDebug) printf("!!!! NO IK SOLUTION !!!!\n");
    return -1;
  }
   prevPos[0] = kinParam.L;// Calculated by arcMotionIK (initial deltaPosue)
   prevPos[1] = kinParam.t1;
   prevPos[2] = kinParam.t2;
  //////////////////////////////////////////////////////////////////////
  // -- Start to calcute the number of pulses, delay time counts (Cn) and direction of motors
  for (int i = 0; i < circleProfile.arcAng; i++)
  {
    
    //-------------------------------------------------------//
    
    //////////////////////////////////////////////////////////
    // Calculate the I.K.
    if(!arcMotionIK(circleProfile.radius, circleProfile.cenPosX, circleProfile.cenPosY, circleProfile.EETheta*DEG2RAD ,  (i+1)*DEG2RAD * rotDir))
    {
      // if(bDebug) printf("!!!! NO IK SOLUTION !!!!\n");
      return -1;
    }
    ////////////////////////////////////////////////
    // Handing 3 motors...
    for(int CH=0; CH<3; CH++)
    {
      //////////////////////////////////////////////////
    // // Handling Acceleration and Deceleration by adding more time
    //   if(i<10) {
    //     deltaT = deltaT_per_1deg*double(acc_area[CH])*0.5;
    //     acc_area[CH]--;
    //   }
    // else if(i> (nAng-10)) {
    //     deltaT = deltaT_per_1deg*double(acc_area[CH])*0.5;
    //     acc_area[CH]++;
    // }
    // else {
    //     deltaT = deltaT_per_1deg;
    //     acc_area[CH]=2;
    //   }
      /////////////////////////////////////////
      // Calculate Delta Position
      // 0: X - Linear Motion (mm)
      if(CH==0) {
        deltaPos[CH] = (kinParam.L - prevPos[CH]); // [mm]
        totalDistance[CH] += abs(deltaPos[CH]);
        prevPos[CH] = kinParam.L;
      }
      /////////////////////////////////////////
      // 1: R1 - Rotation Motion (rad)
      else if(CH==1) {
        deltaPos[CH] = (kinParam.t1 - prevPos[CH]); // [mm]
        totalDistance[CH] += abs(deltaPos[CH]);
        prevPos[CH] = kinParam.t1;
      }
      /////////////////////////////////////////
      // 2: R2 - Rotation Motion (rad)
      else if(CH==2) {
        deltaPos[CH] = (kinParam.t2 - prevPos[CH]); // [mm]
        totalDistance[CH] += abs(deltaPos[CH]);
        prevPos[CH] = kinParam.t2;
      }
      // "sum_deltaPos[CH]"" is for checking if there is difference between real step count and calculate count...
      sum_deltaPos[CH] += deltaPos[CH];
    //%---------------------------------------------------------------------//
    //% Only accept more than one step if not, calculate the residue and pass
    //% it to next step...
      if(i==0) {
          deltaPos_residue[CH] =  deltaPos[CH];
      } else {
          deltaPos_residue[CH] =  deltaPos[CH] + prevResidue[CH]; // Add Residue
      }
      
      abs_step[CH] = abs(DIST2STEP[CH]*deltaPos_residue[CH]);
      if(abs_step[CH]>=1.0) {
        abs_step_1[CH] = round(abs_step[CH]);
        abs_step_residue[CH] = abs_step[CH] - abs_step_1[CH];
        steps[CH] = SIGN(deltaPos[CH])*abs_step_1[CH];
        residue[CH] = SIGN(deltaPos[CH]) * abs_step_residue[CH] * STEP2DIST[CH];
      }
      else {
        steps[CH]=0;
        residue[CH] =deltaPos_residue[CH]; 
      }
      prevResidue[CH] = residue[CH];
      abs_sum_steps[CH] += abs(steps[CH]);

      // prevResidue = SIGN(deltaPos) * (incPos - abs_sum_steps[CH]*STEP2DIST[CH]);
      
      ///////////////////////////////////////////////

      // if(bDebug) printf("---- step[%d] --- %f, abs_step=%f, val=%f\n",i,steps[CH],abs_step[CH], deltaPos_residue[CH]); 

      //--------------------------------------------------------------------//
      // Based on number of steps, calcuate Cn(count for delay) and direction
      // of motor and number of steps for this duration...
      if (steps[CH] == 0) {
        cnt_zeros[CH]++;
      }
      else
      {
        if(cnt_zeros[CH]>0){
          kinData[CH].motionData[index_step[CH]].Cn = round(tick_freq * deltaT * cnt_zeros[CH] / abs(steps[CH]));
          // if(bDebug) printf(" -------- zeros[%d], cnt_zeros=%d, Cn=%d\n", index_step[CH], cnt_zeros[CH], motionData[CH][index_step[CH]].Cn);
          cnt_zeros[CH]=0;
        } else {
            kinData[CH].motionData[index_step[CH]].Cn = round(tick_freq * deltaT / abs(steps[CH])); 
            
        }
        kinData[CH].motionData[index_step[CH]].steps = abs((steps[CH]));
        kinData[CH].motionData[index_step[CH]].dir = SIGN(deltaPos[CH]); // stepper motor direction...
        // "sum_stepPos[CH]"" is for checking if there is difference between real step count and calculate count...
        sum_stepPos[CH] += double(kinData[CH].motionData[index_step[CH]].steps)*double(kinData[CH].motionData[index_step[CH]].dir);

        index_step[CH]++;

        // if(bDebug) printf("--- Cn[%d]: %d, %d, dir=%d\n", 
        //   index_step[CH], motionData[CH][index_step[CH]-1].steps, motionData[CH][index_step[CH]-1].Cn,   motionData[CH][index_step[CH]-1].dir ); //[mm]
      }

      //%----------------------------------------------------------------------
      //% Since the motor direction is not changing right time due to the
      //% numerical error, we need to correct it...
      //% if diffSteps is larger than 0.5, compensate it by adding one more step
      //% with correcting the direction of motor...
      // !!!!! NOT WORKING HERE!!!!!
      // if(index_step[CH]>0) {
      //    diffSteps = sum_deltaPos[CH]*DIST2STEP[CH]- sum_stepPos[CH];
      //    if(abs(round(diffSteps))>=1.0) {
      //       motionData[CH][index_step[CH]].Cn = tick_freq * deltaT / abs(round(diffSteps));
      //       if(diffSteps>0) motionData[CH][index_step[CH]].dir =  1;
      //       else            motionData[CH][index_step[CH]].dir = -1;
      //       index_step[CH]++;
      //    }

      // }


      //%-----------------------------------------------------------------------------
      //% Lastly check if there is unfinised pulse, if so add one more pulse
      //% with previous direction...
      if((int(circleProfile.arcAng)-1)==i) {
        kinData[CH].totalSteps = int32_t(round(totalDistance[CH] * DIST2STEP[CH]));
        if(cnt_zeros[CH]>0)
        {
          kinData[CH].motionData[index_step[CH]].Cn = round(tick_freq * deltaT * (cnt_zeros[CH])) ;
          kinData[CH].motionData[index_step[CH]].steps=1;
          kinData[CH].motionData[index_step[CH]].dir = kinData[CH].motionData[index_step[CH]-1].dir; // stepper motor direction...
          abs_sum_steps[CH] += 1;
          steps[CH] = kinData[CH].totalSteps - (abs_sum_steps[CH]);
          if(steps[CH]>0){
            kinData[CH].motionData[index_step[CH]].Cn = round(tick_freq * deltaT * cnt_zeros[CH] / abs(steps[CH])) ;
            kinData[CH].motionData[index_step[CH]].steps=abs(steps[CH]);
            kinData[CH].motionData[index_step[CH]].dir = kinData[CH].motionData[index_step[CH]-1].dir; // stepper motor direction...
          }
          abs_sum_steps[CH] += abs(steps[CH]);

          //if(bDebug) printf("last--- Cn[%d]: %d, %d,  cnt_zeros=%d\n", index_step[CH], motionData[CH][index_step[CH]].steps, motionData[CH][index_step[CH]].Cn,   cnt_zeros[CH]); //[mm]
          index_step[CH]++;
        }
        else {
          steps[CH] = kinData[CH].totalSteps - abs_sum_steps[CH];
          if(steps[CH]>0){
            kinData[CH].motionData[index_step[CH]].Cn = round(tick_freq * deltaT / abs(steps[CH])) ;
            kinData[CH].motionData[index_step[CH]].steps=abs(steps[CH]);
            kinData[CH].motionData[index_step[CH]].dir = kinData[CH].motionData[index_step[CH]-1].dir; // stepper motor direction...
            abs_sum_steps[CH] += abs(steps[CH]);
            index_step[CH]++;
          }
        }// ----------else
         kinData[CH].dataSize = index_step[CH];
        //  kinData[CH].dataSize =  uint16_t(abs(circleProfile.cenPosY));//test
        //  kinData[CH].totalSteps = uint32_t( abs_sum_steps[CH]);//test
        if(kinData[CH].totalSteps!=abs_sum_steps[CH]) bCheckSum=false;
        // if(bDebug) printf( "totalsteps=%d, abs_sum_steps=%d,  diff(totalSteps-abs_sum_steps)=%f, motionDataSize=%d\n", 
        //                 totalSteps[CH], int(abs_sum_steps[CH]), totalSteps[CH]-abs_sum_steps[CH], motionDataSize[CH]);

        // if(bDebug) printf("-- Joint ID = %d::result ----------------- totalDistance=%4.3f, totalsteps=%d\n\n\n\n", CH, totalDistance[CH], totalSteps[CH]);

      }// --------  if((nAng-1)==i) { 
      /*
      if((nAng-1)==i) {
        totalSteps[CH] = round(totalDistance[CH] * DIST2STEP[CH]);
        if(cnt_zeros[CH]>0)
        {
          motionData[CH][index_step[CH]].Cn = tick_freq * deltaT * (cnt_zeros[CH]) ;
          motionData[CH][index_step[CH]].steps=1;
          motionData[CH][index_step[CH]].dir = motionData[CH][index_step[CH]-1].dir; // stepper motor direction...
          abs_sum_steps[CH] += 1;

          // if(bDebug) printf("last--- Cn[%d]: %d, %d,  cnt_zeros=%d\n", index_step[CH], motionData[CH][index_step[CH]].steps, motionData[CH][index_step[CH]].Cn,   cnt_zeros[CH]); //[mm]
          index_step[CH]++;
        }
          motionDataSize[CH] = index_step[CH];
          double diffSteps = sum_deltaPos[CH]*DIST2STEP[CH]- sum_stepPos[CH];
          if(abs(diffSteps)>0.001) bCheckSum=false;
          // if(bDebug) printf( "diffSteps=%2.3f, totalsteps=%d, abs_sum_steps=%d,  diff(totalSteps-abs_sum_steps)=%f, motionDataSize=%d\n", 
          //                 diffSteps, totalSteps[CH], int(abs_sum_steps[CH]), totalSteps[CH]-abs_sum_steps[CH], motionDataSize[CH]);

          // if(bDebug) printf("-- Joint ID = %d::result ----------------- totalDistance=%4.3f, totalsteps=%d\n\n\n\n", CH, totalDistance[CH], totalSteps[CH]);
      }// --------  if((nAng-1)==i) { 
      */
    }// --------- for(CH)
  }

  // compensate the extra  or missing couple of steps on the steps because of round off error...
  
   if(bCheckSum)
    return 1; // Calculation is right
   else return 0; // total steps and sum of step is different (Calculation is wrong)...
}
#endif
#if 0
int MKVelProfile::createArcMotion(double vel, double radius, double cenPos[2], double EndEffectorAng , int nAng)
{
  // ++ deltaT_Lx is duration when EE rotate 1deg ++
  //int CH=1;
  double DIST2STEP[4]={16.0, 1527.89, 763.944,80.0}; //[X, R1, R2, Z]
  double STEP2DIST[4]={1.0/16.0, 1.0/1527.89, 1.0/763.944, 1.0/80.0}; //[X, R1, R2, Z]
  
  int8_t rotDir = SIGN(nAng);
  double deltaPos[3]={0}, residue[3]={0}, prevResidue[3]={0};
  double deltaPos_residue[3]={0};
  double prevPos[3]={0}, steps[3]={0};
  double abs_sum_steps[3]={0};
  double totalDistance[3]={0};
  //double dist[nAng+1] ;
  int cnt_zeros[3]={0};
  int index_step[3]={0};



double deltaT_per_1deg = 1.0/vel;
double abs_step[3]={0};
double abs_step_1[3]={0},abs_step_residue [3]={0};
int acc_dec_area = 8;//round(fabs(nAng)*5.0/100.0); // accelleration area from start (x%)
double deltaT = deltaT_per_1deg;
int acc_area=acc_dec_area;
bool bCheckSum=true;
bool bDebug=0;
/////////////////////////////////////////////////////////////////////////
// Calculation for index 0
  if(!arcMotionIK(radius, cenPos, EndEffectorAng*DEG2RAD, 0))
  {
    if(bDebug) printf("!!!! NO IK SOLUTION !!!!\n");
    return -1;
  }
   prevPos[0] = kinParam.L;// Calculated by arcMotionIK (initial deltaPosue)
   prevPos[1] = kinParam.t1;
   prevPos[2] = kinParam.t2;
  //////////////////////////////////////////////////////////////////////

  for (int i = 0; i < nAng; i++)
  {
    //////////////////////////////////////////////////
    // Handling Acceleration and Deceleration by adding more time
  //   if(i<acc_dec_area) {
  //      deltaT = deltaT_per_1deg*acc_area;
  //      acc_area--;
  //   }
  //  else if(i> (nAng-acc_dec_area)) {
  //      deltaT = deltaT_per_1deg*acc_area;
  //      acc_area++;
  //  }
  //  else {
  //      deltaT = deltaT_per_1deg;
  //      acc_area=2;
  //   }
    //-------------------------------------------------------//
    
    //////////////////////////////////////////////////////////
    // Calculate the I.K.
    if(!arcMotionIK(radius, cenPos, EndEffectorAng*DEG2RAD ,  (i+1)*DEG2RAD * rotDir))
    {
      if(bDebug) printf("!!!! NO IK SOLUTION !!!!\n");
      return -1;
    }
    for(int CH=0; CH<3; CH++)
    {
      /////////////////////////////////////////
      // Calculate Delta Position
      // 0: X - Linear Motion (mm)
      if(CH==0) {
        deltaPos[CH] = (kinParam.L - prevPos[CH]); // [mm]
        totalDistance[CH] += abs(deltaPos[CH]);
        prevPos[CH] = kinParam.L;
      }
      /////////////////////////////////////////
      // 1: R1 - Rotation Motion (rad)
      else if(CH==1) {
        deltaPos[CH] = (kinParam.t1 - prevPos[CH]); // [mm]
        totalDistance[CH] += abs(deltaPos[CH]);
        prevPos[CH] = kinParam.t1;
      }
      /////////////////////////////////////////
      // 2: R2 - Rotation Motion (rad)
      else if(CH==2) {
        deltaPos[CH] = (kinParam.t2 - prevPos[CH]); // [mm]
        totalDistance[CH] += abs(deltaPos[CH]);
        prevPos[CH] = kinParam.t2;
      }
    // incPos = incPos + abs( deltaPos);
      if(i==0) {
          deltaPos_residue[CH] =  deltaPos[CH];
      } else {
          deltaPos_residue[CH] =  deltaPos[CH] + prevResidue[CH]; // Add Residue
      }
      
      abs_step[CH] = abs(DIST2STEP[CH]*deltaPos_residue[CH]);
      if(abs_step[CH]>=1.0) {
        abs_step_1[CH] = round(abs_step[CH]);
        abs_step_residue[CH] = abs_step[CH] - abs_step_1[CH];
        steps[CH] = SIGN(deltaPos[CH])*abs_step_1[CH];
        residue[CH] = SIGN(deltaPos[CH]) * abs_step_residue[CH] * STEP2DIST[CH];
      }
      else {
        steps[CH]=0;
        residue[CH] =deltaPos_residue[CH]; 
      }
      prevResidue[CH] = residue[CH];
      abs_sum_steps[CH] += abs(steps[CH]);

      // prevResidue = SIGN(deltaPos) * (incPos - abs_sum_steps[CH]*STEP2DIST[CH]);
      
      ///////////////////////////////////////////////

      if(bDebug) printf("---- step[%d] --- %f, abs_step=%f, val=%f\n",i,steps[CH],abs_step[CH], deltaPos_residue[CH]); 
      // duration for each step(Cn Timer Counter Input: larger number is more delay by counting)
      if (steps[CH] == 0) {
        cnt_zeros[CH]++;
      }
      else
      {
        if(cnt_zeros[CH]>0){
          motionData[CH][index_step[CH]].Cn = (tick_freq * deltaT * cnt_zeros[CH] / abs(steps[CH]));
          if(bDebug) printf(" -------- zeros[%d], cnt_zeros=%d, Cn=%d\n", index_step[CH], cnt_zeros[CH], motionData[CH][index_step[CH]].Cn);
          cnt_zeros[CH]=0;
        } else {
            motionData[CH][index_step[CH]].Cn = (tick_freq * deltaT / abs(steps[CH])); //(1/tick_freq)*deltaT_per_1deg/abs(steps);
            
        }
        motionData[CH][index_step[CH]].steps = abs((steps[CH]));
        motionData[CH][index_step[CH]].dir = SIGN(deltaPos[CH]); // stepper motor direction...
        index_step[CH]++;

        if(bDebug) printf("--- Cn[%d]: %d, %d, dir=%d\n", 
          index_step[CH], motionData[CH][index_step[CH]-1].steps, motionData[CH][index_step[CH]-1].Cn,   motionData[CH][index_step[CH]-1].dir ); //[mm]
      }
        
        
      if((nAng-1)==i) {
        totalSteps[CH] = round(totalDistance[CH] * DIST2STEP[CH]);
        if(cnt_zeros[CH]>0)
        {
          motionData[CH][index_step[CH]].Cn = tick_freq * deltaT * (cnt_zeros[CH]) ;
          motionData[CH][index_step[CH]].steps=1;
          motionData[CH][index_step[CH]].dir = motionData[CH][index_step[CH]-1].dir; // stepper motor direction...
          abs_sum_steps[CH] += 1;
          steps[CH] = totalSteps[CH] - (abs_sum_steps[CH]);
          if(steps[CH]>0){
            motionData[CH][index_step[CH]].Cn = tick_freq * deltaT * cnt_zeros[CH] / abs(steps[CH]) ;
            motionData[CH][index_step[CH]].steps=abs(steps[CH]);
            motionData[CH][index_step[CH]].dir = motionData[CH][index_step[CH]-1].dir; // stepper motor direction...
          }
          abs_sum_steps[CH] += abs(steps[CH]);

          if(bDebug) printf("last--- Cn[%d]: %d, %d,  cnt_zeros=%d\n", index_step[CH], motionData[CH][index_step[CH]].steps, motionData[CH][index_step[CH]].Cn,   cnt_zeros[CH]); //[mm]
          index_step[CH]++;
        }
        else {
          steps[CH] = totalSteps[CH] - abs_sum_steps[CH];
          if(steps[CH]>0){
            motionData[CH][index_step[CH]].Cn = tick_freq * deltaT / abs(steps[CH]) ;
            motionData[CH][index_step[CH]].steps=abs(steps[CH]);
            motionData[CH][index_step[CH]].dir = motionData[CH][index_step[CH]-1].dir; // stepper motor direction...
            abs_sum_steps[CH] += abs(steps[CH]);
            index_step[CH]++;
          }
        }// ----------else
         motionDataSize[CH] = index_step[CH];
        if(totalSteps[CH]!=abs_sum_steps[CH]) bCheckSum=false;
        if(bDebug) printf( "totalsteps=%d, abs_sum_steps=%d,  diff(totalSteps-abs_sum_steps)=%f, motionDataSize=%d\n", 
                        totalSteps[CH], int(abs_sum_steps[CH]), totalSteps[CH]-abs_sum_steps[CH], motionDataSize[CH]);

        if(bDebug) printf("-- Joint ID = %d::result ----------------- totalDistance=%4.3f, totalsteps=%d\n\n\n\n", CH, totalDistance[CH], totalSteps[CH]);

      }// --------  if((nAng-1)==i) { 
    }// --------- for(CH)
  }

  // compensate the extra  or missing couple of steps on the steps because of round off error...
  
   if(bCheckSum)
    return 1; // Calculation is right
   else return 0; // total steps and sum of step is different (Calculation is wrong)...
  // // mkSerial.println(msg);
  
  //activatedEE = 1;
}
#endif
#if 0
int MKVelProfile::createArcMotion(double vel, double radius, double cenPos[2], double EndEffectorAng , int nAng)
{
  // ++ deltaT_Lx is duration when EE rotate 1deg ++
  int CH=0;
  double DIST2STEP[4]={16.0, 1527.89, 763.944,80.0}; //[X, R1, R2, Z]
  double STEP2DIST[4]={1.0/16.0, 1.0/1527.89, 1.0/763.944, 1.0/80.0}; //[X, R1, R2, Z]
  
  int8_t rotDir = SIGN(nAng);
  double deltaPos=0, residue=0, prevResidue=0;
  double deltaPos_residue=0;
  double prevPos=0, steps=0;
  double abs_sum_steps[3]={0};
  double totalDistance[3]={0};
  //double dist[nAng+1] ;
  uint8_t cnt_zeros=0;
  uint16_t index_step=0;



double deltaT_per_1deg = 1.0/vel;
double abs_step=0;
double abs_step_1=0,abs_step_residue = 0;
//int acc_dec_area = round(fabs(nAng)*5.0/100.0); // accelleration area from start (x%)
double deltaT = deltaT_per_1deg;
//int acc_area=acc_dec_area;
double sum_deltaPos=0, sum_stepPos=0;
//double incPos = 0;// increamental joint position
bool bCheckSum=true;
/////////////////////////////////////////////////////////////////////////
// Calculation for index 0
  if(!arcMotionIK(radius, cenPos, EndEffectorAng*DEG2RAD, 0))
  {
   
    return -1;
  }
  if(CH==0) prevPos = kinParam.L;// Calculated by arcMotionIK (initial deltaPosue)
  else if(CH==1) prevPos = kinParam.t1;
  else if(CH==2) prevPos = kinParam.t2;
  //////////////////////////////////////////////////////////////////////
  // memset(totalDistance, 0,sizeof(double)*3);
  // memset(abs_sum_steps, 0,sizeof(double)*3);
  // memset(prevResidue, 0,sizeof(double)*3);
  for (int i = 0; i < nAng; i++)
  {
    //////////////////////////////////////////////////
    // Handling Acceleration and Deceleration by adding more time
  //   if(i<acc_dec_area) {
  //      deltaT = deltaT_per_1deg*acc_area;
  //      acc_area--;
  //   }
  //  else if(i> (nAng-acc_dec_area)) {
  //      deltaT = deltaT_per_1deg*acc_area;
  //      acc_area++;
  //  }
  //  else {
  //      deltaT = deltaT_per_1deg;
  //      acc_area=2;
  //   }
    //-------------------------------------------------------//
    
    //////////////////////////////////////////////////////////
    // Calculate the I.K.
    if(!arcMotionIK(radius, cenPos, EndEffectorAng*DEG2RAD ,  (i+1)*DEG2RAD * rotDir))
    {
      
      return -1;
    }
    /////////////////////////////////////////
    // Calculate Delta Position
    // 0: X - Linear Motion (mm)
    if(CH==0) {
      deltaPos = (kinParam.L - prevPos); // [mm]
      totalDistance[CH] += abs(deltaPos);
      prevPos = kinParam.L;
    }
    /////////////////////////////////////////
    // 1: R1 - Rotation Motion (rad)
    else if(CH==1) {
      deltaPos = (kinParam.t1 - prevPos); // [mm]
      totalDistance[CH] += abs(deltaPos);
      prevPos = kinParam.t1;
    }
    /////////////////////////////////////////
    // 2: R2 - Rotation Motion (rad)
    else if(CH==2) {
      deltaPos = (kinParam.t2 - prevPos); // [mm]
      totalDistance[CH] += abs(deltaPos);
      prevPos = kinParam.t2;
    }
    // "sum_deltaPos[CH]"" is for checking if there is difference between real step count and calculate count...
    sum_deltaPos += deltaPos;
    //%---------------------------------------------------------------------//
    //% Only accept more than one step if not, calculate the residue and pass
    //% it to next step...
    if(i==0) {
        deltaPos_residue =  deltaPos;
    } else {
        deltaPos_residue =  deltaPos + prevResidue; // Add Residue
    }
    
    abs_step = abs(DIST2STEP[CH]*deltaPos_residue);
    if(abs_step>=1.0) {
      abs_step_1 = round(abs_step);
      abs_step_residue = abs_step - abs_step_1;
      steps = SIGN(deltaPos)*abs_step_1;
      residue = SIGN(deltaPos) * abs_step_residue * STEP2DIST[CH];
    }
    else {
      steps=0;
      residue =deltaPos_residue; 
    }
    prevResidue = residue;
    abs_sum_steps[CH] += abs(steps);

    //--------------------------------------------------------------------//
    // Based on number of steps, calcuate Cn(count for delay) and direction
    // of motor and number of steps for this duration...
    if (steps == 0) {
      cnt_zeros++;
    }
    else
    {
      if(cnt_zeros>0){
        motionData[CH][index_step].Cn = (tick_freq * deltaT * cnt_zeros / fabs(steps));
        cnt_zeros=0;
      } else {
          motionData[CH][index_step].Cn = (tick_freq * deltaT / fabs(steps)); //(1/tick_freq)*deltaT_per_1deg/abs(steps);
          
      }
      motionData[CH][index_step].steps = abs((steps));
      motionData[CH][index_step].dir = SIGN(deltaPos); // stepper motor direction...
      // "sum_stepPos[CH]"" is for checking if there is difference between real step count and calculate count...
      sum_stepPos += double(motionData[CH][index_step].steps) * double(motionData[CH][index_step].dir);
      index_step++;

    }
    //%----------------------------------------------------------------------
    //% Since the motor direction is not changing right time due to the
    //% numerical error, we need to correct it...
    //% if diffSteps is larger than 0.5, compensate it by adding one more step
    //% with correcting the direction of motor...
    if(index_step>0) {
        double diffSteps = sum_deltaPos*DIST2STEP[CH]- sum_stepPos;
        if(round(fabs(diffSteps))>=1.0) {
         motionData[CH][index_step].Cn = tick_freq * deltaT / abs(round(diffSteps));
          if(diffSteps>0) motionData[CH][index_step].dir =  1;
          else            motionData[CH][index_step].dir = -1;
          index_step++;
        }

    }  
    //%-----------------------------------------------------------------------------
    //% Lastly check if there is unfinised pulse, if so add one more pulse
    //% with previous direction...   
    if((nAng-1)==i) {
       totalSteps[CH] = round(totalDistance[CH] * DIST2STEP[CH]);
      if(cnt_zeros>0)
      {
        motionData[CH][index_step].Cn = tick_freq * deltaT * (cnt_zeros) ;
        motionData[CH][index_step].steps=1;
        motionData[CH][index_step].dir = motionData[CH][index_step-1].dir; // stepper motor direction...
        abs_sum_steps[CH] += 1;
        index_step++;
      }
          motionDataSize[CH] = index_step+1;
          double diffSteps = sum_deltaPos*DIST2STEP[CH]- sum_stepPos;
          if(abs(diffSteps)>0.001) bCheckSum=false;

        
      
    }// --------  if((nAng-1)==i) { 

    
  }


 
 
   if(bCheckSum)
    return 1; // Calculation is right
   else return 0; // total steps and sum of step is different (Calculation is wrong)...
  // // mkSerial.println(msg);
  
  //activatedEE = 1;
}
#endif
#if 0 
int MKVelProfile::createArcMotion(double vel/*[deg/sec]*/, double radius, double cenPos[2]/* [mm] */, double EndEffectorAng /* [deg] */, int nAng/* [deg] */)
#endif
/*
{
  // ++ deltaT_Lx is duration when EE rotate 1deg ++
  int CH=1;
  double DIST2STEP[4]={16.0, 1527.89, 763.944,80.0}; //[X, R1, R2, Z]
  double STEP2DIST[4]={1.0/16.0, 1.0/1527.89, 1.0/763.944, 1.0/80.0}; //[X, R1, R2, Z]
  
  int8_t rotDir = SIGN(nAng);
  double deltaPos[3]={0}, residue[3]={0}, prevResidue[3]={0};
  double prevPos[3] = {0}, steps[3] = {0};
  uint32_t abs_sum_steps[3] = {0};
  //double dist[nAng+1] ;
  int cnt_zeros[3]={0};
  int index_step[3]={0};
  double totalDistance[3] = {0};


double deltaT_per_1deg = 1.0/vel;
double abs_step[3] = {0};
double abs_step_1[3] = {0},abs_step_residue[3] = {0};
int acc_dec_area = round(fabs(nAng)*5.0/100.0); // accelleration area from start (x%)
double deltaT = deltaT_per_1deg;
int acc_area=acc_dec_area;
bool bDebug=0;
/////////////////////////////////////////////////////////////////////////
// Calculation for index 0
  if(!arcMotionIK(radius, cenPos, EndEffectorAng*DEG2RAD, 0))
  {
    if(bDebug) printf("!!!! NO IK SOLUTION !!!!\n");
    return -1;
  }
  if(CH==0) prevPos[CH] = kinParam.L;// Calculated by arcMotionIK (initial deltaPos[CH]ue)
  else if(CH==1) prevPos[CH] = kinParam.t1;
  else if(CH==2) prevPos[CH] = kinParam.t2;
  //////////////////////////////////////////////////////////////////////
  memset(totalDistance, 0,sizeof(double)*3);
 
  for (int i = 0; i < nAng; i++)
  {
    if(i<acc_dec_area) {
       deltaT = deltaT_per_1deg*double(acc_area)*0.5;
       acc_area--;
    }
   else if(i> (nAng-acc_dec_area)) {
       deltaT = deltaT_per_1deg*double(acc_area)*0.5;
       acc_area++;
   }
   else {
       deltaT = deltaT_per_1deg;
       acc_area=2;
    }
    if(!arcMotionIK(radius, cenPos, EndEffectorAng*DEG2RAD ,  (i+1)*DEG2RAD * rotDir))
    {
      if(bDebug) printf("!!!! NO IK SOLUTION !!!!\n");
      return -1;
    }
    /////////////////////////////////////////
    // 0: X - Linear Motion (mm)
    if(CH==0) {
      deltaPos[CH] = (kinParam.L - prevPos[CH])+ prevResidue[CH]; // [mm]
      totalDistance[CH] += abs(kinParam.L - prevPos[CH]);
      prevPos[CH] = kinParam.L;
    }
    /////////////////////////////////////////
    // 1: R1 - Rotation Motion (rad)
    else if(CH==1) {
      deltaPos[CH] = (kinParam.t1 - prevPos[CH])+ prevResidue[CH]; // [mm]
      totalDistance[CH] += abs(kinParam.t1 - prevPos[CH]);
      prevPos[CH] = kinParam.t1;
    }
    /////////////////////////////////////////
    // 2: R2 - Rotation Motion (rad)
    else if(CH==2) {
      deltaPos[CH] = (kinParam.t2 - prevPos[CH])+ prevResidue[CH]; // [mm]
      totalDistance[CH] += abs(kinParam.t2 - prevPos[CH]);
      prevPos[CH] = kinParam.t2;
    }
    abs_step[CH] = abs(DIST2STEP[CH]*deltaPos[CH]);

    if(abs_step[CH]>=1.0) {
      
       abs_step_1[CH] = round(abs_step[CH]);
       abs_step_residue[CH] = abs_step[CH] - abs_step_1[CH];

      steps[CH] = SIGN(deltaPos[CH])*abs_step_1[CH];

      //remain[i] = SIGN(deltaPos[CH]) * abs_step_residue * STEP2DIST[CH];
      residue[CH] = SIGN(deltaPos[CH]) * abs_step_residue[CH] * STEP2DIST[CH];
    }
    else {
      steps[CH]=0;
      residue[CH] =deltaPos[CH]; 
    }
    prevResidue[CH] = residue[CH];

    // steps[CH] = round(steps[CH]);
    abs_sum_steps[CH]+=abs((steps[CH]));
    ///////////////////////////////////////////////

     if(bDebug) printf("---- step[%d]=%3.2f, abs_step=%f, deltaPos=%f\n",i,steps[CH],abs_step[CH], deltaPos[CH]); 
    // duration for each step(Cn Timer Counter Input: larger number is more delay by counting)
    if (steps[CH] == 0) {
      cnt_zeros[CH]++;
    }
    else
    {
      if(cnt_zeros[CH]>0){
        motionData[CH][index_step[CH]].Cn = (tick_freq * deltaT * double(cnt_zeros[CH]) / abs(steps[CH]));
        if(bDebug) printf(" -------- zeros[%d], cnt_zeros=%d, Cn=%d\n", index_step[CH], cnt_zeros[CH], motionData[CH][index_step[CH]].Cn);
        cnt_zeros[CH]=0;
      } else {
          motionData[CH][index_step[CH]].Cn = (tick_freq * deltaT / abs(steps[CH])); //(1/tick_freq)*deltaT_per_1deg/abs(steps);
          
      }
      motionData[CH][index_step[CH]].steps = abs((steps[CH]));
      motionData[CH][index_step[CH]].dir = SIGN(steps[CH]); // stepper motor direction...
      // abs_sum_steps[CH]+=motionData[CH][index_step[CH]].steps;
      index_step[CH]++;

      if(bDebug) printf("--- Cn[%d]=%d: steps=%d,  dir=%d\n", 
        index_step[CH], motionData[CH][index_step[CH]-1].Cn, motionData[CH][index_step[CH]-1].steps,   motionData[CH][index_step[CH]-1].dir ); //[mm]
    }
      
    // Last pulse handling   
    // if((nAng-1)==i) {
    //   if(cnt_zeros>0)
    //   {
    //     motionData[CH][index_step[CH]].Cn = tick_freq * deltaT *  double(cnt_zeros[CH]) ;
    //     motionData[CH][index_step[CH]].steps=1;
    //     motionData[CH][index_step[CH]].dir = motionData[CH][index_step[CH]-1].dir; // stepper motor direction...
        
    //     // if(bDebug) printf("last--- Cn[%d]: %d, %d,  cnt_zeros=%d\n", index_step, motionData[CH][index_step].steps, motionData[CH][index_step].Cn,   cnt_zeros); //[mm]
    //     index_step[CH]++;
    //   }
    //   else {
    //         totalSteps[CH] = round(totalDistance[CH] * DIST2STEP[CH]);
    //         steps[CH] = totalSteps[CH] - abs_sum_steps[CH];
    //         if(steps[CH]>0) {
    //             motionData[CH][index_step[CH]].Cn = (tick_freq * deltaT * (cnt_zeros[CH])/ abs(steps[CH])) ;
    //             motionData[CH][index_step[CH]].steps=abs((steps[CH]));
    //             motionData[CH][index_step[CH]].dir = motionData[CH][index_step[CH]-1].dir; // stepper motor direction...
    //             abs_sum_steps[CH]+=abs((steps[CH]));
    //             index_step[CH]++;
    //         }
    //   }
    // }
    /////////////////////////////////////////////////
    if((nAng-1)==i) {
      totalSteps[CH] = round(totalDistance[CH] * DIST2STEP[CH]);
      steps[CH] = totalSteps[CH] - abs_sum_steps[CH];
      if(steps[CH]>0) {
        if(cnt_zeros[CH]>0)
        {
          motionData[CH][index_step[CH]].Cn = (tick_freq * deltaT * (cnt_zeros[CH])/ abs(steps[CH])) ;
        }
        else {
             motionData[CH][index_step[CH]].Cn = (tick_freq * deltaT / abs(steps[CH])) ;
        }
        //motionData[CH][index_step[CH]].Cn = round(tick_freq * deltaT * (cnt_zeros[CH])/ abs(steps[CH])) ;
        motionData[CH][index_step[CH]].steps=abs((steps[CH]));
        motionData[CH][index_step[CH]].dir = motionData[CH][index_step[CH]-1].dir; // stepper motor direction...
        
        if(bDebug) printf("last--- Cn[%d]: last steps=%2.2f, %d, Cn=%d,  cnt_zeros=%d\n", 
        index_step[CH], steps[CH], motionData[CH][index_step[CH]].steps, motionData[CH][index_step[CH]].Cn,   cnt_zeros[CH]); //[mm]
        index_step[CH]++;
        //abs_sum_steps[CH]+=abs((steps[CH]));
      } // ----- if(steps[CH]>0) {
    }

    
  }

  // compensate the extra  or missing couple of steps on the steps because of round off error...
  motionDataSize[CH] = index_step[CH];

  // totalSteps[CH] = round(totalDistance[CH] * DIST2STEP[CH]);
  if(bDebug) printf( "totalsteps=%d, abs_sum_steps=%d, , diff(totalSteps-abs_sum_steps)=%d, nM=%d\n", 
  totalSteps[CH], abs_sum_steps[CH], totalSteps[CH]-abs_sum_steps[CH], motionDataSize[CH]);

   if(bDebug) printf("-- result ----------------- totalDistance=%4.3f, totalsteps=%d\n", totalDistance[CH], totalSteps[CH]);
   return (totalSteps[CH]-abs_sum_steps[CH]);
  //  if((totalSteps[CH]-abs_sum_steps[CH])==0)
  //   return 1; // Calculation is right
  //  else return 0; // total steps and sum of step is different (Calculation is wrong)...
  // // // mkSerial.println(msg);
  
  //activatedEE = 1;
}
*/
/////////////////////////////////////////////////////////////////////////////////
//int createArcMotion() ==>  Return: 
// -1: no IK Solution
//  0: step count is not matching from length of move 
//  1: Everything is fine...
/*
int MKVelProfile::createArcMotion(double vel, double radius, double cenPos[2], double EndEffectorAng , int nAng)
{
  // ++ deltaT_Lx is duration when EE rotate 1deg ++
  int CH=1;
  double DIST2STEP[4]={16.0, 1527.89, 763.944,80.0}; //[X, R1, R2, Z]
  double STEP2DIST[4]={1.0/16.0, 1.0/1527.89, 1.0/763.944, 1.0/80.0}; //[X, R1, R2, Z]
  
  int8_t rotDir = SIGN(nAng);
  double deltaPos[3]={0}, residue[3]={0}, prevResidue[3]={0};
  double prevPos[3] = {0}, steps[3] = {0};
  uint32_t abs_sum_steps[3] = {0};
  //double dist[nAng+1] ;
  int cnt_zeros[3]={0};
  int index_step[3]={0};
  double totalDistance[3] = {0};


double deltaT_per_1deg = 1.0/vel;
double abs_step[3] = {0};
double abs_step_1[3] = {0},abs_step_residue[3] = {0};
int acc_dec_area = round(fabs(nAng)*10.0/100.0); // accelleration area from start (x%)
double deltaT = deltaT_per_1deg;
int acc_area=acc_dec_area;
bool bDebug=0;
/////////////////////////////////////////////////////////////////////////
// Calculation for index 0
  if(!arcMotionIK(radius, cenPos, EndEffectorAng*DEG2RAD, 0))
  {
    if(bDebug) printf("!!!! NO IK SOLUTION !!!!\n");
    return -1;
  }
  if(CH==0) prevPos[CH] = kinParam.L;// Calculated by arcMotionIK (initial deltaPos[CH]ue)
  else if(CH==1) prevPos[CH] = kinParam.t1;
  else if(CH==2) prevPos[CH] = kinParam.t2;
  //////////////////////////////////////////////////////////////////////
  memset(totalDistance, 0,sizeof(double)*3);
 
  for (int i = 0; i < nAng; i++)
  {
    if(i<acc_dec_area) {
       deltaT = deltaT_per_1deg*acc_area;
       acc_area--;
    }
   else if(i> (nAng-acc_dec_area)) {
       deltaT = deltaT_per_1deg*acc_area;
       acc_area++;
   }
   else {
       deltaT = deltaT_per_1deg;
       acc_area=2;
    }
    if(!arcMotionIK(radius, cenPos, EndEffectorAng*DEG2RAD ,  (i+1)*DEG2RAD * rotDir))
    {
      if(bDebug) printf("!!!! NO IK SOLUTION !!!!\n");
      return -1;
    }
    /////////////////////////////////////////
    // 0: X - Linear Motion (mm)
    if(CH==0) {
      deltaPos[CH] = (kinParam.L - prevPos[CH])+ prevResidue[CH]; // [mm]
      totalDistance[CH] += abs(kinParam.L - prevPos[CH]);
      prevPos[CH] = kinParam.L;
    }
    /////////////////////////////////////////
    // 1: R1 - Rotation Motion (rad)
    else if(CH==1) {
      deltaPos[CH] = (kinParam.t1 - prevPos[CH])+ prevResidue[CH]; // [mm]
      totalDistance[CH] += abs(kinParam.t1 - prevPos[CH]);
      prevPos[CH] = kinParam.t1;
    }
    /////////////////////////////////////////
    // 2: R2 - Rotation Motion (rad)
    else if(CH==2) {
      deltaPos[CH] = (kinParam.t2 - prevPos[CH])+ prevResidue[CH]; // [mm]
      totalDistance[CH] += abs(kinParam.t2 - prevPos[CH]);
      prevPos[CH] = kinParam.t2;
    }
    abs_step[CH] = abs(DIST2STEP[CH]*deltaPos[CH]);

    if(abs_step[CH]>=1.0) {
      
       abs_step_1[CH] = round(abs_step[CH]);
       abs_step_residue[CH] = abs_step[CH] - abs_step_1[CH];

      steps[CH] = SIGN(deltaPos[CH])*abs_step_1[CH];

      //remain[i] = SIGN(deltaPos[CH]) * abs_step_residue * STEP2DIST[CH];
      residue[CH] = SIGN(deltaPos[CH]) * abs_step_residue[CH] * STEP2DIST[CH];
    }
    else {
      steps[CH]=0;
      residue[CH] =deltaPos[CH]; 
    }
    prevResidue[CH] = residue[CH];
    ///////////////////////////////////////////////

     if(bDebug) printf("---- step[%d] --- %f, abs_step=%f, val=%f\n",i,steps[CH],abs_step[CH], deltaPos[CH]); 
    // duration for each step(Cn Timer Counter Input: larger number is more delay by counting)
    if (steps[CH] == 0) {
      cnt_zeros[CH]++;
    }
    else
    {
      if(cnt_zeros[CH]>0){
        motionData[CH][index_step[CH]].Cn = round(tick_freq * deltaT_per_1deg * cnt_zeros[CH] / abs(steps[CH]));
        if(bDebug) printf(" -------- zeros[%d], cnt_zeros=%d, Cn=%d\n", index_step[CH], cnt_zeros[CH], motionData[CH][index_step[CH]].Cn);
        cnt_zeros[CH]=0;
      } else {
          motionData[CH][index_step[CH]].Cn = round(tick_freq * deltaT_per_1deg / abs(steps[CH])); //(1/tick_freq)*deltaT_per_1deg/abs(steps);
          
      }
      motionData[CH][index_step[CH]].steps = abs((steps[CH]));
      motionData[CH][index_step[CH]].dir = SIGN(steps[CH]); // stepper motor direction...
      abs_sum_steps[CH]+=motionData[CH][index_step[CH]].steps;
      index_step[CH]++;

      if(bDebug) printf("--- Cn[%d]: %d, %d, dir=%d\n", 
        index_step[CH], motionData[CH][index_step[CH]-1].steps, motionData[CH][index_step[CH]-1].Cn,   motionData[CH][index_step[CH]-1].dir ); //[mm]
    }
      
       
    if((nAng-1)==i) {
      if(cnt_zeros[CH]>0)
      {
      motionData[CH][index_step[CH]].Cn = tick_freq * deltaT_per_1deg * (cnt_zeros[CH]) ;
      motionData[CH][index_step[CH]].steps=1;
      motionData[CH][index_step[CH]].dir = motionData[CH][index_step[CH]-1].dir; // stepper motor direction...
      
       if(bDebug) printf("last--- Cn[%d]: %d, %d,  cnt_zeros=%d\n", index_step[CH], motionData[CH][index_step[CH]].steps, motionData[CH][index_step[CH]].Cn,   cnt_zeros[CH]); //[mm]
      index_step[CH]++;
      abs_sum_steps[CH] += 1;
      }
    }

    
  }

  // compensate the extra  or missing couple of steps on the steps because of round off error...
  motionDataSize[CH] = index_step[CH];

  totalSteps[CH] = round(totalDistance[CH] * DIST2STEP[CH]);
  if(bDebug) printf( "totalsteps=%d, abs_sum_steps=%d, , diff(totalSteps-abs_sum_steps)=%d, nM=%d\n", 
  totalSteps[CH], abs_sum_steps[CH], totalSteps[CH]-abs_sum_steps[CH], motionDataSize[CH]);

   if(bDebug) printf("-- result ----------------- totalDistance=%4.3f, totalsteps=%d\n", totalDistance[CH], totalSteps[CH]);
   if((totalSteps[CH]-abs_sum_steps[CH])==0)
    return 1; // Calculation is right
   else return 0; // total steps and sum of step is different (Calculation is wrong)...
  // // mkSerial.println(msg);
  
  //activatedEE = 1;
}
*/
/* // Array size is too big so that this is not working
void MKVelProfile::createLinearMotion(double distance, double speed, double accel, double decel)
{
  char msg[128];
  uint8_t num = 0;
  int8_t dir = SIGN(distance);
  uint32_t steps = round(motorParam[num].DIST2STEP * abs(distance)); // [mm]
  sprintf(msg,"total steps=%d", steps);
  // mkSerial.println(msg);
  activatedEE = -1;
  //if(speed>=MAX_SPEED_STEP) speed=MAX_SPEED_STEP; // Max speed...

  // if(sel_sqrt_cal==SQRT_APPRO){
  //   TICK_FREQ_SQRT_ERROR_COM *= 0.676;//0.69367;//initial error: 0.69367 at n=1, ((sqrt(2)-sqrt(1)))/((1.0-2.0/(4.0+1.0)))
  // }

  //#define SPR 1600 // (200*8); //% step per revolution
  double alpha = (2.0 * M_PI / 1600.0); //  % [rad] angle per step
  double two_alpha = 2.0 * alpha;   // alpha*2

  // % 1. Calcalate Time
  // % Ta = speed/acceleration
  double Ttotal = 0;
  double Ta = speed / accel; //%[sec]
  double Tb = speed / decel; //%[sec]
  double Tc = (steps * alpha - 0.5 * accel * Ta * Ta - 0.5 * decel * Tb * Tb) / speed;
  if (Tc > 0)
  {
    Ttotal = Ta + Tc + Tb;
  }
  else
  {
    Ttotal = Ta + Tb;
  }

  // % 2. Calculate Step
  // % convert Time Zone to Stepper Motor Count
  // % Acceleration Zone
  // % n = speed^2/(2*alpha*accel)

  uint32_t Na = round(speed * speed / (two_alpha * accel));
  uint32_t Nacc = round(steps * decel / (accel + decel));
  uint32_t Nb, Nc, Nac, NNb;
  uint32_t Cn_const_speed, Cn_acc0, Cn_dec0, Cn;
  if (Na < Nacc)
  {
    //%Nb = floor(speed^2/(2*alpha*decel));
    Nb = accel / decel * Na;
    Nc = steps - (Na + Nb);
  }
  else
  {
    Na = Nacc;
    Nb = steps - Nacc;
    Nc = 0;
  }
  Nac = Na + Nc;
  NNb = Nb;
  Cn_const_speed = uint32_t(TICK_FREQ_SQRT_ERROR_COM * sqrt(two_alpha / accel) * (sqrt(Na + 1) - sqrt(Na)));
  Cn_acc0 = uint32_t(TICK_FREQ_SQRT_ERROR_COM * sqrt(two_alpha / accel));
  Cn_dec0 = uint32_t(TICK_FREQ_SQRT_ERROR_COM * sqrt(two_alpha / decel));

  Cn = Cn_acc0;
  totalSteps = Na + Nc + Nb;
  nM = 0; // motion index
  for (uint32_t n = 0; n < totalSteps; n++)
  {
    // ++ Acceleration region ++ //
    if (n < Na)
    {
      if (n == 0)
      {
        motionData[nM].Cn = round(tick_freq * sqrt(2.0 * alpha / accel));
        motionData[nM].dir = dir;
        motionData[nM].steps = 1;
        ++nM;
      }
      else
      {
        //%%% Real SQRT calculation
        motionData[nM].Cn = round(tick_freq * sqrt(2.0 * alpha / accel) * (sqrt(n + 1) - sqrt(n)));
        motionData[nM].dir = dir;
        motionData[nM].steps = 1;
        ++nM;
      }
    }
    // ++ Constant speed region ++ //
    else if (n >= Na && n < (Na + Nc)) {
    //else if (n == Na) {
      if(n==Na) {
        motionData[nM].Cn = Cn_const_speed;
        motionData[nM].dir = dir;
        motionData[nM].steps = Nc;
        ++nM;
      }
    }
    // ++ Decelation Region ++ //
    else if (n >= (Na + Nc))
    {
      // %%% Real SQRT calculation
      motionData[nM].Cn = round(tick_freq * sqrt(2.0 * alpha / decel) * (sqrt(NNb + 1) - sqrt(NNb)));
      motionData[nM].dir = dir;
      motionData[nM].steps = 1;
      ++nM;
      NNb = NNb - 1;
    }
  }

  /////////////////////////////////////////////////////////
  if (dir == 1)
  { // CCW
    IOMap[num][ST_DIR].pIO->PIO_ODSR = IOMap[num][ST_DIR].pin;
    IOMap[num][ST_DIR].pIO->PIO_CODR = IOMap[num][ST_DIR].pin;
  }
  else
  { // CW
    IOMap[num][ST_DIR].pIO->PIO_ODSR ^= IOMap[num][ST_DIR].pin;
    IOMap[num][ST_DIR].pIO->PIO_SODR = IOMap[num][ST_DIR].pin;
  }

  
  sprintf(msg,"nM=%d, total steps=%lu, Na=%lu, Nb=%lu, Nc=%lu\n",nM, totalSteps, Na, Nb, Nc);
  // mkSerial.println(msg);

}
*/
////////////////////////////////////////////////////////////////
/*
void MKVelProfile::createArcMotion(double vel, double radius, double cenPos[2], double EndEffectorAng, int arcAng)
{
  // ++ deltaT_Lx is duration when EE rotate 1deg ++
  int num=0;
  double DIST2STEP=16.0;
  double STEP2DIST=1.0/DIST2STEP;
  double deltaT_Lx = vel * DIST2STEP / (M_PI * M_PI);
  //double deltaT_Lx = 10.0/360.0;//40.0*(180)*DEG2RAD / (60.0*5 * 2.0 * M_PI);
  // double DEG2RAD = M_PI / 180.0;
  int nAng = abs(arcAng);
  int8_t dir = SIGN(arcAng);
  double delAng = 0;
  double val = 0, dist[nAng+1] , remain[nAng+1] ;
  double steps = 0;
  int sum_steps = 0, abs_sum_steps = 0;

  prevLx = 0;
  LxCnt = 0;
  nM = nAng;
  //printf("-----------createArcMotion\n");
  int cnt_zeros=0;
  int index_step=0;
  activatedEE = -1;
  double totalDistance = 0;
  double abs_step=0;
  double abs_step_1=0,abs_step_residue=0;
  for (int i = 0; i <= nAng; i++)
  {

    //arcMotion(0.05, center, -90 * DEG2RAD, DEG2RAD * i);
    arcMotionIK(radius, cenPos, EndEffectorAng, DEG2RAD * i * dir);
    // printf("L pos[%d]: %2.5f\n",i,kinParam.L);
    if (i == 0)
    {
      prevLx = kinParam.L;
    }
    else
    {
      dist[i - 1] = (kinParam.L - prevLx); // [mm]
      totalDistance += abs(dist[i - 1]);
      prevLx = kinParam.L;
      // printf("L dLx[%d]: %2.5f\n",i-1,dist[i - 1]);
    }
  }
  totalSteps = round(totalDistance * DIST2STEP);
  //  printf("------------------- totalDistance %f\n", totalDistance);
  // Takes value which is only over 0.100mm and 0.xxx (3 doubleing digit)
  // and remains are shifting to next calculation (Because stepper motor's minimum resolution is 0.1mm)...

  for (int i = 0; i < nAng; i++)
  {
    Lx[i]=0;
    remain[i]=0;
    if (i == 0)
    {
      val = dist[i];
    }
    else
    {
      val = dist[i] + remain[i - 1];
    }

    abs_step = abs(DIST2STEP*val);
    if(abs_step>=1.0) { // not 1.0 because numerical error...
       abs_step_1 = round(abs_step);
       abs_step_residue = abs_step - abs_step_1;

      steps = SIGN(val)*abs_step_1;
      Lx[i] = steps*STEP2DIST;
      remain[i] = SIGN(val)*abs_step_residue*STEP2DIST;
    }
    else {
      steps=0;
       Lx[i] = 0;
       remain[i] =  val; 
      //  remain[i] =0; 
    }
 if(max_step<=steps)
    {
        max_is=i;
        max_step = steps;
    }
   //steps = round(steps);
    if (steps == 0) {
      cnt_zeros++;
    }
    else
    {
      if(cnt_zeros>0){
        motionData[index_step].Cn = round(abs(tick_freq * cnt_zeros * deltaT_Lx / steps));
        cnt_zeros=0;
      } else {
          motionData[index_step].Cn = round(abs(tick_freq * deltaT_Lx  / steps)); //(1/tick_freq)*deltaT_Lx/abs(steps);
          
      }
      motionData[index_step].steps = round(abs(steps));
      motionData[index_step].dir = SIGN(steps); // stepper motor direction...
      index_step++;

    }
      
       
    if((nAng-1)==i) {
      if(cnt_zeros>0){
      motionData[index_step].Cn = tick_freq * deltaT_Lx * (cnt_zeros) ;
      motionData[index_step].steps=1;
      motionData[index_step].dir = motionData[index_step-1].dir; // stepper motor direction...
      index_step++;
      }
    }
  }
  nM = index_step;
}
*/
/*
void MKVelProfile::createArcMotion(double vel, double radius, double cenPos[2], double EndEffectorAng, int arcAng)
{
  // ++ deltaT_Lx is duration when EE rotate 1deg ++
  int num=0;
  double DIST2STEP=16.f;
  double deltaT_Lx = vel * DIST2STEP / (M_PI * M_PI);
  //double deltaT_Lx = 10.0/360.0;//40.0*(180)*DEG2RAD / (60.0*5 * 2.0 * M_PI);
  // double DEG2RAD = M_PI / 180.0;
  uint16_t nAng = abs(arcAng);
  int8_t dir = SIGN(arcAng);
  double delAng = 0;
  // double val = 0, dist[nAng] = {0.0}, remain[nAng] = {0.0};
  double val = 0, dist[nAng], remain[nAng];
  double steps = 0;
  int sum_steps = 0, abs_sum_steps = 0;

  prevLx = 0;
  LxCnt = 0;
  nM = nAng;
  char msg[128];
  //printf("-----------createArcMotion\n");

  activatedEE = -1;
  double totalDistance = 0;
  for (int i = 0; i <= nAng; i++)
  {

    //arcMotion(0.05, center, -90 * DEG2RAD, DEG2RAD * i);
    arcMotionIK(radius, cenPos, EndEffectorAng, DEG2RAD * i * dir);
    //printf("L pos[%d]: %2.5f\n",angCnt,mkRobotKin.kinParam.L);
    if (i == 0)
    {
      prevLx = kinParam.L;
    }
    else
    {
      dist[i - 1] = (kinParam.L - prevLx); // [mm]
      totalDistance += abs(dist[i - 1]);
      prevLx = kinParam.L;
    }
  }
  totalSteps = round(totalDistance * DIST2STEP);
  // Takes value which is only over 0.100mm and 0.xxx (3 doubleing digit)
  // and remains are shifting to next calculation (Because stepper motor's minimum resolution is 0.1mm)...

  for (int i = 0; i < nAng; i++)
  {
    motionData[i].Cn =i;

    if (i == 0)
    {
      val = dist[i];
    }
    else
    {
      val = dist[i] + remain[i - 1];
    }

    if (abs(val) >= 0.1)
    {
      Lx[i] = round(val * 100.0) * 0.01;
      remain[i] = (val * 100.0 - round(val * 100.0)) * 0.01;
      steps = Lx[i] * DIST2STEP;
    }
    else
    {

      if (abs(val) >= 0.05)
        steps = SIGN(val) * floor(abs(val) * DIST2STEP);
      else
        steps = SIGN(val) * ceil(abs(val) * DIST2STEP);
      Lx[i] = val;
      remain[i] = 0;
    }
    steps = round(steps);
    motionData[i].steps = abs(steps);

    // duration for each step(Cn Timer Counter Input)
    if (steps == 0)
      motionData[i].Cn = tick_freq * deltaT_Lx; //(1/tick_freq)*deltaT_Lx/abs(steps);
    else
      motionData[i].Cn = tick_freq * deltaT_Lx / motionData[i].steps; //(1/tick_freq)*deltaT_Lx/abs(steps);

    motionData[i].dir = SIGN(steps); // stepper motor direction...
    sum_steps += steps;
    abs_sum_steps += abs(steps);
    //printf("--- Cn[%d]: %d, %lu, %2.4f\n", i, motionData[i].steps, motionData[i].Cn,  dist[i]); //[mm]
   
  }
  // compensate the extra  or missing couple of steps on the steps because of round off error...
  motionData[nAng - 1].steps += sum_steps;
 
  // sprintf(msg, "totalsteps=%d, abs step sum=%d, step_sum=%d\n", totalSteps, abs_sum_steps, sum_steps);
  // // mkSerial.println(msg);
  //activatedEE = 1;
}
*/