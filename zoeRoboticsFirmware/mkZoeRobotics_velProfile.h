/**
  mkZoeRobotics_velProfile.h 
  Copyright (c) 2020 Mike Kim (ZOE Robotics).  All right reserved.
  Originated at 28 August 2020 by Mike Kim
*/
#ifndef _MKZOEROBOTICS_VELPROFILE_H_
#define _MKZOEROBOTICS_VELPROFILE_H_
#include <stdint.h>
#define MAX_MOTION_DATA 1500
#include "mkZoeRobotics_globalDataStruct.h"
//#include "mkZoeRobotics_define.h"
typedef struct _MOTOR_PARAMS
{
  uint16_t MICROSTEPPING;
  uint32_t DIST2STEP;
  double STEP2DIST;
  double alpha;
  double two_alpha;
}MOTOR_PARAMS;
class MKVelProfile
{
public:
// Velocity Profile Params:
  //static MOTOR_PARAMS motorParams[4];
  /////////////////////////////
  // static double l1 = 215.0; // Link1 length[m]
  // static double l2 = 250.0; // Link2 length[m]
   KIN_PARAM kinParam;
  ////////////////////////////////////////////////
  // ----------------------------------------
  // volatile bool activated=false;
  // volatile bool pulseTick=false;
  // volatile bool pulseDown=false;
  //MOTIONDATA motionData[3][MAX_MOTION_DATA]={};
  // uint16_t motionDataSize[3]={0};
  // uint32_t totalSteps[3]={0};
 //////////////////////////////////////////////////
  static MOTOR_PARAMS motorParams[2];
public:
  MKVelProfile()
  {
    // motorParams[0].MICROSTEPPING = X_MICROSTEPPING;
    // motorParams[0].DIST2STEP = X_MICROSTEPPING/X_DIST2STEP_20T5MM;
    // motorParams[0].STEP2DIST = float(X_DIST2STEP_20T5MM)/float(X_MICROSTEPPING);
    // motorParams[0].alpha = 2.0*M_PI/motorParams[0].MICROSTEPPING;
    // motorParams[0].two_alpha = motorParams[0].alpha*2.0;

    // motorParams[1].MICROSTEPPING = Z_MICROSTEPPING;
    // motorParams[1].DIST2STEP = Z_MICROSTEPPING/Z_DIST2STEP_20D5MM;
    // motorParams[1].STEP2DIST = float(Z_DIST2STEP_20D5MM)/float(Z_MICROSTEPPING);
    // motorParams[1].alpha = 2.0*M_PI/motorParams[1].MICROSTEPPING;
    // motorParams[1].two_alpha = motorParams[1].alpha*2.0;

  }
  ~MKVelProfile()
  {
  }
  static int gen_linear_profile_old(LINEARProfile & linearProfile);
  static int gen_linear_profile(LINEARProfile & linearProfile);
  static int gen_EErotation_profile(EEROTATIONProfile & eeRotationProfile);
  static int gen_circl_profile( CIRCLEProfile & circleProfile);
  static int gen_spiral_profile( SPIRALProfile & spiralProfile);
  static void set_speed_profile( SPEEDProfile &speedProfile);
  static void gen_speed_profile(uint16_t num, double distance, double speed, double accel, double decel);
  static void update_speed_only(uint16_t num, uint32_t steps);
  //void resetPos();

   bool invKin(double x, double y, double theta);
   bool invKin(KIN_PARAM &input);
   void forKin(double L, double t1, double t2);
   
   bool arcMotionIK(double radius, double cenPosX, double cenPosY, double EndEffectorAng, double rotAng);
  //int createArcMotion(double vel, double radius, double cenPosX, double cenPosY, double EndEffectorAng, int nAng=360);
  
};


#endif /*_MKZOEROBOTICS_VELPROFILE_H_*/