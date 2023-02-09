#include <stdio.h>
#include <math.h>
#include <string.h>
#include "mkZoeRobotics_velProfile_test.h"
//gcc -Wall -g   -std=C++11 main_IStacker_Server.cpp -o final  
extern MKVelProfile mkVelProfile;
KIN_DATA kinData[3];
int main()
{

 
  CIRCLEProfile  circleProfile;
  circleProfile.speed=360.0;
  circleProfile.radius=50;
  circleProfile.cenPosX=1000;
  circleProfile.cenPosY=-250;
  circleProfile.EETheta=-90;
  circleProfile.arcAng=360;
  int rev = mkVelProfile.gen_circl_profile(circleProfile);//A  
  int ich=0;
  char temp[128];
   sprintf(temp,"-- Last(rev=%d) Cn=%d, dataSize=%d, totalS=%d",
            rev,
            kinData[ich].motionData[kinData[ich].dataSize-1].Cn, 
            kinData[ich].dataSize, 
            kinData[ich].totalSteps
            );

  printf("end main, %d, %s\n", rev, temp);
  printf("-----------------------------------------------------\n");
  LINEARProfile linearProfile;
  // X889.149 Y943.000 Z-222.584 T-354.980 V-1.571 A150.000 
  //"G2 X943.015 Y943.000 Z-120.008 T-354.980 V-1.571 A150.000 J154 N1\n"
  // linearProfile.EEx[0] = 889.149;
  // linearProfile.EEx[1] = 943.000;

  // linearProfile.EEy[0] = -222.584;
  // linearProfile.EEy[1] = -354.980;

  linearProfile.EEx[0] = 943.015;
  linearProfile.EEx[1] = 943.000;

  linearProfile.EEy[0] = -222.584;
  linearProfile.EEy[1] = -354.980;

  linearProfile.EETheta = -1.571;
  linearProfile.Vel = 150.000;
// W487.624 X635.145 Y-355.011 Z-355.073 V-1.571 A150.000
  linearProfile.EEx[0] = 487.624;
  linearProfile.EEx[1] = 635.145 ;

  linearProfile.EEy[0] = -355.011;
  linearProfile.EEy[1] = -355.073;

  linearProfile.EETheta = -1.571;
  linearProfile.Vel = 150.000;

  //"G2 X943.245 Y943.030 Z-354.968 T-120.030 V-1.570 A150.000 J329 N1\n"
  linearProfile.EEx[0] = 943.245;
  linearProfile.EEx[1] = 943.030 ;

  linearProfile.EEy[0] = -354.968;
  linearProfile.EEy[1] = -120.030;

  linearProfile.EETheta = -1.571;
  linearProfile.Vel = 150.000;

  mkVelProfile.gen_linear_profile(linearProfile);
   return 0;
}


