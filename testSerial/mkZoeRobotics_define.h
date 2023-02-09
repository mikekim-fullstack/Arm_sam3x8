/**
  mkZoeRobotics_define.h 
  Copyright (c) 2020 Mike Kim (ZOE Robotics).  All right reserved.
  Modified 5 August 2020 by Mike Kim
*/
#ifndef _MKZOEROBOTICS_DEFINE_H_
#define _MKZOEROBOTICS_DEFINE_H_

//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
#ifndef F_CPU
  #define F_CPU 84000000UL // or whatever may be your frequency
#endif
#define TICK_PRESCALE 128

#ifndef TICK_FREQ_SQRT_ERROR_COM
//root square error compansation value 0.676
  #define  TICK_FREQ_SQRT_ERROR_COM  84000000.0/TICK_PRESCALE*0.676 //28392000u //84000000/8*0.676 //10500000u;//F_CPU/8* ;
#endif 

#ifndef LINK_1
  #define LINK_1 215.0
#endif

#ifndef LINK_2
  #define LINK_2 250.0
#endif

#ifndef MAX_MOTOR_NUM
  #define MAX_MOTOR_NUM 4
#endif 

#ifndef tick_freq
  #define tick_freq (F_CPU / TICK_PRESCALE)
#endif 

#ifndef SIGN
  #define SIGN(a) ((a < 0) ? -1 : 1)
#endif

#ifndef M_PI
  #define M_PI           3.14159265358979323846 
#endif

#ifndef DEG2RAD
  #define DEG2RAD 0.017453292519943 // (M_PI / 180.0)
#endif

//#define DIST2STEP 40 // moving distnce[mm] per one revolution of pulley (belt pitch 2mm* 20 teeth on pulley)

#ifndef DIST2STEP_20T2MM
  #define  DIST2STEP_20T2MM 40 // (20TH*2MM=40MM/REV) MM TO STPEPS
#endif

#ifndef X_DIST2STEP_20T5MM
  #define  X_DIST2STEP_20T5MM 100 //  100mm/rev (=20TH*5MM)
#endif

#ifndef Z_DIST2STEP_20D5MM
  #define  Z_DIST2STEP_20D5MM 5 //  5mm/rev (Ball Screw Lead pitch)
#endif

#ifndef X_MICROSTEPPING
  #define  X_MICROSTEPPING 1600*2 //1600 (= 200 STEPS X 8 MICROSTEP)
#endif

#ifndef Z_MICROSTEPPING
  #define  Z_MICROSTEPPING 1600*2 // 200 STEPS X 1 MICROSTEP
  // Old cofiguration for Ball Screw axis
  // #define  Z_MICROSTEPPING 400 // 200 STEPS X 1 MICROSTEP
#endif


#ifndef HIGH
  #define HIGH 0x1
#endif

#ifndef LOW
  #define LOW 0x0
#endif

#ifndef PULLUP_INPUT
  #define PULLUP_INPUT 1
#endif

#ifndef PULLDOWN_INPUT
  #define PULLDOWN_INPUT 0
#endif
//////////////////////////////////////////////////////////////////

#ifndef Stopped
  #define  Stopped false
#endif

#ifndef MAX_MOTIONDATA_SIZE
  #define MAX_MOTIONDATA_SIZE 1500
#endif


#ifndef BUFSIZE
  #define BUFSIZE 24
#endif
#ifndef MAX_CMD_SIZE
  #define MAX_CMD_SIZE 96
#endif

#endif