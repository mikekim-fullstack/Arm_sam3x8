#include "due_can.h"
#ifndef __MKCANENCODER_H__
#define __MKCANENCODER_H__

class mkCANEncoder: public CANRaw {
   private:
   CAN_FRAME output;
   unsigned long lastTime = 0;
   bool sel=true;
   public:
   bool bReadSignal[2]={false};
   bool bReadyRead[2]={false};
   int32_t encoderValue[2]={0};//raw data degree
   mkCANEncoder() ;
   //virtual ~mkCANEncoder() ;
   void initCAN();
   void setReadEncoderData(int sel) ;
   bool readEncoderData(int sel) ;
   void setZeroPos(int sel);
   void test_loopCAN() ;
};

// extern mkCANEncoder Can0;
// extern mkCANEncoder Can1;



#endif