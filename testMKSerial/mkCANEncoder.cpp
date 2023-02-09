#include "mkCANEncoder.h"
#include "mkZoeRobotics_serial.h"
#include "variant.h"
#define ENCODER_CONVERSION 0.087890625 //1//4096.0*360.0

mkCANEncoder mkCAN;
mkCANEncoder::mkCANEncoder():CANRaw()
{

}
// ~mkCANEncoder::mkCANEncoder() 
// {

// }
void mkCANEncoder::initCAN()
{
   Can0.begin(CAN_BPS_500K);
   Can1.begin(CAN_BPS_500K);
   //The default is to allow nothing through if nothing is specified
   Can0.watchFor();
   Can1.watchFor();
}
void mkCANEncoder::setReadEncoderData(int sel) 
{
      // Prepare transmit ID, data and data length in CAN0 mailbox 0
   output.id = 0x01;//TEST1_CAN_TRANSFER_ID;
   output.length = 8;//MAX_CAN_FRAME_DATA_LEN;
   output.extended = false;
   output.rtr = false;

   if(sel==0) {
      // bSentRead[0] = true;
      output.data.byte[0] = 0x04;// Data Length
      output.data.byte[1] = 0x01;// Device ID(Encoder)
      output.data.byte[2] = 0x01;// CMD(1: read encoder value)
      output.data.byte[3] = 0x00;// Data
      Can0.sendFrame(output);
      mkCAN.bReadSignal[0]=false;
      mkCAN.bReadyRead[0]=true;
      
   }
   else if(sel==1) {
      // bSentRead[1] = true;
      output.data.byte[0] = 0x04;// Data Length
      output.data.byte[1] = 0x01;// Device ID(Encoder)
      output.data.byte[2] = 0x01;// CMD(1: read encoder value)
      output.data.byte[3] = 0x00;// Data
      Can1.sendFrame(output);
      mkCAN.bReadSignal[1]=false;
      mkCAN.bReadyRead[1]=true;
   }
}
void mkCANEncoder::setZeroPos(int sel)
{
   if(sel==0){
      output.data.byte[0] = 0x04;// Data Length
      output.data.byte[1] = 0x01;// Device ID(Encoder)
      output.data.byte[2] = 0x06;// CMD(1: read encoder value)
      output.data.byte[3] = 0x00;// Data
      Can0.sendFrame(output);
    }
   else if(sel==1){
      output.data.byte[0] = 0x04;// Data Length
      output.data.byte[1] = 0x01;// Device ID(Encoder)
      output.data.byte[2] = 0x06;// CMD(1: read encoder value)
      output.data.byte[3] = 0x00;// Data
      Can1.sendFrame(output);
    }
}
bool mkCANEncoder::readEncoderData(int sel) 
{
   
   // if(sel==0) {
   if(bReadyRead[0] && sel==0) {
      if(Can0.available()!=0) {
         bReadyRead[0]=false;
         CAN_FRAME incoming;
         Can0.read(incoming);
         int len = incoming.data.byte[0];
         int dataLen=len-4;
         encoderValue[0]=0;
         for(int i=1; i<=len-3; i++){
            encoderValue[0] += (incoming.data.byte[len-i]<<(8*(dataLen+1-i)));
         }
         return (true);
      }
      //encoderValue[0]*=ENCODER_CONVERSION;
      //if(encoderValue[0]>180.0) encoderValue[0] = encoderValue[0]-360.0;
      
   }
   // else if(sel==1) {
   else if(bReadyRead[1] && sel==1) {
      if(Can1.available()!=0) {
         bReadyRead[1]=false;
         CAN_FRAME incoming;
         Can1.read(incoming);
         int len = incoming.data.byte[0];
         int dataLen=len-4;
         encoderValue[1]=0;
         for(int i=1; i<=len-3; i++){
            encoderValue[1] += (incoming.data.byte[len-i]<<(8*(dataLen+1-i)));
         }
         return (true);
      }
      //encoderValue[1]*=ENCODER_CONVERSION;
      //if(encoderValue[1]>180.0) encoderValue[1] = encoderValue[1]-360.0;
      
   }
   return false;
}
void mkCANEncoder::test_loopCAN() {

      if(mkCAN.readEncoderData(0)) {
         mkSerial.print("CAN0 encoderValue=");
         mkSerial.println(mkCAN.encoderValue[0]*ENCODER_CONVERSION);
      }
      if(mkCAN.readEncoderData(1)) {
         mkSerial.print("CAN1 encoderValue=");
         mkSerial.println(mkCAN.encoderValue[1]*ENCODER_CONVERSION);
      }
         
      if ((millis() - lastTime) > 20) 
      {
         lastTime = millis();
         // mkCAN.bSentRead[0]=true;
         setReadEncoderData(0);
         // mkCAN.bSentRead[1]=true;
         setReadEncoderData(1);
         mkSerial.print("\n------------------------------\n");
      }
   }
