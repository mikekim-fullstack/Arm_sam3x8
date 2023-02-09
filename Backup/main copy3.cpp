#include <stdio.h>
#include <math.h>
#include "mkZoeRobotics_define.h"
#include "../include/due_sam3x.init.h"

#include "timetick.h"
#include "mkZoeRobotics_serial.h"
#include "mkZoeRobotics_command.h"
#include "mkZoeRobotics_velProfile.h"
#include "spi.h"
//////////////////////////////////////////////////////////////


speedRampData speedData[MAX_MOTOR_NUM]={0};
portIOPair   IOMap[MAX_MOTOR_NUM][4]={0};
struct GLOBAL_FLAGS status;
unsigned long  elapsedTime[MAX_MOTOR_NUM]={0};
volatile  uint32_t gIRQ_step_count[MAX_MOTOR_NUM]={0};
volatile  uint16_t gIRQ_flag[MAX_MOTOR_NUM]={0};
static int nTimer = TC_0;
///////////////////////////////////////////////////////////////
// Speed ramp states
#define STOP  0
#define ACCEL 1
#define DECEL 2
#define RUN   3
////////////////////////////////////////////////////////////////////////
inline static  void writePluse(Pio *portIO, uint32_t pin)
{
   portIO->PIO_ODSR ^= pin;
   portIO->PIO_ODSR ^= pin;
}
//////////////////////////////////////////////////////////
void TC0_Handler()
{
  TC_GetStatus(TC0, 0);
  if(speedData[nTimer].activated)
  {
      // ...GIVE AN IMPULSE TO MOTOR...
      writePluse(IOMap[nTimer][ST_PULSE].pIO, IOMap[nTimer][ST_PULSE].pin);

      // ...CLEAR TC STATUS REGISTER...
      TC_SetRC(TC0, 0, speedData[nTimer].Cn);
      ++speedData[nTimer].step_count;
      // ... ACCELERATION AREA ....
      if(speedData[nTimer].step_count<speedData[nTimer].Na)
      {
            if(speedData[nTimer].step_count==0)
            {
                speedData[nTimer].Cn = speedData[nTimer].Cn_acc0;
                speedData[nTimer].rest = 0;
            }
            else
            {
              //%%% SQRT approximation of speedData[num].Cn = speedData[num].Cn_acc0*(sqrt(step_count+1)-sqrt(step_count));
              speedData[nTimer].C =  speedData[nTimer].Cn*(1- 2.0/(4 * speedData[nTimer].step_count + 1))+speedData[nTimer].rest ;
              speedData[nTimer].Cn = floor(speedData[nTimer].C);
              speedData[nTimer].rest =  speedData[nTimer].C-speedData[nTimer].Cn;
            }
      }
      // ... DECELERATION AREA ...
      else if( speedData[nTimer].step_count>=speedData[nTimer].Nac && speedData[nTimer].step_count<speedData[nTimer].steps)
      {
        
          //%%% SQRT approximation
          if(speedData[nTimer].step_count==speedData[nTimer].Nac)
          {
              speedData[nTimer].rest=0;
          }
          else {
            speedData[nTimer].C =  speedData[nTimer].Cn*(1 + 2.0/(4 * speedData[nTimer].NNb - 1))+speedData[nTimer].rest ;
            speedData[nTimer].Cn = floor(speedData[nTimer].C);
            speedData[nTimer].rest =  speedData[nTimer].C-speedData[nTimer].Cn;
          }
          speedData[nTimer].NNb--;
      }
      // ... FINISHED SPEED PROFILE AND NOTIFY IN MAIN LOOP...
      else if(speedData[nTimer].step_count>=speedData[nTimer].steps)
      {
          gIRQ_flag[nTimer]=1;
      }
  }// ---------end of if(speedData[nTimer].activated) 
}
// -------------------- END OF TC0_Handler ---------------------------

void TC1_Handler()
{

 ////////////////// EXAMPLE FOR OTHER TIMER ////////////////////////////
  TC_GetStatus(TC0, 1);
   //////////////////////////////////////////////
  NVIC_DisableIRQ(TC1_IRQn);
  TC_SetRC(TC0, 1, speedData[1].Cn);
  NVIC_EnableIRQ(TC1_IRQn);
  //////////////////////////////////////////////
  writePluse(IOMap[TC_1][ST_PULSE].pIO, IOMap[TC_1][ST_PULSE].pin);
  //pulse_stepper_motor(0,speedData[num].dir);
  ++gIRQ_step_count[1];
  gIRQ_flag[1]=1;
}
/////////////////////////////////////////////////////////////////////
inline static  void setupIO_Output(Pio *portIO, uint32_t pin)
{
  portIO->PIO_IDR = pin; // disable interrupt
  portIO->PIO_MDDR = pin;// Disable to connect multi devices 
  portIO->PIO_OWER = pin;// Enable writing on ODSR

  portIO->PIO_PER = pin; // PIO ENABLE
  portIO->PIO_OER = pin; // OUTPUT ENALBE
  portIO->PIO_PUDR = pin;// PULL-UP DISALBLE
}
////////////////////////////////////////////////////////
 static  void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency=350) 
 {
    pmc_set_writeprotect(false);
    pmc_enable_periph_clk((uint32_t)irq);
    //TC_Configure(tc, channel,  TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK2 | TC_CMR_ACPC_CLEAR | TC_CMR_ASWTRG_CLEAR);//  | TC_CMR_CPCSTOP);
    TC_Configure(tc, channel,  TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK1);
    uint32_t rc = F_CPU/8/frequency; //128 because we selected TIMER_CLOCK4 above
    TC_SetRA(tc, channel, rc); 
    TC_SetRC(tc, channel, rc);
    TC_Start(tc, channel);
    tc->TC_CHANNEL[channel].TC_IER=TC_IER_CPCS;
    tc->TC_CHANNEL[channel].TC_IDR=~TC_IER_CPCS;
    NVIC_ClearPendingIRQ(irq);
    NVIC_EnableIRQ(irq);
}

/////////////////////////////////////////////////////

extern MKSerial mkSerial;
extern MKCommand mkCommand;
extern MKVelProfile mkVelProfile;
int main(void)
{
  char buffer[128];
  init_controller();
  


  // SET IO MAP FOR MOTOR #1 ON TIMER 0
  IOMap[TC_0][ST_PULSE].pIO = PIOB; IOMap[TC_0][ST_PULSE].pin = PIO_OER_P26; // PULSE FOR STEPPER MOTOR ON PIN 22
  IOMap[TC_0][ST_DIR].pIO =   PIOA; IOMap[TC_0][ST_DIR].pin = PIO_OER_P14;     // DIRECTION FOR STEPPER MOTOR ON PIN 23
  IOMap[TC_0][HOMESTOP].pIO = PIOA; IOMap[TC_0][HOMESTOP].pin = PIO_OER_P15; // HOME STOP FOR STEPPER MOTOR ON PIN 24
  IOMap[TC_0][ENDSTOP].pIO = PIOD;  IOMap[TC_0][ENDSTOP].pin = PIO_OER_P0;  // END STOP FOR STEPPER MOTOR ON PIN 24

  // ENABLE IO PIN AS AN OUTPUT
  for(int timer=0; timer<MAX_MOTOR_NUM; timer++)
  {
    for (int map=0; map<4; map++)
      setupIO_Output(IOMap[timer][map].pIO, IOMap[timer][map].pin);
  }

  mkCommand.setCallBack_gen_speed_profile(mkVelProfile.gen_speed_profile);
  mkCommand.setCallBack_update_speed_only(mkVelProfile.update_speed_only);
  mkSerial.begin(115200);
  mkSerial.println("FROM SAM3X8 CONTROLLER!");

  //TC0 channel 0, the IRQ0 for that channel and the desired frequency
  startTimer(TC0, 0, TC0_IRQn); 
  ///////////////////////////////////////////////////


  /* Main loop */
  while(1) {
    if(mkCommand.buflen < (BUFSIZE-1)){
        mkCommand.getCommand();
    }
    if(mkCommand.buflen) {
      mkSerial.println(mkCommand.buflen);
      mkCommand.process_commands();
      mkCommand.buflen = (mkCommand.buflen-1);
      mkCommand.bufindr = (mkCommand.bufindr + 1)%BUFSIZE;
    }
    ////////////////////////////////////////////
    if(gIRQ_flag[nTimer]==1)
    {
      NVIC_DisableIRQ(TC0_IRQn);
      elapsedTime[nTimer] = millis() - elapsedTime[nTimer];
      float current = elapsedTime[nTimer]*0.001;
      float speed_mmPsec = speedData[nTimer].step_count*0.025/current;
      
      mkSerial.print("time[sec]: ");
      mkSerial.println(current);
      mkSerial.print("speed[mm/sec]: ");
      mkSerial.println(speed_mmPsec);
      mkSerial.println("----------------END OF MOTOR #1  -------------");
      speedData[nTimer].step_count = 0;
      gIRQ_flag[nTimer]=0;
      // ...NOTIFY TO HOST...
      mkSerial.println("READY");
    } 
    
  }
  return 0;
}
/////////////////////////////////////////////

