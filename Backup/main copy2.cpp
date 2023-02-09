#include <stdio.h>
#include <math.h>
#include "mkZoeRobotics_define.h"
#include "../include/due_sam3x.init.h"
#include "timetick.h"
#include "mkZoeRobotics_serial.h"
#include "mkZoeRobotics_command.h"
#define _SAM3XA_
//////////////////////////////////////////////////////////////
#define MAXI 30
#define VERSION        (1)  // firmware version
#define MAX_BUF        (64)  // What is the longest message Arduino can store?.
#define MAX_FEEDRATE   (50000) 
#define MIN_FEEDRATE   (0.01)
#define PI 3.1415926535
#define STEPS_MM  40
#define  TICK_FREQ  28392000u //42000000u*0.676 //10500000u;//F_CPU/8;

int ACCEL =    1500 ;  // mm/s^2

bool flip = false;
// for arc directions
#define ARC_CW          (1)
#define ARC_CCW         (-1)
// Arcs are split into many line segments.  How long are the segments?
#define MM_PER_SEGMENT  (1)
volatile int cs=0; // current step
long accel_steps=1000; // how many steps during acceleration
long coast_steps=40000; // how many steps during coasting
long total_steps=0; // total number of steps for the current movement
float t0=0.1;   // determines initial speed
float cn;
bool diry,dirx;
long over;
float dx,dy;

volatile bool busy = false;
//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------

char buffer[MAX_BUF];  // where we store the message until we get a newline
int sofar;  // how much is in the buffer

float px, py;  // location

// speeds
float fr=1000;  // human version
long step_delay;  // machine version

// settings
char mode_abs=1;  // absolute mode?


///////////////////////////////////////////////////////////////

// Speed ramp states
#define STOP  0
#define ACCEL 1
#define DECEL 2
#define RUN   3

#define TRUE 1
#define FALSE 0

// Direction of stepper motor movement
#define CW  0
#define CCW 1

#define MAX_SPEED_STEP 110
static enum  {SQRT_APPRO, SQRT_REAL} SQRT_CAL;
static bool  sel_sqrt_cal =  SQRT_APPRO;

struct GLOBAL_FLAGS {
  //! True when stepper motor is running.
  unsigned char running:1;
  //! True when uart has received a string (ended with '/r').
  unsigned char cmd:1;
  //! Dummy bits to fill up a byte.
  unsigned char dummy:6;
};
typedef struct {
  Pio *pIO;// Direction Pin
  uint32_t pin;
}portIOPair;
typedef struct {
    bool activated;
    float rest;
    float Ta, Tb, Tc, Ttotal;
    uint32_t Na;// Step count for acceleration Zone
    uint32_t Nb;// Step count for deceleration Zone
    uint32_t Nc;// Step count for conatant speed Zone
    uint32_t Nac; //Nac=Na+Nc
    uint32_t NNb;// Same as Nb for deceleration
    uint32_t Cn_const_speed ; //time delay(OCR1A) for constant speed zone
    uint32_t Cn_acc0; // time delay at the starting acceleration.
    uint32_t Cn_dec0; // time delay at the starting deceleration.

    volatile uint32_t Cn:1000; // time delay by N step
    volatile uint32_t steps;// total steps
    volatile uint32_t step_count;
    float C;


  //! What part of the speed ramp we are in.
  unsigned char run_state : 3;
  //! Direction stepper motor should move.
  unsigned char dir : 1;// CCW
  
  
} speedRampData;
static speedRampData srd[8]={0};
static portIOPair   pIODir[8]={0};
struct GLOBAL_FLAGS status;
unsigned long  elapsedTime[8]={0};
volatile static uint32_t gIRQ_step_count[8]={0};
volatile static uint16_t gIRQ_flag[8]={0};


static void gen_speed_profile(uint16_t num, float steps,  float speed, float accel, float decel, int dir=1);
static void modify_speed_profile(uint16_t num,   uint32_t steps);
inline static void  pulse_stepper_motor(char n_motor, signed char inc);
//////////////////////////////////////////////////////////////////
/**
 * Set the feedrate (speed motors will move)
 * @input nfr the new speed in steps/second
 */
void feedrate(float nfr) {
  if(fr==nfr) return;  // same as last time?  quit now.
  if(nfr>MAX_FEEDRATE || nfr<MIN_FEEDRATE) {  // don't allow crazy feed rates
    mkSerial.print(("New feedrate must be greater than "));
    mkSerial.print(MIN_FEEDRATE);
    mkSerial.print(("steps/s and less than "));
    mkSerial.print(MAX_FEEDRATE);
    mkSerial.println(("steps/s."));
    return;
  }
  step_delay = 1000000.0/nfr;
  fr=nfr;
  cn = t0 = sqrt( 2.0 /** STEPS_MM*/ / ACCEL );
}
/**
 * Set the logical position
 * @input npx new position x
 * @input npy new position y
 */
void position(float npx,float npy) {
  // here is a good place to add sanity tests
  px=npx;
  py=npy;
}

/**
 * Uses bresenham's line algorithm to move both motors
 * @input newx the destination x position
 * @input newy the destination y position
 **/
void line(float newx,float newy) {
  dx=newx-px;
  dy=newy-py;
  dirx=dx>0;
  diry=dy>0;  // because the motors are mounted in opposite directions
  dx=abs(dx);
  dy=abs(dy);
// Serial.print("line: ");Serial.print(dx); Serial.print(","); Serial.println(dy);
//  move(dx,dirx,dy,diry);
  long i;
  over=0;

  total_steps = max ( dx, dy ) * STEPS_MM; 
  float ta = fr / 60.0 / ACCEL;
  float ea = ( fr / 60.0 ) * ta / 2 ;
  accel_steps = min(ea * STEPS_MM , total_steps/2); // just in case feedrate cannot be reached 
  coast_steps = total_steps - accel_steps * 2; // acceleration
  // digitalWrite(D13, dirx); // direction of X motor
  // digitalWrite(D12, diry);
  cs = 0; // let the motion start :-)
  cn = t0;

  if(dx>dy) over=dx/2; else over=dy/2;
  busy=true;
  // wait till the command ends
  //while(busy) { /*doServo();*/ delay(0); }
  px=newx;
  py=newy;
}
// returns angle of dy/dx as a value from 0...2PI
float atan3(float dy,float dx) {
  float a=atan2(dy,dx);
  if(a<0) a=(PI*2.0)+a;
  return a;
}
// This method assumes the limits have already been checked.
// This method assumes the start and end radius match.
// This method assumes arcs are not >180 degrees (PI radians)
// cx/cy - center of circle
// x/y - end position
// dir - ARC_CW or ARC_CCW to control direction of arc
void arc(float cx,float cy,float x,float y,float dir) {
  // get radius
  float dx = px - cx;
  float dy = py - cy;
  float radius=sqrt(dx*dx+dy*dy);

  // find angle of arc (sweep)
  float angle1=atan3(dy,dx);
  float angle2=atan3(y-cy,x-cx);
  float theta=angle2-angle1;
  
  if(dir>0 && theta<0) angle2+=2*PI;
  else if(dir<0 && theta>0) angle1+=2*PI;
  
  theta=angle2-angle1;
  
  // get length of arc
  // float circ=PI*2.0*radius;
  // float len=theta*circ/(PI*2.0);
  // simplifies to
  float len = abs(theta) * radius;

  int i, segments = ceil( len * MM_PER_SEGMENT );
 
  float nx, ny, angle3, scale;

  for(i=0;i<segments;++i) {
    // interpolate around the arc
    scale = ((float)i)/((float)segments);
    
    angle3 = ( theta * scale ) + angle1;
    nx = cx + cos(angle3) * radius;
    ny = cy + sin(angle3) * radius;
    // send it to the planner
    line(nx,ny);
  }
  
  line(x,y);
}
/**
 * Look for character /code/ in the buffer and read the float that immediately follows it.
 * @return the value found.  If nothing is found, /val/ is returned.
 * @input code the character to look for.
 * @input val the return value if /code/ is not found.
 **/
float parsenumber(char code,float val) {
  char *ptr=buffer;
  while(ptr && *ptr && ptr<buffer+sofar) {
    if(*ptr==code) {
      return atof(ptr+1);
    }
    ptr++; //ptr=strchr(ptr,' ')+1;
  }
  return val;
} 
void output(const char *code,float val) {
  mkSerial.print(code);
  mkSerial.println(val);
}
void where() {
  output("X",px);
  output("Y",py);
  output("F",fr);
  mkSerial.println(mode_abs?"ABS":"REL");
} 
/**
 * Read the input buffer and find any recognized commands.  One G or M command per line.
 */
void processCommand() {
  int cmd = parsenumber('G',-1); 
  switch(cmd) {
  case  0:
  case  1: { // line
    feedrate(parsenumber('F',fr));
    line( parsenumber('X',(mode_abs?px:0)) + (mode_abs?0:px),
          parsenumber('Y',(mode_abs?py:0)) + (mode_abs?0:py) );
    break;
    }
  case 2:
  case 3: {  // arc
      feedrate(parsenumber('F',fr));
      arc(parsenumber('I',(mode_abs?px:0)) + (mode_abs?0:px),
          parsenumber('J',(mode_abs?py:0)) + (mode_abs?0:py),
          parsenumber('X',(mode_abs?px:0)) + (mode_abs?0:px),
          parsenumber('Y',(mode_abs?py:0)) + (mode_abs?0:py),
          (cmd==2) ? -1 : 1);
      break;
    }
  case  4:  break;//pause(parsenumber('P',0)*1000);  break;  // dwell
  case 90:  mode_abs=1;  break;  // absolute mode
  case 91:  mode_abs=0;  break;  // relative mode
  case 92:  // set logical position
    position( parsenumber('X',0),
              parsenumber('Y',0) );
    break;
  default:  break;
  }

//   cmd = parsenumber('M',-1); 
//   switch(cmd) {
//   case 3:servo = ( parsenumber('S',1500) ); break; // sets the servo value in microseconds, it only works while inside loop :-(
//   case 18:  // disable motors
// //    disable();
//     break;
//   case 100:  help();  break;
//   case 114:  where();  break;
//   case 201: ACCEL = parsenumber('A',0); feedrate(fr); Serial.print(ACCEL); break; // M201 A<accel> change acceleration
//   default:  break;
//   }
}




////////////////////////////////////////////////////////////////////////
inline static  void toggleIO(Pio *portIO, uint32_t pin)
{

   //PIOB->PIO_ODSR ^= PIO_ODSR_P27;   // Toggle LED BY OUTPUT STATUS REGISTER
   portIO->PIO_ODSR ^= pin;

}

void TC0_Handler()
{
  TC_GetStatus(TC0, 0);
  if(srd[0].activated)
  {

      toggleIO(PIOB, PIO_OER_P26);
      toggleIO(PIOB, PIO_OER_P26);

     // NVIC_DisableIRQ(TC0_IRQn);
      TC_SetRC(TC0, 0, srd[0].Cn);
     // NVIC_EnableIRQ(TC0_IRQn);

++srd[0].step_count;
// C =  srd[num].Cn*(1.0- 2.0/(4.0 * step_count + 1.0))+srd[num].rest ;
// srd[num].Cn = floor(C);
//srd[num].rest =  C-srd[num].Cn;
 
if(srd[0].step_count<srd[0].Na)
  {
        if(srd[0].step_count==0)
        {
            srd[0].Cn = srd[0].Cn_acc0;
            srd[0].rest = 0;
        }
        else
        {
          //%%% SQRT approximation of srd[num].Cn = srd[num].Cn_acc0*(sqrt(step_count+1)-sqrt(step_count));
          //srd[0].C =  srd[0].Cn*(1.0- 2.0/(4.0 * srd[0].step_count + 1.0))+srd[0].rest ;

          srd[0].C =  srd[0].Cn*(1- 2/(4.0 * srd[0].step_count + 1.0))+srd[0].rest ;

          srd[0].Cn = floor(srd[0].C);
          srd[0].rest =  srd[0].C-srd[0].Cn;
          //srd[num].Cn = srd[num].Cn*(1.0- 2.0/(4.0 * step_count + 1.0));
          // uint32_t Cn = srd[num].Cn;
          // srd[num].Cn =  Cn+ Cn*( 2 + srd[num].rest )/(4 * step_count + 1);
          // srd[num].rest = (2 * Cn + srd[num].rest)%(4 * step_count +1);
          // mkSerial.println( srd[num].Cn);

        }
  }

  
  // else if(srd[0].step_count>=srd[0].Na && srd[0].step_count<(srd[0].Nac))
  // {
  //     //%%% SQRT approximation 

  //       srd[0].Cn = srd[0].Cn_const_speed ;
      
  // }
 
  else if( srd[0].step_count>=srd[0].Nac && srd[0].step_count<srd[0].steps)
  {
    
      //%%% SQRT approximation
      if(srd[0].step_count==srd[0].Nac)
      {
          srd[0].rest=0;
      }
      
      else {
        srd[0].C =  srd[0].Cn*(1.0+2.0/(4.0*srd[0].NNb-1.0))+srd[0].rest ;
        srd[0].Cn = floor(srd[0].C);
       srd[0].rest =  srd[0].C-srd[0].Cn;

       //srd[num].Cn = srd[num].Cn*(1.0+2.0/(4.0*srd[num].NNb-1.0));
       

       //uint32_t Cn = srd[num].Cn;
      // srd[num].Cn =  Cn+ Cn*(2+srd[num].rest )/(4 * srd[num].NNb - 1);
       //srd[num].rest = (2 * Cn + srd[num].rest)%(4 * srd[num].NNb +1);
      }
      srd[0].NNb--;
  }
  
  else if(srd[0].step_count>=srd[0].steps)
  {
    //  // NVIC_DisableIRQ(TC0_IRQn);
    //   // mkSerial.print("Total time[" );
    //   // mkSerial.print(num);
    //   // mkSerial.print("]=:" );
    //   elapsedTime[0] = millis() - elapsedTime[0];
    //   mkSerial.println(elapsedTime[0]);
    //   //mkSerial.println(step_count);
    //   mkSerial.println("----------------stoped end -------------");
    //   srd[0].step_count = 0;
      gIRQ_flag[0]=1;

  }
  // else{
  //   //srd[0].Cn = srd[0].Cn_const_speed ;
  // }
}


      // ++gIRQ_step_count[0];
      // gIRQ_flag[0]=1;
    ////////////////////////////  
}
void TC3_Handler()
{
 TC_GetStatus(TC1, 0);
if(srd[1].activated)
  {
      // mkSerial.print("TC3: ");
      // mkSerial.println(srd[1].Cn);
  //mkSerial.println("TC1");
  //if(srd[1].activated)
  //{
     
      

      toggleIO(PIOA, PIO_OER_P15);
      toggleIO(PIOA, PIO_OER_P15);

     // NVIC_DisableIRQ(TC1_IRQn);
      TC_SetRC(TC1, 0, srd[1].Cn);
     // NVIC_EnableIRQ(TC1_IRQn);

      // ++gIRQ_step_count[1];
      // gIRQ_flag[1]=1;
  //}  ////////////////////////////  
   //
   ++srd[1].step_count;
   
  if(srd[1].step_count<srd[1].Na)
  {
        if(srd[1].step_count==0)
        {
            srd[1].Cn = srd[1].Cn_acc0;
            srd[1].rest = 0;
        }
        else
        {
          //%%% SQRT approximation of srd[num].Cn = srd[num].Cn_acc0*(sqrt(step_count+1)-sqrt(step_count));
         srd[1].C =  srd[1].Cn*(1- 2.0/(4 * srd[1].step_count + 1))+srd[1].rest ;
          srd[1].Cn = floor(srd[1].C);
          srd[1].rest =  srd[1].C-srd[1].Cn;
        }
  }

 

  else if( srd[1].step_count>=srd[1].Nac && srd[1].step_count<srd[1].steps)
  {
    
      //%%% SQRT approximation
      if(srd[1].step_count==srd[1].Nac)
      {
          srd[1].rest=0;
      }

      else {
       srd[1].C =  srd[1].Cn*(1.0+2.0/(4.0*srd[1].NNb-1.0))+srd[1].rest ;
        srd[1].Cn = floor(srd[1].C);
        srd[1].rest =  srd[1].C-srd[1].Cn;
      }
      srd[1].NNb--;
  }

  else if(srd[1].step_count>=srd[1].steps)
  {

    gIRQ_flag[1]=1;

  }
  

  } 
}
void TC2_Handler()
{
  static volatile uint32_t step_count=0;
      float C=0;
      uint16_t num=2;
      IRQn_Type IRQn=TC2_IRQn;
  //if(srd[2].activated)
  //{
      TC_GetStatus(TC0, 2);

      toggleIO(PIOD, PIO_OER_P1);
      toggleIO(PIOD, PIO_OER_P1);
      
      NVIC_DisableIRQ(TC2_IRQn);
      TC_SetRC(TC0, 2, srd[2].Cn);
      NVIC_EnableIRQ(TC2_IRQn);

      

      // ++gIRQ_step_count[2];
      // gIRQ_flag[2]=1;
      if(step_count<srd[num].Na)
  {
        if(step_count==0)
        {
            srd[num].Cn = srd[num].Cn_acc0;
            srd[num].rest = 0;
        }
        else
        {
          //%%% SQRT approximation of srd[num].Cn = srd[num].Cn_acc0*(sqrt(step_count+1)-sqrt(step_count));
          C =  srd[num].Cn*(1.0- 2.0/(4.0 * step_count + 1.0))+srd[num].rest ;
          srd[num].Cn = floor(C);
          srd[num].rest =  C-srd[num].Cn;
        }
  }

  else if(step_count>=srd[num].Na && step_count<(srd[num].Nac))
  {
      //%%% SQRT approximation 
      if(step_count==srd[num].Na){
        srd[num].Cn_const_speed = srd[num].Cn;
      } 
  }
  
  else if( step_count>=srd[num].Nac && step_count<srd[num].steps)
  {
    
      //%%% SQRT approximation
      if(step_count==srd[num].Nac)
      {
          srd[num].rest=0;
      }
      // C =  srd[num].Cn*(1.0+2.0/(4.0*srd[num].NNb-1.0))+srd[num].rest ;
      // srd[num].Cn = floor(C);
      // srd[num].rest =  C-srd[num].Cn;
      // srd[num].NNb--;
      else if(srd[num].NNb<0 || step_count>=srd[num].steps)
      {
        mkSerial.println("----------------stoped-------------");
        elapsedTime[num] = millis() - elapsedTime[num];
        mkSerial.println(elapsedTime[num]);
        mkSerial.println(step_count);

        //step_count8 = 0;
        NVIC_DisableIRQ(TC8_IRQn);

      }
      else if(srd[num].NNb<0 ||  srd[num].NNb==4295000000UL) {
          
          mkSerial.println("----------------stoped: rollover-------------");
          elapsedTime[num] = millis() - elapsedTime[num];
         mkSerial.println(elapsedTime[num]);
          mkSerial.println(step_count);
          mkSerial.println(elapsedTime[num]);
        //step_count = 0;
        NVIC_DisableIRQ(IRQn);

      }
      else {
        C =  srd[num].Cn*(1.0+2.0/(4.0*srd[num].NNb-1.0))+srd[num].rest ;
        srd[num].Cn = floor(C);
        srd[num].rest =  C-srd[num].Cn;
      }
      srd[num].NNb--;
  }
  
  else if(step_count>=srd[num].steps)
  {
      NVIC_DisableIRQ(IRQn);
      mkSerial.print("Total time[" );
      mkSerial.print(num);
      mkSerial.print("]=:" );
      elapsedTime[num] = millis() - elapsedTime[num];
      mkSerial.println(elapsedTime[num]);
      mkSerial.println(step_count);
      mkSerial.println("----------------stoped end -------------");
      //step_count8 = 0;

  }
  //}  ////////////////////////////  
  // volatile static uint32_t Rc=1000;
  // TC_GetStatus(TC0, 2);
  // toggleIO(PIOD, PIO_ODSR_P8);
  // toggleIO(PIOD, PIO_ODSR_P8);

  //   Rc -=1;
  //   if(Rc<=262)
  //   {
  //     Rc=262;
  //   }
      
  // NVIC_DisableIRQ(TC2_IRQn);
  // TC_SetRC(TC0, 2, Rc);
  // NVIC_EnableIRQ(TC2_IRQn);
        
}
void TC1_Handler()
{
  //if(srd[3].activated)
  //{


//////////////////////////////////////////////
TC_GetStatus(TC0, 1);
if(srd[2].activated)
  {
      // mkSerial.print("TC3: ");
      // mkSerial.println(srd[1].Cn);
  //mkSerial.println("TC1");
  //if(srd[1].activated)
  //{
     
      

      toggleIO(PIOD, PIO_OER_P1);
      toggleIO(PIOD, PIO_OER_P1);

     // NVIC_DisableIRQ(TC1_IRQn);
      TC_SetRC(TC0, 1, srd[2].Cn);
     // NVIC_EnableIRQ(TC1_IRQn);

      // ++gIRQ_step_count[1];
      // gIRQ_flag[1]=1;
  //}  ////////////////////////////  
   //
   ++srd[2].step_count;
   
  if(srd[2].step_count<srd[2].Na)
  {
        if(srd[2].step_count==0)
        {
            srd[2].Cn = srd[2].Cn_acc0;
            srd[2].rest = 0;
        }
        else
        {
          //%%% SQRT approximation of srd[num].Cn = srd[num].Cn_acc0*(sqrt(step_count+1)-sqrt(step_count));
         srd[2].C =  srd[2].Cn*(1.0- 2.0/(4.0 * srd[2].step_count + 1.0))+srd[2].rest ;
          srd[2].Cn = floor(srd[2].C);
          srd[2].rest =  srd[2].C-srd[2].Cn;
        }
  }

 

  else if( srd[2].step_count>=srd[2].Nac && srd[2].step_count<srd[2].steps)
  {
    
      //%%% SQRT approximation
      if(srd[2].step_count==srd[2].Nac)
      {
          srd[2].rest=0;
      }

      else {
       srd[2].C =  srd[2].Cn*(1.0+2.0/(4.0*srd[2].NNb-1.0))+srd[2].rest ;
        srd[2].Cn = floor(srd[2].C);
        srd[2].rest =  srd[2].C-srd[2].Cn;
      }
      srd[2].NNb--;
  }

  else if(srd[2].step_count>=srd[2].steps)
  {

    gIRQ_flag[2]=1;

  }
  

  } 

        
}
void TC4_Handler()
{
  volatile static uint32_t Rc=1000;
  TC_GetStatus(TC1, 1);
  toggleIO(PIOD, PIO_ODSR_P9);
  toggleIO(PIOD, PIO_ODSR_P9);

    Rc -=1;
    if(Rc<=262)
    {
      Rc=262;
    }
      
  NVIC_DisableIRQ(TC4_IRQn);
  TC_SetRC(TC1, 1, Rc);
  NVIC_EnableIRQ(TC4_IRQn);
        
}
void TC5_Handler()
{
  volatile static uint32_t Rc=1000;
  TC_GetStatus(TC1, 2);
  toggleIO(PIOD, PIO_ODSR_P8);
  toggleIO(PIOD, PIO_ODSR_P8);

    Rc -=1;
    if(Rc<=262)
    {
      Rc=262;
    }
      
  NVIC_DisableIRQ(TC5_IRQn);
  TC_SetRC(TC1, 2, Rc);
  NVIC_EnableIRQ(TC5_IRQn);
        
}

void TC6_Handler()
{
  volatile static uint32_t Rc=1000;
  TC_GetStatus(TC2, 0);
  toggleIO(PIOD, PIO_ODSR_P8);
  toggleIO(PIOD, PIO_ODSR_P8);

    Rc -=1;
    if(Rc<=262)
    {
      Rc=262;
    }
      
  NVIC_DisableIRQ(TC6_IRQn);
  TC_SetRC(TC2, 0, Rc);
  NVIC_EnableIRQ(TC6_IRQn);
        
}

void TC7_Handler()
{
  volatile static uint32_t Rc=1000;
  TC_GetStatus(TC2, 1);
  toggleIO(PIOD, PIO_ODSR_P8);
  toggleIO(PIOD, PIO_ODSR_P8);

    Rc -=1;
    if(Rc<=262)
    {
      Rc=262;
    }
      
  NVIC_DisableIRQ(TC7_IRQn);
  TC_SetRC(TC2, 1, Rc);
  NVIC_EnableIRQ(TC7_IRQn);
        
}

void TC8_Handler()
{
  
  
  // toggleIO(PIOD, PIO_ODSR_P8);
  // toggleIO(PIOD, PIO_ODSR_P8);
 
 //////////////////////////////////////////////
  

  TC_GetStatus(TC2, 2);
   //////////////////////////////////////////////
  NVIC_DisableIRQ(TC8_IRQn);
  TC_SetRC(TC2, 2, srd[7].Cn);
  NVIC_EnableIRQ(TC8_IRQn);
  //////////////////////////////////////////////
  toggleIO(PIOA, PIO_ODSR_P15);
  toggleIO(PIOA, PIO_ODSR_P15);

  //pulse_stepper_motor(0,srd[num].dir);
  ++gIRQ_step_count[7];
  gIRQ_flag[7]=1;

        
}
inline static  void setupIO_Output(Pio *portIO, uint32_t pin)
{
  //SETUP:  (PIN 13) B.27 
//  PIOB->PIO_PER = PIO_OER_P27; // PIO ENABLE
//  PIOB->PIO_OER = PIO_OER_P27; // OUTPUT ENALBE
//  PIOB->PIO_PUDR = PIO_OER_P27;// PULL-UP DISALBLE
  portIO->PIO_IDR = pin; // disable interrupt
  portIO->PIO_MDDR = pin;// Disable to connect multi devices 
  portIO->PIO_OWER = pin;// Enable writing on ODSR
  

  portIO->PIO_PER = pin; // PIO ENABLE
  portIO->PIO_OER = pin; // OUTPUT ENALBE
  portIO->PIO_PUDR = pin;// PULL-UP DISALBLE
  
}
 static  void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency) {
        pmc_set_writeprotect(false);
        pmc_enable_periph_clk((uint32_t)irq);
        //TC_Configure(tc, channel,  TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK2 | TC_CMR_ACPC_CLEAR | TC_CMR_ASWTRG_CLEAR);//  | TC_CMR_CPCSTOP);
        TC_Configure(tc, channel,  TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK1);//| TC_CMR_ACPC_CLEAR| TC_CMR_ASWTRG_CLEAR);// MCLK/8
        uint32_t rc = F_CPU/8/frequency; //128 because we selected TIMER_CLOCK4 above
        TC_SetRA(tc, channel, rc); 
        TC_SetRC(tc, channel, rc);
        TC_Start(tc, channel);
        tc->TC_CHANNEL[channel].TC_IER=TC_IER_CPCS;
        tc->TC_CHANNEL[channel].TC_IDR=~TC_IER_CPCS;
        NVIC_ClearPendingIRQ(irq);
        NVIC_EnableIRQ(irq);
}

 inline static bool velProfile(uint32_t step_count, uint16_t num, IRQn_Type IRQn)
{
  float C=0;
if(step_count<srd[num].Na)
  {
        if(step_count==0)
        {
            srd[num].Cn = srd[num].Cn_acc0;
            srd[num].rest = 0;
        }
        else
        {
          //%%% SQRT approximation of srd[num].Cn = srd[num].Cn_acc0*(sqrt(step_count+1)-sqrt(step_count));
          C =  srd[num].Cn*(1.0- 2.0/(4.0 * step_count + 1.0))+srd[num].rest ;
          srd[num].Cn = floor(C);
          srd[num].rest =  C-srd[num].Cn;
        }
  }

  else if(step_count>=srd[num].Na && step_count<(srd[num].Nac))
  {
      //%%% SQRT approximation 
      if(step_count==srd[num].Na){
        srd[num].Cn_const_speed = srd[num].Cn;
      } 
  }
  
  else if( step_count>=srd[num].Nac && step_count<srd[num].steps)
  {
    
      //%%% SQRT approximation
      if(step_count==srd[num].Nac)
      {
          srd[num].rest=0;
      }
      // C =  srd[num].Cn*(1.0+2.0/(4.0*srd[num].NNb-1.0))+srd[num].rest ;
      // srd[num].Cn = floor(C);
      // srd[num].rest =  C-srd[num].Cn;
      // srd[num].NNb--;
      else if(srd[num].NNb<0 || step_count>=srd[num].steps)
      {
        mkSerial.println("----------------stoped-------------");
        elapsedTime[num] = millis() - elapsedTime[num];
        mkSerial.println(elapsedTime[num]);
        mkSerial.println(step_count);

        //step_count8 = 0;
        NVIC_DisableIRQ(TC8_IRQn);
        return true;
      }
      else if(srd[num].NNb<0 ||  srd[num].NNb==4295000000UL) {
          
          mkSerial.println("----------------stoped: rollover-------------");
          elapsedTime[num] = millis() - elapsedTime[num];
         mkSerial.println(elapsedTime[num]);
          mkSerial.println(step_count);
          mkSerial.println(elapsedTime[num]);
        //step_count = 0;
        NVIC_DisableIRQ(IRQn);
        return true;
      }
      else {
        C =  srd[num].Cn*(1.0+2.0/(4.0*srd[num].NNb-1.0))+srd[num].rest ;
        srd[num].Cn = floor(C);
        srd[num].rest =  C-srd[num].Cn;
      }
      srd[num].NNb--;
      
      
     
  }
  
  else if(step_count>=srd[num].steps)
  {
      NVIC_DisableIRQ(IRQn);
      mkSerial.print("Total time[" );
      mkSerial.print(num);
      mkSerial.print("]=:" );
      elapsedTime[num] = millis() - elapsedTime[num];
      mkSerial.println(elapsedTime[num]);
      mkSerial.println(step_count);
      mkSerial.println("----------------stoped end -------------");
      //step_count8 = 0;
      
      return true;
  }
  
  return false;
}

/////////////////////////////////////////////////////

extern MKSerial mkSerial;
extern MKCommand mkCommand;

int main(void)
{
  char buffer[128];
  init_controller();
  mkCommand.setCallBack_gen_speed_profile(gen_speed_profile);
  mkCommand.setCallBack_update_speed_only(modify_speed_profile);
  mkSerial.begin(115200);
  mkSerial.println("Hello Mike!");

//PIO_Configure(PIOD, PIO_OUTPUT_1, PIO_PD8, PIO_DEFAULT);
  setupIO_Output(PIOB, PIO_OER_P26);// Pulse signal for Motor #1
  setupIO_Output(PIOA, PIO_OER_P14);// Direction signal for Motor #1
  pIODir[0].pIO = PIOA; pIODir[0].pin = PIO_OER_P14;


  setupIO_Output(PIOA, PIO_OER_P15);// Pulse signal for Motor #2
  setupIO_Output(PIOD, PIO_OER_P0);// Direction signal for Motor #2
  pIODir[1].pIO = PIOD; pIODir[1].pin = PIO_OER_P0;


  setupIO_Output(PIOD, PIO_OER_P1);// Pulse signal for Motor #3
  setupIO_Output(PIOD, PIO_OER_P2);// Direction signal for Motor #3
  pIODir[2].pIO = PIOD; pIODir[2].pin = PIO_OER_P2;

  setupIO_Output(PIOD, PIO_OER_P3);// Pulse signal for Motor #4
  setupIO_Output(PIOA, PIO_OER_P6);// Direction signal for Motor #4
  pIODir[3].pIO = PIOA; pIODir[3].pin = PIO_OER_P6;
 
  for(int i=0; i<8; i++) srd[i].Cn=256;// for testing
   
  startTimer(TC0, 0, TC0_IRQn, 350); //TC0 channel 0, the IRQ0 for that channel and the desired frequency
  // startTimer(TC1, 0, TC3_IRQn, 350); // TC3
  // startTimer(TC0, 1, TC1_IRQn, 350);// TC1
  // startTimer(TC0, 2, TC2_IRQn, 350);

  
  
  //startTimer(TC1, 1, TC4_IRQn, 1000);
  //startTimer(TC1, 2, TC5_IRQn, 350);
  //          
  //startTimer(TC2, 0, TC6_IRQn, 350);
  //startTimer(TC2, 1, TC7_IRQn, 350);
  //startTimer(TC2, 2, TC8_IRQn, 350);

   float scale=1;
  


  //  for(int i=0; i<10; i++);
  //  gen_speed_profile(1, 1600*30, 50.0, 80.0*scale, 80.0*scale);
  //  for(int i=0; i<10; i++);
  //  gen_speed_profile(2, 1600*30, 50.0, 80.0*scale, 80.0*scale);
  //  for(int i=0; i<10; i++);
  //  gen_speed_profile(1, 1600*60, 250.0, 80.0*4, 80.0*4);
  //  gen_speed_profile(2, 1600*60, 250.0, 80.0*4, 80.0*4);
  //  gen_speed_profile(3, 1600*60, 200.0, 80.0*4, 80.0*4);

//gen_speed_profile(7, 1600*30, 250.0, 80.0*5, 80.0*5);

  
  mkSerial.print("srd[num].Cn_acc0=");
  mkSerial.println(srd[8].Cn_acc0);
  ///////////////////////////////////////////////////


  /* Main loop */
  while(1) {
   // mkSerial.println("Hello Mike!");

    if(mkCommand.buflen < (BUFSIZE-1)){
        mkCommand.getCommand();
        
    }
    if(mkCommand.buflen) {
      mkSerial.println(mkCommand.buflen);
      mkCommand.process_commands();
      
       //gen_speed_profile(0, 1600*10, 100, 80.0*scale, 80.0*scale);
      //processCommand(); 
      mkCommand.buflen = (mkCommand.buflen-1);
      mkCommand.bufindr = (mkCommand.bufindr + 1)%BUFSIZE;
    }
    ////////////////////////////////////////////
    if(gIRQ_flag[0]==1)
    {
      NVIC_DisableIRQ(TC0_IRQn);
      // mkSerial.print("Total time[" );
      // mkSerial.print(num);
      // mkSerial.print("]=:" );
      elapsedTime[0] = millis() - elapsedTime[0];
      float eTime = elapsedTime[0]*0.001;
      float speed_mmPsec = srd[0].step_count*0.025/eTime;
      
      mkSerial.print("time[sec]: ");
      mkSerial.println(eTime);
      mkSerial.print("speed[mm/sec]: ");
      mkSerial.println(speed_mmPsec);
      //mkSerial.println(step_count);
      mkSerial.println("----------------0. toped end -------------");
      srd[0].step_count = 0;
      gIRQ_flag[0]=0;

      mkSerial.println("READY");
    } 
    
  }
  return 0;
}
/////////////////////////////////////////////
static void modify_speed_profile(uint16_t num,  uint32_t steps)
{
/*
  //#define SPR 1600 // (200*8); //% step per revolution
  float  alpha = (2*PI/1600);//  % [rad] angle per step
  float two_alpha = 2.0*alpha ;// alpha*2
  
  // % 1. Calcalate Time
  // % Ta = speed/acceleration 
  srd[num].Ta = speed/accel; //%[sec]
  srd[num].Tb = speed/decel; //%[sec]
  srd[num].Tc = (srd[num].step_count*alpha - 0.5*accel*srd[num].Ta*srd[num].Ta - 0.5*decel*srd[num].Tb*srd[num].Tb)/speed;
  if(srd[num].Tc>0)
  {
      srd[num].Ttotal = srd[num].Ta+srd[num].Tc+srd[num].Tb;
  }
  else
  {
      srd[num].Ttotal = srd[num].Ta+srd[num].Tb;
  }
  

  // % 2. Calculate Step
  // % convert Time Zone to Stepper Motor Count
  // % Acceleration Zone
  // % n = speed^2/(2*alpha*accel)

  srd[num]. Na = floor(speed*speed/(two_alpha*accel));
  uint32_t Nacc = floor(srd[num].steps*decel/(accel+decel));

  
  if(srd[num].Na<Nacc){
      //%Nb = floor(speed^2/(2*alpha*decel));
      srd[num].Nb = accel/decel*srd[num].Na;
      srd[num].Nc = srd[num].step_count - (srd[num].Na+srd[num].Nb);
  }
  else{
      srd[num].Na = Nacc;
      srd[num].Nb = srd[num].steps - Nacc;
      srd[num].Nc = 0;
  }
  srd[num].Nac = srd[num].Na + srd[num].Nc;
  srd[num].NNb = srd[num].Nb;
  srd[num].Cn_const_speed = uint32_t(TICK_FREQ*sqrt(two_alpha/accel)*(sqrt(srd[num].Na+1)-sqrt(srd[num].Na)));
  srd[num].Cn_acc0 = uint32_t(TICK_FREQ*sqrt(two_alpha/accel));
  srd[num].Cn_dec0 = uint32_t(TICK_FREQ*sqrt(two_alpha/decel));

  srd[num].Cn = srd[num].Cn_const_speed;
  */
  srd[num].Cn = steps;
  //mkSerial.println("modify_speed_profile");

}
 static  void gen_speed_profile(uint16_t num, float steps,  float speed, float accel, float decel, int dir)
{

  float distance = 0.025*steps;// [mm]
  mkSerial.print("Moving Distance[mm]=");
  mkSerial.println(distance);


  //if(speed>=MAX_SPEED_STEP) speed=MAX_SPEED_STEP; // Max speed...
  
  // if(sel_sqrt_cal==SQRT_APPRO){
  //   TICK_FREQ *= 0.676;//0.69367;//initial error: 0.69367 at n=1, ((sqrt(2)-sqrt(1)))/((1.0-2.0/(4.0+1.0)))
  // }
  
  //#define SPR 1600 // (200*8); //% step per revolution
  float  alpha = (2*PI/1600);//  % [rad] angle per step
  float two_alpha = 2.0*alpha ;// alpha*2
  
  // % 1. Calcalate Time
  // % Ta = speed/acceleration 
  srd[num].Ta = speed/accel; //%[sec]
  srd[num].Tb = speed/decel; //%[sec]
  srd[num].Tc = (steps*alpha - 0.5*accel*srd[num].Ta*srd[num].Ta - 0.5*decel*srd[num].Tb*srd[num].Tb)/speed;
  if(srd[num].Tc>0)
  {
      srd[num].Ttotal = srd[num].Ta+srd[num].Tc+srd[num].Tb;
  }
  else
  {
      srd[num].Ttotal = srd[num].Ta+srd[num].Tb;
  }
  

  // % 2. Calculate Step
  // % convert Time Zone to Stepper Motor Count
  // % Acceleration Zone
  // % n = speed^2/(2*alpha*accel)
  srd[num].steps = steps;
  srd[num]. Na = floor(speed*speed/(two_alpha*accel));
  uint32_t Nacc = floor(steps*decel/(accel+decel));

  
  if(srd[num].Na<Nacc){
      //%Nb = floor(speed^2/(2*alpha*decel));
      srd[num].Nb = accel/decel*srd[num].Na;
      srd[num].Nc = steps - (srd[num].Na+srd[num].Nb);
  }
  else{
      srd[num].Na = Nacc;
      srd[num].Nb = steps - Nacc;
      srd[num].Nc = 0;
  }
  srd[num].Nac = srd[num].Na + srd[num].Nc;
  srd[num].NNb = srd[num].Nb;
  srd[num].Cn_const_speed = uint32_t(TICK_FREQ*sqrt(two_alpha/accel)*(sqrt(srd[num].Na+1)-sqrt(srd[num].Na)));
  srd[num].Cn_acc0 = uint32_t(TICK_FREQ*sqrt(two_alpha/accel));
  srd[num].Cn_dec0 = uint32_t(TICK_FREQ*sqrt(two_alpha/decel));

  srd[num].Cn = srd[num].Cn_acc0;

  //gIRQ_step_count[num]=0;
  srd[num].step_count=0; 
  if(dir==1){
      pIODir[num].pIO->PIO_ODSR = pIODir[num].pin;
      pIODir[num].pIO->PIO_CODR = pIODir[num].pin;
  }
  else {
      pIODir[num].pIO->PIO_ODSR ^= pIODir[num].pin;
      pIODir[num].pIO->PIO_SODR = pIODir[num].pin;
  }
  
  mkSerial.println("----------------start -------------");
  mkSerial.print("srd[num].Cn=");
  mkSerial.println(srd[num].Cn);
  mkSerial.print("Total time[" );
  mkSerial.print(num);
  mkSerial.print("]=:" );
  mkSerial.println(srd[num].Ttotal,4 );
  mkSerial.println(steps);
  mkSerial.println(srd[num].Cn_acc0);
  mkSerial.println(srd[num].Na);
  mkSerial.println(srd[num].Nb);
  mkSerial.println(srd[num].Nc);
  
  elapsedTime[num] = millis();
  TC_SetRA(TC0, 0, srd[num].Cn); 
  TC_SetRC(TC0, 0, srd[num].Cn);
  TC_Start(TC0, 0);
  NVIC_ClearPendingIRQ(TC0_IRQn);
  NVIC_EnableIRQ(TC0_IRQn);
  srd[num].activated=true;
}


