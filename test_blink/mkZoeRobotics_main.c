/**
  mkZoeRobotics_main.c
  Copyright (c) 2020 Mike Kim (ZOE Robotics).  All right reserved.
  Modified 20 August 2020 by Mike Kim
*/
#include <stdint.h>
#include "stdio.h"
#include "due_sam3x.h"
#include "sam3.h"
#include "mkZoeRobotics_serial.h"
#include "timetick.h"
#define  __SAM3X8E__



#define MAX_CMD_SIZE 96
#define BUFSIZE 4
#define byte (unsigned char)
//////////////////////////////////////////////////////
/* Systick Register address, refer datasheet for more info */
#define STCTRL      (*( ( volatile unsigned long *) 0xE000E010 ))
#define STRELOAD    (*( ( volatile unsigned long *) 0xE000E014 ))
#define STCURR      (*( ( volatile unsigned long *) 0xE000E018 ))  

/*******STCTRL bits*******/
#define SBIT_ENABLE     0
#define SBIT_TICKINT    1
#define SBIT_CLKSOURCE  2


/* 84000000Mhz * 1ms = 840000 - 1 */
#define RELOAD_VALUE  83999

static volatile uint32_t dwTickCount=0 ;
// void SysTick_Handler (void) __attribute__ ((weak));
// extern void SysTick_Handler(void) 
// {
//   ++dwTickCount;
//   mkSerial.write("tick.\n");
// }

static __INLINE uint32_t mkSysTick_Config(uint32_t ticks)
{
  if (ticks > SysTick_LOAD_RELOAD_Msk)  return (1);            /* Reload value impossible */

  SysTick->LOAD  = (ticks & SysTick_LOAD_RELOAD_Msk) - 1;      /* set reload register */
  //NVIC_SetPriority (SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 1);  /* set Priority for Cortex-M0 System Interrupts */
  SysTick->VAL   = 0;                                          /* Load the SysTick Counter Value */
  SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
                   SysTick_CTRL_TICKINT_Msk   |
                   SysTick_CTRL_ENABLE_Msk;                    /* Enable SysTick IRQ and SysTick Timer */
  NVIC_EnableIRQ((IRQn_Type) SysTick_IRQn);
  return (0);                                                  /* Function successful */

}
//////////////////////////////////////////////////////
static void setup();
static void loop();
static int cnt=0;
int main()
{
    setup();
    while(1)
    {
        loop();
    }
    
    return 0;
}
///////////////////////////////////////////////////////////////////////////////
extern MKSerial mkSerial;
static int buflen = 0;
static uint8_t serial_char;
static int serial_count = 0;
static bool comment_mode = false;

static bool relative_mode = false;  //Determines Absolute or Relative Coordinates

static char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];
static bool fromsd[BUFSIZE];
static int bufindr = 0;
static int bufindw = 0;
static char *strchr_pointer; // just a pointer to find chars in the command string like X, Y, Z, E, etc

//static float tt = 0;
//static float bt = 0;

//Inactivity shutdown variables
static unsigned long previous_millis_cmd = 0;
static unsigned long max_inactive_time = 0;
//static unsigned long stepper_inactive_time = DEFAULT_STEPPER_DEACTIVE_TIME*1000l;


static unsigned long timebuff=0;
static unsigned long deltatime=0;
static unsigned long lasttime=0;
bool Stopped=false;

static void getCommand();
//extern uint32_t GetTickCount( void );
// extern void SysTick_Handler( void )
// {
//   // Increment tick count each ms
//   TimeTick_Increment() ;
// }
////////////////////////////////////////////////////////////////////////////////
static volatile uint32_t time_prev;
extern uint32_t SystemCoreClock;
void setup()
{
    //** Initializing System:: so important **//
    SystemInit();
    
    WDT->WDT_MR = WDT_MR_WDDIS;
    //** ---------------------------------- **//
     mkSerial.begin(250000);
    TimeTick_Configure(F_CPU) ; // Reload value for 1ms tick

  //   /* Enable the Systick, Systick Interrup and select CPU Clock Source */
  //   STCTRL = (1<<SBIT_ENABLE) | (1<<SBIT_TICKINT) | (1<<SBIT_CLKSOURCE);
  //   NVIC_EnableIRQ((IRQn_Type) SysTick_IRQn);







     //  time_prev= GetTickCount() ;
   //   time_prev= GetTickCount() ;
//SysTick_Config(0);
   
  //  mkSerial.begin(115200);
    mkSerial.write("Hello Mike.\n");
    // mkSerial.println(time_prev);
}
void loop()
{
    if(buflen < (BUFSIZE-1)){
        getCommand();
    }
    mkSerial.println("Hello Mike.");
    Sleep(100);
}
/////////////////////////////////////////////////////////////////////////////////////
void getCommand()
{
   while( mkSerial.available() > 0  && buflen < BUFSIZE) {
    serial_char = mkSerial.read();
         //uint32_t time_current = dwTickCount - time_prev ;
         

    if(serial_char == '\n' ||
       serial_char == '\r' ||
       (serial_char == ':' && comment_mode == false) ||
       serial_count >= (MAX_CMD_SIZE - 1) )
    {
      
    //   if(!serial_count) { //if empty line
    //     comment_mode = false; //for new command
    //     return;
    //   }
      //SERIAL_PROTOCOLLNPGM(MSG_OK);
      cmdbuffer[bufindw][serial_count] = 0; //terminate string
      //if(!comment_mode)
      {
       
        comment_mode = false; //for new command
        fromsd[bufindw] = false;
        

        mkSerial.print("I got this: ");
        mkSerial.println(cmdbuffer[bufindw]);
        if((strchr(cmdbuffer[bufindw], 'G') != NULL)){
          //mkSerial.println("Found G");
          strchr_pointer = strchr(cmdbuffer[bufindw], 'G');
          switch((int)((strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL)))){
          case 0:
          case 1:
          case 2:
          case 3:
            if(Stopped == false) { // If printer is stopped by an error the G[0-3] codes are ignored.
              //mkSerial.println(MSG_OK);
            }
            else {
             // mkSerial.println(MSG_ERR_STOPPED);
            }
            break;
          default:
            break;
          }

        }
        bufindw = (bufindw + 1)%BUFSIZE;
        buflen += 1;
      }
      serial_count = 0; //clear buffer
      buflen=0;
    }
    else
    {
      if(serial_char == ';') comment_mode = true;
      if(!comment_mode)
      {
       cmdbuffer[bufindw][serial_count++] = serial_char;
      //  mkSerial.print(serial_char);
      //  mkSerial.print('\n');
       }
    }
    
  }

}