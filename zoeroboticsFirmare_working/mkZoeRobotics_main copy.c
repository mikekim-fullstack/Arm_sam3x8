/**
  mkZoeRobotics_main.c
  Copyright (c) 2020 Mike Kim (ZOE Robotics).  All right reserved.
  Modified 20 August 2020 by Mike Kim
*/
#include <stdint.h>
#include "stdio.h"
#include "sam3.h"
#include "mkZoeRobotics_serial.h"
#include "UARTClass.h"
#define MAX_CMD_SIZE 96
#define BUFSIZE 4
#define byte (unsigned char)

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

////////////////////////////////////////////////////////////////////////////////
void setup()
{
    //** Initializing System:: so important **//
    SystemInit();
    WDT->WDT_MR = WDT_MR_WDDIS;
    //** ---------------------------------- **//
  

    mkSerial.begin(250000);
  //  mkSerial.begin(115200);
    mkSerial.write("Hello Mike.\n");

}
void loop()
{
    if(buflen < (BUFSIZE-1)){
        getCommand();
    }
}
/////////////////////////////////////////////////////////////////////////////////////
void getCommand()
{
   while( mkSerial.available() > 0  && buflen < BUFSIZE) {
    serial_char = mkSerial.read();

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