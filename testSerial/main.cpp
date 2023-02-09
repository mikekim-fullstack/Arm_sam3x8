/**
mkMain.cpp
Copyright (c) 2020 Mike Kim (ZOE Robotics).  All right reserved.
4 Sept 2020 by Mike Kim
*/
#include <stdio.h>
#include <math.h>
#include <string.h>
// #include <iostream>
// #include <sstream>
#include <assert.h>
#include "mkZoeRobotics_define.h"
#include "../include/due_sam3x.init.h"

//#include "timetick.h"

#include "mkZoeRobotics_globalDataStruct.h"
#include "mkZoeRobotics_serial.h"

SERIAL_BUFFER_DATA serialSendBuf;
////////////////////////////////////////////////////////
// Handling Velocity Profile
void init_interrupt()
{
    pmc_enable_periph_clk(ID_PIOA);
    NVIC_DisableIRQ(PIOA_IRQn);

    NVIC_ClearPendingIRQ(PIOA_IRQn);
    NVIC_SetPriority(PIOA_IRQn, 0);
    NVIC_EnableIRQ(PIOA_IRQn);
    
    pmc_enable_periph_clk(ID_PIOB);
    NVIC_DisableIRQ(PIOB_IRQn);
    NVIC_ClearPendingIRQ(PIOB_IRQn);
    NVIC_SetPriority(PIOB_IRQn, 0);
    NVIC_EnableIRQ(PIOB_IRQn);
    
    pmc_enable_periph_clk(ID_PIOC);
    NVIC_DisableIRQ(PIOC_IRQn);
    NVIC_ClearPendingIRQ(PIOC_IRQn);
    NVIC_SetPriority(PIOC_IRQn, 0);
    NVIC_EnableIRQ(PIOC_IRQn);
    
    pmc_enable_periph_clk(ID_PIOD);
    NVIC_DisableIRQ(PIOD_IRQn);
    NVIC_ClearPendingIRQ(PIOD_IRQn);
    NVIC_SetPriority(PIOD_IRQn, 0);
    NVIC_EnableIRQ(PIOD_IRQn);
}


#include <stdbool.h>
int main(void)
{
    //////////////////////////////////////////////////////
    //char buffer[128];
    
    init_controller();
    //mkSerial.begin(115200);
    mkSerial.begin(250000);
       // mkSerial.begin(500000);
    serialSendBuf.reset();  
    // mkSerial.println("FROM ZOEROBOTICS CONTROLLER!");  
    //serialSendBuf.write("FROM ZOEROBOTICS CONTROLLER!");//test  

    while(1){
        
       // if (serialSendBuf.buflen)
        {
            //if(mkSerial.isEmpty())
            {
                //mkSerial.println(serialSendBuf.read());
                 mkSerial.println("test");
                 for(long int i =0; i<200; i++){
                   // mkSerial.flush();
                 }

                Sleep(100);
            }
            
        }// -------------- End of ALL Processes-------------
        ////////////////////////////////////////////////////////
        
    }// ---------- End of While(1) --------------
    return 0;
}

