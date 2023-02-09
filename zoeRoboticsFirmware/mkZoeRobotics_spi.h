/**
  mkZoeRobotics_spi.h 
  Copyright (c) 2020 Mike Kim (ZOE Robotics).  All right reserved.
  Modified 31 August 2020 by Mike Kim
*/

#ifndef _MKZOEROBOTICS_SPI_H
#define _MKZOEROBOTICS_SPI_H
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "mkZoeRobotics_globalDataStruct.h"
class MKSPI {
    public:
    volatile char DataReceived;
    volatile bool Flag_SPI;
    public:
    MKSPI(){

    }
    ~MKSPI() {

    }
    void init0Slave();
    void write(uint8_t val);
    void write(uint8_t * data, int len);
    void startWrite();// NSS lower(pulldown)
    void endWrite();// NSS Higher(pullup)

    char databuf[64];
    int recordlen[10];
    int count = 0;
};

extern MKSPI mkSPI;
#endif