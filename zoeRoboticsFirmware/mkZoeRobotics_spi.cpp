/**
  mkZoeRobotics_spi.cpp 
  Copyright (c) 2020 Mike Kim (ZOE Robotics).  All right reserved.
  Modified 31 August 2020 by Mike Kim
*/
#include "mkZoeRobotics_spi.h"
#include "mkZoeRobotics_define.h"
MKSPI mkSPI;
void MKSPI::init0Slave()
{
    // --------------------- SELECT SPI0 AS SLAVE ------------------------

    // Enable the peripheral uart controller
    PMC->PMC_PCER0 |= 1u << ID_SPI0;

    // Disable the PIO of SPI pins so that the peripheral controller can use them
    PIOA->PIO_PDR = PIO_PDR_P25 | PIO_PDR_P26 | PIO_PDR_P27 | PIO_PDR_P28;

    // MOSI A26 (INPUT)
    // CLK A27 (INPUT)
    // MISO A25 (OUTPUT)
    // NPCS A28 (OUT SIGNAL: LOW -> TRANSFER DATA)

    // Define  INPUT 
    PIOA->PIO_ODR |=PIO_PA26A_SPI0_MOSI | PIO_PA27A_SPI0_SPCK;

    // Define  OUTPUT 
    PIOA->PIO_OER |=PIO_PA25A_SPI0_MISO ;

    // Read current peripheral AB select register and set SPI pins to 0 (Peripheral A function)
    PIOA->PIO_ABSR &= ~(PIO_PA25A_SPI0_MISO
                      | PIO_PA26A_SPI0_MOSI
                      | PIO_PA27A_SPI0_SPCK
                      | PIO_PA28A_SPI0_NPCS0);
    
    // // Enable the pull up on the SPI pin
    // PIOA->PIO_PUER = PIO_PA25A_SPI0_MISO 
    //                 | PIO_PA26A_SPI0_MOSI 
    //                 | PIO_PA27A_SPI0_SPCK 
    //                 | PIO_PA28A_SPI0_NPCS0 ;

    PIOA->PIO_PUDR |= PIO_PA27;	//SPCK - Active low
	  PIOA->PIO_PUER |= PIO_PA28;	//(PULLUP ACTIVE HIGH)
    // Slave mode
    SPI0->SPI_MR &= ~SPI_MR_MSTR; // 0

    // Variable Peripheral Selection for selecting active from 4 selection 
    // TD: Transmit Data
    // SPI0->SPI_MR |= SPI_MR_PS;

    // Fixed Peripheral Selection
    SPI0->SPI_MR &= ~SPI_MR_PS; // 0

    //
    
    // Initially SPI Disabled
    SPI0->SPI_CR = SPI_CR_SPIDIS;// SPI is in slave mode after software reset !!
    
    // Perform a SPI software reset twice, like SAM does.
    SPI0->SPI_CR = SPI_CR_SWRST;
    SPI0->SPI_CR = SPI_CR_SWRST;

    // // Data transfer parameters
    // SPI Bus Protocol Mode 
    // SPI0 MOde [CPOL=0,  NCPHA=1]
    SPI0->SPI_CSR[0] |= SPI_CSR_NCPHA       // Data is captured on the leading edge of SPCK and changed on the following edge
                      | SPI_CSR_CSNAAT      // Chip select active after transfer
                      | SPI_CSR_BITS_8_BIT // Bits per transfer
                      | SPI_CSR_SCBR(0)   // Delay Before SPCK 0: 1/CLK
                      | SPI_CSR_DLYBS(0); // 0: no delay 

    // Receive Data Register Full Interrupt and
    // Transmit Data Register Empty Interrupt Enable
    SPI0->SPI_IER |= SPI_IER_RDRF | SPI_IER_TDRE;
    NVIC_ClearPendingIRQ(SPI0_IRQn);
    //NVIC_EnableIRQ(SPI0_IRQn);
    //spi_set_writeprotect(SPI0, 1);
    SPI0->SPI_CR = SPI_CR_SPIEN;
}
void MKSPI::write(uint8_t val)
{
   // uint32_t status = SPI0->SPI_SR;
	//Wait for previous transfer to complete
	//while ((SPI0->SPI_SR & SPI_SR_TXEMPTY) == 0);
	
	//load the Transmit Data Register with the value to transmit
	SPI0->SPI_TDR = val;	
	
	//Wait for data to be transferred to serializer
	while ((SPI0->SPI_SR & SPI_SR_TDRE) == 0);
}
void MKSPI::write( uint8_t * data, int len)
{
    for(int i=0; i<len; i++)
    {
        write(*(data+i));
    }
}
void MKSPI::startWrite()
{
  
  PIOA->PIO_CODR = PIO_PA28;//LOWER VOLTAGE //PIO_PA28A_SPI0_NPCS0;
}
void MKSPI::endWrite()
{
  PIOA->PIO_SODR = PIO_PA28;// HIGHER VOLTAGE //PIO_PA28A_SPI0_NPCS0;
}
