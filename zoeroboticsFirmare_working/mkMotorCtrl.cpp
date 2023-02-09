
#include <string.h>
#include "mkMotorCtrl.h"


#include "mkZoeRobotics_serial.h"
//#include "mkZoeRobotics_define.h"

extern MKSerial mkSerial;

MKStepperMotorCtrl::MKStepperMotorCtrl()
{
   IOMapDir.pIO=NULL;
   IOMapDir.pin=0;
   IOMapPulse.pIO=NULL;
   IOMapPulse.pin=0;

  //  setPulseMap(   PIO_OER_P13);
  //  setDirectionMap( PIO_OER_P12);
}
MKStepperMotorCtrl::~MKStepperMotorCtrl()
{

}
void MKStepperMotorCtrl::setDirectionMap(Pio * pIO, int pinNum)
{
  mkSerial.println("setDirectionMap!");
  IOMapDir.pIO = (Pio *)pIO;
  IOMapDir.pin = pinNum; 
  pinModeAsOutput(pIO, pinNum);
}
void MKStepperMotorCtrl::setDirectionMap(uint32_t pinNum)
{
  pinDir = pinNum; 
}
void MKStepperMotorCtrl::setPulseMap(uint32_t pinNum)
{
  pinPulse = pinNum; 
}
void MKStepperMotorCtrl::setPulseMap(Pio * pIO, int pinNum)
{
   mkSerial.println("setPulseMap!");
   IOMapPulse.pIO = (Pio *)pIO;
   IOMapPulse.pin = pinNum;
   pinModeAsOutput(pIO, pinNum);
}
void MKStepperMotorCtrl::writeMotorDirection(uint32_t dir)
{
   mkSerial.println("setMotorDirection()");
  if (dir)
  {
    IOMapDir.pIO->PIO_SODR = IOMapDir.pin;
  }
  else
  {
    IOMapDir.pIO->PIO_CODR = IOMapDir.pin;
  }

//   return;
//   if (dir == 1)
//   { // CCW
//    pioWriteOutput(IOMapDir.pIO, IOMapDir.pin, 0x1);
//   }
//   else
//   { // CW
//     pioWriteOutput(IOMapDir.pIO, IOMapDir.pin, 0x0);
//   }
}
void MKStepperMotorCtrl::writeMotorPulse(uint32_t value)
{
  mkSerial.println("writeMotorPulse()");
   if (value)
  {
    IOMapPulse.pIO->PIO_SODR = IOMapDir.pin;
  }
  else
  {
    IOMapPulse.pIO->PIO_CODR = IOMapDir.pin;
  }

   //return;
   // pioWriteOutput(IOMapPulse.pIO, IOMapPulse.pin, value);
}
void MKStepperMotorCtrl::pioWriteOutput(Pio *pPio, uint32_t pin, uint32_t val)
{
  // Disable Interrupt...
  // pPio->PIO_IDR = pin; //PIO_OER_P26 ;
  // // setup PULLUP (NOMALLY LOW)
  // pPio->PIO_PUER = pin;
  /* Set default value */
  if (val)
  {
    ((Pio *)pPio)->PIO_SODR = pin;
  }
  else
  {
    ((Pio *)pPio)->PIO_CODR = pin;
  }
  /* Configure pin(s) as output(s) */
  // pPio->PIO_OER = pin;
  // pPio->PIO_PER = pin;
}
uint8_t MKStepperMotorCtrl::pioReadInput(Pio *pPio, uint32_t pin)
{
  if (( ((Pio *)pPio)->PIO_PDSR & pin) == 0)
    return 0;
  else
    return 1;
}

void MKStepperMotorCtrl::pinModeAsOutput(Pio *portIO, uint32_t pin)
{
  portIO->PIO_IDR = pin;  // disable interrupt
  portIO->PIO_MDDR = pin; // Disable to connect multi devices
  portIO->PIO_OWER = pin; // Enable writing on ODSR

  portIO->PIO_PER = pin;  // PIO ENABLE
  portIO->PIO_OER = pin;  // OUTPUT ENALBE
  portIO->PIO_PUDR = pin; // PULL-UP DISALBLE
  //portIO->PIO_PUER = pin; // PULL-UP ENALBLE
}
void MKStepperMotorCtrl::pinModeAsInput(Pio *portIO, uint32_t pin)
{
  // pmc_enable_periph_clk(PIOB_IRQn);
  // NVIC_DisableIRQ(PIOB_IRQn);
  // NVIC_ClearPendingIRQ(PIOB_IRQn);
  // NVIC_SetPriority(PIOB_IRQn, 1);
  // NVIC_EnableIRQ(PIOB_IRQn);

  portIO->PIO_PER = pin;  // PIO ENABLE
  portIO->PIO_ODR = pin;  // OUTPUT DISABLE (INPUT ENABLED)
  portIO->PIO_IDR = pin;  // disable interrupt
  portIO->PIO_PUDR = pin; // PULL-UP  DISABLE
  // portIO->PIO_PUER = pin; // PULL-UP  ENABLE
  //portIO->PIO_IFER = pin; // INPUTFILTER ENABLE
  portIO->PIO_IFDR = pin;

  //portIO->PIO_MDDR = pin; // Disable to connect multi devices
  //portIO->PIO_OWER = pin; // Enable writing on ODSR

  //

  // portIO->PIO_AIMDR = pin; //any change

  // portIO->PIO_ESR = pin;    // "Edge" Select Register
  // portIO->PIO_FELLSR = pin; // "Falling Edge / Low Level" Select Register

  // portIO->PIO_LSR = pin;    // "Level" Select Register
  // portIO->PIO_FELLSR = pin; // "Falling Edge / Low Level" Select Register

  // portIO->PIO_IER = pin; // Enable interrupt

  // portIO->PIO_LSR = pin;    // "Level" Select Register
  // 		portIO->PIO_REHLSR = pin; // "Rising Edge / High Level" Select Register
}