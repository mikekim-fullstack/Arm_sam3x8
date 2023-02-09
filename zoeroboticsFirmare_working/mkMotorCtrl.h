
#ifndef _MKSTEPPERMOTORCTRL_
#define _MKSTEPPERMOTORCTRL_

   #include "mkZoeRobotics_globalDataStruct.h"
   class MKStepperMotorCtrl
   {
      public:
      portIOPair IOMapDir;
      portIOPair IOMapPulse;
      uint32_t pinDir, pinPulse;
      public:
      MKStepperMotorCtrl();
      ~MKStepperMotorCtrl();
      void setDirectionMap(Pio * pIO, int pinNum);
      void setDirectionMap(uint32_t pinNum);
      void setPulseMap(Pio * pIO, int pinNum);
      void setPulseMap(uint32_t pinNum);
      inline void writeMotorDirection(uint32_t dir);
      inline void writeMotorPulse(uint32_t value);
      void pioWriteOutput(Pio *pPio, uint32_t pin, uint32_t val);
      uint8_t pioReadInput(Pio *pPio, uint32_t pin);
      void pinModeAsOutput(Pio *portIO, uint32_t pin);
      void pinModeAsInput(Pio *portIO, uint32_t pin);
   };
   

#endif //_MKSTEPPERMOTORCTRL_