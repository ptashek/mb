/**********************************************************************************************
   Based on Arduino PID Library - Version 1.2.1
   by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
   
   https://github.com/br3ttb/Arduino-PID-Library
 **********************************************************************************************/

#ifndef PID_CONTROLLER_LIBRARY_H
#define PID_CONTROLLER_LIBRARY_H
#define LIBRARY_VERSION	1.0.0

#include "PIDConfig.h"

namespace pid {

  class PIDConfig;
  
  class PIDController {
    public:
      void init(PIDConfig *kConfig, float *kInput, float *kSetPoint, float *kOutput);
      void setMode(PIDConfig::PIDFlagType mMode);
      void compute();

    private:
      void init();

      PIDConfig *mConfig;
      float *mInput;
      float *mOutput;
      float *mSetPoint;

      uint32_t lastTime;
      float lastInput;
      
      float lastLastInput;

      float integrator;
      bool inAuto;
  };
}
#endif
