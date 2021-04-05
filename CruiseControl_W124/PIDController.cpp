/**********************************************************************************************
   Based on Arduino PID Library - Version 1.2.1
   by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
   
   https://github.com/br3ttb/Arduino-PID-Library
 **********************************************************************************************/
#include "PIDController.h"

namespace pid {

    void PIDController::init(PIDConfig *kConfig, float *kInput, float *kSetPoint, float *kOutput) {
      mConfig = kConfig;
      mOutput = kOutput;
      mInput = kInput;
      mSetPoint = kSetPoint;

      lastInput = *mInput;
      inAuto = mConfig->getMode() == PIDConfig::kModeAutomatic;
      lastTime = mConfig->getClock() - mConfig->getSampleTime();

      if (inAuto) {
        integrator = *mSetPoint - *mInput;
      }
    }

    void PIDController::init() {
      float outMin = mConfig->getOutputMinLimit();
      float outMax = mConfig->getOutputMaxLimit();

      integrator = *mOutput;
      lastInput = *mInput;

      if (integrator > outMax) {
        integrator = outMax;
      } else if (integrator < outMin) {
        integrator = outMin;
      }
    }
  
    void PIDController::compute() {
      if (!inAuto) return;

      uint32_t now = mConfig->getClock();
      uint32_t timeChange = (now - lastTime);

      if (timeChange < mConfig->getSampleTime()) {
        return;
      }

      float outMin = mConfig->getOutputMinLimit();
      float outMax = mConfig->getOutputMaxLimit();

      
      float input = *mInput;
      float error = *mSetPoint - input;
      float dInput = (input - lastInput);
      
      integrator += mConfig->getKi() * error;

      if (integrator > outMax) {
        integrator = outMax;
      } else if (integrator < outMin) {
        integrator = outMin;
      }

      float output = mConfig->getKp() * error + integrator - mConfig->getKd() * dInput;

      if (output > outMax) { 
        output = outMax;
      } else if (output < outMin) {
        output = outMin;
      }

      *mOutput = output;

      lastLastInput = lastInput;
      lastInput = input;
      lastTime = now;
    }

    void PIDController::setMode(PIDConfig::PIDFlagType mMode) {
      bool newAuto = (mMode == PIDConfig::kModeAutomatic);
      
      if (newAuto && !inAuto) {
        init();
      } else {
        integrator = 0;
      }
      
      inAuto = newAuto;
      mConfig->setMode(mMode);
    }
}
