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

      integrator = constrain(*mOutput, outMin, outMax);
      lastInput = *mInput;
    }
  
    bool PIDController::compute() {
      if (!inAuto) return false;

      uint32_t now = mConfig->getClock();
      uint32_t timeChange = (now - lastTime);

      if (timeChange < mConfig->getSampleTime()) {
        return false;
      }

      float outMin = mConfig->getOutputMinLimit();
      float outMax = mConfig->getOutputMaxLimit();

      float input;

      if (inputFilterEnabled) {
        input = mPrevInputRatio * lastInput + mCurInputRatio * *mInput;
      } else {
        input = *mInput;
      }
      
      float error = *mSetPoint - input;
      float dInput = (input - lastInput);

      integrator = constrain(integrator + mConfig->getKi() * error, outMin, outMax);
      float output = constrain(mConfig->getKp() * error + integrator - mConfig->getKd() * dInput, outMin, outMax);

      *mOutput = output;

      lastInput = input;
      lastTime = now;

      return true;
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

   void PIDController::enableInputFilter(float kPrevInputRatio) {
    if (kPrevInputRatio < 1) {
      mPrevInputRatio = kPrevInputRatio;
      mCurInputRatio = 1 - kPrevInputRatio;
      inputFilterEnabled = true;
    } else {
      inputFilterEnabled = false;
    }
   }

   void PIDController::disableInputFilter() {
    inputFilterEnabled = false;
    mPrevInputRatio = 0;
    mCurInputRatio = 0;
   }

}
