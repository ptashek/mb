/*
 * Author: 
 * Lukasz Szmit <ptashek@gmail.com>
 * 
 * License:
 * Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International 
 * CC BY-NC-SA 4.0
 * 
 */
#ifndef PID_CONFIG_LIBRARY_H
#define PID_CONFIG_LIBRARY_H

#include <Arduino.h>

namespace pid {
  class PIDConfig {

    public:
      typedef uint8_t PIDFlagType;

      // PIDController operation mode
      static const PIDFlagType kModeManual = 0x01;
      static const PIDFlagType kModeAutomatic = 0x02;

      // PIDController action mode
      static const PIDFlagType kActionDirect = 0x03;
      static const PIDFlagType kActionReverse = 0x04;
      
      virtual uint32_t getClock();
      
      uint16_t getClockTicksPerSecond();

      uint16_t getSampleTime();

      float getKp();

      float getKi();

      float getKd();

      float getOutputMinLimit();

      float getOutputMaxLimit();

      PIDFlagType getMode() const;

      PIDFlagType getControllerAction() const;

      void setClockTicksPerSecond(uint16_t kClockTicksPerSecond);

      void setOutputLimits(float kOutputMinLimit, float kOutputMaxLimit);

      void setGains(float Kp, float Ki, float Kd, PIDFlagType kAction);

      void setGains(float Kp, float Ki, float Kd);

      void setControllerAction(PIDFlagType kAction);

      void setSampleTime(uint16_t kSampleTime);

      void setMode(PIDFlagType kMode);

    private:
      PIDFlagType mMode = kModeManual;
      PIDFlagType mAction = kActionDirect;

      float mKp, mKi, mKd;
      uint16_t mSampleTime = 100;
      uint16_t mClockTicksPerSecond = 1000;

      float mOutputMinLimit = 0;
      float mOutputMaxLimit = 255;
  };
}
#endif
