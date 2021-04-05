#include "PIDConfig.h"

namespace pid {
  
      uint32_t PIDConfig::getClock() { 
        return 0; 
      }

      uint16_t PIDConfig::getClockTicksPerSecond() {
        return mClockTicksPerSecond;
      }

      uint16_t PIDConfig::getSampleTime() {
        return mSampleTime;
      }

      float PIDConfig::getKp() {
        return mKp;
      }

      float PIDConfig::getKi() {
        return mKi;
      }

      float PIDConfig::getKd() {
        return mKd;
      }

      float PIDConfig::getOutputMinLimit() {
        return mOutputMinLimit;
      }

      float PIDConfig::getOutputMaxLimit() {
        return mOutputMaxLimit;
      }

      PIDConfig::PIDFlagType PIDConfig::getMode() const {
        return mMode;
      }

      PIDConfig::PIDFlagType PIDConfig::getControllerAction() const {
        return mAction;
      }

      void PIDConfig::setClockTicksPerSecond(uint16_t kClockTicksPerSecond) {
        mClockTicksPerSecond = kClockTicksPerSecond;
      }

      void PIDConfig::setOutputLimits(float kOutputMinLimit, float kOutputMaxLimit) {
        if (kOutputMinLimit >= kOutputMaxLimit) {
          return;
        }

        mOutputMinLimit = kOutputMinLimit;
        mOutputMaxLimit = kOutputMaxLimit;
      }

      void PIDConfig::setGains(float Kp, float Ki, float Kd, PIDFlagType kAction) {
        if (Kp < 0 || Ki < 0 || Kd < 0) return;

        float sampleTimeInSec = ((float) mSampleTime) / ((float) mClockTicksPerSecond);

        mKp = Kp;
        mKi = Ki * sampleTimeInSec;
        mKd = Kd / sampleTimeInSec;

        if (mMode == PIDConfig::kActionReverse) {
          mKp = (0 - mKp);
          mKi = (0 - mKi);
          mKd = (0 - mKd);
        } 
      }

      void PIDConfig::setGains(float Kp, float Ki, float Kd) {
        setGains(Kp, Ki, Kd, mAction);
      }

      void PIDConfig::setControllerAction(PIDFlagType kAction) {
        mAction = kAction;
      }

      void PIDConfig::setSampleTime(uint16_t kSampleTime) {
        if (kSampleTime > 0) {
          float ratio  = ((float) kSampleTime) / ((float) mSampleTime);
          mKi *= ratio;
          mKd /= ratio;
          mSampleTime = kSampleTime;

          setGains(mKp, mKi, mKd, mAction);
        }
      }

      void PIDConfig::setMode(PIDFlagType kMode) {
        mMode = kMode;
      }
}
