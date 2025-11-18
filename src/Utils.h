#pragma once

// #include "app.h"
#include "common.h"
#include "daisysp.h"
#include <bitset>

// Utils class
class Utils
{
  public:
    // Function to test if a touchpad state has changed
    static bool hasTouchStateChanged(const std::bitset<16> &current, const std::bitset<16> &previous, size_t index)
    {
      // Check if the state at the given index has changed
      return current.test(index) != previous.test(index);
    }

    // Function to test if a touchpad state has changed to pressed
    static bool hasTouchStateChangedToPressed(const std::bitset<16> &current, const std::bitset<16> &previous, size_t index)
    {
      // Check if the state at the given index has changed
      return current.test(index) && !previous.test(index);
    }

    // Function to test if a touchpad state has changed to released
    static bool hasTouchStateChangedToReleased(const std::bitset<16> &current, const std::bitset<16> &previous, size_t index)
    {
      // Check if the state at the given index has changed
      return !current.test(index) && previous.test(index);
    }

    // Function to test if the Alt pad is pressed
    static bool isAltPadPressed(const std::bitset<16> &current)
    {
      // Check if the Alt pad (index 11) is pressed
      return isTouchPadPressed(current, 11);
    }

    // Function to test if the Spotykach pad is pressed
    static bool isSpotykachPadPressed(const std::bitset<16> &current)
    {
      // Check if the Spotykach pad (index 10) is pressed
      return isTouchPadPressed(current, 10);
    }

    static bool isTouchPadPressed(const std::bitset<16> &current, size_t index)
    {
      // Check if the Touch pad is pressed
      return current.test(index);
    }

    static void audioBlockLerp (const float *in1, const float *in2, float *out, float mix, size_t blockSize)
    {
      // Perform linear interpolation between two audio blocks
      for (size_t i = 0; i < blockSize; i++)
      {
        out[i] = infrasonic::lerp(in1[i], in2[i], mix);
      }
    }

    // SmoothValue class for smoothing parameter changes
    class SmoothValue
    {
      public:
        SmoothValue () : SmoothValue(0.0f, 0.0f, 0.0f) {};

        SmoothValue (float smoothTimeMs, float updatePeriodMs) : SmoothValue(0.0f, smoothTimeMs, updatePeriodMs) { }

        SmoothValue (float currentValue, float smoothTimeMs, float updatePeriodMs)
        {
          smoothing_    = false;
          currentValue_ = currentValue;
          targetValue_  = currentValue;

          // coeff = 100.0 / (time * sample_rate), where time is in seconds
          float updateRate = 1000.0f / updatePeriodMs;
          filterCoeff_     = 100.0f / ((smoothTimeMs / 1000.0f) * updateRate);
        }

        // Overload assignment to set targetValue
        SmoothValue &operator=(float v)
        {
          if (std::abs(v - targetValue_) > 0.01f)
          {
            targetValue_ = v;
            // Check if the value is undergoing smoothing
            setSmoothing(targetValue_, currentValue_);
          }
          return *this;
        }

        // Overload the += operator to add to the target value
        SmoothValue &operator+=(float v)
        {
          targetValue_ += v;
          // Check if the value is undergoing smoothing
          setSmoothing(targetValue_, currentValue_);
          return *this;
        }

        // Get the smoothed value
        float getSmoothVal ()
        {
          daisysp::fonepole(currentValue_, targetValue_, filterCoeff_);
          // Check if the value is undergoing smoothing
          setSmoothing(targetValue_, currentValue_);
          return currentValue_;
        }

        // Get the target value
        float getTargetVal () const { return targetValue_; }

        // Check if the value has smoothing
        bool isSmoothing () const { return smoothing_; }

      private:
        bool  smoothing_;
        float currentValue_;
        float targetValue_;
        float filterCoeff_;

        // Determine if the value has smoothing
        void setSmoothing (float oldValue, float currentValue)
        {
          // If the current value is more that 1% away from the old value, mark as smoothing
          if (std::abs((currentValue - oldValue) / oldValue) > 0.01f)
          {
            smoothing_ = true;
          }
          else
          {
            smoothing_ = false;
          }
        }
    };
};
