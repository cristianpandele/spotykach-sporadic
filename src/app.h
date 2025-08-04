#pragma once
#include "hardware.h"
#include <daisy_seed.h>
namespace spotykach_hwtest
{
  // Definitions
  #define PRINT_CPU_LOAD

  // Constants
  constexpr size_t kSampleRate           = 48000;
  constexpr size_t kBlockSize            = 16;

  constexpr size_t kNumberSpotykachSides = 2;
  constexpr size_t kNumberChannelsStereo = 2;
  constexpr size_t kNumberChannelsMono   = 1;

  // Enum for Size/Pos switch position
  enum SizePosSwitchState
  {
    SIZE,
    POSITION,
    BOTH
  };

  // SmoothValue class for smoothing parameter changes
  class SmoothValue
  {
    public:
      SmoothValue (float smoothTimeMs, float sampleRate)
      {
        smoothing    = false;
        currentValue = 0.0f;
        targetValue  = 0.0f;
        filterCoeff  = 100000.0f / (smoothTimeMs * sampleRate);
      }

      // Overload assignment to set targetValue
      SmoothValue &operator=(float v)
      {
        targetValue = v;
        // Check if the value is undergoing smoothing
        setSmoothing();
        return *this;
      }

      // Overload the += operator to add to the target value
      SmoothValue &operator+=(float v)
      {
        targetValue += v;
        // Check if the value is undergoing smoothing
        setSmoothing();
        return *this;
      }

      // Get the smoothed value
      float getSmoothVal ()
      {
        daisysp::fonepole(currentValue, targetValue, filterCoeff);
        // Check if the value is undergoing smoothing
        setSmoothing();
        return currentValue;
      }

      // Get the target value
      float getTargetVal () const { return targetValue; }

      // Check if the value has smoothing
      bool isSmoothing () const { return smoothing; }

    private:
      bool  smoothing;
      float currentValue;
      float targetValue;
      float filterCoeff;

      // Determine if the value has smoothing
      void setSmoothing ()
      {
        // If the current value is more that 1% away from the target, mark as smoothing
        if (std::abs(currentValue - targetValue) / (std::abs(targetValue) > 0.01f))
        {
          smoothing = true;
        }
        else
        {
          smoothing = false;
        }
      }
  };

  // Application class
  class Application
  {
    public:
      Application ()  = default;
      ~Application () = default;

      void Init ();
      void Loop ();

    private:
      Application (const Application &a)           = delete;
      Application &operator=(const Application &a) = delete;
  };

  using namespace daisy;
  using namespace spotykach;

  // Class for application implementation
  class AppImpl
  {
    public:
      // Application routing modes
      enum AppMode
      {
        OFF,
        ROUTING_DUAL_MONO,
        ROUTING_DUAL_STEREO,
        ROUTING_GENERATIVE,
        ROUTING_LAST
      };

      AppImpl ()  = default;
      ~AppImpl () = default;

      void init ();
      void loop ();

      // Audio processing functions for the Spotykach looper and Sporadic effect
      void processAudioLogic (AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size);

      // This is the main audio processing function that will be called from the AudioCallback
      void processAudio (AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size);

    private:
      Hardware       hw;
      StopwatchTimer led_timer;
      StopwatchTimer midi_timer;

      UiEventQueue ui_queue;

      PotMonitor<Hardware, Hardware::kNumAnalogControls> pot_monitor;

      // Routing mode changed flag and current routing mode
      bool    routingModeChanged                      = false;
      AppMode currentRoutingMode                      = AppMode::OFF;

      // Mix controls for the two sides
      float   mixControls[kNumberSpotykachSides]     = { 0.5f };

      // Smooth pitch for each side
      SmoothValue pitchControls[kNumberSpotykachSides] = { SmoothValue(150.0f, kSampleRate), SmoothValue(150.0f, kSampleRate) };

      // Position knob and position/size switch state for each side
      float positionControls[kNumberSpotykachSides] = {0.0f, 0.0f};
      SizePosSwitchState sizePosSwitch[kNumberSpotykachSides] = {SizePosSwitchState::SIZE, SizePosSwitchState::SIZE};

      uint16_t last_pot_moved_a;
      uint16_t last_pot_moved_b;

      bool    test_note_on;
      bool    midi_in_note_on;
      uint8_t midi_in_nn;

      daisysp::Oscillator osc[8];

#if DEBUG
      StopwatchTimer log_timer;
      void           logDebugInfo ();
#endif
      // Set the current operating mode of the application
      void setRoutingMode (AppMode mode);

      void processUIQueue ();
      void processMidi ();

      void drawRainbowRoad ();

      void handleAnalogControls ();
      void handleDigitalControls ();
      void handleDisplay ();

      void testSDCard ();

      AppImpl (const AppImpl &a)           = delete;
      AppImpl &operator=(const AppImpl &a) = delete;
  };
}    // namespace spotykach_hwtest
