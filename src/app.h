#pragma once
#include "Effect.h"
#include "Modulation.h"
#include "hardware.h"
#include <bitset>
#include <daisy_seed.h>

using spotykach::Hardware;
namespace spotykach_hwtest
{
  // Definitions
  #define PRINT_CPU_LOAD

  // Constants
  constexpr size_t kSampleRate           = 48000;
  constexpr size_t kBlockSize            = 16;

  constexpr uint8_t kNumberEffectSlots    = Hardware::kNumInPair;
  constexpr uint8_t kNumberChannelsStereo = 2;
  constexpr uint8_t kNumberChannelsMono   = 1;

  constexpr size_t kDebugLogPeriodMs      = 500;

  // Enum for Size/Pos switch position
  enum SizePosSwitchState
  {
    SIZE,
    POSITION,
    BOTH
  };

  // Utils class
  using ChannelConfig = Effect::ChannelConfig;
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

      // SmoothValue class for smoothing parameter changes
      class SmoothValue
      {
        public:
          SmoothValue (float smoothTimeMs, float sampleRate)
          {
            smoothing_    = false;
            currentValue_ = 0.0f;
            targetValue_  = 0.0f;
            // Frequency for smoothing these values is 500 Hz - roughly 100 times slower than the sample rate
            filterCoeff_  = 100.0f * (1000.0f / (smoothTimeMs * sampleRate));
          }

          // Overload assignment to set targetValue
          SmoothValue &operator=(float v)
          {
            targetValue_ = v;
            // Check if the value is undergoing smoothing
            setSmoothing(targetValue_, currentValue_);
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

      // Mapping from pad bit position (electrode) to LED
      // for illuminating on touch for test purposes
      // Bit position acts as index into array
      static constexpr Hardware::LedId kPadMapping[12] = {Hardware::LED_PLAY_A,
                                                          Hardware::LED_REV_A,
                                                          Hardware::LED_ORBIT_A,
                                                          Hardware::LED_DRIFT_A,
                                                          Hardware::LED_CYCLE_A,
                                                          Hardware::LED_CYCLE_B,
                                                          Hardware::LED_DRIFT_B,
                                                          Hardware::LED_ORBIT_B,
                                                          Hardware::LED_REV_B,
                                                          Hardware::LED_PLAY_B,
                                                          Hardware::LED_SPOTY_PAD,
                                                          Hardware::LED_ALT_A};

      static constexpr size_t  kPadMappingSize = sizeof(kPadMapping) / sizeof(kPadMapping[0]);
      static constexpr size_t  kPadMapPlayIds[kNumberEffectSlots]  = {0, 9};
      static constexpr size_t  kPadMapRevIds[kNumberEffectSlots]   = {1, 8};
      static constexpr size_t  kPadMapOrbitIds[kNumberEffectSlots] = {2, 7};
      static constexpr size_t  kPadMapDriftIds[kNumberEffectSlots] = {3, 6};
      static constexpr size_t  kPadMapCycleIds[kNumberEffectSlots] = {4, 5};
      static constexpr size_t  kPadMapSpotyId                      = 10;
      static constexpr size_t  kPadMapAltId                        = 11;

      AppImpl ()  = default;
      ~AppImpl () = default;

      void init ();
      void loop ();

      // Update the control frame with the current values
      void updateAnalogControlFrame (Effect::AnalogControlFrame &c, size_t effectSlot);

      // Update the control frame with the current values
      void updateDigitalControlFrame (Effect::DigitalControlFrame &c, size_t effectSlot);

      // Push the control frame to the effects
      void pushAnalogEffectControls (Effect::AnalogControlFrame &c, size_t effectSlot);

      // Push the control frame to the effects
      void pushDigitalEffectControls (Effect::DigitalControlFrame &c, size_t effectSlot);

      // Process the modulation controls for the specified effect slot
      void processModulatorControls (size_t slot);

      // Audio processing functions for the Spotykach looper and Sporadic effect
      void processAudioLogic (AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t blockSize);

      // This is the main audio processing function that will be called from the AudioCallback
      void processAudio (AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t blockSize);

    private:
      using ModType = ModulationEngine::ModType;
      static constexpr ModType modulationTypes[kNumberEffectSlots][ModulationEngine::kNumModsPerSide] =
        {
          {ModType::ENV_FOLLOWER,  ModType::S_H, ModType::SQUARE},
          {ModType::ENV_FOLLOWER, ModType::SINE,   ModType::RAMP},
        };

      Hardware       hw;
      StopwatchTimer led_timer;
      StopwatchTimer midi_timer;

      UiEventQueue ui_queue;

      PotMonitor<Hardware, Hardware::CTRL_LAST> pot_monitor;

      // Routing mode changed flag and current routing mode
      bool    routingModeChanged = false;
      AppMode currentRoutingMode = AppMode::OFF;

      // Mix controls for the two sides
      using SmoothValue = Utils::SmoothValue;
      SmoothValue mixControls[kNumberEffectSlots] = {SmoothValue(25.0f, kSampleRate),
                                                     SmoothValue(25.0f, kSampleRate)};

      // Mix control alt latch for the two sides
      bool mixAltLatch[kNumberEffectSlots] = {false};

      // Smooth pitch for each side
      SmoothValue pitchControls[kNumberEffectSlots] {SmoothValue(75.0f, kSampleRate),
                                                     SmoothValue(75.0f, kSampleRate)};

      // Position knob for each side
      SmoothValue positionControls[kNumberEffectSlots] {SmoothValue(150.0f, kSampleRate),
                                                        SmoothValue(150.0f, kSampleRate)};
      // Position/size switch state for each side
      SizePosSwitchState sizePosSwitch[kNumberEffectSlots] {SizePosSwitchState::SIZE, SizePosSwitchState::SIZE};

      // Size controls for each side
      SmoothValue sizeControls[kNumberEffectSlots] {SmoothValue(250.0f, kSampleRate),
                                                    SmoothValue(250.0f, kSampleRate)};

      // Shape controls for each side
      SmoothValue shapeControls[kNumberEffectSlots] {SmoothValue(250.0f, kSampleRate),
                                                     SmoothValue(250.0f, kSampleRate)};

      // Modulation amount controls for each side
      SmoothValue modulationAmount[kNumberEffectSlots] {SmoothValue(75.0f, kSampleRate),
                                                        SmoothValue(75.0f, kSampleRate)};

      // Modulation frequency controls for each side
      SmoothValue modulationFreq[kNumberEffectSlots] {SmoothValue(75.0f, kSampleRate),
                                                      SmoothValue(75.0f, kSampleRate)};

      // Modulation frequency alt latch for each side
      bool modFreqAltLatch[2] = {false};

      // Mode switch changed flag and current effect mode for each side
      using EffectMode                                    = Effect::EffectMode;
      bool       effectModeChanged[kNumberEffectSlots] {false};
      EffectMode currentEffectMode[kNumberEffectSlots] {EffectMode::MODE_1, EffectMode::MODE_1};

      // Mod type switch flag and current mod type for each side
      bool    modTypeChanged[kNumberEffectSlots] {false};
      ModType currentModType[kNumberEffectSlots] {ModType::MOD_TYPE_LAST,
                                                  ModType::MOD_TYPE_LAST};

      // Reverse flag for each side
      bool reverseStateChanged[kNumberEffectSlots] {false};
      bool currentReverseState[kNumberEffectSlots] {false};

      // Play flag for each side
      bool playStateChanged[kNumberEffectSlots] {false};
      bool currentPlayState[kNumberEffectSlots] {false};

      // Alt + Play flag for each side
      bool altPlayStateChanged[kNumberEffectSlots] {false};
      bool currentAltPlayState[kNumberEffectSlots] {false};

      // Spotykach + Play flag for each side
      bool spotyPlayStateChanged[kNumberEffectSlots] {false};
      bool currentSpotyPlayState[kNumberEffectSlots] {false};

      // Pad touch states
      std::bitset<16> padTouchStates;
      std::bitset<16> padTouchStatesPrev;

      // Modulator CV value for each side
      float    modCv[kNumberEffectSlots] {0.0f};

      // LED Phase for blinking LEDs
      uint8_t padLedPhase = 0;

      uint16_t last_pot_moved[kNumberEffectSlots] {0};

      bool    test_note_on;
      bool    midi_in_note_on;
      uint8_t midi_in_nn;

      // Modulator instances for each side
      Modulator modulator[kNumberEffectSlots] = {
        Modulator(modulationTypes[0], ModulationEngine::kNumModsPerSide, kSampleRate),
        Modulator(modulationTypes[1], ModulationEngine::kNumModsPerSide, kSampleRate)
      };

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
