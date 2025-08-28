#pragma once
#include "Deck.h"
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
  constexpr float  kSamplePeriodMs       = 1000.0f / kSampleRate;
  constexpr size_t kBlockSize            = 48;

  constexpr uint8_t kNumberDeckSlots      = Hardware::kNumInPair;
  constexpr uint8_t kNumberChannelsStereo = 2;
  constexpr uint8_t kNumberChannelsMono   = 1;

  // Timers
  constexpr size_t kDebugLogPeriodMs      = 500;
  constexpr size_t kLedUpdatePeriodMs     = 2;
  constexpr size_t kLedUpdateRate         = (1.0f / (kLedUpdatePeriodMs / 1000.0f));

  // Enum for Size/Pos switch position
  enum SizePosSwitchState
  {
    SIZE,
    POSITION,
    BOTH
  };

  // Utils class
  using ChannelConfig = Deck::ChannelConfig;
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
          SmoothValue (float smoothTimeMs, float updatePeriodMs)
          {
            smoothing_    = false;
            currentValue_ = 0.0f;
            targetValue_  = 0.0f;

            // coeff = 100.0 / (time * sample_rate), where time is in seconds
            float updateRate = 1000.0f / updatePeriodMs;
            filterCoeff_     = 100.0f / ((smoothTimeMs / 1000.0f) * updateRate);
          }

          // Overload assignment to set targetValue
          SmoothValue &operator=(float v)
          {
            if (std::abs(v - targetValue_) > 0.015f)
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
                                                          Hardware::LED_GRIT_A,
                                                          Hardware::LED_FLUX_A,
                                                          Hardware::LED_CYCLE_A,
                                                          Hardware::LED_CYCLE_B,
                                                          Hardware::LED_FLUX_B,
                                                          Hardware::LED_GRIT_B,
                                                          Hardware::LED_REV_B,
                                                          Hardware::LED_PLAY_B,
                                                          Hardware::LED_SPOTY_PAD,
                                                          Hardware::LED_ALT_A};

      static constexpr size_t kPadMappingSize                   = sizeof(kPadMapping) / sizeof(kPadMapping[0]);
      static constexpr size_t kPadMapPlayIds[kNumberDeckSlots]  = {0, 9};
      static constexpr size_t kPadMapRevIds[kNumberDeckSlots]   = {1, 8};
      static constexpr size_t kPadMapGritIds[kNumberDeckSlots]  = {2, 7};
      static constexpr size_t kPadMapFluxIds[kNumberDeckSlots]  = {3, 6};
      static constexpr size_t kPadMapCycleIds[kNumberDeckSlots] = {4, 5};
      static constexpr size_t kPadMapSpotyId                    = 10;
      static constexpr size_t kPadMapAltId                      = 11;

      AppImpl ()  = default;
      ~AppImpl () = default;

      void init ();
      void loop ();

      // Update the control frame with the current values
      void updateAnalogControlFrame (Deck::AnalogControlFrame &c, size_t deckSlot);

      // Update the control frame with the current values
      void updateDigitalControlFrame (Deck::DigitalControlFrame &c, size_t deckSlot);

      // Push the control frame to the decks
      void pushAnalogDeckControls (Deck::AnalogControlFrame &c, size_t deckSlot);

      // Push the control frame to the decks
      void pushDigitalDeckControls (Deck::DigitalControlFrame &c, size_t deckSlot);

      // Process the modulation controls for the specified deck slot
      void processModulatorControls (size_t slot);

      // Audio processing functions for the Spotykach looper and Sporadic deck
      void processAudioLogic (AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t blockSize);

      // This is the main audio processing function that will be called from the AudioCallback
      void processAudio (AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t blockSize);

    private:
      using ModType = ModulationEngine::ModType;
      static constexpr ModType modulationTypes[kNumberDeckSlots][ModulationEngine::kNumModsPerSide] =
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

      // Mix controls for the two sides (updated at audio sample rate)
      using SmoothValue                         = Utils::SmoothValue;
      SmoothValue mixControls[kNumberDeckSlots] = {SmoothValue(25.0f, kSamplePeriodMs),
                                                   SmoothValue(25.0f, kSamplePeriodMs)};

      // Mix control alt latch for the two sides
      bool mixAltLatch[kNumberDeckSlots] = {false};

      // Mix knob Grit latch
      bool mixGritLatch[kNumberDeckSlots] = {false};

      // Position knob Grit latch
      bool positionGritLatch[kNumberDeckSlots] = {false};

      // Size knob Grit latch
      bool sizeGritLatch[kNumberDeckSlots] = {false};

      // Shape knob Grit latch
      bool shapeGritLatch[kNumberDeckSlots] = {false};

      // Spotykach slider Spotykach latch
      bool spotySpotyLatch = false;

      // Smooth Spotykach slider (updated at audio block rate)
      SmoothValue spotyControl {SmoothValue(75.0f, kSamplePeriodMs * kBlockSize)};

      // Smooth pitch for each side (updated at audio sample rate)
      SmoothValue pitchControls[kNumberDeckSlots]{SmoothValue(75.0f, kSamplePeriodMs),
                                                  SmoothValue(75.0f, kSamplePeriodMs)};

      // Position knob for each side (updated at audio sample rate)
      SmoothValue positionControls[kNumberDeckSlots]{SmoothValue(150.0f, kSamplePeriodMs),
                                                     SmoothValue(150.0f, kSamplePeriodMs)};
      // Position/size switch state for each side
      SizePosSwitchState sizePosSwitch[kNumberDeckSlots]{SizePosSwitchState::SIZE, SizePosSwitchState::SIZE};

      // Size controls for each side (updated at audio sample rate)
      SmoothValue sizeControls[kNumberDeckSlots]{SmoothValue(250.0f, kSamplePeriodMs),
                                                 SmoothValue(250.0f, kSamplePeriodMs)};

      // Shape controls for each side (updated at audio sample rate)
      SmoothValue shapeControls[kNumberDeckSlots]{SmoothValue(250.0f, kSamplePeriodMs),
                                                  SmoothValue(250.0f, kSamplePeriodMs)};

      // Modulation amount controls for each side (updated at audio block rate)
      SmoothValue modulationAmount[kNumberDeckSlots]{SmoothValue(75.0f, kSamplePeriodMs *kBlockSize),
                                                     SmoothValue(75.0f, kSamplePeriodMs *kBlockSize)};

      // Modulation frequency controls for each side (updated at audio block rate)
      SmoothValue modulationFreq[kNumberDeckSlots]{SmoothValue(75.0f, kSamplePeriodMs *kBlockSize),
                                                   SmoothValue(75.0f, kSamplePeriodMs *kBlockSize)};

      // Modulation frequency alt latch for each side
      bool modFreqAltLatch[2] = {false};

      // Mode switch changed flag and current deck mode for each side
      using DeckMode = Deck::DeckMode;
      bool     deckModeChanged[kNumberDeckSlots]{false};
      DeckMode currentDeckMode[kNumberDeckSlots]{DeckMode::MODE_1, DeckMode::MODE_1};

      // Mod type switch flag and current mod type for each side
      bool    modTypeChanged[kNumberDeckSlots]{false};
      ModType currentModType[kNumberDeckSlots]{ModType::MOD_TYPE_LAST, ModType::MOD_TYPE_LAST};

      // Reverse flag for each side
      bool reverseStateChanged[kNumberDeckSlots]{false};
      bool currentReverseState[kNumberDeckSlots]{false};

      // Play flag for each side
      bool playStateChanged[kNumberDeckSlots]{false};
      bool currentPlayState[kNumberDeckSlots]{false};

      // Alt + Play flag for each side
      bool altPlayStateChanged[kNumberDeckSlots]{false};
      bool currentAltPlayState[kNumberDeckSlots]{false};

      // Spotykach + Play flag for each side
      bool spotyPlayStateChanged[kNumberDeckSlots]{false};
      bool currentSpotyPlayState[kNumberDeckSlots]{false};

      // Flux flag for each side
      bool fluxStateChanged[kNumberDeckSlots]{false};
      bool currentFluxState[kNumberDeckSlots]{false};

      // Alt+Flux combo toggle flag for each side
      bool altFluxStateChanged[kNumberDeckSlots]{false};
      bool currentAltFluxState[kNumberDeckSlots]{false};

      // Grit flag for each side
      bool gritStateChanged[kNumberDeckSlots]{false};
      bool currentGritState[kNumberDeckSlots]{false};

      // Alt+Grit combo toggle flag for each side
      bool altGritStateChanged[kNumberDeckSlots]{false};
      bool currentAltGritState[kNumberDeckSlots]{false};

      // Pad touch states
      std::bitset<16> padTouchStates;
      std::bitset<16> padTouchStatesPrev;

      // Modulator CV value for each side
      float modCv[kNumberDeckSlots]{0.0f};

      // Crossfade mix between deck slot outputs (0.0 = slot 0 only, 1.0 = slot 1 only)
      float deckMix_ = 0.5f;

      // Per-deck temporary output buffers for crossfading (stereo, blocksize frames)
      float deckOutputs_[kNumberDeckSlots][kNumberChannelsStereo][kBlockSize]{};

      // LED Phase for blinking LEDs
      uint8_t padLedPhase = 0;

      uint16_t last_pot_moved[kNumberDeckSlots]{0};

      bool    test_note_on;
      bool    midi_in_note_on;
      uint8_t midi_in_nn;

      // Modulator instances for each side
      Modulator modulator[kNumberDeckSlots] = {
        Modulator(modulationTypes[0], ModulationEngine::kNumModsPerSide, kLedUpdateRate),
        Modulator(modulationTypes[1], ModulationEngine::kNumModsPerSide, kLedUpdateRate)
      };

      StopwatchTimer log_timer;
#if DEBUG
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
