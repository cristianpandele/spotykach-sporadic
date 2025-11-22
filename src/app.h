#pragma once
#include "Deck.h"
#include "Modulation.h"
#include "Utils.h"
#include "constants.h"
#include "hardware.h"
#include <bitset>
#include <daisy_seed.h>

namespace spotykach_hwtest
{
  // Definitions
  #define PRINT_CPU_LOAD

  // Timers
  constexpr size_t   kDebugLogPeriodMs    = 500;
  constexpr size_t   kLedUpdatePeriodMs   = 2;
  constexpr size_t   kLedUpdateRate       = (1000.0f / kLedUpdatePeriodMs);
  constexpr uint32_t kTakeoverAttackMs    = 50;
  constexpr uint32_t kTakeoverDecayMs     = 100;

  // Enum for Size/Pos switch position
  enum SizePosSwitchState
  {
    SIZE,
    POSITION,
    BOTH
  };

  enum ModTarget
  {
    MIX,
    FLUX,
    GRIT,
    MOD_TARGET_LAST
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

      // Mix controls for the two sides (updated at audio block rate)
      using SmoothValue                         = Utils::SmoothValue;
      SmoothValue mixControls[kNumberDeckSlots] = {SmoothValue(75.0f, kSamplePeriodMs * kBlockSize),
                                                   SmoothValue(75.0f, kSamplePeriodMs * kBlockSize)};

      // Mix control alt latch for the two sides
      bool mixAltLatch[kNumberDeckSlots] = {false};

      // Pitch knob Grit latch
      bool pitchGritLatch[kNumberDeckSlots] = {false};

      // Position knob Grit latch
      bool positionGritLatch[kNumberDeckSlots] = {false};

      // Size knob Grit latch
      bool sizeGritLatch[kNumberDeckSlots] = {false};

      // Shape knob Grit latch
      bool shapeGritLatch[kNumberDeckSlots] = {false};

      // Smooth Spotykach slider (updated at audio block rate)
      SmoothValue spotyControl {SmoothValue(150.0f, kSamplePeriodMs * kBlockSize)};

      // Smooth pitch for each side (updated at audio block rate)
      SmoothValue pitchControls[kNumberDeckSlots]{SmoothValue(75.0f, kSamplePeriodMs * kBlockSize),
                                                  SmoothValue(75.0f, kSamplePeriodMs * kBlockSize)};

      // Position knob for each side (updated at audio block rate)
      SmoothValue positionControls[kNumberDeckSlots]{SmoothValue(75.0f, kSamplePeriodMs * kBlockSize),
                                                     SmoothValue(75.0f, kSamplePeriodMs * kBlockSize)};
      // Position/size switch state for each side
      SizePosSwitchState sizePosSwitch[kNumberDeckSlots]{SizePosSwitchState::SIZE, SizePosSwitchState::SIZE};

      // Size controls for each side (updated at audio block rate)
      SmoothValue sizeControls[kNumberDeckSlots]{SmoothValue(75.0f, kSamplePeriodMs * kBlockSize),
                                                 SmoothValue(75.0f, kSamplePeriodMs * kBlockSize)};

      // Shape controls for each side (updated at audio block rate)
      SmoothValue shapeControls[kNumberDeckSlots]{SmoothValue(75.0f, kSamplePeriodMs * kBlockSize),
                                                  SmoothValue(75.0f, kSamplePeriodMs * kBlockSize)};
      // Modulation amount controls for each side (updated at audio block rate)
      SmoothValue modulationAmount[kNumberDeckSlots]{SmoothValue(75.0f, kSamplePeriodMs * kBlockSize),
                                                     SmoothValue(75.0f, kSamplePeriodMs * kBlockSize)};

      // Modulation frequency controls for each side (updated at audio block rate)
      SmoothValue modulationFreq[kNumberDeckSlots]{SmoothValue(75.0f, kSamplePeriodMs * kBlockSize),
                                                   SmoothValue(75.0f, kSamplePeriodMs * kBlockSize)};

      // Modulation frequency alt latch for each side
      bool modFreqAltLatch[2] = {false};

      // Mod target switch changed flag and current mod target for each side
      bool        modTargetChanged[kNumberDeckSlots]{false};
      SmoothValue modTargetSmooth[ModTarget::MOD_TARGET_LAST]{SmoothValue(75.0f, kSamplePeriodMs),
                                                              SmoothValue(75.0f, kSamplePeriodMs),
                                                              SmoothValue(75.0f, kSamplePeriodMs)};
      ModTarget   currentModTarget[kNumberDeckSlots]{ModTarget::MIX, ModTarget::MIX};

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

      // Soft takeover indicator for each side
      bool currentTakeoverState[kNumberDeckSlots]{false};
      bool takeoverPulseActive_[kNumberDeckSlots]{false};

      // Soft takeover envelope generators
      daisysp::AdEnv takeoverEnv_[kNumberDeckSlots];

      // Pad touch states
      std::bitset<16> padTouchStates;
      std::bitset<16> padTouchStatesPrev;

      // Modulator CV value for each side
      float modCv[kNumberDeckSlots]{0.0f};

      // Crossfade mix between deck slot outputs (0.0 = slot 0 only, 1.0 = slot 1 only)
      SmoothValue deckMix_ = SmoothValue(0.5f, 150.0f, kLedUpdatePeriodMs);

      // Per-deck temporary output buffers for crossfading (stereo, blocksize frames)
      float deckOutputs_[kNumberDeckSlots][kNumberChannelsStereo][kBlockSize] = {{{0.0f}}};

      // LED Phase for blinking LEDs
      uint8_t padLedPhase = 0;

      uint16_t last_pot_moved[kNumberDeckSlots]{0};

      // Flag for feeding the envelope follower with input signal
      bool    envelopeFeed = false;

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

      ///////////
      NOCOPY (AppImpl);
  };
}    // namespace spotykach_hwtest
