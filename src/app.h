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

  constexpr size_t kNumberSpotykachSides = 1;
  constexpr size_t kNumberChannelsStereo = 2;
  constexpr size_t kNumberChannelsMono   = 1;

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

      bool    routingModeChanged = false;
      AppMode currentRoutingMode = AppMode::OFF;

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

      void handleControls ();
      void handleDisplay ();

      void testSDCard ();

      AppImpl (const AppImpl &a)           = delete;
      AppImpl &operator=(const AppImpl &a) = delete;
  };
}    // namespace spotykach_hwtest
