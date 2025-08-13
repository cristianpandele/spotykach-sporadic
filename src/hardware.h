#pragma once

#include "common.h"
#include "daisy_seed.h"
#include "sr_165.h"
#include "ws2812.h"

namespace spotykach
{
  class Hardware
  {
    public:
      // LED indexes in order of chain
      static constexpr uint8_t kNumLedsPerRing = 32;

      enum LedId : uint16_t
      {
        LED_SPOTY_PAD,
        LED_ALT_A,
        LED_PLAY_A,
        LED_REV_A,

        // Reserved 32 LEDs starting at top of ring,
        // clockwise order - add desired offset (0 - 31)
        LED_RING_A,
        LED_RING_A_LAST = LED_RING_A + kNumLedsPerRing - 1,

        LED_ORBIT_A,
        LED_DRIFT_A,
        LED_GATE_OUT_A,
        LED_CYCLE_A,

        LED_ROUTING_CENTER,
        LED_ROUTING_LEFT,
        LED_CLOCK_IN,
        LED_ROUTING_RIGHT,

        LED_CYCLE_B,
        LED_GATE_OUT_B,
        LED_DRIFT_B,
        LED_ORBIT_B,

        LED_RING_B,
        LED_RING_B_LAST = LED_RING_B + kNumLedsPerRing - 1,

        LED_REV_B,
        LED_PLAY_B,
        LED_SPOTY_SLIDER_B,
        LED_SPOTY_SLIDER_A,
        LED_ALT_B,

        LED_LAST
      };

      // Pots/sliders - these are on muxes
      enum AnalogControlId : uint16_t
      {
        CTRL_SOS_A,
        CTRL_MOD_FREQ_A,
        CTRL_MOD_AMT_A,
        CTRL_SIZE_A,
        CTRL_PITCH_A,
        CTRL_POS_A,
        CTRL_SHAPE_A,

        CTRL_SOS_B,
        CTRL_MOD_FREQ_B,
        CTRL_MOD_AMT_B,
        CTRL_SIZE_B,
        CTRL_PITCH_B,
        CTRL_POS_B,
        CTRL_SHAPE_B,

        CTRL_SPOTYKACH,

        CTRL_LAST
      };

      // These are in order as they are labeled on the schematic
      enum CvInputId : uint16_t
      {
        CV_SIZE_POS_A,
        CV_V_OCT_A,
        CV_SOS_IN_A,

        CV_SPOTYKACH,

        CV_SIZE_POS_B,
        CV_V_OCT_B,
        CV_SOS_IN_B,

        CV_LAST
      };

      Hardware ()                          = default;
      ~Hardware ()                         = default;

      void Init (float sr, size_t blocksize);
      void StartAdcs ();

      // Process the analog controls - do this in audiocallback,
      // it is not blocking
      void ProcessAnalogControls ();

      // Process the shift register, read from Mpr121, etc
      // This is a blocking call, don't do it in audiocallback
      void ProcessDigitalControls ();

      // Unipolar 0.0 - 1.0
      float GetAnalogControlValue (AnalogControlId id);

      // Bipolar -1.0 - 1.0 (nominally around 0.0)
      float GetControlVoltageValue (CvInputId id);

      // Adapter for direct use as PotMonitor Backend
      inline float GetPotValue (uint16_t pot_id) { return GetAnalogControlValue((AnalogControlId)pot_id); }

      uint32_t GetBootButtonHeldTime () const;
      uint32_t GetBootButtonReleased () const;

      bool GetClockInputState ();
      bool GetGateInputAState ();
      bool GetGateInputBState ();

      inline void SetGateOutA (bool state) { gate_out_a_.Write(state); }

      inline void SetGateOutB (bool state) { gate_out_b_.Write(state); }

      // TODO: I'd recommend abstracting this with more readable enums
      //       and abstracted bit testing for switches - not just reading raw bytes
      //       from the shift register (this is just for quick hardware testing code)
      inline uint8_t GetShiftRegState (uint8_t idx) const
      {
        if (idx > 1)
        {
          return 0;
        }
        return shiftreg_.Read(idx);
      }

      // TODO: same as above, this is just quick code for testing - recommend
      //       implementing enumerated pad logic internal to hw class
      inline uint16_t GetMpr121TouchStates () { return mpr121_.Touched(); }

      inline void WriteCVOut (size_t channel, float val)
      {
        seed.dac.WriteValue(static_cast<daisy::DacHandle::Channel>(channel), __USAT(val * (1 << 12), 12));
      }

      // Left public for easy direct handle access
      // but all core peripherals are initialized by this class
      daisy::DaisySeed seed;

      // Using device class directly, maybe easier
      // to wrap inside a helper for color/index management etc
      infrasonic::Ws2812 leds;

      daisy::MidiUartHandler midi_uart;

    private:
      infrasonic::ShiftRegister165 shiftreg_;

      daisy::Mpr121I2C mpr121_;

      daisy::GPIO clock_in_;
      daisy::GPIO gate_in_a_;
      daisy::GPIO gate_in_b_;
      daisy::GPIO gate_out_a_;
      daisy::GPIO gate_out_b_;

      daisy::Switch boot_btn_;

      daisy::AnalogControl controls_[CTRL_LAST];
      daisy::AnalogControl cvinputs_[CV_LAST];

      Hardware (const Hardware &a)           = delete;
      Hardware &operator=(const Hardware &a) = delete;
  };

}    // namespace spotykach
