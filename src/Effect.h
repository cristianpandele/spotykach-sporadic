#pragma once

#include "common.h"
#include "daisy_seed.h"
#include "hardware.h"
using namespace daisy;
using namespace spotykach;
using namespace infrasonic;

// Base class for Effect implementation
class Effect
{
  public:
    enum ChannelConfig
    {
      OFF,
      MONO_LEFT,
      MONO_RIGHT,
      STEREO,
      CH_CONFIG_LAST
    };

    enum EffectMode
    {
      MODE_1,
      MODE_2,
      MODE_3,
      MODE_LAST
    };

    // Analog Control payload pushed on change (at most once per block) from AppImpl
    struct AnalogControlFrame
    {
      float mix;
      bool  mixAlt;      // latched Alt pad at move time
      float pitch;
      float position;
      float size;
      float shape;
    };

    // Digital control payload pushed  on change (at most once per loop iteration) from AppImpl
    struct DigitalControlFrame
    {
      bool  reverse;
      bool  play;
      bool  altPlay;      // latched Alt pad when Play pressed
      bool  spotyPlay;    // latched Spotykach pad when Play pressed
    };

    // Render-ready view the UI can draw without peeking internals
    static constexpr uint8_t kMaxRingLayers = 4;
    static constexpr uint8_t kMaxLedPhases  = 2;

    struct RingSpan
    {
      uint8_t          start;
      uint8_t          end;
      LedRgbBrightness led[Hardware::kNumLedsPerRing];
    };

    struct DisplayState
    {
      // Flags indicating the state of the pads
      bool reverseActive   = false;
      bool playActive      = false;
      bool altPlayActive   = false;
      bool spotyPlayActive = false;

      // Up to a few ring spans to draw; 0..count-1 valid
      uint8_t  layerCount = 0;
      RingSpan rings[kMaxRingLayers];

      // Flags indicating the colours for the two LED phases
      LedRgbBrightness playLedColors[kMaxLedPhases];
      LedRgbBrightness reverseLedColors[kMaxLedPhases];
    };

    Effect ()            = default;
    virtual ~Effect ()   = default;

    virtual void init () = 0;
    virtual void setChannelConfig (ChannelConfig cfg);
    virtual void setMode (EffectMode m);

    // Unified controls update
    virtual void updateAnalogControls (const AnalogControlFrame &c) = 0;
    virtual void updateDigitalControls (const DigitalControlFrame &c) = 0;
    // Digital controls fetch
    virtual void getDigitalControls (DigitalControlFrame &c) = 0;
    // Update the display state with the current values
    virtual void updateDisplayState() = 0;
    // Pull display state snapshot (thread-safe, non-blocking)
    virtual bool getDisplayState (DisplayState &out) const;

    // Audio
    virtual void processAudio (AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t blockSize) = 0;

  protected:
    // Channel configuration for the effect
    ChannelConfig channelConfig_ = ChannelConfig::OFF;

    // Update the display state with the current values
    void publishDisplay (const DisplayState &state);

    // Mix control
    float mix_ = 0.5f;

    // V.Oct pitch control
    float pitch_ = 0.0f;

    // Position control
    float position_ = 0.0f;

    // Size control
    float size_ = 0.0f;

    // Shape control
    float shape_ = 0.0f;

    // Reverse playback flag
    bool reverse_ = false;

    // Play flag
    bool play_ = false;

    // Current effect mode
    EffectMode mode_ = MODE_1;

    // Setters for effect parameters
    virtual void setMix (float m) { mix_ = infrasonic::unitclamp(m); }
    virtual void setPitch (float p) { pitch_ = infrasonic::unitclamp(p); }
    virtual void setPosition (float p) { position_ = infrasonic::unitclamp(p); }
    virtual void setSize (float s) { size_ = infrasonic::unitclamp(s); }
    virtual void setShape (float s) { shape_ = infrasonic::unitclamp(s); }
    virtual void setReverse (bool r) { reverse_ = r; }
    virtual void setPlay (bool p) { play_ = p; }

    bool isChannelActive (size_t ch) const;

  private:
    // Double-buffer technique to handle display state updates and publish them externally
    struct DisplayBuf
    {
      DisplayState state;
      uint32_t     cnt;
    };

    DisplayBuf       dispBuf_[2];
    mutable uint32_t cntRead_  = 0;
    volatile uint8_t dispWIdx_ = 0;

    Effect (const Effect &)           = delete;
    Effect &operator=(const Effect &) = delete;
};
