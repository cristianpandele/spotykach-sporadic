#pragma once

#include "InputSculpt.h"
#include "common.h"
#include "constants.h"
#include "daisy_seed.h"
#include "hardware.h"
using namespace daisy;
using namespace spotykach;
using namespace infrasonic;

// Base class for Deck implementation
class Deck
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

    enum DeckMode
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
      bool  mixAlt;       // latched Alt pad at move time for Mix knob
      bool  mixGrit;      // latched Flux pad at move time for Mix knob
      float pitch;
      float position;
      bool  positionGrit; // latched Flux pad at move time for Position knob
      float size;
      bool  sizeGrit;     // latched Flux pad at move time for Size knob
      float shape;
      bool  shapeGrit;    // latched Flux pad at move time for Shape knob
    };

    // Digital control payload pushed  on change (at most once per loop iteration) from AppImpl
    struct DigitalControlFrame
    {
      // Simple pad presses
      bool  reverse;
      bool  play;
      bool  flux;
      bool  grit;
      // Supported pad combinations
      bool  altPlay;      // latched Alt pad when Play pressed
      bool  spotyPlay;    // latched Spotykach pad when Play pressed
      bool  altFlux;      // latched Alt+Flux combo toggle (enters flux mode / overrides flux)
      bool  altGrit;      // latched Alt+Grit combo toggle (enters grit mode / overrides grit)
    };

    // Render-ready view the UI can draw without peeking internals
    static constexpr uint8_t kMaxRingLayers = 4;
    static constexpr uint8_t kMaxLedPhases  = 2;

    // Timeout for double-tap detection
    static constexpr uint32_t kDoubleTapTimeoutMs = 425;
    // Timeout for held detection
    static constexpr uint32_t kHeldTimeoutMs = 150;

    struct RingSpan
    {
      uint8_t          start;
      uint8_t          end;
      LedRgbBrightness led[Hardware::kNumLedsPerRing];
    };

    struct DisplayState
    {
      // Flags indicating the state of the pad LEDs
      bool reverseActive   = false;
      bool playActive      = false;
      bool altActive       = false;
      bool fluxActive      = false;
      bool gritActive      = false;

      // Up to a few ring spans to draw; 0..count-1 valid
      uint8_t  layerCount = 0;
      RingSpan rings[kMaxRingLayers];

      // Flags indicating the colours for the two LED phases
      LedRgbBrightness fluxLedColors[kMaxLedPhases];
      LedRgbBrightness gritLedColors[kMaxLedPhases];
      LedRgbBrightness reverseLedColors[kMaxLedPhases];
      LedRgbBrightness playLedColors[kMaxLedPhases];
      LedRgbBrightness altLedColors[kMaxLedPhases];
    };

    Deck (size_t sampleRate, size_t blockSize) : sampleRate_(sampleRate), blockSize_(blockSize) { }
    Deck ()            = delete;
    virtual ~Deck ()   = default;

    virtual void init () = 0;
    virtual void setChannelConfig (ChannelConfig cfg);
    virtual void setMode (DeckMode m);

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
    // Channel configuration for the deck
    ChannelConfig channelConfig_ = ChannelConfig::OFF;

    // Sample rate for the deck
    size_t sampleRate_;

    // Block size for the deck
    size_t blockSize_;

    // Input sculpting bandpass filter
    InputSculpt inputSculpt_;

    // Internal working buffers (single block) to avoid per-callback allocations.
    float inputSculptBuf_[kNumberChannelsStereo][kBlockSize]{};

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

    // Flux flag
    bool flux_ = false;

    // Grit flag
    bool grit_ = false;

    // Current deck mode
    DeckMode mode_ = MODE_1;

    // Update the display state with the current values
    void publishDisplay (const DisplayState &state);

    // Update the Grit pad LED state
    void updateGritPadLedState (DisplayState &view);

    // Update the Grit display state specifically
    void updateGritRingState (DisplayState &view);

    // Update the Flux pad LED state
    void updateFluxPadLedState (DisplayState &view);

    // // Update the Flux display state specifically
    // void updateFluxRingState (DisplayState &view);

    // Update the LED states for the effect pads
    void updateEffectDisplayStates (DisplayState &view);

    // Check if there is an update to the held state of the effect pads
    void detectEffectPadsHeld ();

    // Setters for deck parameters
    virtual void setMix (float m) { mix_ = infrasonic::unitclamp(m); }
    virtual void setPitch (float p) { pitch_ = infrasonic::unitclamp(p); }
    virtual void setPosition (float p) { position_ = infrasonic::unitclamp(p); }
    virtual void setSize (float s) { size_ = infrasonic::unitclamp(s); }
    virtual void setShape (float s) { shape_ = infrasonic::unitclamp(s); }
    virtual void setReverse (bool r) { reverse_ = r; }
    virtual void setPlay (bool p) { play_ = p; }
    virtual void setFlux (bool f) { flux_ = f; }
    virtual void setGrit (bool g) { grit_ = g; }

    // Toggles for the effects
    virtual void toggleFluxMenu () { fluxMenuOpen_ = !fluxMenuOpen_; }
    virtual void toggleGritMenu () { gritMenuOpen_ = !gritMenuOpen_; }
    virtual void toggleFluxActive() { fluxActive_ = !fluxActive_; }
    virtual void toggleGritActive() { gritActive_ = !gritActive_; }

    // Getters for the effects active states
    virtual bool getFluxActive () const { return fluxActive_; }
    virtual bool getGritActive () const { return gritActive_; }

    // Setters for the effects menu states
    virtual void setFluxMenuOpen (bool f) { fluxMenuOpen_ = f; }
    virtual void setGritMenuOpen (bool g) { gritMenuOpen_ = g; }
    // Getters for the effects menu open states
    virtual bool getFluxMenuOpen () const { return fluxMenuOpen_; }
    virtual bool getGritMenuOpen () const { return gritMenuOpen_; }
    virtual bool isEffectMenuOpen () const { return (getFluxMenuOpen() || getGritMenuOpen()); }

    // Getters for the effects pad held states
    virtual bool getFluxHeld () const { return fluxHeld_; }
    virtual bool getGritHeld () const { return gritHeld_; }
    virtual bool isEffectPadHeld () const { return (getFluxHeld() || getGritHeld()); }

    // Getters for effects being displayed
    virtual bool isFluxDisplayed () const { return (getFluxMenuOpen() || getFluxHeld()); }
    virtual bool isGritDisplayed () const { return (getGritMenuOpen() || getGritHeld()); }
    virtual bool isEffectDisplayed () const { return (isEffectMenuOpen() || isEffectPadHeld()); }

    // Getters for the effects playing states
    virtual bool isFluxPlaying () const { return (getFluxHeld() || getFluxMenuOpen() || getFluxActive()); }
    virtual bool isGritPlaying () const { return (getGritHeld() || getGritMenuOpen() || getGritActive()); }
    virtual bool isEffectPlaying () const { return (isFluxPlaying() || isGritPlaying()); }

    // Utility functions
    bool isChannelActive (size_t ch) const;
    // Detect if the Flux pad is held; returns true if held
    bool detectFluxHeld ();
    // Detect if the Grit pad is held; returns true if held
    bool detectGritHeld ();
    // Handle Flux tap/hold detection; returns states via references
    void handleFluxTap (const bool flux, bool &doubleTap, bool &held);
    // Handle Grit tap/hold detection; returns states via references
    void handleGritTap (const bool grit, bool &doubleTap, bool &held);

  private:
    // Double-buffer technique to handle display state updates and publish them externally
    struct DisplayBuf
    {
      DisplayState state;
      uint32_t     cnt;
    };

    enum FilterType
    {
      kLowPass,
      kHighPass,
      kBandPass,
    };

    // Display buffer for state updates
    DisplayBuf       dispBuf_[2];
    mutable uint32_t cntRead_  = 0;
    volatile uint8_t dispWIdx_ = 0;

    // Frequency filter range for Grit
    static constexpr float gritFilterMinFreq = 50.0f;
    static constexpr float gritFilterMaxFreq = 18000.0f;

    ///////////
    // Flux pad states
    bool           fluxActive_               = false;
    bool           fluxHeld_                 = false;
    bool           fluxDoubleTapTimerActive_ = false;
    bool           fluxHeldTimerActive_      = false;
    bool           fluxMenuOpen_             = false;
    StopwatchTimer fluxDoubleTapTimer_;
    StopwatchTimer fluxHeldTimer_;

    // Grit pad states
    bool           gritActive_               = false;
    bool           gritHeld_                 = false;
    bool           gritDoubleTapTimerActive_ = false;
    bool           gritHeldTimerActive_      = false;
    bool           gritMenuOpen_             = false;
    StopwatchTimer gritDoubleTapTimer_;
    StopwatchTimer gritHeldTimer_;

    ///////////
    // Held detection
    void detectHeld (StopwatchTimer &timer, bool &timerActive, bool &held);
    // General tap/hold handler. Returns true if a double-tap detected; updates "held"; stops timer if pressed == false.
    void handleTap (const bool      padPressed,
                    StopwatchTimer &heldTimer,
                    bool           &heldTimerActive,
                    StopwatchTimer &doubleTapTimer,
                    bool           &doubleTapTimerActive,
                    bool           &held,
                    bool           &doubleTap);

    ///////////
    // Grit display handling functions
    uint8_t freqToLed (float f, uint8_t numLeds, float fMin, float fMax);
    void    ledBrightnessFilterGradient (
         FilterType type, uint8_t ringSize, uint8_t spanSize, float minBrightness, float maxBrightness, float *gradValues);
    // Falling: full max until cutoff LED index, then linear descent over Q-based width to min
    void ledBrightnessFallingGradient (
      uint8_t ringSize, uint8_t spanSize, float minBrightness, float maxBrightness, float *gradValues);
    // Ramp: linear ascent over Q-based width up to cutoff LED index, then full max
    void ledBrightnessRampGradient (
      uint8_t ringSize, uint8_t spanSize, float minBrightness, float maxBrightness, float *gradValues);
    void populateGritLedRing (Deck::RingSpan  &ringSpan,
                          uint8_t          ringSize,
                          LedRgbBrightness colorBright,
                          uint8_t          spanStart,
                          uint8_t          spanSize,
                          bool             gradient = false);

    float calculateFilterHalfBandwidth (float centerFreq, float Q);
    void  calculateFilterRingSpanSize (FilterType type, const uint8_t numLeds, uint8_t &start, uint8_t &end);

    // Helpers to derive LED indices from filter params
    uint8_t computeCutoffIdx (uint8_t ringSize);
    uint8_t computeWidthFromQ (uint8_t ringSize);

    ///////////
    Deck (const Deck &)           = delete;
    Deck &operator=(const Deck &) = delete;
};
