#pragma once

#include "InputSculpt.h"
#include "Utils.h"
#include "common.h"
#include "constants.h"
#include "daisy_seed.h"
#include <daisysp.h>
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
      float pitch;
      bool  pitchGrit;    // latched Grit pad at move time for Pitch knob
      float position;
      bool  positionGrit; // latched Grit pad at move time for Position knob
      float size;
      bool  sizeGrit;     // latched Grit pad at move time for Size knob
      float shape;
      bool  shapeGrit;    // latched Grit pad at move time for Shape knob
      float spoty;
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
      // Soft takeover notification
      bool  takeover;     // asserted when a soft takeover just occurred
    };

    // Render-ready view the UI can draw without peeking internals
    static constexpr uint8_t kMaxRingLayers = std::max(kMaxNumDelayProcs, kMaxNutrientBands) + 2; // +2 for canvas and any additional span
    static constexpr uint8_t kMaxLedPhases  = 2;

    // Timeout for double-tap detection
    static constexpr uint32_t kDoubleTapTimeoutMs = 425;
    // Timeout for held detection
    static constexpr uint32_t kHeldTimeoutMs = 150;
    // Timeout for long-held detection (3 seconds)
    static constexpr uint32_t kLongHeldTimeoutMs = 3000;

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
      std::array<RingSpan, kMaxRingLayers> rings;

      // Flags indicating the colours for the two LED phases
      std::array<LedRgbBrightness, kMaxLedPhases> fluxLedColors;
      std::array<LedRgbBrightness, kMaxLedPhases> gritLedColors;
      std::array<LedRgbBrightness, kMaxLedPhases> reverseLedColors;
      std::array<LedRgbBrightness, kMaxLedPhases> playLedColors;
      std::array<LedRgbBrightness, kMaxLedPhases> altLedColors;
    };

    Deck (size_t sampleRate, size_t blockSize);
    Deck ()              = delete;
    virtual ~Deck ()     = default;

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
    struct SoftTakeoverState
    {
      bool  initialized = false;
      bool  waiting     = false;
      float lastControl = 0.0f;
      float targetValue = 0.0f;
    };

    struct DualLayerSoftTakeover
    {
      SoftTakeoverState primary;
      SoftTakeoverState alternate;
      bool              hasActiveLayer       = false;
      bool              activeLayerAlternate = false;
    };

    bool prepareSoftTakeover(DualLayerSoftTakeover &state,
                             bool                    usingAlternateLayer,
                             float                   controlValue,
                             float                   currentValue);

    void finalizeSoftTakeover(DualLayerSoftTakeover &state,
                              bool                    usingAlternateLayer,
                              float                   controlValue,
                              float                   newValue);

    bool consumeTakeoverFlag ()
    {
      bool triggered    = takeoverTriggered_;
      takeoverTriggered_ = false;
      return triggered;
    }

    // Channel configuration for the deck
    ChannelConfig channelConfig_ = ChannelConfig::OFF;

    // Sample rate for the deck
    size_t sampleRate_;

    // Block size for the deck
    size_t blockSize_;

    // Input sculpting bandpass filter (per-channel)
    InputSculpt inputSculpt_[kNumberChannelsStereo];

    // Internal working buffers (single block) to avoid per-callback allocations.
    float inputSculptBuf_[kNumberChannelsStereo][kBlockSize]{};
    // Center frequency for input sculpting
    float inputSculptCenterFreq_ = 0.0f;

    // Mix control
    float mixControl_ = 0.5f;
    float mix_        = 0.5f;

    // V.Oct pitch control
    float pitchControl_ = 0.0f;
    float pitch_        = 0.0f;

    // Position control
    float positionControl_ = 0.0f;
    float position_        = 0.0f;

    // Size control
    float sizeControl_ = 0.0f;
    float size_        = 0.0f;

    // Shape control
    float shapeControl_ = 0.0f;
    float shape_        = 0.0f;

    // Spotykach slider control
    float spotyControl_ = 0.0f;
    float spoty_        = 0.0f;

    // Soft takeover states for the controls
    DualLayerSoftTakeover positionSoftTakeover_{};
    DualLayerSoftTakeover sizeSoftTakeover_{};
    DualLayerSoftTakeover shapeSoftTakeover_{};
    DualLayerSoftTakeover pitchSoftTakeover_{};
    DualLayerSoftTakeover mixSoftTakeover_{};

    // Grit layer controls
    float mixAltControl_       = 0.0f;
    float positionGritControl_ = 0.0f;
    float sizeGritControl_     = 0.0f;
    float shapeGritControl_    = 0.0f;
    float pitchGritControl_    = 0.0f;

    // Feedback level
    float feedback_ = 0.0f;

    // Reverse playback flag
    bool reverse_ = false;

    // Play flag
    bool play_ = false;

    // Flux flag
    bool flux_ = false;

    // Grit flag
    bool grit_ = false;

    // Soft takeover notification flag consumed by digital plumbing
    bool takeoverTriggered_ = false;

    // Current deck mode
    DeckMode mode_ = MODE_1;

    // Reset the display state
    void resetDisplayRingLayers (DisplayState &state) { state.layerCount = 0; };

    // Update the display state with the current values
    void publishDisplay (const DisplayState &state);

    // LED brightness gradient depending on shape_ and span size
    void ledBrightnessGradientLog (uint8_t spanSize, float minBrightness, float maxBrightness, float *gradValues);

    // Get the Grit LED color based on the drive factor
    LedRgbBrightness getGritLedColour ();

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

    // Update the digital controls for effects based on the control frame
    void updateDigitalControlsEffects (const DigitalControlFrame &c);

    #define MAKE_GRIT_CHANGE_STATUS(x, xControl, gritLatch)                                                                \
      bool x##Changed                  = std::abs(x - xControl) > kParamChThreshold;                                       \
      bool x##ChangedWhileGritLatched  = x##Changed && gritLatch;                                                          \
      bool x##ChangedWhileGritMenuOpen = x##Changed && getGritMenuOpen();

    // Helper for implementing soft takeover/catch between primary and alternate layers
    void setSoftTakeoverControl (DualLayerSoftTakeover &state,
                                 bool                    usingAlternateLayer,
                                 float                   incomingValue,
                                 float                  &primaryValue,
                                 float                  &alternateValue,
                                 bool                   &changed,
                                 bool                   &changedAlt);

    // Setters for deck parameters
    void         setMix (float m) { mix_ = infrasonic::unitclamp(m); }
    void         setMix (float m, bool altLatch = false);
    void         setFeedback (float fb) { feedback_ = std::clamp(fb, 0.0f, 0.99f); }
    virtual void setPitch (float p) { pitch_ = infrasonic::unitclamp(p); }
    void         setPitch (float p, bool gritLatch, bool &pitchChanged, bool &pitchChangedGrit);
    virtual void setPosition (float p) { position_ = infrasonic::unitclamp(p); }
    void         setPosition (float p, bool gritLatch, bool &positionChanged, bool &positionChangedGrit);
    virtual void setSize (float s) { size_ = infrasonic::unitclamp(s); }
    void         setSize (float s, bool gritLatch, bool &sizeChanged, bool &sizeChangedGrit);
    virtual void setShape (float s) { shape_ = infrasonic::unitclamp(s); }
    void         setShape (float s, bool gritLatch, bool &shapeChanged, bool &shapeChangedGrit);
    virtual void setSpoty (float s) { spoty_ = infrasonic::unitclamp(s); }
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

    // Handle long-hold modulation (for each effect)
    void processFluxLongHoldModulation ();
    void processGritLongHoldModulation ();

    // Utility functions
    bool isChannelActive (size_t ch) const;
    // Detect if the Flux pad is held; returns true if held
    void detectFluxHeld ();
    // Detect if the Grit pad is held; returns true if held
    void detectGritHeld ();
    // Handle Flux tap/hold detection; returns states via references (doubleTap, held, longHeld)
    void handleFluxTap (const bool flux, bool &doubleTap, bool &held, bool &longHeld);
    // Handle Grit tap/hold detection; returns states via references (doubleTap, held, longHeld)
    void handleGritTap (const bool grit, bool &doubleTap, bool &held, bool &longHeld);
    // Map frequency to LED index
    uint8_t freqToLed (float f, uint8_t numLeds, float fMin, float fMax);
    // Interpolate between 4 values based on shape_
    inline void ledsFourShapeInterpolator (
      float minBrightness, float maxBrightness, float blend, float *shapes, float *gradValues);
    // Populate the LED ring with the specified parameters
    void populateLedRing (Deck::RingSpan  &ringSpan,
                          uint8_t          ringSize,
                          LedRgbBrightness colorBright,
                          uint8_t          start,
                          uint8_t          spanSize,
                          bool             gradient = false,
                          bool             invert = false);

    // Frequency filter range for Grit
    static constexpr float gritFilterMinFreq = InputSculpt::kMinFreq;
    static constexpr float gritFilterMaxFreq = InputSculpt::kMaxFreq;

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

    ///////////
    // Flux pad states
    bool           fluxActive_               = false;
    // Pad held state (150ms)
    bool           fluxHeld_                 = false;
    // Long-held state (3s)
    bool           fluxLongHeld_             = false;
    // Double-tap and held timers & states
    bool           fluxDoubleTapTimerActive_ = false;
    bool           fluxHeldTimerActive_      = false;
    StopwatchTimer fluxDoubleTapTimer_;
    StopwatchTimer fluxHeldTimer_;
    // Flux menu state
    bool           fluxMenuOpen_             = false;

    // Grit pad states
    bool           gritActive_               = false;
    // Pad held state (150ms)
    bool           gritHeld_                 = false;
    // Long-held state (3s)
    bool           gritLongHeld_             = false;
    // Double-tap and held timers & states
    bool           gritDoubleTapTimerActive_ = false;
    bool           gritHeldTimerActive_      = false;
    StopwatchTimer gritDoubleTapTimer_;
    StopwatchTimer gritHeldTimer_;
    // Grit menu state
    bool           gritMenuOpen_             = false;

    ///////////
    // Held detection
    void detectHeld (StopwatchTimer &timer, bool &timerActive, bool &held, bool &longHeld);

    ///////////
    // Long-hold modulation state
    using SmoothValue = Utils::SmoothValue;

    static constexpr float kLongHoldRampSec = 500.0f;    // ramp time for freq/amp from min->max

    daisysp::Oscillator fluxOsc_;
    SmoothValue         fluxFreqSmoother_ = SmoothValue(kLongHoldRampSec * 1000.0f, (1000.f * blockSize_ / sampleRate_));
    SmoothValue         fluxAmpSmoother_  = SmoothValue(kLongHoldRampSec * 1000.0f, (1000.f * blockSize_ / sampleRate_));
    bool                fluxLongHeldPrev_ = false;

    daisysp::Oscillator gritOsc_;
    SmoothValue         gritFreqSmoother_ = SmoothValue(kLongHoldRampSec * 1000.0f, (1000.f * blockSize_ / sampleRate_));
    SmoothValue         gritAmpSmoother_  = SmoothValue(kLongHoldRampSec * 1000.0f, (1000.f * blockSize_ / sampleRate_));
    bool                gritLongHeldPrev_ = false;
    // General tap/hold handler. Returns double-tap state; updates "held" and "longHeld"; stops timers if pressed == false.
    void handleTap (const bool      padPressed,
                    StopwatchTimer &heldTimer,
                    bool           &heldTimerActive,
                    StopwatchTimer &doubleTapTimer,
                    bool           &doubleTapTimerActive,
                    bool           &held,
                    bool           &longHeld,
                    bool           &doubleTap);

    ///////////
    // Grit display handling functions
    void ledBrightnessGradientFilter (
      FilterType type, uint8_t ringSize, uint8_t spanSize, float minBrightness, float maxBrightness, float *gradValues);
    // Falling: full max until cutoff LED index, then linear descent over Q-based width to min
    void ledBrightnessFallingGradient (
      uint8_t ringSize, uint8_t spanSize, float minBrightness, float maxBrightness, float *gradValues);
    // Ramp: linear ascent over Q-based width up to cutoff LED index, then full max
    void ledBrightnessRampGradient (
      uint8_t ringSize, uint8_t spanSize, float minBrightness, float maxBrightness, float *gradValues);
    // Populate the Grit LED ring with the specified parameters
    void populateGritLedRing (Deck::RingSpan &ringSpan, uint8_t ringSize, uint8_t spanStart, uint8_t spanSize);

    float calculateFilterHalfBandwidth (float centerFreq, float Q);
    void  calculateFilterRingSpanSize (FilterType type, const uint8_t numLeds, uint8_t &start, uint8_t &end);

    // Helpers to derive LED indices from filter params
    uint8_t computeCutoffIdx (uint8_t ch, uint8_t ringSize);

    ///////////
    NOCOPY (Deck);
};
