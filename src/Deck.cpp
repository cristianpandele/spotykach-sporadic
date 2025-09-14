#include "common.h"
#include "Deck.h"

using infrasonic::Log;

void Deck::setChannelConfig (ChannelConfig mode)
{
  if (mode < ChannelConfig::OFF || mode >= ChannelConfig::CH_CONFIG_LAST)
  {
    Log::PrintLine("Invalid operating mode: %d", mode);
    return;
  }
  channelConfig_ = mode;
}

void Deck::setMode (DeckMode m)
{
  if (m < DeckMode::MODE_1 || m >= DeckMode::MODE_LAST)
  {
    Log::PrintLine("Invalid deck mode: %d", m);
    return;
  }
  // Set the deck mode
  mode_ = m;
}

void Deck::setPosition (float position, bool gritLatch, bool &changed, bool &changedGrit)
{
  position = infrasonic::unitclamp(position);
  MAKE_GRIT_CHANGE_STATUS(position, positionControl_, gritLatch);
  changed          = positionChanged;
  changedGrit      = positionChangedWhileGritLatched || positionChangedWhileGritMenuOpen;
  positionControl_ = positionChanged ? position : positionControl_;

  if (changedGrit)
  {
    inputSculptCenterFreq_ =
      daisysp::fmap(positionControl_, InputSculpt::kMinFreq, InputSculpt::kMaxFreq, Mapping::LOG);

    // If grit latched, set input sculpt frequency instead of position
    for (size_t ch = 0; ch < kNumberChannelsStereo; ++ch)
    {
      if (isChannelActive(ch))
      {
        inputSculpt_[ch].setFreq(inputSculptCenterFreq_);
      }
    }
  }
}

void Deck::setSize (float size, bool gritLatch, bool &changed, bool &changedGrit)
{
  size = infrasonic::unitclamp(size);
  MAKE_GRIT_CHANGE_STATUS(size, sizeControl_, gritLatch);
  changed      = sizeChanged;
  changedGrit  = sizeChangedWhileGritLatched || sizeChangedWhileGritMenuOpen;
  sizeControl_ = sizeChanged ? size : sizeControl_;

  if (changedGrit)
  {
    // If grit latched, set input sculpt width instead of size
    for (size_t ch = 0; ch < kNumberChannelsStereo; ++ch)
    {
      if (isChannelActive(ch))
      {
        inputSculpt_[ch].setWidth(sizeControl_);
      }
    }
  }
}

void Deck::setShape (float shape, bool gritLatch, bool &changed, bool &changedGrit)
{
  shape = infrasonic::unitclamp(shape);
  MAKE_GRIT_CHANGE_STATUS(shape, shapeControl_, gritLatch);
  changed       = shapeChanged;
  changedGrit   = shapeChangedWhileGritLatched || shapeChangedWhileGritMenuOpen;
  shapeControl_ = shapeChanged ? shape : shapeControl_;

  if (changedGrit)
  {
    // If grit latched, set input sculpt shape instead of shape
    for (size_t ch = 0; ch < kNumberChannelsStereo; ++ch)
    {
      if (isChannelActive(ch))
      {
        inputSculpt_[ch].setShape(shapeControl_);
      }
    }
  }
}

void Deck::setPitch (float pitch, bool gritLatch, bool &changed, bool &changedGrit)
{
  pitch                   = infrasonic::unitclamp(pitch);
  MAKE_GRIT_CHANGE_STATUS(pitch, pitchControl_, gritLatch);
  changed       = pitchChanged;
  changedGrit   = pitchChangedWhileGritLatched || pitchChangedWhileGritMenuOpen;
  pitchControl_ = pitchChanged ? pitch : pitchControl_;

  if (changedGrit)
  {
    // If grit latched, set drive instead of pitch
    for (size_t ch = 0; ch < kNumberChannelsStereo; ++ch)
    {
      if (isChannelActive(ch))
      {
        inputSculpt_[ch].setOverdrive(pitchControl_);
      }
    }
  }
}

////////////////////////
// Display state and publish functions
bool Deck::getDisplayState (DisplayState &out) const
{
  uint8_t     r   = dispWIdx_;    // read the last published buffer index
  const auto &src = dispBuf_[r];
  if (src.cnt == cntRead_)    // unchanged since last fetch
  {
    return false;
  }
  out      = src.state;
  cntRead_ = src.cnt;
  return true;
}

void Deck::publishDisplay(const DisplayState &state)
{
  uint8_t w         = dispWIdx_ ^ 1;  // toggle write index
  dispBuf_[w].state = state;
  dispBuf_[w].cnt   = dispBuf_[dispWIdx_].cnt + 1; // increment generation count
  dispWIdx_         = w; // update write index
}

// Helper to determine if a channel should be processed in the current channel configuration
bool Deck::isChannelActive (size_t ch) const
{
  // Example logic: only process left channel in MONO_LEFT, right in MONO_RIGHT, both in STEREO
  switch (channelConfig_)
  {
    case ChannelConfig::MONO_LEFT:
    {
      return (ch == 0);
    }
    case ChannelConfig::MONO_RIGHT:
    {
      return (ch == 1);
    }
    case ChannelConfig::STEREO:
    {
      return ((ch == 0 || ch == 1));
    }
    default:
    {
      return false;
    }
  }
}

///////////
// Handle effect pad taps
bool Deck::detectFluxHeld ()
{
  detectHeld(fluxHeldTimer_, fluxHeldTimerActive_, fluxHeld_);
  return fluxHeld_;
}

bool Deck::detectGritHeld ()
{
  detectHeld(gritHeldTimer_, gritHeldTimerActive_, gritHeld_);
  return gritHeld_;
}

void Deck::handleFluxTap (const bool flux, bool &doubleTap, bool &held)
{
  handleTap(flux,
            fluxHeldTimer_,
            fluxHeldTimerActive_,
            fluxDoubleTapTimer_,
            fluxDoubleTapTimerActive_,
            fluxHeld_,
            doubleTap);
  held = fluxHeld_;
}

void Deck::handleGritTap (const bool grit, bool &doubleTap, bool &held)
{
  handleTap(grit,
            gritHeldTimer_,
            gritHeldTimerActive_,
            gritDoubleTapTimer_,
            gritDoubleTapTimerActive_,
            gritHeld_,
            doubleTap);
  held = gritHeld_;
}

void Deck::detectHeld (StopwatchTimer &timer, bool &heldTimerActive, bool &held)
{
  held = ((heldTimerActive) && timer.HasPassedMs(kHeldTimeoutMs));
}

void Deck::handleTap (const bool      padPressed,
                        StopwatchTimer &heldTimer,
                        bool           &heldTimerActive,
                        StopwatchTimer &doubleTapTimer,
                        bool           &doubleTapTimerActive,
                        bool           &held,
                        bool           &doubleTap)
{
  // If button released, stop timer and clear held
  if (!padPressed)
  {
    heldTimerActive = false;
    held            = false;
    return;
  }
  else
  {
    // Handle held pad taps
    bool heldTimerExpired = heldTimer.HasPassedMs(kHeldTimeoutMs);
    if (!heldTimerActive)
    {
      heldTimer.Init();
      heldTimerActive = true;
      // infrasonic::Log::PrintLine("Held timer started");
    }
    else
    {
      if (heldTimerExpired)
      {
        // Held if we exceeded the held threshold
        heldTimerActive = false;
        held            = true;
      }
    }

    // Handle double presses
    bool doubleTapTimerExpired = doubleTapTimer.HasPassedMs(kDoubleTapTimeoutMs);
    if (!doubleTapTimerActive || doubleTapTimerExpired)
    {
      doubleTapTimer.Init();
      doubleTapTimerActive = true;
      // infrasonic::Log::PrintLine("Double tap timer started");
    }
    else
    {
      if (!doubleTapTimerExpired)
      {
        // infrasonic::Log::PrintLine("Double-tap detected");
        doubleTapTimerActive = false;    // reset after double-tap
        doubleTap            = true;
        return;
      }
      // infrasonic::Log::PrintLine("Double-tap timer stopped");
    }
    doubleTap = false;
    return;
  }
}

void Deck::detectEffectPadsHeld ()
{
  detectFluxHeld();
  detectGritHeld();
}

void Deck::updateEffectDisplayStates (DisplayState &view)
{
  // Flux pad LEDs and ring display
  if (isFluxDisplayed() || getFluxActive())
  {
    updateFluxPadLedState(view);
  }

  if (isFluxDisplayed())
  {
    // updateFluxRingState(view);
    view.layerCount = 0; // Clear any other layers if flux is displayed (TODO, remove this)
  }

  // Grit pad LEDs
  if (isGritDisplayed() || getGritActive())
  {
    updateGritPadLedState(view);
  }

  // Grit ring display
  if (isGritDisplayed())
  {
    updateGritRingState(view);
  }
}

void Deck::updateDigitalControlsEffects (const DigitalControlFrame &c)
{
  // Update the digital deck parameters based on the control frame
  setReverse(c.reverse);
  setPlay(c.play);
  setFlux(c.flux);
  setGrit(c.grit);
  // Hold Alt+Flux state
  if (c.altFlux)
  {
    toggleFluxActive();
  }
  else
  // Always feed flux state so release stops timer
  {
    bool dbl = false;
    bool h   = false;
    handleFluxTap(c.flux, dbl, h);
    if (dbl || h)
    {
      toggleFluxMenu();
    }
  }

  // Hold Alt+Grit state
  if (c.altGrit)
  {
    toggleGritActive();
    if (!getGritActive())
    {
      // If grit is deactivated, disable the grit menu
      setGritMenuOpen(false);
    }
  }
  else
  // Always feed grit state so release stops timer
  {
    bool dbl = false;
    bool h   = false;
    handleGritTap(c.grit, dbl, h);
    if (dbl || h)
    {
      toggleGritMenu();
    }
  }
}

///////////
// Display handling functions
// Interpolate between 4 values based on a 0..1 blend factor
void Deck::ledsFourShapeInterpolator (float minValue, float maxValue, float blend, float *values, float *gradValues)
{
  float v;
  if (blend < 0.33333334f)
  {
    float u = blend / 0.33333334f;
    v       = infrasonic::lerp(values[0], values[1], u);
  }
  else if (blend < 0.6666667f)
  {
    float u = (blend - 0.33333334f) / 0.33333334f;
    v       = infrasonic::lerp(values[1], values[2], u);
  }
  else
  {
    float u = (blend - 0.6666667f) / 0.33333334f;
    v       = infrasonic::lerp(values[2], values[3], u);
  }
  *gradValues = daisysp::fclamp(v, minValue, maxValue);
}

// LED brightness gradient depending on shape_ and span size
void Deck::ledBrightnessGradientLog (uint8_t spanSize, float minBrightness, float maxBrightness, float *gradValues)
{
  // Per-LED gradient using analytic envelope within [0, spanSize)
  const uint8_t gradLen = std::min<uint8_t>(spanSize, Hardware::kNumLedsPerRing);

  if (gradLen == 0)
  {
    return;
  }

  std::memset(gradValues, 0, sizeof(float) * Hardware::kNumLedsPerRing);

  for (uint8_t i = 0; i < gradLen; ++i)
  {
    float x = (gradLen > 1) ? ((maxBrightness - minBrightness) * (float)i / (float)(gradLen - 1)) : 0.0f;
    float ledSquareGradient   = maxBrightness;
    float ledFallingGradient  = (maxBrightness - x);
    float ledTriangleGradient = (x <= 0.5f) ? minBrightness + (x / 0.5f) : maxBrightness - (x / 0.5f);
    float ledRampGradient     = minBrightness + x;
    float values[4]           = {ledSquareGradient, ledFallingGradient, ledTriangleGradient, ledRampGradient};
    ledsFourShapeInterpolator(minBrightness, maxBrightness, shape_, values, &gradValues[i]);
    // Logarithmic mapping of the gradient values
    *(&gradValues[i]) = daisysp::fmap(gradValues[i], minBrightness, maxBrightness, Mapping::LOG);
  }
}

// Logarithmic frequency to LED index mapping (integer LED index 0..numLeds-1).
uint8_t Deck::freqToLed (float f, uint8_t numLeds, float filterMinFreq, float filterMaxFreq)
{
  f       = daisysp::fclamp(f, filterMinFreq, filterMaxFreq);
  float t = logf(f / filterMinFreq) / logf(filterMaxFreq / filterMinFreq);    // 0..1
  t       = infrasonic::unitclamp(t);
  return static_cast<uint8_t>(t * (numLeds - 1) + 1);
};

void Deck::populateLedRing (Deck::RingSpan  &ringSpan,
                            uint8_t          ringSize,
                            LedRgbBrightness colorBright,
                            uint8_t          start,
                            uint8_t          spanSize,
                            bool             gradient,
                            bool             invert)
{
  constexpr uint8_t N = spotykach::Hardware::kNumLedsPerRing;
  LedRgbBrightness  ledColor[N];
  // Clear ledColor
  std::fill(std::begin(ledColor), std::end(ledColor), LedRgbBrightness{0x000000, kOffLedBrightness});
  // Populate the current span with the specified color (at indicated brightness)
  std::fill(ledColor + start, std::min(ledColor + start + spanSize, std::end(ledColor)), colorBright);

  // Compute per-LED gradient values, using the brightness indicated in colorBright as maximum value
  if (gradient)
  {
    float ledGradient[N] = {0.0f};
    const float minBrightness  = colorBright.brightness / 4.0f;
    const float maxBrightness  = colorBright.brightness;
    ledBrightnessGradientLog(spanSize, minBrightness, maxBrightness, ledGradient);

    for (uint8_t i = 0; i < spanSize; ++i)
    {
      if (!invert)
      {
        // Copy resulting gradients to the ledColor array
        ledColor[start + i] = {colorBright.rgb, ledGradient[i]};
      }
      else
      {
        // Copy resulting gradients to the ledColor array in reverse order
        ledColor[start + i] = {colorBright.rgb, ledGradient[spanSize - 1 - i]};
      }
    }
  }

  ringSpan.start = start;
  ringSpan.end   = std::min(static_cast<uint8_t>(start + spanSize), ringSize);
  std::copy(std::begin(ledColor), std::end(ledColor), std::begin(ringSpan.led));
}

///////////
// Grit display handling functions

// LED brightness gradient for a particular filter type
void Deck::ledBrightnessGradientFilter (
  FilterType type, uint8_t ringSize, uint8_t spanSize, float minBrightness, float maxBrightness, float *gradValues)
{
  const uint8_t gradLen = std::min<uint8_t>(spanSize, ringSize);

  if (gradLen == 0)
  {
    return;
  }

  float triCenter = (gradLen - 1) * 0.5f;
  float triHalf   = triCenter > 0.0f ? triCenter : 1.0f;

  for (uint8_t i = 0; i < gradLen; ++i)
  {
    float t = 0.0f;
    switch (type)
    {
      case kLowPass:
      {
        // Falling ramp of brightness
        t = 1.0f - static_cast<float>(i) / static_cast<float>(gradLen - 1);
        break;
      }
      case kHighPass:
      {
        // Rising ramp of brightness
        t = static_cast<float>(i) / static_cast<float>(gradLen - 1);
        break;
      }
      case kBandPass:
      {
        // Symmetric triangle around midpoint (spanSize-1)/2 in linear LED index space.
        float dist = fabsf((float)i - triCenter);
        t          = 1.0f - (dist / triHalf);    // 1 at center, 0 at edges
        t          = infrasonic::unitclamp(t);
        break;
      }
    }
    // Map brightness values to log scale
    gradValues[i] = daisysp::fmap(t, minBrightness, maxBrightness, Mapping::LOG);
  }
}

uint8_t Deck::computeCutoffIdx (uint8_t ch, uint8_t ringSize)
{
  float cf = inputSculpt_[ch].getCenterFreq();
  return freqToLed(cf, ringSize, gritFilterMinFreq, gritFilterMaxFreq);
}

float Deck::calculateFilterHalfBandwidth (float centerFreq, float Q)
{
  // Approximate 3dB bandwidth for filter: BW = centerFreq / Q.
  float bandwidthHz = centerFreq / std::max(Q, 0.1f);

  return bandwidthHz * 0.5f;
}

void Deck::calculateFilterRingSpanSize (FilterType type, const uint8_t numLeds, uint8_t &start, uint8_t &end)
{
  uint8_t channel  = channelConfig_ == isChannelActive(1) ? 1 : 0;
  float centerFreq = inputSculpt_[channel].getCenterFreq();
  float Q          = inputSculpt_[channel].getQ();
  float halfBW     = calculateFilterHalfBandwidth(centerFreq, Q);

  // Convert additive bandwidth into a multiplicative ratio for log symmetry.
  // ratio r such that centerFreq / r = fLo and centerFreq * r = fHi.
  float fLoLin;
  float fHiLin;
  switch (type)
  {
    case kBandPass:
    {
      fLoLin = daisysp::fmax(gritFilterMinFreq, centerFreq - halfBW);
      fHiLin = daisysp::fmin(gritFilterMaxFreq, centerFreq + halfBW);
      break;
    }

    case kLowPass:
    {
      fLoLin = gritFilterMinFreq;
      fHiLin = daisysp::fmin(gritFilterMaxFreq, centerFreq + halfBW);
      break;
    }

    case kHighPass:
    {
      // Hacky way of approximating the high-pass filter's bandwidth
      fLoLin = daisysp::fmax(gritFilterMinFreq, centerFreq - halfBW / 6.0f);
      fHiLin = gritFilterMaxFreq;
      break;
    }

    default:
    {
      fLoLin = gritFilterMinFreq;
      fHiLin = gritFilterMaxFreq;
      break;
    }
  }

  // Translate to ratios from center (avoid divide by zero)
  float rLo = daisysp::fmax(1.0001f, centerFreq / daisysp::fmax(1.0f, fLoLin));
  float rHi = daisysp::fmax(1.0001f, fHiLin / daisysp::fmax(1.0f, centerFreq));
  float fLo = 0.0f;
  float fHi = 0.0f;

  if (type == kBandPass)
  {
    // Use geometric mean ratio for symmetry.
    float r = sqrtf(rLo * rHi);
    fLo     = centerFreq / r;
    fHi     = centerFreq * r;
    fLo     = daisysp::fclamp(fLo, gritFilterMinFreq, gritFilterMaxFreq);
    fHi     = daisysp::fclamp(fHi, gritFilterMinFreq, gritFilterMaxFreq);
  }
  else
  {
    fLo = daisysp::fclamp(fLoLin, gritFilterMinFreq, gritFilterMaxFreq);
    fHi = daisysp::fclamp(fHiLin, gritFilterMinFreq, gritFilterMaxFreq);
  }

  start = freqToLed(fLo, numLeds, gritFilterMinFreq, gritFilterMaxFreq);
  end   = freqToLed(fHi, numLeds, gritFilterMinFreq, gritFilterMaxFreq);
  // If we decide to switch to linear plotting:
  // uint8_t start = (fLo / (gritFilterMaxFreq - gritFilterMinFreq)) * (N - 1);
  // uint8_t end   = (fHi / (gritFilterMaxFreq - gritFilterMinFreq)) * (N - 1);
  if (end < start)
  {
    end = start;
  }
  end = std::min<uint8_t>(end, numLeds);
}

void Deck::updateFluxPadLedState (DisplayState &view)
{
  // Set flux pad LED state and color
  view.fluxActive       = true;
  view.fluxLedColors[0] = LedRgbBrightness{0x00ffff, kMaxLedBrightness};
  // If flux is not latched active, set the second phase to black
  view.fluxLedColors[1] =
    getFluxActive() ? LedRgbBrightness{0x00ffff, kMaxLedBrightness} : LedRgbBrightness{0x000000, kOffLedBrightness};
}

LedRgbBrightness Deck::getGritLedColour ()
{
  uint8_t      channel = channelConfig_ == isChannelActive(1) ? 1 : 0;
  InputSculpt &sculpt  = inputSculpt_[channel];
  // Purple color indicating the bandpass area (fade to red with overdrive)
  LedRgbBrightness ledColor  = {0xff00ff, kMaxLedBrightness};
  float            od        = sculpt.getOverdrive();    // 0..0.2
  uint8_t          blueLevel = static_cast<uint8_t>(map(od, sculpt.kMinDriveAmt, sculpt.kMaxDriveAmt, 255.0f, 0.0f));
  ledColor.rgb               = (ledColor.rgb & 0xffffff00) | blueLevel;
  return ledColor;
}

void Deck::updateGritPadLedState (DisplayState &view)
{
  // Set grit pad LED state and color
  view.gritActive       = true;
  LedRgbBrightness gritColor = getGritLedColour();
  view.gritLedColors[0] = gritColor;
  // If grit is not latched active, set the second phase to black
  view.gritLedColors[1] = getGritActive() ? gritColor : LedRgbBrightness{0x000000, kOffLedBrightness};
}

// Grit LED Ring has special gradient handling mechanism, thus separate from populateLedRing
void Deck::populateGritLedRing (Deck::RingSpan &ringSpan, uint8_t ringSize, uint8_t spanStart, uint8_t spanSize)
{
  if (ringSize == 0)
  {
    return;
  }

  LedRgbBrightness ledColor[ringSize];
  LedRgbBrightness gritColor = getGritLedColour();
  // Clear ledColor brightness, set rgb
  std::fill(ledColor, ledColor + ringSize, LedRgbBrightness{gritColor.rgb, kOffLedBrightness});

  // Compute per-LED gradient values, using the brightness indicated in gritColor as maximum value
  float ledSquareGradient[ringSize]   = {0.0f};
  float ledFallingGradient[ringSize]  = {0.0f};
  float ledTriangleGradient[ringSize] = {0.0f};
  float ledRampGradient[ringSize]     = {0.0f};

  const float minBrightness           = gritColor.brightness / 4.0f;
  const float maxBrightness           = gritColor.brightness;

  uint8_t channel                     = channelConfig_ == isChannelActive(1) ? 1 : 0;
  uint8_t cutoffIdx                   = computeCutoffIdx(channel, ringSize);

  // Square gradient: full max to all LEDs
  std::fill(ledSquareGradient, ledSquareGradient + ringSize, kMaxLedBrightness);

  // Falling gradient: full max to cutoff LED, then descend over Q-derived width
  std::fill(ledFallingGradient + spanStart, ledFallingGradient + cutoffIdx, maxBrightness);
  // Calculate the gradient size
  uint8_t gradientSize = daisysp::fclamp(spanSize - (cutoffIdx + spanStart), 1, spanSize - cutoffIdx);
  ledBrightnessGradientFilter(kLowPass,
                              gradientSize,
                              gradientSize,
                              minBrightness,
                              maxBrightness,
                              ledFallingGradient + cutoffIdx);

  // Triangle gradient (symmetric around span center)
  gradientSize = spanSize;
  ledBrightnessGradientFilter(kBandPass,
                              gradientSize,
                              gradientSize,
                              minBrightness,
                              maxBrightness,
                              ledTriangleGradient + spanStart);

  // Ramp gradient: rise up to cutoff over Q-derived width, then full max
  std::fill(ledRampGradient + cutoffIdx, ledRampGradient + ringSize, maxBrightness);
  // Scale the filter pass band to the display for a meaningful appearance
  float scaledFilterBand = static_cast<float>(cutoffIdx - spanStart) / (static_cast<float>(ringSize));
  gradientSize           = static_cast<uint8_t>(daisysp::fmap(scaledFilterBand, 1.0f, ringSize));
  // Calculate the gradient start index
  uint8_t gradientStart = daisysp::fmax(static_cast<uint8_t>(cutoffIdx - gradientSize), 0);
  ledBrightnessGradientFilter(kHighPass,
                              gradientSize,
                              gradientSize,
                              minBrightness,
                              maxBrightness,
                              ledRampGradient + gradientStart);

  for (uint8_t i = 0; i < ringSize; ++i)
  {
    // Interpolate between the LED gradient values
    float values[4] = {ledSquareGradient[i], ledFallingGradient[i], ledTriangleGradient[i], ledRampGradient[i]};
    ledsFourShapeInterpolator(minBrightness,
                              maxBrightness,
                              inputSculpt_[channel].getShape(),    // Use the Input Sculpt shape
                              values,
                              &ledColor[i].brightness);
  }

  // Update the ring span information
  ringSpan.start = spanStart;
  ringSpan.end   = std::min(static_cast<uint8_t>(spanStart + spanSize), ringSize);
  std::copy(ledColor, ledColor + ringSize, std::begin(ringSpan.led));
}

void Deck::updateGritRingState (DisplayState &view)
{
  constexpr uint8_t N = spotykach::Hardware::kNumLedsPerRing;
  Deck::RingSpan    ringSpan;

  // Yellow area indicating the frequency range
  populateLedRing(ringSpan, N, {0xffff00, kMaxLedBrightness}, 0, N);
  view.rings[view.layerCount++] = ringSpan;

  uint8_t squareSpanStart       = 0;
  uint8_t squareSpanEnd         = N;

  // Falling shape span: start at 0, end at cutoff + half-width (bounded)
  uint8_t fallingSpanStart;
  uint8_t fallingSpanEnd;
  calculateFilterRingSpanSize(kLowPass, N, fallingSpanStart, fallingSpanEnd);

  // Compute bandpass span from center frequency and width (Q).
  uint8_t triangleSpanStart;
  uint8_t triangleSpanEnd;
  calculateFilterRingSpanSize(kBandPass, N, triangleSpanStart, triangleSpanEnd);

  // Ramp shape span: start at max(cutoff - half-width, 0) up to end
  uint8_t rampSpanStart;
  uint8_t rampSpanEnd;
  calculateFilterRingSpanSize(kHighPass, N, rampSpanStart, rampSpanEnd);

  float spanStart;
  float spanEnd;

  float shapeSpanStart[4] = {(float)squareSpanStart, (float)fallingSpanStart, (float)triangleSpanStart, (float)rampSpanStart};
  float shapeSpanEnd[4]   = {(float)squareSpanEnd, (float)fallingSpanEnd, (float)triangleSpanEnd, (float)rampSpanEnd};
  // Interpolate between the 4 filter shape span starts and ends
  uint8_t channel = channelConfig_ == isChannelActive(1) ? 1 : 0;
  ledsFourShapeInterpolator(0, N, inputSculpt_[channel].getShape(), shapeSpanStart, &spanStart);
  ledsFourShapeInterpolator(0, N, inputSculpt_[channel].getShape(), shapeSpanEnd, &spanEnd);

  // Purple span indicating the filter area
  uint8_t          filterSpanSize = static_cast<uint8_t>(daisysp::fclamp(spanEnd - spanStart, 0, N - spanStart));
  populateGritLedRing(ringSpan, N, spanStart, filterSpanSize);
  view.rings[view.layerCount++] = ringSpan;
}