#include "app.h"
#include "Sporadic.h"
#include <cmath>

void Sporadic::init ()
{
  inputSculpt_.init(sampleRate_);
  delayNetwork_.init(sampleRate_, kNumBands, kBlockSize);
}

void Sporadic::setMix (float m, bool gritLatch)
{
  // If grit latched, set drive instead of mix
  if (gritLatch)
  {
    inputSculpt_.setOverdrive(m);
  }
  else
  {
    mix_ = m;
  }
}

void Sporadic::setPosition (float p, bool gritLatch)
{
  // If grit latched, set input sculpt frequency instead of position
  if (gritLatch)
  {
    // Map the frequency to the input sculpt
    inputSculpt_.setFreq(p);
  }
  else
  {
    position_ = p;
  }
}

void Sporadic::setSize (float s, bool gritLatch)
{
  // If grit latched, set input sculpt width instead of size
  if (gritLatch)
  {
    // Map the width to the input sculpt
    inputSculpt_.setWidth(s);
  }
  else
  {
    size_ = s;
  }
}

void Sporadic::setShape (float s, bool gritLatch)
{
  // If grit latched, set input sculpt shape instead of shape
  if (gritLatch)
  {
    // Map the shape to the input sculpt
    inputSculpt_.setShape(s);
  }
  else
  {
    shape_ = s;
  }
}

void Sporadic::updateAnalogControls(const AnalogControlFrame &c)
{
  // Update the analog deck parameters based on the control frame
  // Use grit modifiers (pad latch or grit menu) to route to InputSculpt
  setMix(c.mix, c.mixGrit || getGritMenuOpen());
  setPitch(c.pitch);
  setPosition(c.position, c.positionGrit || getGritMenuOpen());
  setSize(c.size, c.sizeGrit || getGritMenuOpen());
  setShape(c.shape, c.shapeGrit || getGritMenuOpen());
}

void Sporadic::updateDigitalControls (const DigitalControlFrame &c)
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
    if (!getGritActive ())
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

void Sporadic::getDigitalControls (DigitalControlFrame &c)
{
  // c.reverse   = reverse_;
  // c.play      = play_;
  c.altPlay   = false;    // Not used in this deck
  c.spotyPlay = false;    // Not used in this deck
  // c.flux      = flux_;
  c.altFlux   = false;    // Alt+Flux is just a toggle
  // c.grit      = grit_;
  c.altGrit   = false;    // Alt+Grit is just a toggle
}

// Logarithmic frequency to LED index mapping (integer LED index 0..numLeds-1).
uint8_t Sporadic::freqToLed(float f, uint8_t numLeds, float filterMinFreq, float filterMaxFreq)
{
  f       = daisysp::fclamp(f, filterMinFreq, filterMaxFreq);
  float t = logf(f / filterMinFreq) / logf(filterMaxFreq / filterMinFreq); // 0..1
  t       = daisysp::fclamp(t, 0.0f, 1.0f);
  return static_cast<uint8_t>(t * (numLeds - 1) + 1);
};

void Sporadic::ledBrightnessFilterGradient (
  FilterType type, uint8_t ringSize, uint8_t spanSize, float minBrightness, float maxBrightness, float *gradValues)
{
  const uint8_t gradLen = std::min<uint8_t>(spanSize, ringSize);

  if (gradLen == 0)
    return;

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
        t          = daisysp::fclamp(t, 0.0f, 1.0f);
      }
      break;
    }
    float val     = minBrightness + (maxBrightness - minBrightness) * t;
    gradValues[i] = daisysp::fclamp(val, minBrightness, maxBrightness);
  }
}

uint8_t Sporadic::computeCutoffIdx (uint8_t ringSize)
{
  float           cf   = inputSculpt_.getCenterFreq();
  return freqToLed(cf, ringSize, filterMinFreq, filterMaxFreq);
}

void Sporadic::populateLedRing (Deck::RingSpan  &ringSpan,
                                uint8_t          ringSize,
                                LedRgbBrightness colorBright,
                                uint8_t          spanStart,
                                uint8_t          spanSize,
                                bool             gradient)
{
  LedRgbBrightness ledColor[ringSize];
  // Clear ledColor
  std::fill(ledColor, ledColor + ringSize, LedRgbBrightness{0x000000, 0.0f});

  // Compute per-LED gradient values, using the brightness indicated in colorBright as maximum value
  if (gradient)
  {
    float ledSquareGradient[ringSize]   = {0.0f};
    float ledFallingGradient[ringSize]  = {0.0f};
    float ledTriangleGradient[ringSize] = {0.0f};
    float ledRampGradient[ringSize]     = {0.0f};

    const float minBrightness = colorBright.brightness / 4.0f;
    const float maxBrightness = colorBright.brightness;

    uint8_t cutoffIdx = computeCutoffIdx(ringSize);

    // Square gradient: full max to all LEDs
    std::fill(ledSquareGradient, ledSquareGradient + ringSize, 1.0f);

    // Falling gradient: full max to cutoff LED, then descend over Q-derived width
    std::fill(ledFallingGradient + spanStart, ledFallingGradient + cutoffIdx, maxBrightness);
    // Calculate the gradient size
    uint8_t gradientSize = daisysp::fclamp(spanSize - (cutoffIdx + spanStart), 1, spanSize - cutoffIdx);
    ledBrightnessFilterGradient(kLowPass,
                                gradientSize,
                                gradientSize,
                                minBrightness,
                                maxBrightness,
                                ledFallingGradient + cutoffIdx);

    // Triangle gradient (symmetric around span center)
    gradientSize = spanSize;
    ledBrightnessFilterGradient(kBandPass,
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
    uint8_t gradientStart  = daisysp::fmax(static_cast<uint8_t>(cutoffIdx - gradientSize), 0);
    ledBrightnessFilterGradient(kHighPass,
                                gradientSize,
                                gradientSize,
                                minBrightness,
                                maxBrightness,
                                ledRampGradient + gradientStart);

    for (uint8_t i = 0; i < ringSize; ++i)
    {
      // Interpolate between the LED gradient shapes
      float t = inputSculpt_.getShape(); // 0..1
      float v;
      if (t < 0.33333334f)
      {
        float u = t / 0.33333334f;
        v       = infrasonic::lerp(ledSquareGradient[i], ledFallingGradient[i], u);
      }
      else if (t < 0.6666667f)
      {
        float u = (t - 0.33333334f) / 0.33333334f;
        v       = infrasonic::lerp(ledFallingGradient[i], ledTriangleGradient[i], u);
      }
      else
      {
        float u = (t - 0.6666667f) / 0.33333334f;
        v       = infrasonic::lerp(ledTriangleGradient[i], ledRampGradient[i], u);
      }
      // Copy resulting brightness into the ledColor array
      ledColor[i] = {colorBright.rgb, v};
    }
  }
  else
  {
    // Populate the current span with the specified color (at indicated brightness)
    std::fill(ledColor + spanStart,
              ledColor + std::min(static_cast<uint8_t>(spanStart + spanSize), ringSize),
              colorBright);
  }

  // Update the ring span information
  ringSpan.start = spanStart;
  ringSpan.end   = std::min(static_cast<uint8_t>(spanStart + spanSize), ringSize);
  std::copy(ledColor, ledColor + ringSize, std::begin(ringSpan.led));
}

float Sporadic::calculateFilterHalfBandwidth (float centerFreq, float Q)
{
  // Approximate 3dB bandwidth for filter: BW = centerFreq / Q.
  float           bandwidthHz = centerFreq / std::max(Q, 0.1f);

  return bandwidthHz * 0.5f;
}

void Sporadic::calculateFilterRingSpanSize (FilterType type, const uint8_t numLeds, uint8_t &start, uint8_t &end)
{
  float centerFreq = inputSculpt_.getCenterFreq();
  float Q          = inputSculpt_.getQ();
  float halfBW     = calculateFilterHalfBandwidth(centerFreq, Q);

  // Convert additive bandwidth into a multiplicative ratio for log symmetry.
  // ratio r such that centerFreq / r = fLo and centerFreq * r = fHi.
  float fLoLin;
  float fHiLin;
  switch (type)
  {
    case kBandPass:
    {
      fLoLin = daisysp::fmax(filterMinFreq, centerFreq - halfBW);
      fHiLin = daisysp::fmin(filterMaxFreq, centerFreq + halfBW);
      break;
    }

    case kLowPass:
    {
      fLoLin = filterMinFreq;
      fHiLin = daisysp::fmin(filterMaxFreq, centerFreq + halfBW);
      break;
    }

    case kHighPass:
    {
      // Hacky way of approximating the high-pass filter's bandwidth
      fLoLin = daisysp::fmax(filterMinFreq, centerFreq - halfBW / 6.0f);
      fHiLin = filterMaxFreq;
      break;
    }

    default:
    {
      fLoLin = filterMinFreq;
      fHiLin = filterMaxFreq;
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
    float r       = sqrtf(rLo * rHi);
    fLo           = centerFreq / r;
    fHi           = centerFreq * r;
    fLo           = daisysp::fclamp(fLo, filterMinFreq, filterMaxFreq);
    fHi           = daisysp::fclamp(fHi, filterMinFreq, filterMaxFreq);
  }
  else
  {
    fLo = daisysp::fclamp(fLoLin, filterMinFreq, filterMaxFreq);;
    fHi = daisysp::fclamp(fHiLin, filterMinFreq, filterMaxFreq);;
  }

  start = freqToLed(fLo, numLeds, filterMinFreq, filterMaxFreq);
  end   = freqToLed(fHi, numLeds, filterMinFreq, filterMaxFreq);
  // If we decide to switch to linear plotting:
  // uint8_t start = (fLo / (filterMaxFreq - filterMinFreq)) * (N - 1);
  // uint8_t end   = (fHi / (filterMaxFreq - filterMinFreq)) * (N - 1);
  if (end < start)
  {
    end = start;
  }
  end = std::min<uint8_t>(end, numLeds);
}

void Sporadic::updateGritDisplayState (DisplayState& view)
{
  // Purple color indicating the bandpass area (fade to red with overdrive)
  LedRgbBrightness ledColor  = {0xff00ff, 1.0f};
  float            od        = inputSculpt_.getOverdrive();    // 0..0.2
  uint8_t          blueLevel = static_cast<uint8_t>(map(od, inputSculpt_.kMinDriveAmt, inputSculpt_.kMaxDriveAmt, 255.0f, 0.0f));
  ledColor.rgb               = (ledColor.rgb & 0xffffff00) | blueLevel;

  if (getGritHeld() || getGritMenuOpen())
  {
    constexpr uint8_t N = spotykach::Hardware::kNumLedsPerRing;
    Deck::RingSpan    ringSpan;

    // Yellow area indicating the frequency range
    populateLedRing(ringSpan, N, {0xffff00, 1.0f}, 0, N);
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

    float   t = inputSculpt_.getShape();    // 0..1
    uint8_t spanStart;
    uint8_t spanEnd;
    if (t < 0.33333334f)
    {
      float u   = t / 0.33333334f;
      spanStart = infrasonic::lerp(squareSpanStart, fallingSpanStart, u);
      spanEnd   = infrasonic::lerp(squareSpanEnd, fallingSpanEnd, u);
    }
    else if (t < 0.6666667f)
    {
      float u   = (t - 0.33333334f) / 0.33333334f;
      spanStart = infrasonic::lerp(fallingSpanStart, triangleSpanStart, u);
      spanEnd   = infrasonic::lerp(fallingSpanEnd, triangleSpanEnd, u);
    }
    else
    {
      float u   = (t - 0.6666667f) / 0.33333334f;
      spanStart = infrasonic::lerp(triangleSpanStart, rampSpanStart, u);
      spanEnd   = infrasonic::lerp(triangleSpanEnd, rampSpanEnd, u);
    }

    // Purple span indicating the filter area
    uint8_t filterSpanSize = static_cast<uint8_t>(daisysp::fclamp(spanEnd - spanStart, 0, N - spanStart));
    populateLedRing(ringSpan, N, ledColor, spanStart, filterSpanSize, true);
    view.rings[view.layerCount++] = ringSpan;
  }

  // Set grit pad LED state and color
  view.gritActive = true;
  view.gritLedColors[0] = LedRgbBrightness{ledColor.rgb, 1.0f};
  if (getGritActive())
  {
    // If grit is latched active, set the second phase to the same color
    view.gritLedColors[1] = LedRgbBrightness{ledColor.rgb, 1.0f};    // Green for active state
  }
  else
  {
    // If grit is not latched active, set the second phase to black
    view.gritLedColors[1] = LedRgbBrightness{0x000000, 1.0f};
  }
}

void Sporadic::updateDisplayState ()
{
  DisplayState view{};

  // Check if there is an update to the held state
  detectGritHeld();

  if (isGritPlaying())
  {
    updateGritDisplayState(view);
  }
  publishDisplay(view);
}

void Sporadic::processAudio (AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t blockSize)
{
  // Pass-through if channelConfig_ is not MONO_LEFT, MONO_RIGHT, or STEREO
  if (channelConfig_ == ChannelConfig::OFF || channelConfig_ >= ChannelConfig::CH_CONFIG_LAST)
  {
    return;
  }

  // Fill sculpted input (per-channel enable) into member scratch buffers
  for (size_t ch = 0; ch < kNumberChannelsStereo; ++ch)
  {
    if (isChannelActive(ch))
    {
      if (isGritPlaying())
      {
        inputSculpt_.processBlockMono(in[ch], inputSculptBuf_[ch], blockSize);
      }
      else
      {
        // If the input sculpting is not active, copy the input to the scratch buffer
        std::copy(in[ch], in[ch] + blockSize, inputSculptBuf_[ch]);
      }
      delayNetwork_.processBlockMono(inputSculptBuf_[ch], delayNetworkBuf_[ch], blockSize);

      // Apply dry-wet mix
      Utils::audioBlockLerp(in[ch], delayNetworkBuf_[ch], out[ch], mix_, blockSize);
    }
  }
}