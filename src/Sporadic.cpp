#include "app.h"
#include "Sporadic.h"
#include <cmath>

void Sporadic::init ()
{
  inputSculpt_.init(sampleRate_);
  delayNetwork_.init(sampleRate_, kNumBands, kBlockSize);
}

void Sporadic::processAudio (AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t blockSize)
{
  // Pass-through if channelConfig_ is not MONO_LEFT, MONO_RIGHT, or STEREO
  if (channelConfig_ == ChannelConfig::OFF || channelConfig_ >= ChannelConfig::CH_CONFIG_LAST)
  {
    return;
  }

  // Fill sculpted input (per-channel enable) into member scratch buffers
  for (size_t i = 0; i < blockSize; i++)
  {
    for (size_t ch = 0; ch < kNumberChannelsStereo; ++ch)
    {
      if (isChannelActive(ch))
      {
        inputSculptBuf_[ch][i] = inputSculpt_.processSample(in[ch][i]);
      }
      else
      {
        inputSculptBuf_[ch][i] = in[ch][i];
      }
    }
  }

  // inputSculpt_.processBlock(inputSculptBuf_[0], inputSculptBuf_[1], delayNetworkBuf_[0], delayNetworkBuf_[1], blockSize);

  delayNetwork_.processBlock(inputSculptBuf_[0], inputSculptBuf_[1], delayNetworkBuf_[0], delayNetworkBuf_[1], blockSize);

  for (size_t i = 0; i < blockSize; ++i)
  {
    for (size_t ch = 0; ch < kNumberChannelsStereo; ++ch)
    {
      if (isChannelActive(ch))
      {
        // Apply dry-wet mix
        out[ch][i] = infrasonic::lerp(in[ch][i], delayNetworkBuf_[ch][i], mix_);
      }
    }
  }
}

void Sporadic::setMix (float m, bool fluxLatch)
{
  // If flux latch is pressed, set drive instead of mix
  if (fluxLatch)
  {
    inputSculpt_.setOverdrive(m);
  }
  else
  {
    mix_ = m;
  }
}

void Sporadic::setPosition (float p, bool fluxLatch)
{
  // If flux latch is pressed, set input sculpt frequency instead of position
  if (fluxLatch)
  {
    // Map the frequency to the input sculpt
    inputSculpt_.setFreq(p);
  }
  else
  {
    position_ = p;
  }
}

void Sporadic::setSize (float s, bool fluxLatch)
{
  // If flux latch is pressed, set input sculpt width instead of size
  if (fluxLatch)
  {
    // Map the width to the input sculpt
    inputSculpt_.setWidth(s);
  }
  else
  {
    size_ = s;
  }
}

void Sporadic::updateAnalogControls(const AnalogControlFrame &c)
{
  // Update the analog effect parameters based on the control frame
  setMix(c.mix, c.mixFlux || getFluxMenuOpen ());
  setPitch(c.pitch);
  setPosition(c.position, c.positionFlux || getFluxMenuOpen ());
  setSize(c.size, c.sizeFlux || getFluxMenuOpen ());
  setShape(c.shape);
}

void Sporadic::updateDigitalControls (const DigitalControlFrame &c)
{
  // Update the digital effect parameters based on the control frame
  setReverse(c.reverse);
  setPlay(c.play);
  setFlux(c.flux);

  // Hold Alt+Flux state
  if (c.altFlux)
  {
    toggleFluxActive();
    if (!getFluxActive ())
    {
      // If flux is deactivated, disable the flux menu
      setFluxMenuOpen(false);
    }
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
  c.altPlay   = false;    // Not used in this effect
  c.spotyPlay = false;    // Not used in this effect
  // c.flux      = flux_;
  c.altFlux   = false;    // Alt+Flux is just a toggle
  // c.grit      = grit_;
  c.altGrit   = false;    // Alt+Grit is just a toggle
}

// Logarithmic frequency to LED index mapping (integer LED index 0..numLeds-1).
uint8_t Sporadic::freqToLed(float f, uint8_t numLeds, float fMin, float fMax)
{
  f       = daisysp::fclamp(f, fMin, fMax);
  float t = logf(f / fMin) / logf(fMax / fMin); // 0..1
  t       = daisysp::fclamp(t, 0.0f, 1.0f);
  return static_cast<uint8_t>(t * (numLeds - 1) + 1);
};

void Sporadic::ledBrightnessTriangleGradient (uint8_t spanSize, float minBrightness, float maxBrightness, float *gradValues)
{
  // Symmetric triangle around midpoint (spanSize-1)/2 in linear LED index space.
  const uint8_t gradLen = std::min<uint8_t>(spanSize, Hardware::kNumLedsPerRing);
  std::memset(gradValues, 0, sizeof(float) * Hardware::kNumLedsPerRing);

  if (gradLen == 0)
    return;

  float center = (gradLen - 1) * 0.5f;
  float half   = center > 0.0f ? center : 1.0f;

  for (uint8_t i = 0; i < gradLen; ++i)
  {
    float dist    = fabsf((float)i - center);
    float triUnit = 1.0f - (dist / half);    // 1 at center, 0 at edges
    triUnit       = daisysp::fclamp(triUnit, 0.0f, 1.0f);
    float val     = minBrightness + (maxBrightness - minBrightness) * triUnit;
    gradValues[i] = daisysp::fclamp(val, minBrightness, maxBrightness);
  }
}

void Sporadic::populateLedRing (Effect::RingSpan &ringSpan,
                                uint8_t           ringSize,
                                LedRgbBrightness  colorBright,
                                uint8_t           start,
                                uint8_t           spanSize,
                                bool              gradient)
{
  LedRgbBrightness ledColor[Hardware::kNumLedsPerRing];
  // Clear ledColor
  std::fill(std::begin(ledColor), std::end(ledColor), LedRgbBrightness{0x000000, 0.0f});
  // Populate the current span with the specified color (at indicated brightness)
  std::fill(ledColor + start, std::min(ledColor + start + spanSize, std::end(ledColor)), colorBright);

  // Compute per-LED gradient values, using the brightness indicated in colorBright as maximum value
  if (gradient)
  {
    float ledGradient[Hardware::kNumLedsPerRing] = {0.0f};
    ledBrightnessTriangleGradient(spanSize, (colorBright.brightness / 3.0f), colorBright.brightness, ledGradient);

    for (uint8_t i = 0; i < spanSize; ++i)
    {
      // Copy resulting gradients to the ledColor array
      ledColor[start + i] = {colorBright.rgb, ledGradient[i]};
    }
  }

  ringSpan.start = start;
  ringSpan.end   = std::min(static_cast<uint8_t>(start + spanSize), ringSize);
  std::copy(std::begin(ledColor), std::end(ledColor), std::begin(ringSpan.led));
}

void Sporadic::updateBandpassDisplay (const uint8_t numLeds, uint8_t &start, uint8_t &end)
{
  float centerFreq = inputSculpt_.getCenterFreq();
  float Q          = inputSculpt_.getQ();
  // Approximate 3dB bandwidth for bandpass: BW = centerFreq / Q.
  float           bandwidthHz = centerFreq / std::max(Q, 0.1f);
  constexpr float fMin        = 50.0f;
  constexpr float fMax        = 18000.0f;
  // Convert additive bandwidth into a multiplicative ratio for log symmetry.
  // ratio r such that centerFreq / r = fLo and centerFreq * r = fHi.
  float halfBW = bandwidthHz * 0.5f;
  float fLoLin = daisysp::fmax(fMin, centerFreq - halfBW);
  float fHiLin = daisysp::fmin(fMax, centerFreq + halfBW);
  // Translate to ratios from center (avoid divide by zero)
  float rLo = daisysp::fmax(1.0001f, centerFreq / daisysp::fmax(1.0f, fLoLin));
  float rHi = daisysp::fmax(1.0001f, fHiLin / daisysp::fmax(1.0f, centerFreq));
  // Use geometric mean ratio for symmetry.
  float r       = sqrtf(rLo * rHi);
  float fLo     = centerFreq / r;
  float fHi     = centerFreq * r;
  fLo           = daisysp::fclamp(fLo, fMin, fMax);
  fHi           = daisysp::fclamp(fHi, fMin, fMax);

  start = freqToLed(fLo, numLeds, fMin, fMax);
  end   = freqToLed(fHi, numLeds, fMin, fMax);
  // If we decide to switch to linear plotting:
  // uint8_t start = (fLo / (fMax - fMin)) * (N - 1);
  // uint8_t end   = (fHi / (fMax - fMin)) * (N - 1);
  if (end < start)
  {
    end = start;
  }
  end = std::min<uint8_t>(end, numLeds);
}

void Sporadic::updateFluxDisplayState (DisplayState& view)
{
  // Purple color indicating the bandpass area (fade to red with overdrive)
  LedRgbBrightness ledColor  = {0xff00ff, 1.0f};
  float            od        = inputSculpt_.getOverdrive();    // 0..0.2
  uint8_t          blueLevel = static_cast<uint8_t>(map(od, 0.5f, 0.7f, 255.0f, 0.0f));
  ledColor.rgb               = (ledColor.rgb & 0xffffff00) | blueLevel;

  if (getFluxHeld() || getFluxMenuOpen())
  {
    constexpr uint8_t N = spotykach::Hardware::kNumLedsPerRing;
    Effect::RingSpan  ringSpan;

    // Yellow area indicating the frequency range
    populateLedRing(ringSpan, N, {0xffff00, 1.0f}, 0, N);
    view.rings[view.layerCount++] = ringSpan;

    // Compute bandpass span from center frequency and width (Q).
    uint8_t bandpassSpanStart;
    uint8_t bandpassSpanEnd;
    updateBandpassDisplay(N, bandpassSpanStart, bandpassSpanEnd);

    // Purple span indicating the bandpass area
    populateLedRing(ringSpan, N, ledColor, bandpassSpanStart, bandpassSpanEnd - bandpassSpanStart, true);
    view.rings[view.layerCount++] = ringSpan;
  }

  // Set flux pad LED state and color
  view.fluxActive = true;
  view.fluxLedColors[0] = LedRgbBrightness{ledColor.rgb, 1.0f};
  if (getFluxActive())
  {
    // If flux is latched active, set the second phase to the same color
    view.fluxLedColors[1] = LedRgbBrightness{ledColor.rgb, 1.0f};    // Green for active state
  }
  else
  {
    // If flux is not latched active, set the second phase to black
    view.fluxLedColors[1] = LedRgbBrightness{0x000000, 1.0f};
  }
}

void Sporadic::updateDisplayState ()
{
  DisplayState view{};

  // Check if there is an update to the held state
  detectFluxHeld();

  if (isFluxPlaying())
  {
    updateFluxDisplayState(view);
  }
  publishDisplay(view);
}

void Sporadic::getDigitalControls (DigitalControlFrame &c)
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
      if (isFluxPlaying())
      {
        inputSculpt_.processBlockMono(in[ch], inputSculptBuf_[ch], blockSize);
      }
      else
      {
        // If the input sculpting is not active, copy the input to the scratch buffer
        std::copy(in[ch], in[ch] + blockSize, inputSculptBuf_[ch]);
      }
      delayNetwork_.processBlockMono(inputSculptBuf_[ch], delayNetworkBuf_[ch], blockSize);
    }
  }

  delayNetwork_.processBlock(inputSculptBuf_[0],
                             inputSculptBuf_[1],
                             delayNetworkBuf_[0],
                             delayNetworkBuf_[1],
                             blockSize);

  for (size_t i = 0; i < blockSize; ++i)
  {
    for (size_t ch = 0; ch < kNumberChannelsStereo; ++ch)
    {
      if (isChannelActive(ch))
      {
        // Apply dry-wet mix
        out[ch][i] = infrasonic::lerp(in[ch][i], delayNetworkBuf_[ch][i], mix_);
      }
    }
  }
}