#include "app.h"
#include "Sporadic.h"
#include <cmath>

void Sporadic::init ()
{
  inputSculpt_.init(sampleRate_);
}

void Sporadic::processAudio (AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t blockSize)
{
  // Pass-through if channelConfig_ is not MONO_LEFT, MONO_RIGHT, or STEREO
  if (channelConfig_ == ChannelConfig::OFF || channelConfig_ >= ChannelConfig::CH_CONFIG_LAST)
  {
    std::copy(IN_L, IN_L + blockSize, OUT_L);
    std::copy(IN_R, IN_R + blockSize, OUT_R);
    return;
  }

  // Process the Sporadic effect audio: for now just apply bandpass (mono or stereo)
  for (size_t i = 0; i < blockSize; i++)
  {
    for (size_t ch = 0; ch < kNumberChannelsStereo; ++ch)
    {
      if (isChannelActive(ch))
      {
        float wet  = inputSculpt_.processSample(in[ch][i]);

        // Apply dry-wet mix
        out[ch][i] = infrasonic::lerp(in[ch][i], wet, mix_);
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
  setMix(c.mix, c.mixFlux);
  setPitch(c.pitch);
  setPosition(c.position, c.positionFlux);
  setSize(c.size, c.sizeFlux);
  setShape(c.shape);
}

void Sporadic::updateDigitalControls (const DigitalControlFrame &c)
{
  // Update the digital effect parameters based on the control frame
  setReverse(c.reverse);
  setPlay(c.play);
  setFlux(c.flux);
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

void Sporadic::updateFluxDisplayState (DisplayState& view)
{
  // Build yellow base ring (full ring)
  constexpr uint8_t N = spotykach::Hardware::kNumLedsPerRing;
  Effect::RingSpan  ringSpan;

  // Yellow area indicating the frequency range
  LedRgbBrightness ledColor = {0xffff00, 0.5f};
  populateLedRing(ringSpan, N, ledColor, 0, N);
  view.rings[view.layerCount++] = ringSpan;

  // Compute bandpass span from center frequency and width (Q).
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

  uint8_t start = freqToLed(fLo, N, fMin, fMax);
  uint8_t end   = freqToLed(fHi, N, fMin, fMax);
  // If we decide to switch to linear plotting:
  // uint8_t start = (fLo / (fMax - fMin)) * (N - 1);
  // uint8_t end   = (fHi / (fMax - fMin)) * (N - 1);
  if (end < start)
  {
    end = start;
  }
  end = std::min<uint8_t>(end, N);

  // Purple span indicating the bandpass area (fade to red with overdrive)
  ledColor = {0xff00ff, 1.0f};
  float     od        = inputSculpt_.getOverdrive(); // 0..0.2
  uint8_t   blueLevel = static_cast<uint8_t>(map(od, 0.5f, 0.7f, 255.0f, 0.0f));
  ledColor.rgb        = (ledColor.rgb & 0xffffff00) | blueLevel;

  populateLedRing(ringSpan, N, ledColor, start, end - start, true);
  view.rings[view.layerCount++] = ringSpan;
}

void Sporadic::updateDisplayState ()
{
  DisplayState view{};

  if (flux_)
  {
    updateFluxDisplayState(view);
  }
  publishDisplay(view);
}

void Sporadic::getDigitalControls (DigitalControlFrame &c)
{
  c.reverse = reverse_;
  c.play = play_;
  c.altPlay = false;  // Not used in this effect
  c.spotyPlay = false;  // Not used in this effect
}