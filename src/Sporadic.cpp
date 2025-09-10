#include "app.h"
#include "Sporadic.h"
#include <cmath>

void Sporadic::init ()
{
  for (size_t ch = 0; ch < kNumberChannelsStereo; ++ch)
  {
    // Initialize the input sculpt effect
    inputSculpt_[ch].init(sampleRate_);

    // Initialize the edge tree for envelope following
    edgeTree_[ch].init(sampleRate_);
  }
  // Initialize the delay network
  delayNetwork_.init(sampleRate_, blockSize_, kMaxNutrientBands, kMaxNumDelayProcs);
}

//////////
// Handle parameter changes
void Sporadic::setPosition (float p, bool gritLatch)
{
  bool positionChanged;
  bool positionChangedGrit;
  Deck::setPosition(p, gritLatch, positionChanged, positionChangedGrit);

  if (positionChangedGrit)
  {
    setDelayNetworkParameters(inputSculptCenterFreq_, spoty_);
  }
  else if (positionChanged && !getGritMenuOpen())
  {
    position_ = positionControl_;
  }
}

void Sporadic::setSize (float s, bool gritLatch)
{
  bool sizeChanged;
  bool sizeChangedGrit;
  Deck::setSize(s, gritLatch, sizeChanged, sizeChangedGrit);

  if (sizeChangedGrit)
  {
    return;
  }
  else if (sizeChanged && !getGritMenuOpen())
  {
    size_ = sizeControl_;
  }
}

void Sporadic::setShape (float s, bool gritLatch)
{
  bool shapeChanged;
  bool shapeChangedGrit;
  Deck::setShape(s, gritLatch, shapeChanged, shapeChangedGrit);

  if (shapeChangedGrit)
  {
    return;
  }
  else if (shapeChanged && !getGritMenuOpen())
  {
    shape_ = shapeControl_;
  }
}

void Sporadic::setPitch (float p, bool gritLatch)
{
  bool pitchChanged;
  bool pitchChangedGrit;
  Deck::setPitch(p, gritLatch, pitchChanged, pitchChangedGrit);

  if (pitchChangedGrit)
  {
    return;
  }
  // else... // TODO: map pitch to scarcity/abundance in Sporadic
  //{
  //   pitch_ = p;
  //}
}

void Sporadic::setSpoty (float s)
{
  s = infrasonic::unitclamp(s);
  bool spotyChanged = (std::abs(s - spotyControl_) > kParamChThreshold);
  spotyControl_ = spotyChanged ? s : spotyControl_;

  if (spotyChanged)
  {
    spoty_ = spotyControl_;
    setDelayNetworkParameters(inputSculptCenterFreq_, spoty_);
  }
}

void Sporadic::getBandFrequencies (std::vector<float> &frequencies) const
{
  delayNetwork_.getBandFrequencies(frequencies);
}

void Sporadic::setDelayNetworkParameters(float centerFreq, float stretch)
{
  delayNetwork_.setParameters({.numBands   = kMaxNutrientBands,
                               .numProcs   = kMaxNumDelayProcs,
                               .centerFreq = centerFreq,
                               .stretch    = stretch});
}

void Sporadic::updateAnalogControls(const AnalogControlFrame &c)
{
  // Update the analog deck parameters based on the control frame
  // Use grit modifiers (pad latch or grit menu) to route to InputSculpt
  setMix(c.mix);
  setPitch(c.pitch, c.pitchGrit);
  setPosition(c.position, c.positionGrit);
  setSize(c.size, c.sizeGrit);
  setShape(c.shape, c.shapeGrit);
  setSpoty(c.spoty);
}

void Sporadic::updateDigitalControls (const DigitalControlFrame &c)
{
  // Update the digital deck parameters based on the control frame
  setReverse(c.reverse);
  setPlay(c.play);
  setFlux(c.flux);
  setGrit(c.grit);

  Deck::updateDigitalControlsEffects(c);
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

void Sporadic::updateDisplayState ()
{
  DisplayState view{};
  // view.reverseActive = reverse_;
  // view.playActive    = play_;
  // view.altActive     = false; // not used

  // Check if there is an update to the held state of the effect pads
  detectEffectPadsHeld();

  if (isEffectPlaying())
  {
    // Flux/Grit pad LEDs and ring display
    updateEffectDisplayStates(view);

    // Overlay dark red LEDs indicating the Diffusion filter cutoff frequencies
    updateDiffusionRingState(view);
  }

  if (!isEffectDisplayed())
  {
    // Sporadic fold window visualization
    updateFoldWindowState(view);
  }

  // Publish the state of the display
  publishDisplay(view);

  // TODO: sporadic state display
}

using namespace spotykach_hwtest;
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
        inputSculpt_[ch].processBlockMono(in[ch], inputSculptBuf_[ch], blockSize);
      }
      else
      {
        // If the input sculpting is not active, copy the input to the scratch buffer
        std::copy(in[ch], in[ch] + blockSize, inputSculptBuf_[ch]);
      }

      // First, modulate input volume using EdgeTree per-sample
      edgeTree_[ch].processBlockMono(inputSculptBuf_[ch], modulatedInputBuf_[ch], blockSize);

      delayNetwork_.processBlockMono(modulatedInputBuf_[ch], ch, delayNetworkBuf_[ch], blockSize);

      // Apply dry-wet mix
      Utils::audioBlockLerp(inputSculptBuf_[ch], delayNetworkBuf_[ch], out[ch], mix_, blockSize);
    }
  }
}

void Sporadic::updateDiffusionRingState (DisplayState &view)
{
  constexpr uint8_t N = spotykach::Hardware::kNumLedsPerRing;
  Deck::RingSpan    ringSpan;
  LedRgbBrightness  ledColor[N];

  // Clear the LED color array
  std::fill(std::begin(ledColor), std::end(ledColor), LedRgbBrightness{0x000000, 0.0f});

  // Overlay dark red LEDs indicating the Diffusion filter cutoff frequencies
  std::vector<float> cutoffFreqs(kMaxNutrientBands);
  delayNetwork_.getBandFrequencies(cutoffFreqs);

  for (size_t i = 0; i < cutoffFreqs.size(); ++i)
  {
    if (view.layerCount == kMaxRingLayers)
    {
      break;
    }
    float freq = cutoffFreqs[i];
    if (freq > 0.0f)
    {
      uint8_t ledIdx   = freqToLed(freq, N, gritFilterMinFreq, gritFilterMaxFreq);
      ledColor[ledIdx] = {0xff0000, 0.5f};
      // Update the ring span information
      ringSpan.start = ledIdx;
      ringSpan.end   = ledIdx + 1;
      std::copy(ledColor, ledColor + N, std::begin(ringSpan.led));
      // Place the filter cutoff ring span into the view
      view.rings[view.layerCount++] = ringSpan;
    }
  }
}