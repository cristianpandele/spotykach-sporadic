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
    setDelayNetworkParameters(play_, reverse_, inputSculptCenterFreq_, spoty_);
  }
  else if (positionChanged && !getGritMenuOpen())
  {
    position_ = positionControl_;
    // Set fold window dirty flag to update visualization
    foldWindowDirty_ = true;
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
    // Set fold window dirty flag to update visualization
    foldWindowDirty_ = true;
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
    // Set fold window dirty flag to update visualization
    foldWindowDirty_ = true;
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
  if (pitchChanged && !getGritMenuOpen())
  {
    // Set the tree density in the delay network
    delayNetwork_.setTreeDensity(pitchControl_);

    // Set the tree size of the edge trees
    float edgeTreeSize = 0.0f;
    if (pitchControl_ < 0.5f)
    {
      edgeTreeSize = infrasonic::map(pitchControl_, 0.0f, 0.5f, 1.0f, 0.4f);
    }
    else
    {
      edgeTreeSize = infrasonic::map(pitchControl_, 0.5f, 1.0f, 0.4f, 1.0f);
    }
    for (auto &et : edgeTree_)
    {
      et.setTreeSize(edgeTreeSize);
    }
    // Set fold window dirty flag to update visualization
    foldWindowDirty_ = true;
  }
}

void Sporadic::setPlay (bool p)
{
  bool playChanged = (p != play_);
  play_ = p;

  if (playChanged)
  {
    setDelayNetworkParameters(play_, reverse_, inputSculptCenterFreq_, spoty_);
  }
  foldWindowDirty_ = true;
}

void Sporadic::setSpotyPlay (bool s)
{
  if (s)
  {
    init();
  }
}

void Sporadic::setReverse (bool r)
{
  bool reverseChanged = (r != reverse_);
  reverse_            = r;

  if (reverseChanged)
  {
    setDelayNetworkParameters(play_, reverse_, inputSculptCenterFreq_, spoty_);
  }
}

void Sporadic::setSpoty (float s)
{
  s = infrasonic::unitclamp(s);
  bool spotyChanged = (std::abs(s - spotyControl_) > kParamChThreshold);
  spotyControl_ = spotyChanged ? s : spotyControl_;

  if (spotyChanged)
  {
    spoty_ = spotyControl_;
    setDelayNetworkParameters(play_, reverse_, inputSculptCenterFreq_, spoty_);
  }
}

void Sporadic::getBandFrequencies (std::vector<float> &frequencies) const
{
  delayNetwork_.getBandFrequencies(frequencies);
}

#ifdef DEBUG
void Sporadic::getNodeInterconnectionMatrix (std::vector<std::vector<float>> &matrix) const
{
  delayNetwork_.getNodeInterconnectionMatrix(matrix);
}

void Sporadic::getTreePositions (std::vector<float> &positions) const
{
  delayNetwork_.getTreePositions(positions);
}
#endif

void Sporadic::setDelayNetworkParameters(bool play, bool reverse, float centerFreq, float stretch)
{
  delayNetwork_.setParameters({.play       = play,
                               .reverse    = reverse,
                               .numBands   = kMaxNutrientBands,
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
  setSpotyPlay(c.spotyPlay);
  if (c.spotyPlay)
  {
    setReverse(false);
    setPlay(false);
    return;
  }

  // Update the digital deck parameters based on the control frame
  setReverse(c.reverse);
  setPlay(c.play);
  setFlux(c.flux);
  setGrit(c.grit);

  Deck::updateDigitalControlsEffects(c);
}

void Sporadic::getDigitalControls (DigitalControlFrame &c)
{
  c.reverse   = reverse_;
  c.play      = play_;
  c.altPlay   = false;    // Spotykach+Play is just a toggle
  c.spotyPlay = false;
  c.flux      = flux_;
  c.altFlux   = false;    // Alt+Flux is just a toggle
  c.grit      = grit_;
  c.altGrit   = false;    // Alt+Grit is just a toggle
}

void Sporadic::updateDisplayState ()
{
  DisplayState view{};
  view.reverseActive = reverse_;
  view.playActive    = play_;

  // Check if there is an update to the held state of the effect pads
  detectEffectPadsHeld();

  if (isEffectPlaying())
  {
    // Flux/Grit pad LEDs and ring display
    updateEffectDisplayStates(view);

    if (isGritDisplayed())
    {
      // Overlay dark red LEDs indicating the Diffusion filter cutoff frequencies
      updateDiffusionRingState(view);
    }
  }

  if (!isEffectDisplayed())
  {
    if (foldWindowDirty_)
    {
      resetDisplayRingLayers(foldView_);
      // Sporadic fold window visualization
      updateFoldWindowState(foldView_);

      // Provide the fold window to DelayNetwork for internal gain staging
      delayNetwork_.setFoldWindow(envelopeRing_, spotykach::Hardware::kNumLedsPerRing);

      // Overlay dark red LEDs indicating the tree positions
      updateTreeRingState(foldView_);

      // Update the envelope ring if the fold window changed
      foldWindowDirty_ = false;
    }

    // Copy the fold view rings to the display view
    std::copy(foldView_.rings.begin(), foldView_.rings.end(), view.rings.begin());
    view.layerCount = foldView_.layerCount;
  }

  // Play LED colors
  view.playLedColors.fill({0x000000, kOffLedBrightness});
  if (play_)
  {
    view.playLedColors[0] = {0x00ff00, kMaxLedBrightness};    // Green
  }
  else
  {
    view.playLedColors[0] = {0x000000, kOffLedBrightness};    // Off
  }

  // Reverse pad LED handling
  if (reverse_)
  {
    std::fill(std::begin(view.reverseLedColors),
              std::end(view.reverseLedColors),
              LedRgbBrightness{0x0000ff, kMaxLedBrightness});
  }

  // Publish the state of the display
  publishDisplay(view);
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

      if (pitchControl_ < 0.5f)
      {
        // Skip the edge tree processing if the pitch is in the lower half
        std::copy(inputSculptBuf_[ch], inputSculptBuf_[ch] + blockSize, modulatedInputBuf_[ch]);
      }
      else
      {
        // First, modulate input volume using EdgeTree per-sample
        edgeTree_[ch].processBlockMono(inputSculptBuf_[ch], modulatedInputBuf_[ch], blockSize);
      }

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
  std::fill(std::begin(ledColor), std::end(ledColor), LedRgbBrightness{0x000000, kOffLedBrightness});

  // Overlay dark red LEDs indicating the Diffusion filter cutoff frequencies
  std::vector<float> cutoffFreqs(kMaxNutrientBands);
  delayNetwork_.getBandFrequencies(cutoffFreqs);

  if (cutoffFreqs.empty())
  {
    return;
  }

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
      ledColor[ledIdx] = {0xff0000, kLowLedBrightness};    // Dark Red
      // Update the ring span information
      ringSpan.start = ledIdx;
      ringSpan.end   = ledIdx + 1;
      std::copy(ledColor, ledColor + N, std::begin(ringSpan.led));
      // Place the filter cutoff ring span into the view
      view.rings[view.layerCount++] = ringSpan;
    }
  }
}

void Sporadic::updateTreeRingState (DisplayState &view)
{
  // Only display the canvas when not playing
  if (!play_)
  {
    return;
  }

  constexpr uint8_t N = spotykach::Hardware::kNumLedsPerRing;
  Deck::RingSpan    ringSpan;
  LedRgbBrightness  ledColor[N];

  // Clear the LED color array
  std::fill(std::begin(ledColor), std::end(ledColor), LedRgbBrightness{0x000000, kOffLedBrightness});

  std::vector<float> treePos;
  delayNetwork_.getTreePositions(treePos);

  if (treePos.empty())
  {
    return;
  }

  // Overlay dark red LEDs indicating the tree positions
  for (size_t i = 0; i < treePos.size(); ++i)
  {
    if (view.layerCount == kMaxRingLayers)
    {
      break;
    }

    // Linear mapping: 0..1 -> 0.07..0.89*N
    float t         = treePos[i];
    t               = daisysp::fmap(t, 0.07f * static_cast<float>(N), 0.89f * static_cast<float>(N));
    uint8_t ledIdx  = std::round(t);
    uint8_t channel = channelConfig_ == isChannelActive(1) ? 1 : 0;
    // Use the envelope level for brightness
    float brightness = edgeTree_[channel].getEnvelope();
    brightness       = daisysp::fmap(brightness, kLowLedBrightness, kMidLedBrightness, Mapping::LOG);
    ledColor[ledIdx] = {0xff0000, brightness};    // Dark Red
    // Update the ring span information
    ringSpan.start = ledIdx;
    ringSpan.end   = ledIdx + 1;
    std::copy(std::begin(ledColor), std::end(ledColor), std::begin(ringSpan.led));
    // Place the tree position ring span into the view
    view.rings[view.layerCount++] = ringSpan;
  }
}

void Sporadic::updateFoldWindowState(DisplayState &view)
{
  constexpr uint8_t N = spotykach::Hardware::kNumLedsPerRing;
  Deck::RingSpan    ringSpan;

  // Yellow area for the canvas
  uint8_t          start    = 0;
  LedRgbBrightness ledColor = {0xffff00, kMaxLedBrightness};
  populateLedRing(ringSpan, N, ledColor, start, N);
  view.rings[view.layerCount++] = ringSpan;

  // Green area for the fold window if playing
  if (play_)
  {
    constexpr uint8_t kMinWinLen = 5;
    // Fold window start in LED slots (at at latest N - kMinWinLen)
    start            = std::round(daisysp::fmap(position_, 0.0f, N - kMinWinLen));
    // Fold window length in LED slots (at least kMinWinLen)
    uint8_t winLen   = std::round(daisysp::fmap(size_, kMinWinLen, N + 1));
    winLen           = daisysp::fclamp(winLen, 0, N - start);
    ledColor         = {0x00ff00, kMaxLedBrightness};    // Green

    // Populate the LED Ring
    populateLedRing(ringSpan, N, ledColor, start, winLen, true);
    view.rings[view.layerCount++] = ringSpan;
  }

  // Copy the brightness levels to the envelopeRing array for use in mixing
  for (size_t i = 0; i < N; ++i)
  {
    envelopeRing_[i] = ringSpan.led[i].brightness;
  }
}