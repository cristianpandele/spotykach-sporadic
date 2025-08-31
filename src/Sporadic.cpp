#include "app.h"
#include "Sporadic.h"
#include <cmath>

void Sporadic::init ()
{
  for (size_t ch = 0; ch < kNumberChannelsStereo; ++ch)
  {
    // Initialize the input sculpt effect
    inputSculpt_[ch].init(sampleRate_);
  }
  // Initialize the delay network
  delayNetwork_.init(sampleRate_, blockSize_, kNumBands);

  // Initialize the edge tree for envelope following
  edgeTree_.init(sampleRate_);
}

//////////
// Handle parameter changes

static constexpr float paramChThreshold = 0.001f;
void Sporadic::setMix (float m, bool gritLatch)
{
  bool mixChanged                  = (std::abs(m - mix_) > paramChThreshold);
  bool mixChangedWhileGritMenuOpen = (mixChanged && getGritMenuOpen());

  if (gritLatch || mixChangedWhileGritMenuOpen)
  {
    // If grit latched, set drive instead of mix
    for (size_t ch = 0; ch < kNumberChannelsStereo; ++ch)
    {
      if (isChannelActive(ch))
      {
        inputSculpt_[ch].setOverdrive(m);
      }
    }
  }
  else if (!getGritMenuOpen())
  {
    mix_ = m;
  }
}

void Sporadic::setPosition (float p, bool gritLatch)
{
  bool positionChanged                  = (std::abs(p - position_) > paramChThreshold);
  bool positionChangedWhileGritMenuOpen = (positionChanged && getGritMenuOpen());

  if (gritLatch || positionChangedWhileGritMenuOpen)
  {
    // If grit latched, set input sculpt frequency instead of position
    for (size_t ch = 0; ch < kNumberChannelsStereo; ++ch)
    {
      if (isChannelActive(ch))
      {
        inputSculpt_[ch].setFreq(p);
      }
    }
  }
  else if (!getGritMenuOpen())
  {
    position_ = p;
  }
}

void Sporadic::setSize (float s, bool gritLatch)
{
  bool sizeChanged                  = (std::abs(s - size_) > paramChThreshold);
  bool sizeChangedWhileGritMenuOpen = (sizeChanged && getGritMenuOpen());

  if (gritLatch || sizeChangedWhileGritMenuOpen)
  {
    // If grit latched, set input sculpt width instead of size
    for (size_t ch = 0; ch < kNumberChannelsStereo; ++ch)
    {
      if (isChannelActive(ch))
      {
        inputSculpt_[ch].setWidth(s);
      }
    }
  }
  else if (!getGritMenuOpen())
  {
    size_ = s;
  }
}

void Sporadic::setShape (float s, bool gritLatch)
{
  bool shapeChanged                  = (std::abs(s - shape_) > paramChThreshold);
  bool shapeChangedWhileGritMenuOpen = (shapeChanged && getGritMenuOpen());

  if (gritLatch || shapeChangedWhileGritMenuOpen)
  {
    // If grit latched, set input sculpt shape instead of shape
    for (size_t ch = 0; ch < kNumberChannelsStereo; ++ch)
    {
      if (isChannelActive(ch))
      {
        inputSculpt_[ch].setShape(s);
      }
    }
  }
  else if (!getGritMenuOpen())
  {
    shape_ = s;
  }
}

#if DEBUG
void Sporadic::getBandFrequencies (std::vector<float> &frequencies) const
{
  delayNetwork_.getBandFrequencies(frequencies);
}
#endif

void Sporadic::updateAnalogControls(const AnalogControlFrame &c)
{
  // Update the analog deck parameters based on the control frame
  // Use grit modifiers (pad latch or grit menu) to route to InputSculpt
  setMix(c.mix, c.mixGrit);
  setPitch(c.pitch);
  setPosition(c.position, c.positionGrit);
  setSize(c.size, c.sizeGrit);
  setShape(c.shape, c.shapeGrit);
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

void Sporadic::updateDisplayState ()
{
  DisplayState view{};
  // view.reverseActive = reverse_;
  // view.playActive    = play_;
  // view.altActive     = false; // not used

  // Check if there is an update to the held state of the effect pads
  detectEffectPadsHeld();

  // Flux/Grit pad LEDs and ring display
  updateEffectDisplayStates(view);

  // Publish the state of the display
  publishDisplay(view);

  // TODO: sporadic state display
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
        inputSculpt_[ch].processBlockMono(in[ch], inputSculptBuf_[ch], blockSize);
      }
      else
      {
        // If the input sculpting is not active, copy the input to the scratch buffer
        std::copy(in[ch], in[ch] + blockSize, inputSculptBuf_[ch]);
      }

      // First, modulate input volume using EdgeTree per-sample
      edgeTree_.processBlockMono(inputSculptBuf_[ch], modulatedInputBuf_[ch], blockSize);

      delayNetwork_.processBlockMono(modulatedInputBuf_[ch], ch, delayNetworkBuf_[ch], blockSize);

      // Apply dry-wet mix
      Utils::audioBlockLerp(modulatedInputBuf_[ch], delayNetworkBuf_[ch], out[ch], mix_, blockSize);
    }
  }
}