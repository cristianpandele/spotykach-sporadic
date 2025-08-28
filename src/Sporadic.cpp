#include "app.h"
#include "Sporadic.h"
#include <cmath>

void Sporadic::init ()
{
  // Initialize the input sculpt effect
  inputSculpt_.init(sampleRate_);
  // Initialize the delay network
  delayNetwork_.init(sampleRate_, blockSize_, kNumBands);
}

void Sporadic::setMix (float m, bool gritLatch)
{
  // If grit latched, set drive instead of mix
  if (gritLatch)
  {
    inputSculpt_.setOverdrive(m);
  }
  else if (!getGritMenuOpen())
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
  else if (!getGritMenuOpen())
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
  else if (!getGritMenuOpen())
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
  else if (!getGritMenuOpen())
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