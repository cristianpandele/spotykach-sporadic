#include "app.h"
#include "Sporadic.h"

void Sporadic::init ()
{
  // TODO
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

  // Process the Sporadic effect audio
  // This is a placeholder for the actual Sporadic effect logic
}

void Sporadic::updateAnalogControls(const AnalogControlFrame &c)
{
  // Update the analog effect parameters based on the control frame
  setMix(c.mix); //, c.mixAlt);
  setPitch(c.pitch);
  setPosition(c.position);
  setSize(c.size);
  setShape(c.shape);
}

void Sporadic::updateDigitalControls (const DigitalControlFrame &c)
{
  // Update the digital effect parameters based on the control frame
  setReverse(c.reverse);
  setPlay(c.play);

  // // Handle alternative play and spoty play states
  // if (c.altPlay)
  // {
  //   setAltPlay(true);    // Latch play state if altPlay is pressed
  // }
  // if (c.spotyPlay)
  // {
  //   setSpotyPlay(true);    // Handle spoty play state
  // }
}

void Sporadic::updateDisplayState ()
{
  // TODO
}

void Sporadic::getDigitalControls (DigitalControlFrame &c)
{
  c.reverse = reverse_;
  c.play = play_;
  c.altPlay = false;  // Not used in this effect
  c.spotyPlay = false;  // Not used in this effect
}