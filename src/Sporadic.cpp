#include "app.h"
#include "Sporadic.h"

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
