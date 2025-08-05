#include "app.h"
#include "Sporadic.h"

void Sporadic::processAudio (AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size) override
{
  // Pass-through if channelConfig is not MONO_LEFT, MONO_RIGHT, or STEREO
  if (channelConfig == ChannelConfig::OFF || channelConfig >= ChannelConfig::CH_CONFIG_LAST)
  {
    std::copy(IN_L, IN_L + size, OUT_L);
    std::copy(IN_R, IN_R + size, OUT_R);
    return;
  }

  // Process the Sporadic effect audio
  // This is a placeholder for the actual Sporadic effect logic

}
