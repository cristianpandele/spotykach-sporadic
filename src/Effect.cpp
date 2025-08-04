// #include "app.h"
#include "common.h"
#include "Effect.h"

using infrasonic::Log;

void Effect::setChannelConfig (ChannelConfig mode)
{
  if (mode < ChannelConfig::OFF || mode >= ChannelConfig::CH_CONFIG_LAST)
  {
    Log::PrintLine("Invalid operating mode: %d", mode);
    return;
  }
  channelConfig = mode;
}

void Effect::setMode (EffectMode m)
{
  if (m < EffectMode::MODE_1 || m >= EffectMode::MODE_LAST)
  {
    Log::PrintLine("Invalid effect mode: %d", m);
    return;
  }
  // Set the effect mode
  mode = m;
}

void Effect::processAudio (AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
  // Process the effect audio
  // This is a placeholder for the actual effect logic, just copy the input to the output
  std::copy(IN_L, IN_L + size, OUT_L);
  std::copy(IN_R, IN_R + size, OUT_R);
}