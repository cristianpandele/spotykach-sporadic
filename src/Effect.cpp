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
  channelConfig_ = mode;
}

void Effect::setMode (EffectMode m)
{
  if (m < EffectMode::MODE_1 || m >= EffectMode::MODE_LAST)
  {
    Log::PrintLine("Invalid effect mode: %d", m);
    return;
  }
  // Set the effect mode
  mode_ = m;
}

bool Effect::getDisplayState (DisplayState &out) const
{
  uint8_t     r   = dispWIdx_;    // read the last published buffer index
  const auto &src = dispBuf_[r];
  if (src.cnt == cntRead_)    // unchanged since last fetch
  {
    return false;
  }
  out      = src.state;
  cntRead_ = src.cnt;
  return true;
}

void Effect::publishDisplay(const DisplayState &state)
{
  uint8_t w         = dispWIdx_ ^ 1;  // toggle write index
  dispBuf_[w].state = state;
  dispBuf_[w].cnt   = dispBuf_[dispWIdx_].cnt + 1; // increment generation count
  dispWIdx_         = w; // update write index
}

// Helper to determine if a channel should be processed in the current channel configuration
bool Effect::isChannelActive (size_t ch) const
{
  // Example logic: only process left channel in MONO_LEFT, right in MONO_RIGHT, both in STEREO
  switch (channelConfig_)
  {
    case ChannelConfig::MONO_LEFT:
    {
      return (ch == 0);
    }
    case ChannelConfig::MONO_RIGHT:
    {
      return (ch == 1);
    }
    case ChannelConfig::STEREO:
    {
      return ((ch == 0 || ch == 1));
    }
    default:
    {
      return false;
    }
  }
}