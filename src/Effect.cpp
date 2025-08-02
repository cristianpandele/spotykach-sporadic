// #include "app.h"
#include "common.h"
#include "Effect.h"

using infrasonic::Log;

void Effect::setMode (EffectMode mode)
{
  if (mode < EffectMode::OFF || mode >= EffectMode::MODE_LAST)
  {
    Log::PrintLine("Invalid operating mode: %d", mode);
    return;
  }
  currentMode = mode;
}

void Effect::processAudio (AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
  // Process the effect audio
  // This is a placeholder for the actual effect logic, just copy the input to the output
  std::copy(IN_L, IN_L + size, OUT_L);
  std::copy(IN_R, IN_R + size, OUT_R);
}