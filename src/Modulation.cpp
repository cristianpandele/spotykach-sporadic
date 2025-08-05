#include "Modulation.h"

Modulator::Modulator(const ModType * modTypes, size_t numModTypes)
{
  for (size_t i = 0; i < numModTypes && i < kNumModTypes; ++i)
  {
    modType[i] = modTypes[i];
  }
}

void Modulator::setModType (ModType t)
{
  for (size_t i = 0; i < kNumModTypes; ++i)
  {
    if (modType[i] == t)
    {
      // If in the list of modulation types, set it
      currentModType = t;
      break;
    }
  }
}
