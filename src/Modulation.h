#pragma once
#include "common.h"

// Base class for modulation engines
class ModulationEngine
{
  public:
    static constexpr size_t kNumModTypes = 3; // Number of modulation types
    enum ModType
    {
        ENV_FOLLOWER,
        S_H,
        SQUARE,
        SINE,
        SAW,
        MOD_TYPE_LAST
    };

    ModulationEngine() = default;

    virtual ~ModulationEngine() = default;

    virtual void setModType (ModType t) { currentModType = t; }

  protected:
    ModType modType[kNumModTypes] = {MOD_TYPE_LAST, MOD_TYPE_LAST, MOD_TYPE_LAST};
    ModType currentModType        = MOD_TYPE_LAST;    // Current modulation type
};

// Modulator for each side, interprets ModType differently
class Modulator : public ModulationEngine
{
  public:
    Modulator (const ModType *modTypes, size_t numModTypes);
    void setModType (ModType t) override;
};
