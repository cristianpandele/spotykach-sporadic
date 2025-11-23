#pragma once

#include "Utils.h"
#include <daisysp.h>
#include "common.h"
#include "constants.h"

using namespace spotykach;
using Mapping        = daisysp::Mapping;

enum ModTarget
{
  MIX,
  FLUX,
  GRIT,
  MOD_TARGET_LAST
};

// Modulation source and modulated parameter helpers moved into their own
// header so decks and AppImpl can share the concrete types

// ModulationSources: represents a combination of sources (external CV and soft/assignable)
// with per-source depth, polarity and mapping.
struct ModulationSources
{

  enum Polarity
  {
    UNIPOLAR = 1,
    BIPOLAR  = -1
  };

  enum ModSourceIndex
  {
    SOFT_A,        // Modulator source A
    SOFT_B,        // Modulator source B
    LONG_PRESS,    // Long press modulation (for effects)
    CV,            // External CV source
    MOD_SOURCE_LAST
  };

  // Level of assignable modulation source (unit 0..1 expected)
  float modLevel[MOD_SOURCE_LAST] = {0.0f, 0.0f, 0.0f, 0.0f};

  // Depth per-source
  float modDepth[MOD_SOURCE_LAST] = {1.0f, 1.0f, 1.0f, 1.0f};

  // Polarity per-source: 1 = passthrough, -1 = bipolar mapping (0..1 -> -1..1)
  Polarity modPolarity[MOD_SOURCE_LAST] = {UNIPOLAR, UNIPOLAR, UNIPOLAR, UNIPOLAR};

  // Mapping per-source (linear/log/exp)
  daisysp::Mapping modMapping[MOD_SOURCE_LAST] = {
    daisysp::Mapping::LINEAR, daisysp::Mapping::LINEAR, daisysp::Mapping::LINEAR, daisysp::Mapping::LINEAR};

  // Compute mapped value for CV or soft (applies polarity/bipolar conversion and depth)
  static float mapWithPolarity (float v, ModulationSources::Polarity polarity, daisysp::Mapping mapping)
  {
    v = infrasonic::unitclamp(v);
    v = daisysp::fmap(v, 0.0f, 1.0f, mapping);
    if (polarity == BIPOLAR)
    {
      // map 0..1 -> -1..1
      return infrasonic::map(v, 0.0f, 1.0f, -1.0f, 1.0f);
    }
    return v;
  }

  float evalMod(ModSourceIndex index) const
  {
    return mapWithPolarity(modLevel[index], modPolarity[index], modMapping[index]) * modDepth[index];
  }
};

using ModSourceIndex = ModulationSources::ModSourceIndex;
using Polarity       = ModulationSources::Polarity;

// ModulatedEffectParams: container for multiple modulated parameters (up to 4)
// Each parameter's base value is handled externally. All parameters share the same modulation sources
// (CV, SOFT_A, SOFT_B, LONG_PRESS).
// Per-parameter ParamConfigs (in the effect implementations) control how each parameter responds to modulation.
class ModulatedEffectParams
{
  public:
    static constexpr size_t kMaxParams = 4;

    enum ModEffectParamsIdx
    {
      PITCH_IX,
      POS_IX,
      SIZE_IX,
      SHAPE_IX
    };

    struct ParamConfig
    {
        float    depth    = 1.0f;
        Polarity polarity = ModulationSources::UNIPOLAR;
        Mapping  mapping  = Mapping::LINEAR;
    };

    ModulatedEffectParams() = default;

    // Set per-parameter modulation config (depth, polarity, mapping)
    void setParamConfig (size_t index, const ParamConfig &cfg)
    {
      if (index < kMaxParams)
      {
        paramConfigs_[index] = cfg;
      }
    }

    // Update the modulation source levels (call this once per block from AppImpl)
    void updateModulationSources (const ModulationSources &sources, size_t index)
    {
      if (index >= kMaxParams)
        return;

      modSources_[index] = sources;
    }

    // Get the modulation value for parameter at index
    // Returns the raw (unsmoothed) modulation value
    float getModValue (size_t ch, size_t index) const
    {
      if (index >= kMaxParams)
        return 0.0f;

      const ParamConfig &cfg             = paramConfigs_[index];
      float              modContribution = 0.0f;

      // Sum all modulation sources, scaled by this parameter's depth/polarity/mapping
      for (size_t src = 0; src < ModulationSources::MOD_SOURCE_LAST; ++src)
      {
        ModSourceIndex srcIdx = static_cast<ModSourceIndex>(src);
        // Use the modulation frame's source mapping/polarity/depth where provided
        const ModulationSources &msrc = modSources_[index];
        float srcVal                  = msrc.modLevel[srcIdx];
        Mapping srcMap                = msrc.modMapping[srcIdx];
        float  srcDepth               = msrc.modDepth[srcIdx];

        // Map the raw source value using the desired mapping. Force the mapping to unipolar to keep the value between 0 and 1 for now
        float mapped = ModulationSources::mapWithPolarity(srcVal, Polarity::UNIPOLAR, srcMap);

        // Combine source-level depth and per-parameter depth. This allows
        // frames to provide per-destination tuning while still respecting
        // the configured parameter sensitivity in paramConfigs_.
        float combinedDepth = srcDepth * cfg.depth;

        modContribution += mapped * combinedDepth;
      }

      // Apply modulation with opposite signs (per channel) modulation and clamp
      float sign         = (ch == 0) ? 1.0f : -1.0f;
      float effectiveVal = sign * modContribution;
      return infrasonic::unitclamp(effectiveVal);
    }

    // Get effective value, but apply a fixed smoothing to the result
    // (Create internal SmoothValues per-param if high-frequency updates are noisy)
    float getModSmoothValue (size_t ch, size_t index)
    {
      if (index >= kMaxParams)
        return 0.0f;

      float rawTarget = getModValue(ch, index);

      // Apply smoothing to the computed target value
      smoothers_[index] = rawTarget;
      return smoothers_[index].getSmoothVal();
    }

  private:
    ParamConfig                paramConfigs_[kMaxParams];
    ModulationSources          modSources_[kMaxParams];
    mutable Utils::SmoothValue smoothers_[kMaxParams] = {
      Utils::SmoothValue(75.0f, kSamplePeriodMs * kBlockSize),
      Utils::SmoothValue(75.0f, kSamplePeriodMs * kBlockSize),
      Utils::SmoothValue(75.0f, kSamplePeriodMs * kBlockSize),
      Utils::SmoothValue(75.0f, kSamplePeriodMs * kBlockSize)};
};

// Concrete modulated parameter entity.
class ModulatedParam
{
  public:
    // Construct with smoothing params for the effective value (ms, updatePeriodMs)
    ModulatedParam (float effectiveSmoothTimeMs = 75.0f, float effectiveUpdatePeriodMs = kSamplePeriodMs * kBlockSize)
      : base_(nullptr),
        effective_(0.0f, effectiveSmoothTimeMs, effectiveUpdatePeriodMs),
        effectiveSmoothTimeMs_(effectiveSmoothTimeMs),
        effectiveUpdatePeriodMs_(effectiveUpdatePeriodMs)
    {
    }

    enum ModTarget
    {
      MIX,
      FLUX,
      GRIT,
      MOD_TARGET_LAST
    };

    // Attach to an existing SmoothValue that acts as the base value
    void attachBase (Utils::SmoothValue *base)
    {
      base_ = base;
      if (base_)
      {
        // Initialize effective to the base's target value
        effective_.setCurrentValForce(base_->getTargetVal());
      }
    }

    // Get the smoothed effective value
    // Compute the effective smooth value = base value + mapped CV contribution + mapped soft contribution.
    // The resulting effective target is clamped to [0..1] and written to the effective SmoothValue.
    float getEffectiveSmoothVal ()
    {
      if (!base_)
        return effective_.getSmoothVal();

      // Always read fresh base value
      float baseVal = base_->getSmoothVal();

      float modContribution = 0.0f;
      // Map CV and soft sources (from 0...1 to either 0..1 or -1..1 depending on polarity, multiplied by depth)
      for (size_t i = 0; i < ModulationSources::MOD_SOURCE_LAST; i++)
      {
        modContribution += modSources_.evalMod(static_cast<ModSourceIndex>(i));
      }
      // Compute effective target
      float effectiveTarget = baseVal + modContribution;
      // Clamp to unit range
      effectiveTarget = infrasonic::unitclamp(effectiveTarget);

      // Smooth the computed target
      effective_ = effectiveTarget;

      return effective_.getSmoothVal();
    }

    // Helper function to add soft modulation to a modulated parameter
    void addCvModulation (float value, Mapping mapping = Mapping::LINEAR)
    {
      // Polarity defaults to unipolar.
      Polarity polarity = Polarity::UNIPOLAR;
      // CV modulation depth set to 1.0 (full)
      float depth = 1.0f;

      // Add modulation source and wire into modulated params.
      modSources_.modDepth[ModulationSources::CV]    = depth;
      modSources_.modPolarity[ModulationSources::CV] = polarity;
      modSources_.modLevel[ModulationSources::CV] =
        ModulationSources::mapWithPolarity(value, polarity, mapping) * depth;
    }

    // Helper function to add CV modulation to a modulated parameter
    void addSoftModulation (ModSourceIndex index,
                            float          value,
                            float          depth    = 1.0f,
                            Mapping        mapping  = Mapping::LINEAR,
                            Polarity       polarity = ModulationSources::UNIPOLAR)
    {
      if (index < ModulationSources::CV)
      {
        // Add modulation source and wire into modulated params.
        modSources_.modDepth[index]    = depth;
        modSources_.modPolarity[index] = polarity;
        modSources_.modLevel[index]    = ModulationSources::mapWithPolarity(value, polarity, mapping) * depth;
      }
    }

    // Expose isSmoothing on the effective value
    bool isSmoothing () const { return effective_.isSmoothing(); }

    // Latch flags
    bool altLatch  = false;
    bool gritLatch = false;
    bool fluxLatch = false;

  private:
    Utils::SmoothValue *base_ = nullptr;
    Utils::SmoothValue  effective_;
    ModulationSources   modSources_;

    // tuning for the effective smoothing (configurable per-instance)
    float effectiveSmoothTimeMs_;
    float effectiveUpdatePeriodMs_;
};
