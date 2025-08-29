#pragma once

#include "common.h"
#include "daisy_seed.h"
#include "sr_165.h"
#include "ws2812.h"

namespace spotykach
{
    // Constants for the application
    constexpr size_t kSampleRate     = 48000;
    constexpr float  kSamplePeriodMs = 1000.0f / kSampleRate;
    constexpr size_t kBlockSize      = 48;
};    // namespace spotykach
