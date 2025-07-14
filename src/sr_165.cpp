#include <algorithm>
#include "sr_165.h"

using namespace infrasonic;
using namespace daisy;

void ShiftRegister165::Init(const Config& config, size_t num_daisy_chained)
{
    // Initialize Pins
    data_.Init(config.data, GPIO::Mode::INPUT, GPIO::Pull::NOPULL);
    clk_.Init(config.clk, GPIO::Mode::OUTPUT);
    load_.Init(config.load, GPIO::Mode::OUTPUT);
    std::fill(state_, state_ + kMaxSr165DaisyChain, 0x00);
    num_devices_ = std::min(num_daisy_chained, kMaxSr165DaisyChain);

    clk_.Write(false);
    load_.Write(true);
}

void ShiftRegister165::Update()
{
    load_.Write(false);
    System::DelayTicks(1);
    load_.Write(true);
    System::DelayTicks(1);

    for(size_t i = 0; i < num_devices_ * 8; i++)
    {
        uint8_t didx = i / 8;
        uint8_t &val = state_[didx];
        clk_.Write(false);
        System::DelayTicks(1);
        val = ((val << 1) | (data_.Read() ? 1 : 0));
        clk_.Write(true);
        System::DelayTicks(1);
    }
}
