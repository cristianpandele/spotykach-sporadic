#include "hardware.h"

using namespace spotykach;
using namespace daisy;

namespace spotykach
{
  static constexpr Pin kLEDDataPin                      = seed::D17;

  static constexpr Pin kClockInputPin                   = seed::D3;
  static constexpr Pin kClockOutputPin                  = seed::D15;
  static constexpr Pin kBootButtonPin                   = seed::D2;
  static constexpr Pin kTapButtonPin                    = seed::D32;

  static constexpr Pin kSW1APin                         = seed::D16;
  static constexpr Pin kSW1BPin                         = seed::D31;
  static constexpr Pin kSW2APin                         = seed::D0;
  static constexpr Pin kSW2BPin                         = seed::D7;

  static constexpr Pin kI2CSdaPin                       = seed::D12;
  static constexpr Pin kI2CSclPin                       = seed::D11;

  static constexpr Pin kMuxSignalPin                    = seed::A10; // Pots 1-8, multiplexed by addr pins
  static constexpr Pin kMuxAddrAPin                     = seed::D8;
  static constexpr Pin kMuxAddrBPin                     = seed::D9;
  static constexpr Pin kMuxAddrCPin                     = seed::D10;

  static constexpr Pin kPot9Pin                         = seed::A8;
  static constexpr Pin kPot10Pin                        = seed::A3;

  static constexpr Pin kMidiUartRxPin                   = seed::D14;
  static constexpr Pin kMidiUartTxPin                   = seed::D13;

  static constexpr size_t kNumAdcChannels               = 3;    // 2 Pots + 1 Mux

}    // namespace spotykach

void Hardware::Init (float sr, size_t blocksize)
{
  const float kProcessRate = sr / blocksize;

  seed.Init(true);

  // --- Buttons ---
  boot_btn_.Init(kBootButtonPin, 0, Switch::TYPE_MOMENTARY, Switch::POLARITY_INVERTED, GPIO::Pull::NOPULL);
  tap_btn_.Init(kTapButtonPin, 0, Switch::TYPE_MOMENTARY, Switch::POLARITY_INVERTED, GPIO::Pull::NOPULL);

  // --- Switches ---
  direction_switch_[0].Init(kSW1APin, kSW1BPin);
  direction_switch_[1].Init(kSW2APin, kSW2BPin);

  // --- LEDs ---
  infrasonic::Ws2812::Config led_cfg;
  led_cfg.num_leds    = LED_LAST;
  led_cfg.tim_pin     = kLEDDataPin;
  led_cfg.tim_periph  = TimerHandle::Config::Peripheral::TIM_3;
  led_cfg.tim_channel = infrasonic::Ws2812::Config::CH4;
  leds.Init(led_cfg);
  // leds.SetBrightnessLimit(0.7f);

  // --- GPIO - gate/clk/etc ---

  // --- GPIO - clks ---
  GPIO::Config gpio_cfg;
  gpio_cfg.pull = GPIO::Pull::NOPULL;

  gpio_cfg.pin  = kClockInputPin;
  clock_in_.Init(gpio_cfg);

  gpio_cfg.pin  = kClockOutputPin;
  gpio_cfg.mode = GPIO::Mode::OUTPUT;
  clock_out_.Init(gpio_cfg);

  // --- MPR121 (I2C) ---

  // Default device address is fine
  Mpr121I2C::Config mpr_cfg;
  mpr_cfg.transport_config.periph = I2CHandle::Config::Peripheral::I2C_1;
  mpr_cfg.transport_config.mode   = I2CHandle::Config::Mode::I2C_MASTER;
  mpr_cfg.transport_config.scl    = kI2CSclPin;
  mpr_cfg.transport_config.sda    = kI2CSdaPin;
  mpr_cfg.transport_config.speed  = I2CHandle::Config::Speed::I2C_400KHZ;
  mpr121_.Init(mpr_cfg);

  // --- Init ADCs ---
  // (normally I'd write loopable config structs for this but
  //  this is quick and dirty code)

  // Speed and oversampling can usually be reduced from defaults
  // to increase effective ADC sample rate with no major downsides,
  // but possibly more jitter
  const auto kAdcSpeed = AdcChannelConfig::SPEED_2CYCLES_5;
  const auto kAdcOvs   = AdcHandle::OverSampling::OVS_32;

  AdcChannelConfig adc_cfg[kNumAdcChannels];
  adc_cfg[0].InitMux(kMuxSignalPin, 8, kMuxAddrAPin, kMuxAddrBPin, kMuxAddrCPin, kAdcSpeed);
  adc_cfg[1].InitSingle(kPot9Pin, kAdcSpeed);
  adc_cfg[2].InitSingle(kPot10Pin, kAdcSpeed);

  seed.adc.Init(adc_cfg, kNumAdcChannels, kAdcOvs);

  // --- Analog Controls ---
  constexpr float kPotSmoothTime = 0.02f;

  controls_[CTRL_MOD_AMT_A].Init(seed.adc.GetMuxPtr(0, 0), kProcessRate, false, false, kPotSmoothTime);
  controls_[CTRL_MOD_FREQ_A].Init(seed.adc.GetMuxPtr(0, 1), kProcessRate, false, false, kPotSmoothTime);
  controls_[CTRL_SOS_A].Init(seed.adc.GetMuxPtr(0, 2), kProcessRate, false, false, kPotSmoothTime);
  controls_[CTRL_PITCH_A ].Init(seed.adc.GetMuxPtr(0, 3), kProcessRate, false, false, kPotSmoothTime);
  controls_[CTRL_POS_B].Init(seed.adc.GetMuxPtr(0, 4), kProcessRate, false, false, kPotSmoothTime);
  controls_[CTRL_POS_A].Init(seed.adc.GetMuxPtr(0, 5), kProcessRate, false, false, kPotSmoothTime);
  controls_[CTRL_SIZE_A].Init(seed.adc.GetMuxPtr(0, 6), kProcessRate, false, false, kPotSmoothTime);
  controls_[CTRL_PITCH_B].Init(seed.adc.GetMuxPtr(0, 7), kProcessRate, false, false, kPotSmoothTime);
  controls_[CTRL_SIZE_B].Init(seed.adc.GetPtr(1), kProcessRate, false, false, kPotSmoothTime);
  controls_[CTRL_SPOTYKACH].Init(seed.adc.GetPtr(2), kProcessRate, false, false, kPotSmoothTime);

  // --- UART MIDI ---
  MidiUartHandler::Config midi_cfg;
  midi_cfg.transport_config.periph = UartHandler::Config::Peripheral::USART_1;
  midi_cfg.transport_config.rx     = kMidiUartRxPin;
  midi_cfg.transport_config.tx     = kMidiUartTxPin;
  midi_uart.Init(midi_cfg);
  midi_uart.StartReceive();

  // -- DAC --
  // Setup for polling write currently-
  // Recommend using DMA for production
  DacHandle::Config config;
  config.chn        = DacHandle::Channel::BOTH;
  config.bitdepth   = DacHandle::BitDepth::BITS_12;
  config.mode       = DacHandle::Mode::POLLING;
  config.buff_state = DacHandle::BufferState::DISABLED;
  seed.dac.Init(config);
}

void Hardware::StartAdcs ()
{
  seed.adc.Start();
}

void Hardware::ProcessAnalogControls ()
{
  for (auto &control : controls_)
  {
    control.Process();
  }
}

void Hardware::ProcessDigitalControls ()
{
  boot_btn_.Debounce();
  tap_btn_.Debounce();

  for (uint8_t side = 0; side < kNumberDeckSlots; side++)
  {
    switchState_[side] = direction_switch_[side].Read();
  }
}

float Hardware::GetAnalogControlValue (AnalogControlId id)
{
  // inset scaling for full range
  if (id >= CTRL_LAST)
    return 0.0f;
  float val = infrasonic::map(controls_[id].Value(), 0.05f, 0.93f, 0.0f, 1.0f);
  return infrasonic::unitclamp(val);
}

// These are all inverted due to transistors
bool Hardware::GetClockInputState ()
{
  return !clock_in_.Read();
}

bool Hardware::GetTapButtonState ()
{
  return tap_btn_.Pressed();
}

uint8_t Hardware::GetDirectionSwitchState (uint8_t side)
{
  if (side >= kNumberDeckSlots)
    return 0;
  return switchState_[side];
}

uint32_t Hardware::GetBootButtonHeldTime () const
{
  return boot_btn_.TimeHeldMs();
}

uint32_t Hardware::GetBootButtonReleased () const
{
  return boot_btn_.FallingEdge();
}