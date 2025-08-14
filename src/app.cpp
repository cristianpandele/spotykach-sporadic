#include "app.h"
#include "Effect.h"
#include "Sporadic.h"
#include "Spotykach.h"
#include "color.h"
#include "common.h"
#include "daisysp.h"
#include "hardware.h"
#include <bitset>
#include <cstring>
#include <daisy_seed.h>

using namespace daisy;
using namespace daisysp;
using namespace infrasonic;
using infrasonic::Log;
using spotykach::Hardware;

namespace spotykach_hwtest
{
  // SD Card test stuff
  #define TEST_FILE_NAME "test_data"
  SdmmcHandler   sd;
  FatFSInterface fsi;
  FIL            SDFile;

  // Simulate a one-minute 16bit stereo audio file at 48khz (about 11.5 MB)
  constexpr size_t             kAudioDataBytes = 60 * kSampleRate * 2 * sizeof(int16_t);
  static DSY_SDRAM_BSS uint8_t audioData[kAudioDataBytes];
}    // namespace spotykach_hwtest

static AppImpl   impl;
static Spotykach spotykachLooper[kNumberEffectSlots] = {Spotykach(0), Spotykach(1)};
static Sporadic  sporadic;

// Array of pointers to Effects
Effect* effects[kNumberEffectSlots];

// Control frame for the effects
Effect::AnalogControlFrame analogControlFrames[kNumberEffectSlots];
Effect::DigitalControlFrame digitalControlFrames[kNumberEffectSlots];
// Display state for the effects
Effect::DisplayState displayStates[kNumberEffectSlots];

#if DEBUG
CpuLoadMeter loadMeter;
#endif

#define REBOOT_TO_BOOTLOADER_CMD "reboot"
#define USB_BUFFER_SIZE          256

char    usbBuff[USB_BUFFER_SIZE];
uint8_t usbBuffIx = 0;

static void UsbCallback (uint8_t *buf, uint32_t *len)
{
  if (*len > USB_BUFFER_SIZE - usbBuffIx)
  {
    Log::PrintLine("USB Callback: Buffer overflow, dropping %d bytes and resetting buffer", *len);
    usbBuffIx = 0;
    return;
  }
  memcpy(usbBuff + usbBuffIx, buf, *len);
  usbBuffIx += *len;
}

static void AudioCallback (AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t blockSize)
{
#if DEBUG
  loadMeter.OnBlockStart();
#endif
  impl.processAudio(in, out, blockSize);
#if DEBUG
  loadMeter.OnBlockEnd();
#endif
}

void AppImpl::init ()
{
  hw.Init(kSampleRate, kBlockSize);
  hw.StartAdcs();

  led_timer.Init();
  midi_timer.Init();

  test_note_on = false;

  pot_monitor.Init(ui_queue, hw, 500, 0.005f, 0.002f);
  std::fill(std::begin(last_pot_moved), std::end(last_pot_moved), 0);

  for (size_t i = 0; i < 8; i++)
  {
    osc[i].Init(kSampleRate);
    osc[i].SetWaveform(daisysp::Oscillator::WAVE_POLYBLEP_TRI);
    osc[i].SetAmp(0.25f);
  }

  // Uncomment this if you want to test the SD card -
  // this takes about 10s and is blocking
  // testSDCard();

  hw.seed.usb_handle.SetReceiveCallback(UsbCallback, UsbHandle::FS_EXTERNAL);

  auto &audio = hw.seed.audio_handle;
  audio.SetSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
  audio.SetBlockSize(kBlockSize);

  // Initialize the Spotykach looper
  for (size_t i = 0; i < kNumberEffectSlots; i++)
  {
    spotykachLooper[i].init();
  }
  sporadic.init();

#if DEBUG
  Log::StartLog(false);
  log_timer.Init();
  loadMeter.Init(hw.seed.AudioSampleRate(), hw.seed.AudioBlockSize());
#endif

  audio.Start(AudioCallback);
}

using ChannelConfig = Effect::ChannelConfig;

void AppImpl::setRoutingMode (AppImpl::AppMode mode)
{
  // Pass the mode to the Spotykach looper and Sporadic effect
  if (currentRoutingMode == AppMode::ROUTING_GENERATIVE)
  {
    spotykachLooper[0].setChannelConfig(ChannelConfig::STEREO);
    spotykachLooper[1].setChannelConfig(ChannelConfig::STEREO);
    sporadic.setChannelConfig(ChannelConfig::STEREO);
    effects[0] = &spotykachLooper[0];
    effects[1] = &spotykachLooper[1];
  }
  else if (currentRoutingMode == AppMode::ROUTING_DUAL_MONO)
  {
    spotykachLooper[0].setChannelConfig(ChannelConfig::MONO_LEFT);
    spotykachLooper[1].setChannelConfig(ChannelConfig::OFF);
    sporadic.setChannelConfig(ChannelConfig::MONO_RIGHT);
    effects[0] = &spotykachLooper[0];
    effects[1] = &sporadic;
  }
  else if (currentRoutingMode == AppMode::ROUTING_DUAL_STEREO)
  {
    spotykachLooper[0].setChannelConfig(ChannelConfig::STEREO);
    spotykachLooper[1].setChannelConfig(ChannelConfig::OFF);
    sporadic.setChannelConfig(ChannelConfig::STEREO);
    effects[0] = &spotykachLooper[0];
    effects[1] = &sporadic;
  }
}

void AppImpl::updateAnalogControlFrame(Effect::AnalogControlFrame &frame, size_t slot)
{
  if (slot >= kNumberEffectSlots)
  {
    return; // Invalid slot
  }

  // Update the control frame for the specified effect slot
  frame =
  {
    .mix = mixControls[slot].isSmoothing() ? mixControls[slot].getSmoothVal() : mixControls[slot].getTargetVal(),
    .mixAlt   = mixAltLatch[slot],
    .pitch    = pitchControls[slot].isSmoothing() ? pitchControls[slot].getSmoothVal()
                                                  : pitchControls[slot].getTargetVal(),
    .position = positionControls[slot].isSmoothing() ? positionControls[slot].getSmoothVal()
                                                     : positionControls[slot].getTargetVal(),
    .size =
      sizeControls[slot].isSmoothing() ? sizeControls[slot].getSmoothVal() : sizeControls[slot].getTargetVal(),
    .shape = shapeControls[slot].isSmoothing() ? shapeControls[slot].getSmoothVal()
                                               : shapeControls[slot].getTargetVal()
  };
}

void AppImpl::pushAnalogEffectControls(Effect::AnalogControlFrame &c, size_t slot)
{
  // Push the controls to the Spotykach looper and Sporadic effect
  effects[slot]->updateAnalogControls(c);
}

void AppImpl::updateDigitalControlFrame(Effect::DigitalControlFrame &frame, size_t slot)
{
  if (slot >= kNumberEffectSlots)
  {
    return; // Invalid slot
  }

  // Update the control frame for the specified effect slot
  frame = {
    // Simple pad presses
    .reverse = currentReverseState[slot],
    .play    = currentPlayState[slot],
    // Supported pad combinations
    .altPlay   = currentAltPlayState[slot],
    .spotyPlay = currentSpotyPlayState[slot]
  };
}

void AppImpl::pushDigitalEffectControls(Effect::DigitalControlFrame &c, size_t slot)
{
  // Push the controls to the Spotykach looper and Sporadic effect
  effects[slot]->updateDigitalControls(c);
  // Get back the updated controls
  effects[slot]->getDigitalControls(c);
  currentReverseState[slot]   = c.reverse;
  currentPlayState[slot]      = c.play;
  currentAltPlayState[slot]   = c.altPlay;
  currentSpotyPlayState[slot] = c.spotyPlay;
}

void AppImpl::loop ()
{
  while (true)
  {
    // If boot button held for 3s, reset into bootloader mode for update.
    // Otherwise, restart the app.
    if (hw.GetBootButtonHeldTime() >= 3000)
    {
      System::ResetToBootloader(System::BootloaderMode::DAISY_INFINITE_TIMEOUT);
    }
    else
    {
      if (hw.GetBootButtonReleased())
      {
        System::ResetToBootloader(System::BootloaderMode::DAISY_SKIP_TIMEOUT);
      }
    }

    hw.ProcessDigitalControls();
    pot_monitor.Process();
    processUIQueue();
    processMidi();

    // Every 500ms toggle MIDI note out and gate outs
    if (midi_timer.HasPassedMs(500))
    {
      midi_timer.Restart();
      test_note_on = !test_note_on;
      if (test_note_on)
      {
        hw.midi_uart.EnqueueMessage(MidiTxMessage::NoteOn(0, 60, 127));
      }
      else
      {
        hw.midi_uart.EnqueueMessage(MidiTxMessage::NoteOff(0, 60, 64));
      }
      for (uint8_t i = 0; i < kNumberEffectSlots; i++)
      {
        hw.SetGateOut(i, test_note_on);
      }
    }

    // Modified libDaisy MIDI handlers require explicit call to transmit
    // enqueued messages instead of blocking every time a message is sent
    hw.midi_uart.TransmitEnqueuedMessages();

    // The LED refresh should run at least 200Hz for temporal dithering,
    // but faster is better
    if (led_timer.HasPassedMs(2))
    {
      led_timer.Restart();

      // Controller part of MVC
      handleDigitalControls();

      // Apply changes based on the controls readout
      if (routingModeChanged)
      {
        setRoutingMode(currentRoutingMode);
        routingModeChanged = 0;
      }

      for (size_t i = 0; i < kNumberEffectSlots; i++)
      {
        if (modTypeChanged[i])
        {
          modulator[i].setModType(currentModType[i]);
          modTypeChanged[i] = false;
        }

        if (effectModeChanged[i])
        {
          effects[i]->setMode(currentEffectMode[i]);
          effectModeChanged[i] = false;
        }

        if (reverseStateChanged[i] || playStateChanged[i] || altPlayStateChanged[i] || spotyPlayStateChanged[i])
        {
          updateDigitalControlFrame(digitalControlFrames[i], i);
          pushDigitalEffectControls(digitalControlFrames[i], i);
        }

        /////////
        // Piggy back on this timer (500 Hz, roughly /100 times slower than the sample rate) for very rough CV output demo
        modCv[i] = modulator[i].process();
      }

      // View part of MVC
      hw.leds.Clear();
      // drawRainbowRoad();
      handleDisplay();
      hw.leds.Show();

      // Piggy back on this timer for very rough CV output demo
      for (size_t i = 0; i < kNumberEffectSlots; i++)
      {
        hw.WriteCVOut(i, modCv[i]);
      }
    }
    if (log_timer.HasPassedMs(kDebugLogPeriodMs))
    {
      padLedPhase += 1;
      if (padLedPhase >= Effect::kMaxLedPhases)
      {
        padLedPhase = 0;
      }
#if DEBUG
      logDebugInfo();
#endif
      log_timer.Restart();
    }
  }
}

void AppImpl::processModulatorControls (size_t slot)
{
  if (slot >= kNumberEffectSlots)
  {
    return; // Invalid slot
  }

  // Process the modulation for the specified effect slot
  for (size_t i = 0; i < kNumberEffectSlots; i++)
  {
    // We are rendering the modulator output at 500 Hz, roughly /100 times slower than the sample rate
    // so we need to scale the frequency by 100.0f
    modulator[i].setFrequency(modulationFreq[i].getSmoothVal() * 100.0f, modFreqAltLatch[i]);
    modulator[i].setAmplitude(modulationAmount[i].getSmoothVal());
  }
}

void AppImpl::processAudioLogic (AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t blockSize)
{
  for (size_t i = 0; i < kNumberEffectSlots; i++)
  {
    effects[i]->processAudio(in, out, blockSize);
  }
}

void AppImpl::processAudio (AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t blockSize)
{
  hw.ProcessAnalogControls();

#if 0
  std::copy(IN_L, IN_L + blockSize, OUT_L);
  std::copy(IN_R, IN_R + blockSize, OUT_R);


  // Add test oscillator from MIDI input
  float s;
  for (size_t i = 0; i < blockSize; i++)
  {
    s = 0.0f;
    for (size_t j = 0; j < 8; j++)
    {
      s += osc[j].Process();
    }
    s *= (midi_in_note_on ? 1.0f : 0.0f);
    OUT_L[i] += s;
    OUT_R[i] += s;
  }
#else
  // Handle the analog controls that affect the audio processing
  handleAnalogControls();

  for (size_t i = 0; i < kNumberEffectSlots; i++)
  {
    /////////
    // Apply the analog controls to the effects

    if (mixControls[i].isSmoothing() || pitchControls[i].isSmoothing() ||
        positionControls[i].isSmoothing() || sizeControls[i].isSmoothing() ||
        shapeControls[i].isSmoothing())
    {
      updateAnalogControlFrame(analogControlFrames[i], i);
      pushAnalogEffectControls(analogControlFrames[i], i);
    }

    /////////
    // Apply the analog controls to the modulators
    processModulatorControls(i);
  }

  // Process the audio through the Spotykach/Sporadic logic
  // Routing is dependent on currentRoutingMode as indicated by LED_ROUTING
  processAudioLogic(in, out, blockSize);
#endif
}

#if DEBUG
void AppImpl::logDebugInfo ()
{
  // float val = hw.GetAnalogControlValue(Hardware::CTRL_PITCH_A);
  // float val = hw.GetControlVoltageValue(Hardware::CV_V_OCT_A);
  // Log::PrintLine(FLT_FMT(5), FLT_VAR(5, val));
  // uint16_t touch = hw.GetMpr121TouchStates();
  // Log::PrintLine("0x%x", touch);
  if (usbBuffIx > 0)
  {
    // If the buffer contents end with a newline (LF, CR, or CRLF), print the buffer
    if ((usbBuff[usbBuffIx - 1] == '\n') ||
        (usbBuff[usbBuffIx - 1] == '\r') ||
        (!strncmp((const char *)usbBuff + usbBuffIx - 2, "\r\n", 2)))
    {
      Log::PrintLine("USB Callback: %d bytes received", usbBuffIx);
      for (size_t i = 0; i < usbBuffIx; i++)
      {
        Log::Print("%c ", usbBuff[i]);
      }
      Log::PrintLine("");
      Log::PrintLine("===");

      if (strncmp((const char *)usbBuff, REBOOT_TO_BOOTLOADER_CMD, sizeof(REBOOT_TO_BOOTLOADER_CMD) - 1) == 0)
      {
        Log::PrintLine("Rebooting to bootloader...");
        System::Delay(kDebugLogPeriodMs);    // Give time for the log to flush
        System::ResetToBootloader(System::BootloaderMode::DAISY_INFINITE_TIMEOUT);
      }
      usbBuffIx = 0;    // Reset after processing
    }
  }

#ifdef PRINT_CPU_LOAD
  // get the current load (smoothed value and peak values)
  const float avgLoad = loadMeter.GetAvgCpuLoad();
  const float maxLoad = loadMeter.GetMaxCpuLoad();
  const float minLoad = loadMeter.GetMinCpuLoad();
  // print it to the serial connection (as percentages)
  Log::PrintLine("Processing Load (%%):");
  Log::PrintLine("Max: " FLT_FMT3, FLT_VAR3(maxLoad * 100.0f));
  Log::PrintLine("Avg: " FLT_FMT3, FLT_VAR3(avgLoad * 100.0f));
  Log::PrintLine("Min: " FLT_FMT3, FLT_VAR3(minLoad * 100.0f));
#endif
}
#endif

// This is only used for detecting pot changes
void AppImpl::processUIQueue ()
{
  while (!ui_queue.IsQueueEmpty())
  {
    auto event = ui_queue.GetAndRemoveNextEvent();
    if (event.type == UiEventQueue::Event::EventType::potMoved)
    {
      for (size_t side = 0; side < kNumberEffectSlots; side++)
      {
        if (event.asPotMoved.id == Hardware::kCtrlModFreqIds[side])
          modFreqAltLatch[side] = Utils::isAltPadPressed(padTouchStates);

        if (event.asPotMoved.id == Hardware::kCtrlSosIds[side])
          mixAltLatch[side] = Utils::isAltPadPressed(padTouchStates);

        if (event.asPotMoved.id <= Hardware::kCtrlLastSideIds[side])
          last_pot_moved[side] = event.asPotMoved.id;
      }
    }
  }
}

void AppImpl::processMidi ()
{
  hw.midi_uart.Listen();
  while (hw.midi_uart.HasEvents())
  {
    auto event = hw.midi_uart.PopEvent();
    switch (event.type)
    {
      case MidiMessageType::NoteOn:
        midi_in_note_on = true;
        midi_in_nn      = event.AsNoteOn().note;
        for (size_t i = 0; i < 8; i++)
        {
          osc[i].SetFreq(daisysp::mtof(midi_in_nn + i * 0.05f));
        }
        break;

      case MidiMessageType::NoteOff:
        if (event.AsNoteOff().note == midi_in_nn)
        {
          midi_in_note_on = false;
        }
        break;

      default:
        break;
    }
  }
}

void AppImpl::drawRainbowRoad ()
{
  const float t = System::GetNow() / 1000.f;

  for (size_t i = 0; i < spotykach::Hardware::LED_LAST; i++)
  {
    float             phs = i / 32.0f;
    float             h   = daisysp::fastmod1f(t * 0.2f + phs);
    infrasonic::Color c   = infrasonic::Color::FromHSV(h * 255, 255, 255);
    hw.leds.Set(i, c.Hex(), 0.5f);
  }
}

void AppImpl::handleAnalogControls ()
{
  for (size_t side = 0; side < kNumberEffectSlots; side++)
  {
    // Read and smooth pitch controls for both sides
    pitchControls[side] = hw.GetAnalogControlValue(Hardware::kCtrlPitchIds[side]);
    // Add the pitch CV values
    pitchControls[side] += hw.GetControlVoltageValue(Hardware::kCvVOctIds[side]);

    // Read the mix controls for both sides
    mixControls[side] = hw.GetAnalogControlValue(Hardware::kCtrlSosIds[side]);
    // Add the mix CV values
    mixControls[side] += hw.GetControlVoltageValue(Hardware::kCvSosInIds[side]);

    // Read the position knobs and CVs
    positionControls[side] = hw.GetAnalogControlValue(Hardware::kCtrlPosIds[side]);
    if ((sizePosSwitch[side] == SizePosSwitchState::POSITION) || (sizePosSwitch[side] == SizePosSwitchState::BOTH))
    {
      positionControls[side] += hw.GetControlVoltageValue(Hardware::kCvSizePosIds[side]);
    }

    // Read the size knobs and CVs
    sizeControls[side] = hw.GetAnalogControlValue(Hardware::kCtrlSizeIds[side]);
    if ((sizePosSwitch[side] == SizePosSwitchState::SIZE) || (sizePosSwitch[side] == SizePosSwitchState::BOTH))
    {
      sizeControls[side] += hw.GetControlVoltageValue(Hardware::kCvSizePosIds[side]);
    }

    // Read the shape knobs
    shapeControls[side] = hw.GetAnalogControlValue(Hardware::kCtrlShapeIds[side]);

    // Read the modulation amount knobs
    modulationAmount[side] = hw.GetAnalogControlValue(Hardware::kCtrlModAmtIds[side]);

    // Read the modulation frequency knobs
    modulationFreq[side] = hw.GetAnalogControlValue(Hardware::kCtrlModFreqIds[side]);
  }
}

void AppImpl::handleDigitalControls ()
{
  // --- Switches (Shift registers) ---

  // construct into 8-bit set from inverted bitmask state
  // (all inputs are inverted due to pullups)
  std::bitset<8> sr1 = ~hw.GetShiftRegState(0);
  std::bitset<8> sr2 = ~hw.GetShiftRegState(1);

  // Mode A/B/C switch
  AppMode newMode = currentRoutingMode;
  if (sr1.test(2))
  {
    newMode = AppMode::ROUTING_GENERATIVE;
  }
  else if (sr1.test(3))
  {
    newMode = AppMode::ROUTING_DUAL_MONO;
  }
  else
  {
    newMode = AppMode::ROUTING_DUAL_STEREO;
  }

  if (newMode != currentRoutingMode)
  {
    routingModeChanged = true;
    currentRoutingMode = newMode;
    // Log::PrintLine("Operating mode changed to: %d", currentRoutingMode);
  }

  // Size/Pos A switch
  if (sr1.test(4))
  {
    sizePosSwitch[0] = SizePosSwitchState::SIZE;
  }
  else if (sr1.test(5))
  {
    sizePosSwitch[0] = SizePosSwitchState::POSITION;
  }
  else
  {
    sizePosSwitch[0] = SizePosSwitchState::BOTH;
  }

  // Size/Pos B switch
  if (sr2.test(0))
  {
    sizePosSwitch[1] = SizePosSwitchState::SIZE;
  }
  else if (sr2.test(1))
  {
    sizePosSwitch[1] = SizePosSwitchState::POSITION;
  }
  else
  {
    sizePosSwitch[1] = SizePosSwitchState::BOTH;
  }

  // Mode A switch (sr1 bits 6,7)
  EffectMode newEffectMode[2];
  if (sr1.test(6))
  {
    newEffectMode[0] = EffectMode::MODE_3;
  }
  else if (sr1.test(7))
  {
    newEffectMode[0] = EffectMode::MODE_1;
  }
  else
  {
    newEffectMode[0] = EffectMode::MODE_2;
  }

  // Mode B switch (sr2 bits 2,3)
  if (sr2.test(2))
  {
    newEffectMode[1] = EffectMode::MODE_3;
  }
  else if (sr2.test(3))
  {
    newEffectMode[1] = EffectMode::MODE_1;
  }
  else
  {
    newEffectMode[1] = EffectMode::MODE_2;
  }

  for (size_t i = 0; i < kNumberEffectSlots; i++)
  {
    if (newEffectMode[i] != currentEffectMode[i])
    {
      effectModeChanged[i] = true;
      currentEffectMode[i] = newEffectMode[i];
      // Log::PrintLine("Effect mode changed for side %d to: %d", i, currentEffectMode[i]);
    }
  }

  // Mod type A switch (sr1 bits 0,1)
  using ModType = ModulationEngine::ModType;
  ModType newModType[2];
  if (sr1.test(0))
  {
    newModType[0] = modulationTypes[0][2];
  }
  else if (sr1.test(1))
  {
    newModType[0] = modulationTypes[0][0];
  }
  else
  {
    newModType[0] = modulationTypes[0][1];
  }

  // Mod type B switch (sr2 bits 4,5)
  if (sr2.test(4))
  {
    newModType[1] = modulationTypes[1][2];
  }
  else if (sr2.test(5))
  {
    newModType[1] = modulationTypes[1][0];
  }
  else
  {
    newModType[1] = modulationTypes[1][1];
  }

  for (size_t i = 0; i < kNumberEffectSlots; i++)
  {
    if (newModType[i] != currentModType[i])
    {
      modTypeChanged[i] = true;
      currentModType[i] = newModType[i];
      // Log::PrintLine("Modulator type changed for side %d to: %d", i, currentModType[i]);
    }
  }

  // Touch controls
  padTouchStates = hw.GetMpr121TouchStates();

  for (uint8_t side = 0; side < kNumberEffectSlots; side++)
  {
    if (Utils::hasTouchStateChangedToPressed(padTouchStates, padTouchStatesPrev, kPadMapRevIds[side]))
    {
      // REV_A and B
      reverseStateChanged[side] = true;
      currentReverseState[side] = !currentReverseState[side];
      // Log::PrintLine("Reverse state changed for side %d to: %d", side, currentReverseState[side]);
    }

    if ((Utils::isAltPadPressed(padTouchStates)) &&
        (Utils::hasTouchStateChangedToPressed(padTouchStates, padTouchStatesPrev, kPadMapPlayIds[side])))
    {
      // ALT + PLAY_A or B
      altPlayStateChanged[side] = true;
      currentAltPlayState[side] = !currentAltPlayState[side];
      // Log::PrintLine("Alt+Play state changed for side %d to: %d", side, currentAltPlayState[side]);
    }
    else if ((Utils::isSpotykachPadPressed(padTouchStates)) &&
            (Utils::hasTouchStateChangedToPressed(padTouchStates, padTouchStatesPrev, kPadMapPlayIds[side])))
    {
      // SPOTYKACH + PLAY_A or B
      spotyPlayStateChanged[side] = true;
      currentSpotyPlayState[side] = !currentSpotyPlayState[side];
      // Log::PrintLine("Spotykach+Play state changed for side %d to: %d", side, currentSpotyPlayState[side]);
    }
    else if (Utils::hasTouchStateChangedToPressed(padTouchStates, padTouchStatesPrev, kPadMapPlayIds[side]))
    {
      // PLAY_A or B
      playStateChanged[side] = true;
      currentPlayState[side] = !currentPlayState[side];
      // Log::PrintLine("Play state changed for side %d to: %d", side, currentPlayState[side]);
    }
  }

  // Update the previous touch states
  padTouchStatesPrev = padTouchStates;
}

void AppImpl::handleDisplay ()
{
  // --- Gate I/O ---
  if (hw.GetClockInputState())
  {
    hw.leds.Set(Hardware::LED_CLOCK_IN, 0xff0000, 1.0f);
  }

  for (size_t i = 0; i < kNumberEffectSlots; i++)
  {
    if (hw.GetGateInputState(i))
    {
      hw.leds.Set(Hardware::kLedGateIds[i], 0xff0000, 1.0f);
    }
  }

  // --- Switches (Shift registers) ---
  std::bitset<8> sr2 = ~hw.GetShiftRegState(1);

  // Mode A/B/C switch
  if (currentRoutingMode == AppMode::ROUTING_GENERATIVE)
  {
    hw.leds.Set(Hardware::LED_ROUTING_RIGHT, 0xff0000, 1.0f);
  }
  else if (currentRoutingMode == AppMode::ROUTING_DUAL_MONO)
  {
    hw.leds.Set(Hardware::LED_ROUTING_LEFT, 0xff0000, 1.0f);
  }
  else if (currentRoutingMode == AppMode::ROUTING_DUAL_STEREO)
  {
    hw.leds.Set(Hardware::LED_ROUTING_CENTER, 0xff0000, 1.0f);
  }
  else
  {
    hw.leds.Set(Hardware::LED_ROUTING_LEFT, 0x000000, 1.0f);
    hw.leds.Set(Hardware::LED_ROUTING_RIGHT, 0x000000, 1.0f);
    hw.leds.Set(Hardware::LED_ROUTING_CENTER, 0x000000, 1.0f);
  }

  // Modulator A & B Type switch LED
  using ModType = ModulationEngine::ModType;
  for (size_t side = 0; side < kNumberEffectSlots; side++)
  {
    float modLedBrightness = modCv[side]; //modulationAmount[side].getSmoothVal();
    switch (currentModType[side])
    {
      case ModType::ENV_FOLLOWER:
      {
        hw.leds.Set(Hardware::kLedCycleIds[side], 0x00ff00, modLedBrightness);
        break;
      }
      case ModType::S_H:
      case ModType::SINE:
      {
        hw.leds.Set(Hardware::kLedCycleIds[side], 0x0000ff, modLedBrightness);
        break;
      }
      default:
      {
        hw.leds.Set(Hardware::kLedCycleIds[side], 0xff0000, modLedBrightness);
        break;
      }
    }
  }

  // Mode A & B switches
  for (size_t side = 0; side < kNumberEffectSlots; side++)
  {
    if (currentEffectMode[side] == EffectMode::MODE_3)
    {
      hw.leds.Set(Hardware::kLedOrbitIds[side], 0x00ff00, 1.0f);
    }
    else if (currentEffectMode[side] == EffectMode::MODE_1)
    {
      hw.leds.Set(Hardware::kLedOrbitIds[side], 0x0000ff, 1.0f);
    }
    else
    {
      hw.leds.Set(Hardware::kLedOrbitIds[side], 0xff0000, 1.0f);
    }
  }

  // Manual tempo tap switch
  if (sr2.test(6))
  {
    hw.leds.Set(Hardware::LED_CLOCK_IN, 0xffffff, 1.0f);
  }

  for (size_t side = 0; side < kNumberEffectSlots; side++)
  {
    // If the effect on this side has an updated display state
    if (effects[side]->getDisplayState(displayStates[side]))
    {
      for (size_t layer = 0; layer < std::min(displayStates[side].layerCount, Effect::kMaxRingLayers); ++layer)
      {
        const auto &seg = displayStates[side].rings[layer];
        for (uint8_t i = 0; i < Hardware::kNumLedsPerRing; ++i)
        {
          uint8_t ledIx = Hardware::kLedRingIds[side];
          if ((i >= seg.start) && (i < seg.end))
          {
            // Translate i to the physical index mapping
            // Start at bottom, wrap counterclockwise
            if (i <= 16)
            {
              ledIx += (Hardware::kNumLedsPerRing / 2) - i;
            }
            else
            {
              ledIx += Hardware::kNumLedsPerRing - 1;
              ledIx -= (i - (Hardware::kNumLedsPerRing / 2 + 1));
            }

            hw.leds.Set(ledIx, seg.led[i].rgb, seg.led[i].brightness);
          }
        }
      }
    }
  }

  // Spotykach Slider
  float skval = (hw.GetAnalogControlValue(Hardware::CTRL_SPOTYKACH) * 2.0f - 1.0f) +
                hw.GetControlVoltageValue(Hardware::CV_SPOTYKACH);
  skval = daisysp::fclamp(skval, -1.0f, 1.0f);
  hw.leds.Set(Hardware::LED_SPOTY_SLIDER_B, 0xff0000, daisysp::fmax(skval, 0.0f));
  hw.leds.Set(Hardware::LED_SPOTY_SLIDER_A, 0x0000ff, daisysp::fmax(-skval, 0.0f));

  // --- CV INPUTS ---

  // For these we just add together the 3 CVs on each side and render to drift LEDs
  for (uint8_t side = 0; side < kNumberEffectSlots; side++)
  {
    float cv = 0;
    cv += hw.GetControlVoltageValue(Hardware::kCvSosInIds[side]);
    cv += hw.GetControlVoltageValue(Hardware::kCvVOctIds[side]);
    cv += hw.GetControlVoltageValue(Hardware::kCvSizePosIds[side]);
    hw.leds.Set(Hardware::kLedDriftIds[side], cv >= 0.0f ? 0xff0000 : 0x0000ff, fabsf(cv));
  }

  // --- MIDI INPUT ---
  if (midi_in_note_on)
    hw.leds.Set(Hardware::LED_SPOTY_PAD, 0xff0000);

  // --- TOUCH PADS ---
  for (size_t i = 0; i < kNumberEffectSlots; i++)
  {
    // Alternating phase for REVERSE LEDs
    LedRgbBrightness &curLed = displayStates[i].reverseLedColors[padLedPhase];
    hw.leds.Set(Hardware::kLedRevIds[i], curLed.rgb, curLed.brightness);

    // Alternating phase for PLAY LEDs
    curLed = displayStates[i].playLedColors[padLedPhase];
    hw.leds.Set(Hardware::kLedPlayIds[i], curLed.rgb, curLed.brightness);
  }

  // These will override the corresponding LED of the touchpad with WHITE if the pad
  // is being pressed, otherwise default behavior from above
  for (uint16_t i = 0; i < kPadMappingSize; i++)
  {
    if (Utils::isTouchPadPressed(padTouchStates, i))
    {
      hw.leds.Set(kPadMapping[i], 0xffffff);
      if (i == kPadMapAltId)
      {
        // Alt pads share the same touch electrode
        hw.leds.Set(Hardware::LED_ALT_B, 0xffffff);
      }
    }
  }
}

void AppImpl::testSDCard ()
{
  uint32_t len, byteswritten, bytesread;

  for (size_t i = 0; i < kAudioDataBytes; i++)
  {
    audioData[i] = rand() & 255;
  }

  // Init SD Card
  SdmmcHandler::Config sd_cfg;
  sd_cfg.Defaults();
  sd_cfg.speed = SdmmcHandler::Speed::STANDARD;
  sd_cfg.width = SdmmcHandler::BusWidth::BITS_1;
  sd.Init(sd_cfg);

  // Links libdaisy i/o to fatfs driver.
  fsi.Init(FatFSInterface::Config::MEDIA_SD);

  // Mount SD Card
  if (f_mount(&fsi.GetSDFileSystem(), "/", 1) != FR_OK)
  {
  }

  uint32_t start = System::GetNow();

  // Open and write the test file to the SD Card.
  uint32_t offset = 0;
  Log::PrintLine("Writing ~11.5MB (approx 60s stereo 16-bit audio) to file...");
  if (f_open(&SDFile, TEST_FILE_NAME, (FA_CREATE_ALWAYS) | (FA_WRITE)) == FR_OK)
  {
    Log::PrintLine("Opened file for writing");
    // write in 32kB chunks
    while (offset < kAudioDataBytes)
    {
      len = std::min(kAudioDataBytes - offset, 32768ul);
      f_write(&SDFile, &audioData[offset], len, (UINT *)&byteswritten);
      offset += 32768ul;
    }
  }
  else
  {
    Log::PrintLine("Failed to open file for writing");
    return;
  }
  f_close(&SDFile);

  uint32_t write_time = System::GetNow() - start;
  Log::PrintLine("Write took %u ms", write_time);

  Log::PrintLine("Reading ~11.5MB (approx 60s stereo 16-bit audio) from file...");

  // Read back the test file from the SD Card.
  offset = 0;
  start  = System::GetNow();
  if (f_open(&SDFile, TEST_FILE_NAME, FA_OPEN_EXISTING | FA_READ) == FR_OK)
  {
    Log::PrintLine("Opened file for reading");
    while (offset < kAudioDataBytes)
    {
      FRESULT res;
      res = f_read(&SDFile, &audioData[offset], 32768, (UINT *)&bytesread);
      if (res != FR_OK)
      {
        Log::PrintLine("Chunk read failed: %u", res);
        break;
      }
      // Log::PrintLine("Read %u bytes in %u ms", bytesread, System::GetNow() - start);
      if (bytesread < 32768)
      {
        break;
      }
      offset += bytesread;
    }

    uint32_t read_time = System::GetNow() - start;
    Log::PrintLine("Read took %u ms", read_time);
  }
  else
  {
    Log::PrintLine("Failed to open file for reading");
    return;
  }

  f_close(&SDFile);
}

// ---------------------

void Application::Init ()
{
  impl.init();
}

void Application::Loop ()
{
  impl.loop();
}
