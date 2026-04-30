#include "app.h"
#include "Deck.h"
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

static AppImpl   impl;
static Spotykach spotykachLooper            = Spotykach(kSampleRate, kBlockSize);
static Sporadic  sporadic[kNumberDeckSlots] = {
  {kSampleRate, kBlockSize},
  {kSampleRate, kBlockSize}
};

// Array of pointers to Decks
Deck *decks[kNumberDeckSlots];

// Control frame for the decks
Deck::AnalogControlFrame  analogControlFrames[kNumberDeckSlots];
Deck::DigitalControlFrame digitalControlFrames[kNumberDeckSlots];
// Display state for the decks
Deck::DisplayState displayStates[kNumberDeckSlots];

#if DEBUG
CpuLoadMeter loadMeter;
#endif

#define REBOOT_TO_BOOTLOADER_CMD "reboot"
constexpr uint32_t kUsbBufferSize = 256;

char    usbBuff[kUsbBufferSize];
uint8_t usbBuffIx = 0;

static void UsbCallback (uint8_t *buf, uint32_t *len)
{
  if (*len > kUsbBufferSize - usbBuffIx)
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
  std::fill(std::begin(lastPotMoved), std::end(lastPotMoved), Hardware::CTRL_SOS_A);

  hw.seed.usb_handle.SetReceiveCallback(UsbCallback, UsbHandle::FS_EXTERNAL);

  auto &audio = hw.seed.audio_handle;
  audio.SetSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
  audio.SetBlockSize(kBlockSize);

  // Initialize the Spotykach looper
  spotykachLooper.init();
  sporadic[0].init();
  sporadic[1].init();

  // Initialize modulated parameter wrappers (attach to base SmoothValues)
  initModulatedParams();

  // Initialize the soft takeover envelope generators
  for (size_t i = 0; i < kNumberDeckSlots; i++)
  {
    takeoverEnv_[i].Init(kLedUpdateRate);
    takeoverEnv_[i].SetMin(0.0f);
    takeoverEnv_[i].SetMax(1.0f);
    takeoverEnv_[i].SetTime(ADENV_SEG_ATTACK, 0.050f);    // 50 ms attack
    takeoverEnv_[i].SetTime(ADENV_SEG_DECAY, 0.100f);     // 100 ms decay
  }

#if DEBUG
  Log::StartLog(false);
  log_timer.Init();
  loadMeter.Init(hw.seed.AudioSampleRate(), hw.seed.AudioBlockSize());
#endif

  audio.Start(AudioCallback);
}

void AppImpl::initModulatedParams ()
{
  // Per-side arrays
  for (size_t side = 0; side < kNumberDeckSlots; ++side)
  {
    // Attach ModulatedParam wrappers to their corresponding base SmoothValues.
    modulatedMix[side].attachBase(&mixControls[side]);
    modulatedPitch[side].attachBase(&pitchControls[side]);
    modulatedPosition[side].attachBase(&positionControls[side]);
    modulatedSize[side].attachBase(&sizeControls[side]);
    modulatedShape[side].attachBase(&shapeControls[side]);
    modulatedModAmount[side].attachBase(&modulationAmountControls[side]);
    modulatedModFreq[side].attachBase(&modulationFreqControls[side]);
    // Initialize pointer table for quick lookup (params 0..6 are per-side; 7 is spoty)
    modParamPtrs[side][modParamMixIdx]       = &modulatedMix[side];
    modParamPtrs[side][modParamPitchIdx]     = &modulatedPitch[side];
    modParamPtrs[side][modParamPosIdx]       = &modulatedPosition[side];
    modParamPtrs[side][modParamSizeIdx]      = &modulatedSize[side];
    modParamPtrs[side][modParamShapeIdx]     = &modulatedShape[side];
    modParamPtrs[side][modParamModAmountIdx] = &modulatedModAmount[side];
    modParamPtrs[side][modParamModFreqIdx]   = &modulatedModFreq[side];
    // Spoty is global (no per-side side), point to single instance
    modParamPtrs[side][modParamSpotyIdx] = &modulatedSpoty;
  }

  // Single-sided ModulatedParam controls
  modulatedSpoty.attachBase(&spotyControl);
}

using ChannelConfig = Deck::ChannelConfig;

void AppImpl::setRoutingMode (AppImpl::AppMode mode)
{
  if (currentRoutingMode == AppMode::ROUTING_GENERATIVE)
  {
    // spotykachLooper.setChannelConfig(ChannelConfig::STEREO);
    sporadic[0].setChannelConfig(ChannelConfig::STEREO);
    sporadic[1].setChannelConfig(ChannelConfig::STEREO);
  }
  else if (currentRoutingMode == AppMode::ROUTING_DUAL_MONO)
  {
    // spotykachLooper.setChannelConfig(ChannelConfig::MONO_LEFT);
    sporadic[0].setChannelConfig(ChannelConfig::MONO_LEFT);
    sporadic[1].setChannelConfig(ChannelConfig::MONO_RIGHT);
  }
  else if (currentRoutingMode == AppMode::ROUTING_DUAL_STEREO)
  {
    // spotykachLooper.setChannelConfig(ChannelConfig::STEREO);
    sporadic[0].setChannelConfig(ChannelConfig::STEREO);
    sporadic[1].setChannelConfig(ChannelConfig::STEREO);
  }
  decks[0] = &sporadic[0];
  decks[1] = &sporadic[1];
}

void AppImpl::updateAnalogControlFrame (Deck::AnalogControlFrame &frame, size_t slot)
{
  if (slot >= kNumberDeckSlots)
  {
    return;    // Invalid slot
  }

  // Provide modulated parameter references for decks to use
  frame.mix      = &modulatedMix[slot];
  frame.pitch    = &modulatedPitch[slot];
  frame.position = &modulatedPosition[slot];
  frame.size     = &modulatedSize[slot];
  frame.shape    = &modulatedShape[slot];
  frame.spoty    = &modulatedSpoty;

  // Populate effect modulation sources
  frame.gritModulation = &gritModSources[slot];
  frame.fluxModulation = &fluxModSources[slot];
}

void AppImpl::pushAnalogDeckControls (Deck::AnalogControlFrame &c, size_t slot)
{
  // Push the controls to the Spotykach looper and Sporadic deck
  // decks[slot]->updateAnalogControls(c);
}

void AppImpl::updateDigitalControlFrame (Deck::DigitalControlFrame &frame, size_t slot)
{
  if (slot >= kNumberDeckSlots)
  {
    return;    // Invalid slot
  }

  // Update the control frame for the specified deck slot
  frame = {
            // Simple pad presses
            .reverse = currentReverseState[slot],
            .play    = currentPlayState[slot],
            .flux    = currentFluxState[slot],
            .grit    = currentGritState[slot],
            // Supported pad combinations
            .altPlay   = currentAltPlayState[slot],
            .spotyPlay = currentSpotyPlayState[slot],
            .altFlux   = currentAltFluxState[slot],
            .altGrit   = currentAltGritState[slot],
            // Soft takeover notification
            .takeover = false
          };
}

void AppImpl::pushDigitalDeckControls (Deck::DigitalControlFrame &c, size_t slot)
{
  // Push the controls to the Spotykach looper and Sporadic deck
  decks[slot]->updateDigitalControls(c);
  // Get back the updated controls
  decks[slot]->getDigitalControls(c);
  // Store consequences of the control changes
  currentReverseState[slot]   = c.reverse;
  currentPlayState[slot]      = c.play;
  currentFluxState[slot]      = c.flux;
  currentGritState[slot]      = c.grit;
  currentAltPlayState[slot]   = c.altPlay;
  currentSpotyPlayState[slot] = c.spotyPlay;
  currentAltFluxState[slot]   = c.altFlux;
  currentAltGritState[slot]   = c.altGrit;
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
    // processMidi();

#if 0
    // Every 500ms toggle MIDI note out and gate outs
    if (midi_timer.HasPassedMs(kDebugLogPeriodMs))
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
      for (uint8_t i = 0; i < kNumberDeckSlots; i++)
      {
        hw.SetGateOut(i, test_note_on);
      }
    }

    // Modified libDaisy MIDI handlers require explicit call to transmit
    // enqueued messages instead of blocking every time a message is sent
    hw.midi_uart.TransmitEnqueuedMessages();
#endif

    // The LED refresh should run at least 200Hz for temporal dithering,
    // but faster is better
    if (led_timer.HasPassedMs(kLedUpdatePeriodMs))
    {
      led_timer.Restart();

      // Controller part of MVC
      handleDigitalControls();

      // Apply changes based on the controls readout
      if (routingModeChanged)
      {
        setRoutingMode(currentRoutingMode);
        routingModeChanged = false;
      }

      // Set the flag to feed the envelope follower
      envelopeFeed = true;

      for (size_t side = 0; side < kNumberDeckSlots; side++)
      {
        /////////
        // Modulators
        if (modTypeChanged[side])
        {
          modulator[side].setModType(currentModType[side]);
          modTypeChanged[side] = false;
        }
        // Piggy back on this timer (500 Hz) for very rough CV output demo
        modCv[side] = modulator[side].process();

        /////////
        // Global routing changed
        if (modTargetChanged[side])
        {
          modTargetChanged[side] = false;
          // Update the mod target smoothing values
          for (size_t modIdx = 0; modIdx < ModTarget::MOD_TARGET_LAST; modIdx++)
          {
            modTargetSmooth[side][modIdx] = (modIdx == currentModTarget[side]) ? 1.0f : 0.0f;
          }
        }

        /////////
        // Control state changes
        if (reverseStateChanged[side] || playStateChanged[side] || altPlayStateChanged[side] ||
            spotyPlayStateChanged[side] || fluxStateChanged[side] || altFluxStateChanged[side] ||
            gritStateChanged[side] || altGritStateChanged[side])
        {
          updateDigitalControlFrame(digitalControlFrames[side], side);
          pushDigitalDeckControls(digitalControlFrames[side], side);
          // Reset the change flags
          reverseStateChanged[side]   = false;
          playStateChanged[side]      = false;
          altPlayStateChanged[side]   = false;
          spotyPlayStateChanged[side] = false;
          fluxStateChanged[side]      = false;
          altFluxStateChanged[side]   = false;
          gritStateChanged[side]      = false;
          altGritStateChanged[side]   = false;
        }

        /////////
        // LED Ring display updates
        decks[side]->updateDisplayState();
      }

      // View part of MVC
      hw.leds.Clear();

      handleDisplay();
      hw.leds.Show();

      // Piggy back on this timer for very rough CV output demo
      for (size_t i = 0; i < kNumberDeckSlots; i++)
      {
        hw.WriteCVOut(i, modCv[i]);
      }
    }

    // Debug logging / LED phase advancement
    if (log_timer.HasPassedMs(kDebugLogPeriodMs))
    {
      padLedPhase = (padLedPhase + 1) % Deck::kMaxLedPhases;
#if DEBUG
      logDebugInfo();
#endif
      log_timer.Restart();
    }
  }
}

void AppImpl::processModulatorControls (size_t slot)
{
  if (slot >= kNumberDeckSlots)
  {
    return;    // Invalid slot
  }

  // Process the modulation for the specified deck slot
  for (size_t i = 0; i < kNumberDeckSlots; i++)
  {
    modulator[i].setFrequency(modulationFreqControls[i].getSmoothVal(), modFreqAltLatch[i]);
    modulator[i].setAmplitude(modulationAmountControls[i].getSmoothVal());
  }
}

void AppImpl::processAudioLogic (AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t blockSize)
{
  // Clear temp buffers for each deck/channel
  for (size_t slot = 0; slot < kNumberDeckSlots; ++slot)
  {
    for (size_t ch = 0; ch < kNumberChannelsStereo; ++ch)
    {
      std::fill(std::begin(deckOutputs_[slot][ch]), std::end(deckOutputs_[slot][ch]), 0.0f);
    }
  }

  // Process each deck into its own temporary buffer (deckOutputs_)
  for (size_t slot = 0; slot < kNumberDeckSlots; ++slot)
  {
    const float *slotIn[kNumberChannelsStereo];
    float       *slotOut[kNumberChannelsStereo];
    if (currentRoutingMode == AppMode::ROUTING_GENERATIVE)
    {
      // In generative mode, deck 0 gets the input, deck 1 gets the output of Deck 0
      if (slot == 0)
      {
        // Deck 0 gets input
        for (size_t ch = 0; ch < kNumberChannelsStereo; ++ch)
        {
          slotIn[ch] = in[ch];
        }
      }
      else
      {
        // Deck 1 gets output of Deck 0
        for (size_t ch = 0; ch < kNumberChannelsStereo; ++ch)
        {
          slotIn[ch] = deckOutputs_[0][ch];
        }
      }
    }
    else
    {
      // In dual mono/stereo modes, both decks get the same input
      for (size_t ch = 0; ch < kNumberChannelsStereo; ++ch)
      {
        slotIn[ch] = in[ch];
      }
    }
    for (size_t ch = 0; ch < kNumberChannelsStereo; ++ch)
    {
      slotOut[ch] = deckOutputs_[slot][ch];
    }
    decks[slot]->processAudio(slotIn, slotOut, blockSize);
  }

  // Crossfade / blend between deck 0 and 1 outputs into final out (linear)
  for (size_t ch = 0; ch < kNumberChannelsStereo; ++ch)
  {
    const float *a = deckOutputs_[0][ch];
    const float *b = deckOutputs_[1][ch];
    Utils::audioBlockLerp(a, b, out[ch], deckMix_, blockSize);
  }
}

void AppImpl::processAudio (AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t blockSize)
{
  hw.ProcessAnalogControls();

  // Handle the analog controls that affect the audio processing
  handleAnalogControls();

  // Apply modulator soft modulation to parameters based on stored mappings
  applyModulatorSoftModulation();

  for (size_t i = 0; i < kNumberDeckSlots; i++)
  {
    /////////
    // Apply the analog controls to the decks

    // if (modulatedMix[i].isSmoothing() || modulatedPitch[i].isSmoothing() || modulatedPosition[i].isSmoothing() ||
    //   modulatedSize[i].isSmoothing() || modulatedShape[i].isSmoothing() || modulatedSpoty.isSmoothing())
    // {
    updateAnalogControlFrame(analogControlFrames[i], i);
    pushAnalogDeckControls(analogControlFrames[i], i);
    // }

    /////////
    // Apply the analog controls to the modulators
    processModulatorControls(i);

    // Feed envelope follower input from audio stream.
    if (envelopeFeed)
    {
      modulator[i].setEnvelopeInput(in[0], 1);
    }
  }

  // Reset the envelope follower feed flag
  envelopeFeed = false;

  // Process the audio through the Spotykach/Sporadic logic
  // Routing is dependent on currentRoutingMode as indicated by LED_ROUTING
  processAudioLogic(in, out, blockSize);
}

#if DEBUG
void AppImpl::logDebugInfo ()
{
  // // float val = hw.GetAnalogControlValue(Hardware::CTRL_PITCH_A);
  // // float val = hw.GetControlVoltageValue(Hardware::CV_V_OCT_A);
  // // Log::PrintLine(FLT_FMT(5), FLT_VAR(5, val));
  // // Log::PrintLine(FLT_FMT(5), FLT_VAR(5, positionControls[0].getSmoothVal()));
  // // Log::PrintLine(FLT_FMT(5), FLT_VAR(5, positionControls[0].getTargetVal()));
  Log::PrintLine("Spotykach Slider            : " FLT_FMT(5), FLT_VAR(5, modulatedSpoty.getEffectiveSmoothVal()));
  Log::PrintLine("Position A                  : " FLT_FMT(5), FLT_VAR(5, modulatedPosition[0].getEffectiveSmoothVal()));
  Log::PrintLine("Size A                      : " FLT_FMT(5), FLT_VAR(5, modulatedSize[0].getEffectiveSmoothVal()));
  Log::PrintLine("Shape A                     : " FLT_FMT(5), FLT_VAR(5, modulatedShape[0].getEffectiveSmoothVal()));
  if (modulatedPosition[1].gritLatch)
    Log::PrintLine("Position B grit latched         : true");
  if (modulatedSize[1].gritLatch)
    Log::PrintLine("Size B grit latched             : true");
  if (modulatedShape[1].gritLatch)
    Log::PrintLine("Shape B grit latched            : true");
  if (modulatedPitch[1].gritLatch)
    Log::PrintLine("Pitch B grit latched            : true");
  // // Log::PrintLine("Read Index A   : " FLT_FMT(5), FLT_VAR(5, ((Spotykach *) decks[0])->getReadIx()));
  // // Log::PrintLine("Write Index A  : " FLT_FMT(5), FLT_VAR(5, ((Spotykach *) decks[0])->getWriteIx()));
  // // // Log::PrintLine("Read Window Start : " FLT_FMT(5), FLT_VAR(5, decks[0]->getReadWindowStart()));
  // // // Log::PrintLine("Read Window End   : " FLT_FMT(5), FLT_VAR(5, decks[0]->getReadWindowEnd()));
  // // Log::PrintLine("Ring Start A   : " FLT_FMT(5), FLT_VAR(5, displayStates[0].rings[0].start));
  // // Log::PrintLine("Ring End A     : " FLT_FMT(5), FLT_VAR(5, displayStates[0].rings[0].end));
  // Log::PrintLine("Env A Attack        : " FLT_FMT(5), FLT_VAR(5, modulator[0].getAttackMs()));
  // Log::PrintLine("Env A Release       : " FLT_FMT(5), FLT_VAR(5, modulator[0].getReleaseMs()));
  // Log::PrintLine("Env A Attack Coeff  : " FLT_FMT(5), FLT_VAR(5, modulator[0].getAttackCoefficient()));
  // Log::PrintLine("Env A Release Coeff : " FLT_FMT(5), FLT_VAR(5, modulator[0].getReleaseCoefficient()));
  Log::PrintLine("Mod A Value                 : " FLT_FMT(5), FLT_VAR(5, modulator[0].process()));

  std::vector<float> bandFreqs;
  sporadic[0].getBandFrequencies(bandFreqs);
  for (size_t i = 0; i < bandFreqs.size(); ++i)
  {
    Log::PrintLine("Band %d Frequency: " FLT_FMT(5), i, FLT_VAR(5, bandFreqs[i]));
  }

  std::vector<float> treePositions;
  sporadic[0].getTreePositions(treePositions);
  for (size_t i = 0; i < treePositions.size(); ++i)
  {
    Log::PrintLine("Tree %d Position: " FLT_FMT(5), i, FLT_VAR(5, treePositions[i]));
  }

  // Print the nodeInterconnection matrix
  std::vector<std::vector<float>> matrix;
  sporadic[0].getNodeInterconnectionMatrix(matrix);
  Log::PrintLine("Interconnection Matrix:");
  for (size_t i = 0; i < matrix.size(); ++i)
  {
    for (size_t j = 0; j < matrix[i].size(); ++j)
    {
      Log::Print("  " FLT_FMT(5), FLT_VAR(5, matrix[i][j]));
    }
    Log::PrintLine("");
    // Log::PrintLine("Node %d: %s", i, row.c_str());
  }

  // Print the sidechain levels
  std::vector<float> scLevels;
  sporadic[0].getSidechainLevels(0, scLevels);
  Log::PrintLine("Sidechain Levels:");
  for (size_t i = 0; i < scLevels.size(); ++i)
  {
    Log::PrintLine("Processor %d Level: " FLT_FMT(5), i, FLT_VAR(5, scLevels[i]));
  }

  if (modulatedSpoty.isSmoothing())
    Log::PrintLine("Spotykach Slider Smoothing  : true");
  else
    Log::PrintLine("Spotykach Slider Smoothing  : false");
  // uint16_t touch = hw.GetMpr121TouchStates();
  // Log::PrintLine("0x%x", touch);
  if (usbBuffIx > 0)
  {
    // If the buffer contents end with a newline (LF, CR, or CRLF), print the buffer
    if ((usbBuff[usbBuffIx - 1] == '\n') || (usbBuff[usbBuffIx - 1] == '\r') ||
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
      uint16_t movedPot = event.asPotMoved.id;
      for (size_t targetSide = 0; targetSide < kNumberDeckSlots; targetSide++)
      {
        // If the moved pot belongs to this side, evaluate mapping
        if (!(movedPot <= Hardware::kCtrlLastSideIds[targetSide]))
        {
          continue;
        }

        if (movedPot == Hardware::kCtrlModFreqIds[targetSide])
        {
          // Use Alt pad latch to modify Modulation Frequency
          modulatedModFreq[targetSide].altLatch = Utils::isAltPadPressed(padTouchStates);
        }

        if (movedPot == Hardware::kCtrlSosIds[targetSide])
        {
          // Use Alt pad latch to modify Mix
          modulatedMix[targetSide].altLatch = Utils::isAltPadPressed(padTouchStates);
          // Use Mod pad latch to modify Mix
          modulatedMix[targetSide].modLatch = Utils::isModPadPressed(targetSide, padTouchStates);
        }

        if (movedPot == Hardware::kCtrlPitchIds[targetSide])
        {
          // Use Alt pad latch to modify Pitch
          modulatedPitch[targetSide].altLatch = Utils::isAltPadPressed(padTouchStates);
          // Use Mod pad latch to modify Pitch
          modulatedPitch[targetSide].modLatch = Utils::isModPadPressed(targetSide, padTouchStates);
          // Use Grit pad latch to modify Pitch
          modulatedPitch[targetSide].gritLatch = Utils::isGritPadPressed(targetSide, padTouchStates);
          // Use Flux pad latch to modify Pitch
          modulatedPitch[targetSide].fluxLatch = Utils::isFluxPadPressed(targetSide, padTouchStates);
        }

        if (movedPot == Hardware::kCtrlPosIds[targetSide])
        {
          // Use Alt pad latch to modify Position
          modulatedPosition[targetSide].altLatch = Utils::isAltPadPressed(padTouchStates);
          // Use Mod pad latch to modify Position
          modulatedPosition[targetSide].modLatch =
            Utils::isModPadPressed(0, padTouchStates) || Utils::isModPadPressed(1, padTouchStates);
          // Use Grit pad latch to modify Position
          modulatedPosition[targetSide].gritLatch = Utils::isGritPadPressed(targetSide, padTouchStates);
          // Use Flux pad latch to modify Position
          modulatedPosition[targetSide].fluxLatch = Utils::isFluxPadPressed(targetSide, padTouchStates);
        }

        if (movedPot == Hardware::kCtrlShapeIds[targetSide])
        {
          // Use Alt pad latch to modify Shape
          modulatedShape[targetSide].altLatch = Utils::isAltPadPressed(padTouchStates);
          // Use Mod pad latch to modify Shape
          modulatedShape[targetSide].modLatch = Utils::isModPadPressed(targetSide, padTouchStates);
          // Use Grit pad latch to modify Shape
          modulatedShape[targetSide].gritLatch = Utils::isGritPadPressed(targetSide, padTouchStates);
          // Use Flux pad latch to modify Shape
          modulatedShape[targetSide].fluxLatch = Utils::isFluxPadPressed(targetSide, padTouchStates);
        }

        if (movedPot == Hardware::kCtrlSizeIds[targetSide])
        {
          // Use Alt pad latch to modify Size
          modulatedSize[targetSide].altLatch = Utils::isAltPadPressed(padTouchStates);
          // Use Mod pad latch to modify Size
          modulatedSize[targetSide].modLatch = Utils::isModPadPressed(targetSide, padTouchStates);
          // Use Grit pad latch to modify Size
          modulatedSize[targetSide].gritLatch = Utils::isGritPadPressed(targetSide, padTouchStates);
          // Use Flux pad latch to modify Size
          modulatedSize[targetSide].fluxLatch = Utils::isFluxPadPressed(targetSide, padTouchStates);
        }

        // Remember last moved pot for that side
        lastPotMoved[targetSide] = static_cast<Hardware::AnalogControlId>(movedPot);

        // Check every modulator side to see if its Mod pad is currently pressed.
        for (size_t modSide = 0; modSide < kNumberDeckSlots; ++modSide)
        {
          if (!Utils::isTouchPadPressed(padTouchStates, kPadMapCycleIds[modSide]))
          {
            continue;
          }

          // Determine which parameter index the moved pot (on targetSide) represents
          int paramIdx = -1;
          if (movedPot == Hardware::kCtrlModFreqIds[targetSide])
          {
            paramIdx = modParamModFreqIdx;
          }
          else if (movedPot == Hardware::kCtrlSosIds[targetSide])
          {
            paramIdx = modParamMixIdx;
          }
          else if (movedPot == Hardware::kCtrlPitchIds[targetSide])
          {
            paramIdx = modParamPitchIdx;
          }
          else if (movedPot == Hardware::kCtrlPosIds[targetSide])
          {
            paramIdx = modParamPosIdx;
          }
          else if (movedPot == Hardware::kCtrlShapeIds[targetSide])
          {
            paramIdx = modParamShapeIdx;
          }
          else if (movedPot == Hardware::kCtrlSizeIds[targetSide])
          {
            paramIdx = modParamSizeIdx;
          }

          // Record the moved pot value as modulation depth
          modParamMappings[modSide][targetSide][paramIdx].depth = hw.GetAnalogControlValue(lastPotMoved[targetSide]);
          // If Alt was held during mapping, remember polarity as bipolar
          if (Utils::isAltPadPressed(padTouchStates))
          {
            modParamMappings[modSide][targetSide][paramIdx].polarity = ModulationSources::Polarity::BIPOLAR;
          }
          else
          {
            modParamMappings[modSide][targetSide][paramIdx].polarity = ModulationSources::Polarity::UNIPOLAR;
          }
        }
      }
    }
  }
}

/*
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
*/

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

void AppImpl::applyModulatorSoftModulation ()
{
  // For each modulator side and each param index, apply soft modulation to the mapping's target side.
  for (size_t modSide = 0; modSide < kNumberDeckSlots; ++modSide)
  {
    for (size_t targetSide = 0; targetSide < kNumberDeckSlots; ++targetSide)
    {
      for (size_t paramIdx = 0; paramIdx < kNumModParams; ++paramIdx)
      {
        if (modParamMappings[modSide][targetSide][paramIdx].depth < 0.001f)
        {
          continue;
        }

        float depth    = modParamMappings[modSide][targetSide][paramIdx].depth;
        auto  polarity = modParamMappings[modSide][targetSide][paramIdx].polarity;

        // Which modulated param wrapper to call on the target side
        ModulatedParam *targetParam = nullptr;
        if (paramIdx >= 0 && paramIdx < (int)kNumModParams)
        {
          // For params 0..6 we use per-target side pointers; param 7 (modParamSpotyIdx) resolves
          // to the single modulatedSpoty instance via the pointer table.
          targetParam = modParamPtrs[targetSide][paramIdx];
        }

        if (targetParam != nullptr)
        {
          // For Spoty (single instance) we still call addSoftModulation — the targetSide
          // argument is cast to ModSourceIndex to indicate which soft source to use.
          targetParam->addSoftModulation(static_cast<ModSourceIndex>(modSide),
                                         modCv[modSide],
                                         depth,
                                         Mapping::LINEAR,
                                         polarity);
        }
      }
    }
  }

void AppImpl::applyCvModulation (ModulatedParam &modParam, Hardware::CvInputId cvId, float cvModSmoothLevel)
{
  modParam.addCvModulation(hw.GetControlVoltageValue(cvId) * cvModSmoothLevel);
}

void AppImpl::handleAnalogControls ()
{
  // Spotykach slider
  spotyControl = hw.GetAnalogControlValue(Hardware::CTRL_SPOTYKACH);
  // Apply CV modulation to Spotykach slider
  applyCvModulation(modulatedSpoty, Hardware::CV_SPOTYKACH);
  // Set the deck mix level
  deckMix_ = modulatedSpoty.getEffectiveSmoothVal();

  for (size_t side = 0; side < kNumberDeckSlots; side++)
  {
    // Read and smooth pitch controls for both sides
    if (!modParamModMapping[side][modParamPitchIdx])
    {
      pitchControls[side] = hw.GetAnalogControlValue(Hardware::kCtrlPitchIds[side]);
    }
    // Apply CV modulation to Pitch
    applyCvModulation(modulatedPitch[side], Hardware::kCvVOctIds[side]);

    // Read the mix controls for both sides
    if (!modParamModMapping[side][modParamMixIdx])
    {
      mixControls[side] = hw.GetAnalogControlValue(Hardware::kCtrlSosIds[side]);
    }
    // Apply CV modulation to Mix
    applyCvModulation(modulatedMix[side],
                      Hardware::kCvSosInIds[side],
                      modTargetSmooth[side][ModTarget::MIX].getSmoothVal());

    // Apply CV modulation to Grit and Flux if selected as mod targets
    fluxModSources[side].modLevel[ModSourceIndex::CV] =
      hw.GetControlVoltageValue(Hardware::kCvSosInIds[side]) * modTargetSmooth[side][ModTarget::FLUX].getSmoothVal();
    gritModSources[side].modLevel[ModSourceIndex::CV] =
      hw.GetControlVoltageValue(Hardware::kCvSosInIds[side]) * modTargetSmooth[side][ModTarget::GRIT].getSmoothVal();

    // Read the position knobs and CVs
    if (!modParamModMapping[side][modParamPosIdx])
    {
      positionControls[side] = hw.GetAnalogControlValue(Hardware::kCtrlPosIds[side]);
    }
    if ((sizePosSwitches[side] == SizePosSwitchState::POSITION) || (sizePosSwitches[side] == SizePosSwitchState::BOTH))
    {
      // Add the position CV values when the Size/Pos switch is set to Position or Both
      applyCvModulation(modulatedPosition[side], Hardware::kCvSizePosIds[side]);
    }

    // Read the size knobs and CVs
    if (!modParamModMapping[side][modParamSizeIdx])
    {
      sizeControls[side] = hw.GetAnalogControlValue(Hardware::kCtrlSizeIds[side]);
    }
    if ((sizePosSwitches[side] == SizePosSwitchState::SIZE) || (sizePosSwitches[side] == SizePosSwitchState::BOTH))
    {
      // Add the size CV values when the Size/Pos switch is set to Size or Both
      applyCvModulation(modulatedSize[side], Hardware::kCvSizePosIds[side]);
    }

    // Read the shape knobs
    if (!modParamModMapping[side][modParamShapeIdx])
    {
      shapeControls[side] = hw.GetAnalogControlValue(Hardware::kCtrlShapeIds[side]);
    }

    // Read the modulation amount knobs
    if (!modParamModMapping[side][modParamModAmountIdx])
    {
      modulationAmountControls[side] = hw.GetAnalogControlValue(Hardware::kCtrlModAmtIds[side]);
    }

    // Read the modulation frequency knobs
    if (!modParamModMapping[side][modParamModFreqIdx])
    {
      modulationFreqControls[side] = hw.GetAnalogControlValue(Hardware::kCtrlModFreqIds[side]);
    }
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
    sizePosSwitches[0] = SizePosSwitchState::SIZE;
  }
  else if (sr1.test(5))
  {
    sizePosSwitches[0] = SizePosSwitchState::POSITION;
  }
  else
  {
    sizePosSwitches[0] = SizePosSwitchState::BOTH;
  }

  // Size/Pos B switch
  if (sr2.test(0))
  {
    sizePosSwitches[1] = SizePosSwitchState::SIZE;
  }
  else if (sr2.test(1))
  {
    sizePosSwitches[1] = SizePosSwitchState::POSITION;
  }
  else
  {
    sizePosSwitches[1] = SizePosSwitchState::BOTH;
  }

  // Mode A switch (sr1 bits 6,7)
  ModTarget newModTarget[2];
  if (sr1.test(6))
  {
    newModTarget[0] = ModTarget::GRIT;
  }
  else if (sr1.test(7))
  {
    newModTarget[0] = ModTarget::MIX;
  }
  else
  {
    newModTarget[0] = ModTarget::FLUX;
  }

  // Mode B switch (sr2 bits 2,3)
  if (sr2.test(2))
  {
    newModTarget[1] = ModTarget::GRIT;
  }
  else if (sr2.test(3))
  {
    newModTarget[1] = ModTarget::MIX;
  }
  else
  {
    newModTarget[1] = ModTarget::FLUX;
  }

  for (size_t i = 0; i < kNumberDeckSlots; i++)
  {
    if (newModTarget[i] != currentModTarget[i])
    {
      modTargetChanged[i] = true;
      currentModTarget[i] = newModTarget[i];
      // Log::PrintLine("Modulation target changed for side %d to: %d", i, currentModTarget[i]);
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

  for (size_t i = 0; i < kNumberDeckSlots; i++)
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

  for (uint8_t side = 0; side < kNumberDeckSlots; side++)
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

    if (Utils::hasTouchStateChanged(padTouchStates, padTouchStatesPrev, kPadMapFluxIds[side]))
    {
      // FLUX A or B
      fluxStateChanged[side] = true;
      currentFluxState[side] = Utils::isTouchPadPressed(padTouchStates, kPadMapFluxIds[side]);
      // Log::PrintLine("Flux state changed for side %d to: %d", side, currentFluxState[side]);
    }

    if (Utils::isAltPadPressed(padTouchStates) &&
        Utils::hasTouchStateChangedToPressed(padTouchStates, padTouchStatesPrev, kPadMapFluxIds[side]))
    {
      // ALT + FLUX A or B
      altFluxStateChanged[side] = true;
      currentAltFluxState[side] = !currentAltFluxState[side];
      // Log::PrintLine("Alt+Flux state changed for side %d to: %d", side, currentAltFluxState[side]);
    }

    if (Utils::isAltPadPressed(padTouchStates) &&
        Utils::hasTouchStateChangedToPressed(padTouchStates, padTouchStatesPrev, kPadMapGritIds[side]))
    {
      // ALT + GRIT A or B
      altGritStateChanged[side] = true;
      currentAltGritState[side] = !currentAltGritState[side];
      // Log::PrintLine("Alt+Grit state changed for side %d to: %d", side, currentAltGritState[side]);
    }

    if (Utils::hasTouchStateChanged(padTouchStates, padTouchStatesPrev, kPadMapGritIds[side]))
    {
      // GRIT A or B
      gritStateChanged[side] = true;
      currentGritState[side] = Utils::isTouchPadPressed(padTouchStates, kPadMapGritIds[side]);
      // Log::PrintLine("Grit state changed for side %d to: %d", side, currentGritState[side]);
    }

    if (Utils::hasTouchStateChangedToReleased(padTouchStates, padTouchStatesPrev, kPadMapCycleIds[side]))
    {
      for (size_t paramIdx = 0; paramIdx < kNumModParams; ++paramIdx)
      {
        // clear the modulation mapping flag for all parameters (so base control updates resume)
        modParamModMapping[side][paramIdx] = false;
      }
    }
  }

  // Update the previous touch states
  padTouchStatesPrev = padTouchStates;

  for (size_t side = 0; side < kNumberDeckSlots; side++)
  {
    if (decks[side] != nullptr)
    {
      Deck::DigitalControlFrame takeoverProbe{};
      decks[side]->getDigitalControls(takeoverProbe);
      currentTakeoverState[side] = takeoverProbe.takeover;
    }
  }

  for (size_t side = 0; side < kNumberDeckSlots; side++)
  {
    if (currentTakeoverState[side])
    {
      takeoverPulseActive_[side] = true;
      takeoverEnv_[side].Trigger();
    }
  }
}

void AppImpl::handleDisplay ()
{
  // --- Gate I/O ---
  if (hw.GetClockInputState())
  {
    hw.leds.Set(Hardware::LED_CLOCK_IN, 0xff0000, kMaxLedBrightness);
  }

  for (size_t i = 0; i < kNumberDeckSlots; i++)
  {
    if (hw.GetGateInputState(i))
    {
      hw.leds.Set(Hardware::kLedGateIds[i], 0xff0000, kMaxLedBrightness);
    }
  }

  // --- Switches (Shift registers) ---
  std::bitset<8> sr2 = ~hw.GetShiftRegState(1);

  // Mode A/B/C switch
  if (currentRoutingMode == AppMode::ROUTING_GENERATIVE)
  {
    hw.leds.Set(Hardware::LED_ROUTING_RIGHT, 0xff0000, kMaxLedBrightness);
  }
  else if (currentRoutingMode == AppMode::ROUTING_DUAL_MONO)
  {
    hw.leds.Set(Hardware::LED_ROUTING_LEFT, 0xff0000, kMaxLedBrightness);
  }
  else if (currentRoutingMode == AppMode::ROUTING_DUAL_STEREO)
  {
    hw.leds.Set(Hardware::LED_ROUTING_CENTER, 0xff0000, kMaxLedBrightness);
  }
  else
  {
    hw.leds.Set(Hardware::LED_ROUTING_LEFT, 0x000000, kMaxLedBrightness);
    hw.leds.Set(Hardware::LED_ROUTING_RIGHT, 0x000000, kMaxLedBrightness);
    hw.leds.Set(Hardware::LED_ROUTING_CENTER, 0x000000, kMaxLedBrightness);
  }

  // Modulator A & B Type switch LED
  using ModType = ModulationEngine::ModType;
  for (size_t side = 0; side < kNumberDeckSlots; side++)
  {
    if (takeoverPulseActive_[side])
    {
      float envVal = takeoverEnv_[side].Process();
      hw.leds.Set(Hardware::kLedCycleIds[side], 0xffffff, daisysp::fmap(envVal, kMinLedBrightness, kMaxLedBrightness, Mapping::EXP));
      takeoverPulseActive_[side] = takeoverEnv_[side].IsRunning();
      continue;
    }

    float modLedBrightness = daisysp::fmap(modCv[side], kMinLedBrightness, kMaxLedBrightness, Mapping::LOG);
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

  // Manual tempo tap switch
  if (sr2.test(6))
  {
    hw.leds.Set(Hardware::LED_CLOCK_IN, 0xffffff, kMaxLedBrightness);
  }

  for (size_t side = 0; side < kNumberDeckSlots; side++)
  {
    // If the deck on this side has an updated display state
    if (decks[side]->getDisplayState(displayStates[side]))
    {
      for (size_t layer = 0; layer < std::min(displayStates[side].layerCount, Deck::kMaxRingLayers); ++layer)
      {
        const auto &seg = displayStates[side].rings[layer];
        for (uint8_t i = 0; i < Hardware::kNumLedsPerRing; ++i)
        {
          uint8_t ledIx = Hardware::kLedRingIds[side];
          if ((i >= seg.start) && (i < seg.end))
          {
            // Translate i to the physical index mapping
            // Start at bottom, wrap clockwise
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
  float skval = daisysp::fmap(deckMix_, -1.0f, 1.0f);
  hw.leds.Set(Hardware::LED_SPOTY_SLIDER_B,
              0xff0000,
              skval > 0.0f ? daisysp::fmap(skval, kMinLedBrightness, kMaxLedBrightness, Mapping::LOG)
                           : kOffLedBrightness);
  hw.leds.Set(Hardware::LED_SPOTY_SLIDER_A,
              0x0000ff,
              skval < 0.0f ? daisysp::fmap(-skval, kMinLedBrightness, kMaxLedBrightness, Mapping::LOG)
                           : kOffLedBrightness);

  // // For these we just add together the 3 CVs on each side and render to drift LEDs
  // for (uint8_t side = 0; side < kNumberDeckSlots; side++)
  // {
  //   float cv = 0;
  //   cv += hw.GetControlVoltageValue(Hardware::kCvSosInIds[side]);
  //   cv += hw.GetControlVoltageValue(Hardware::kCvVOctIds[side]);
  //   cv += hw.GetControlVoltageValue(Hardware::kCvSizePosIds[side]);
  //   hw.leds.Set(Hardware::kLedGritIds[side], cv >= 0.0f ? 0xff0000 : 0x0000ff, fabsf(cv));
  // }

  // --- MIDI INPUT ---
  if (midi_in_note_on)
    hw.leds.Set(Hardware::LED_SPOTY_PAD, 0xff0000);

  // --- TOUCH PAD LEDs ---
  for (size_t i = 0; i < kNumberDeckSlots; i++)
  {
    // Alternating phase for FLUX LEDs
    if (displayStates[i].fluxActive)
    {
      LedRgbBrightness &curLed = displayStates[i].fluxLedColors[padLedPhase];
      hw.leds.Set(Hardware::kLedFluxIds[i], curLed.rgb, curLed.brightness);
    }

    // If a Mod pad (cycle) is pressed on either side, display modulation depths
    for (size_t modSide = 0; modSide < kNumberDeckSlots; ++modSide)
    {
      if (Utils::isModPadPressed(modSide, padTouchStates))
      {
        for (size_t targetSide = 0; targetSide < kNumberDeckSlots; targetSide++)
        {
          // For each mod parameter, set the paired LEDs according to the mapping depth
          for (size_t paramIdx = 0; paramIdx < (size_t)kNumModParams; ++paramIdx)
          {
            float depth = 0.0f;

            // Show the depth of the mapping originating from this modSide
            auto modMapping = &modParamMappings[modSide][targetSide][paramIdx];
            depth           = modMapping->depth;

            // Map depth (0..1) to LED brightness
            float brightness = daisysp::fmap(depth, kMinLedBrightness, kMaxLedBrightness, Mapping::LINEAR);

            // For Spoty (last index) there is a single LED
            hw.leds.Set(kModParamMapLed[targetSide][paramIdx], 0xffffff, brightness);
          }
        }
        // Only allow one modulator source's destinations to be highlighted
        break;
      }
    }

    // Alternating phase for GRIT LEDs
    if (displayStates[i].gritActive)
    {
      LedRgbBrightness &curLed = displayStates[i].gritLedColors[padLedPhase];
      hw.leds.Set(Hardware::kLedGritIds[i], curLed.rgb, curLed.brightness);
    }

    // Alternating phase for REVERSE LEDs
    if (displayStates[i].reverseActive)
    {
      LedRgbBrightness &curLed = displayStates[i].reverseLedColors[padLedPhase];
      hw.leds.Set(Hardware::kLedRevIds[i], curLed.rgb, curLed.brightness);
    }

    // Alternating phase for PLAY LEDs
    if (displayStates[i].playActive)
    {
      LedRgbBrightness &curLed = displayStates[i].playLedColors[padLedPhase];
      hw.leds.Set(Hardware::kLedPlayIds[i], curLed.rgb, curLed.brightness);
    }

    // Alternating phase for ALT LEDs
    if (displayStates[i].altActive)
    {
      LedRgbBrightness &curLed = displayStates[i].altLedColors[padLedPhase];
      hw.leds.Set(Hardware::kLedAltIds[i], curLed.rgb, curLed.brightness);
    }
  }

  // These will override the corresponding LED of the touchpad with WHITE if the pad
  // is being pressed, otherwise default behavior from above
  for (uint16_t i = 0; i < kPadMappingSize; i++)
  {
    // Skip Grit and Flux pads
    if (i == kPadMapGritIds[0] || i == kPadMapFluxIds[0] || i == kPadMapGritIds[1] || i == kPadMapFluxIds[1])
    {
      continue;
    }

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

// ---------------------

void Application::Init ()
{
  impl.init();
}

void Application::Loop ()
{
  impl.loop();
}
