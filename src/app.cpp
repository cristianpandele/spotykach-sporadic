#include "app.h"
#include "color.h"
#include "common.h"
#include "hardware.h"
#include <bitset>
#include <cstring>
#include <daisy_seed.h>

using namespace daisy;
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
  constexpr size_t             kAudioDataBytes = 60 * 1000 * 48 * 2 * sizeof(int16_t);
  static DSY_SDRAM_BSS uint8_t audiodata[kAudioDataBytes];

  class AppImpl
  {
    public:
      AppImpl ()  = default;
      ~AppImpl () = default;

      void Init ();
      void Loop ();

      void ProcessAudio (AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size);

    private:
      Hardware       hw;
      StopwatchTimer led_timer;
      StopwatchTimer midi_timer;

      UiEventQueue ui_queue;

      PotMonitor<Hardware, Hardware::kNumAnalogControls> pot_monitor;

      uint16_t last_pot_moved_a;
      uint16_t last_pot_moved_b;

      bool    test_note_on;
      bool    midi_in_note_on;
      uint8_t midi_in_nn;

      daisysp::Oscillator osc[8];

#if DEBUG
      StopwatchTimer log_timer;
      void           logDebugInfo ();
#endif

      void processUIQueue ();
      void processMidi ();

      void drawRainbowRoad ();
      void drawTestState ();

      void testSDCard ();

      AppImpl (const AppImpl &a)           = delete;
      AppImpl &operator=(const AppImpl &a) = delete;
  };
}    // namespace spotykach_hwtest

using namespace spotykach_hwtest;

static AppImpl impl;

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

static void AudioCallback (AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
  impl.ProcessAudio(in, out, size);
}

void AppImpl::Init ()
{
  hw.Init(48000, 16);
  hw.StartAdcs();

#if DEBUG
  Log::StartLog(false);
  log_timer.Init();
#endif

  led_timer.Init();
  midi_timer.Init();

  test_note_on = false;

  pot_monitor.Init(ui_queue, hw, 500, 0.005f, 0.002f);
  last_pot_moved_a = 0;
  last_pot_moved_b = 0;

  for (size_t i = 0; i < 8; i++)
  {
    osc[i].Init(48000);
    osc[i].SetWaveform(daisysp::Oscillator::WAVE_POLYBLEP_TRI);
    osc[i].SetAmp(0.25f);
  }

  // Uncomment this if you want to test the SD card -
  // this takes about 10s and is blocking
  // testSDCard();

  hw.seed.usb_handle.SetReceiveCallback(UsbCallback, UsbHandle::FS_EXTERNAL);

  auto &audio = hw.seed.audio_handle;
  audio.SetSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
  audio.SetBlockSize(16);
  audio.Start(AudioCallback);
}

void AppImpl::Loop ()
{
  float cvphase = 0.0f;

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
      hw.SetGateOutA(test_note_on);
      hw.SetGateOutB(test_note_on);
    }

    // Modified libDaisy MIDI handlers require explicit call to transmit
    // enqueued messages instead of blocking every time a message is sent
    hw.midi_uart.TransmitEnqueuedMessages();

    // The LED refresh should run at least 200Hz for temporal dithering,
    // but faster is better
    if (led_timer.HasPassedMs(2))
    {
      led_timer.Restart();
      hw.leds.Clear();

      // uncomment this and comment out drawTestState()
      // for rainbow cycle on all LEDs
      // drawRainbowRoad();
      drawTestState();
      hw.leds.Show();

      // Piggy back on this timer for very rough CV output demo
      cvphase += 1.0f / 500.0f;
      cvphase -= floorf(cvphase);
      float cv = sinf(cvphase * 2.0f * M_PI) * 0.5f + 0.5f;
      hw.WriteCVOutA(cv);
      hw.WriteCVOutB(cv);
    }
#if DEBUG
    if (log_timer.HasPassedMs(250))
    {
      logDebugInfo();
      log_timer.Restart();
    }
#endif
  }
}

void AppImpl::ProcessAudio (AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
  hw.ProcessAnalogControls();

  std::copy(IN_L, IN_L + size, OUT_L);
  std::copy(IN_R, IN_R + size, OUT_R);

  // Add test oscillator from MIDI input
  float s;
  for (size_t i = 0; i < size; i++)
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
        System::Delay(500);    // Give time for the log to flush
        System::ResetToBootloader(System::BootloaderMode::DAISY_INFINITE_TIMEOUT);
      }
      usbBuffIx = 0;    // Reset after processing
    }
  }
}
#endif

// This is only used for detecting pot changes for the test LED patterns,
// the PotMonitor + queue makes this easy
void AppImpl::processUIQueue ()
{
  while (!ui_queue.IsQueueEmpty())
  {
    auto event = ui_queue.GetAndRemoveNextEvent();
    if (event.type == UiEventQueue::Event::EventType::potMoved)
    {
      if (event.asPotMoved.id <= Hardware::CTRL_ENV_A)
        last_pot_moved_a = event.asPotMoved.id;
      else if (event.asPotMoved.id <= Hardware::CTRL_ENV_B)
        last_pot_moved_b = event.asPotMoved.id;
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

  for (size_t i = 0; i < spotykach::Hardware::kNumLeds; i++)
  {
    float             phs = i / 32.0f;
    float             h   = daisysp::fastmod1f(t * 0.2f + phs);
    infrasonic::Color c   = infrasonic::Color::FromHSV(h * 255, 255, 255);
    hw.leds.Set(i, c.Hex(), 0.5f);
  }
}

void AppImpl::drawTestState ()
{
  // --- Gate I/O ---
  if (hw.GetClockInputState())
  {
    hw.leds.Set(Hardware::LED_CLOCK_IN, 0xff0000, 1.0f);
  }
  if (hw.GetGateInputAState())
  {
    hw.leds.Set(Hardware::LED_GATE_OUT_A, 0xff0000, 1.0f);
  }
  if (hw.GetGateInputBState())
  {
    hw.leds.Set(Hardware::LED_GATE_OUT_B, 0xff0000, 1.0f);
  }

  // --- Switches (Shift registers) ---

  // construct into 8-bit set from inverted bitmask state
  // (all inputs are inverted due to pullups)
  std::bitset<8> sr1 = ~hw.GetShiftRegState(0);
  std::bitset<8> sr2 = ~hw.GetShiftRegState(1);

  // Mode A/B/C switch
  if (sr1.test(2))
  {
    hw.leds.Set(Hardware::LED_MODE_RIGHT, 0xff0000, 1.0f);
  }
  else if (sr1.test(3))
  {
    hw.leds.Set(Hardware::LED_MODE_LEFT, 0xff0000, 1.0f);
  }
  else
  {
    hw.leds.Set(Hardware::LED_MODE_CENTER, 0xff0000, 1.0f);
  }

  // Mod A Type switch
  if (sr1.test(0))
  {
    hw.leds.Set(Hardware::LED_CYCLE_A, 0x00ff00, 1.0f);
  }
  else if (sr1.test(1))
  {
    hw.leds.Set(Hardware::LED_CYCLE_A, 0x0000ff, 1.0f);
  }
  else
  {
    hw.leds.Set(Hardware::LED_CYCLE_A, 0xff0000, 1.0f);
  }

  // Mode A switch
  if (sr1.test(6))
  {
    hw.leds.Set(Hardware::LED_ORBIT_A, 0x00ff00, 1.0f);
  }
  else if (sr1.test(7))
  {
    hw.leds.Set(Hardware::LED_ORBIT_A, 0x0000ff, 1.0f);
  }
  else
  {
    hw.leds.Set(Hardware::LED_ORBIT_A, 0xff0000, 1.0f);
  }

  // Size/Pos A switch
  if (sr1.test(4))
  {
    hw.leds.Set(Hardware::LED_REV_A, 0x00ff00, 1.0f);
  }
  else if (sr1.test(5))
  {
    hw.leds.Set(Hardware::LED_REV_A, 0x0000ff, 1.0f);
  }
  else
  {
    hw.leds.Set(Hardware::LED_REV_A, 0xff0000, 1.0f);
  }

  // Mod B Type switch
  if (sr2.test(4))
  {
    hw.leds.Set(Hardware::LED_CYCLE_B, 0x00ff00, 1.0f);
  }
  else if (sr2.test(5))
  {
    hw.leds.Set(Hardware::LED_CYCLE_B, 0x0000ff, 1.0f);
  }
  else
  {
    hw.leds.Set(Hardware::LED_CYCLE_B, 0xff0000, 1.0f);
  }

  // Mode B switch
  if (sr2.test(2))
  {
    hw.leds.Set(Hardware::LED_ORBIT_B, 0x00ff00, 1.0f);
  }
  else if (sr2.test(3))
  {
    hw.leds.Set(Hardware::LED_ORBIT_B, 0x0000ff, 1.0f);
  }
  else
  {
    hw.leds.Set(Hardware::LED_ORBIT_B, 0xff0000, 1.0f);
  }

  // Size/Pos B switch
  if (sr2.test(0))
  {
    hw.leds.Set(Hardware::LED_REV_B, 0x00ff00, 1.0f);
  }
  else if (sr2.test(1))
  {
    hw.leds.Set(Hardware::LED_REV_B, 0x0000ff, 1.0f);
  }
  else
  {
    hw.leds.Set(Hardware::LED_REV_B, 0xff0000, 1.0f);
  }

  // Manual tempo tap switch
  if (sr2.test(6))
  {
    hw.leds.Set(Hardware::LED_CLOCK_IN, 0xffffff, 1.0f);
  }

  // LED RINGS / POTS
  const uint16_t ring_val_a =
    hw.GetAnalogControlValue((Hardware::AnalogControlId)last_pot_moved_a) * Hardware::kNumLedsPerRing;
  const uint16_t ring_val_b =
    hw.GetAnalogControlValue((Hardware::AnalogControlId)last_pot_moved_b) * Hardware::kNumLedsPerRing;
  for (uint16_t i = 0; i < Hardware::kNumLedsPerRing; i++)
  {
    // start at bottom, wrap clockwise
    uint16_t ledidx_a = Hardware::LED_RING_A;
    uint16_t ledidx_b = Hardware::LED_RING_B;
    // = is_on_left ? Hardware::LED_RING_A : Hardware::LED_RING_B;
    if (i <= 16)
    {
      ledidx_a += (Hardware::kNumLedsPerRing / 2) - i;
      ledidx_b += (Hardware::kNumLedsPerRing / 2) - i;
    }
    else
    {
      ledidx_a += Hardware::kNumLedsPerRing - 1;
      ledidx_a -= (i - (Hardware::kNumLedsPerRing / 2 + 1));

      ledidx_b += Hardware::kNumLedsPerRing - 1;
      ledidx_b -= (i - (Hardware::kNumLedsPerRing / 2 + 1));
    }
    if (ring_val_a > i)
    {
      hw.leds.Set(ledidx_a, 0x00ff00, 0.5f);
    }
    if (ring_val_b > i)
    {
      hw.leds.Set(ledidx_b, 0x00ff00, 0.5f);
    }
  }

  // Spotykach Slider
  float skval = (hw.GetAnalogControlValue(Hardware::CTRL_SPOTYKACH) * 2.0f - 1.0f) +
                hw.GetControlVoltageValue(Hardware::CV_SPOTYKACH);
  skval = daisysp::fclamp(skval, -1.0f, 1.0f);
  hw.leds.Set(Hardware::LED_SPOTY_SLIDER_B, 0xff0000, daisysp::fmax(skval, 0.0f));
  hw.leds.Set(Hardware::LED_SPOTY_SLIDER_A, 0x0000ff, daisysp::fmax(-skval, 0.0f));

  // --- CV INPUTS ---

  // For these we just add together the 3 CVs on each side and
  // render to drift LEDs
  float cv_a = hw.GetControlVoltageValue(Hardware::CV_SOS_IN_A);
  cv_a += hw.GetControlVoltageValue(Hardware::CV_V_OCT_A);
  cv_a += hw.GetControlVoltageValue(Hardware::CV_SIZE_POS_A);
  hw.leds.Set(Hardware::LED_DRIFT_A, cv_a >= 0.0f ? 0xff0000 : 0x0000ff, fabsf(cv_a));

  float cv_b = hw.GetControlVoltageValue(Hardware::CV_SOS_IN_B);
  cv_b += hw.GetControlVoltageValue(Hardware::CV_V_OCT_B);
  cv_b += hw.GetControlVoltageValue(Hardware::CV_SIZE_POS_B);
  hw.leds.Set(Hardware::LED_DRIFT_B, cv_b >= 0.0f ? 0xff0000 : 0x0000ff, fabsf(cv_b));

  // --- MIDI INPUT ---
  if (midi_in_note_on)
    hw.leds.Set(Hardware::LED_SPOTY_PAD, 0xff0000);

  // --- TOUCH PADS ---

  // These will override the corresponding LED of the touchpad with WHITE if the pad
  // is being pressed, otherwise default behavior from above
  std::bitset<16> touch = hw.GetMpr121TouchStates();

  // Mapping from pad bit position (electrode) to LED
  // for illuminating on touch for test purposes
  // Bit position acts as index into array
  constexpr Hardware::LedId kPadMapping[12] = {Hardware::LED_PLAY_A,
                                               Hardware::LED_REV_A,
                                               Hardware::LED_ORBIT_A,
                                               Hardware::LED_DRIFT_A,
                                               Hardware::LED_CYCLE_A,
                                               Hardware::LED_CYCLE_B,
                                               Hardware::LED_DRIFT_B,
                                               Hardware::LED_ORBIT_B,
                                               Hardware::LED_REV_B,
                                               Hardware::LED_PLAY_B,
                                               Hardware::LED_SPOTY_PAD,
                                               Hardware::LED_ALT_A};
  for (uint16_t i = 0; i < 12; i++)
  {
    if (touch.test(i))
    {
      hw.leds.Set(kPadMapping[i], 0xffffff);
      if (i == 11)
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
    audiodata[i] = rand() & 255;
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
      f_write(&SDFile, &audiodata[offset], len, (UINT *)&byteswritten);
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
      res = f_read(&SDFile, &audiodata[offset], 32768, (UINT *)&bytesread);
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
  impl.Init();
}

void Application::Loop ()
{
  impl.Loop();
}
