#pragma once

namespace spotykach_hwtest
{
  // Definitions
  #define PRINT_CPU_LOAD

  // Constants
  constexpr size_t kSampleRate           = 48000;
  constexpr size_t kBlockSize            = 16;

  constexpr size_t kNumberSpotykachSides = 1;
  constexpr size_t kNumberChannelsStereo = 2;
  constexpr size_t kNumberChannelsMono   = 1;

  // Application class
  class Application
  {
    public:
      Application ()  = default;
      ~Application () = default;

      void Init ();
      void Loop ();

    private:
      Application (const Application &a)           = delete;
      Application &operator=(const Application &a) = delete;
  };
}    // namespace spotykach_hwtest
