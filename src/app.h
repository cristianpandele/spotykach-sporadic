#pragma once

namespace spotykach_hwtest
{
  // Definitions
  #define PRINT_CPU_LOAD

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
