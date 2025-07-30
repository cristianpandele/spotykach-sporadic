#include "app.h"
#include <daisy.h>

static spotykach_hwtest::Application app;

extern "C" int main (void)
{
  app.Init();
  app.Loop();
}
