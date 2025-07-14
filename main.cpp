#include <daisy.h>
#include "app.h"

static spotykach_hwtest::Application app;

extern "C" int main(void)
{
    app.Init();
    app.Loop();
}
