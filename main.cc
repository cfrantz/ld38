#include <cstdio>
#include <string>
#include <gflags/gflags.h>

#include "app.h"
#include "util/config.h"


const char kUsage[] =
R"ZZZ(<optional flags>

Description:
  Angry Volcano God

Flags:
  --config <filename> Use an alternate config file.
  --hidpi <n>         Set the scaling factor on hidpi displays (try 2.0).
)ZZZ";

int main(int argc, char *argv[]) {
    gflags::SetUsageMessage(kUsage);
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    ld38::VolcanoApp app("Angry Volcano God");
    app.Init();
    app.Run();
    return 0;
}
