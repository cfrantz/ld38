#include <cstdio>
#include <string>
#include <gflags/gflags.h>

#include "app.h"
//#include "proto/config.pb.h"
#include "util/config.h"

DEFINE_string(config, "", "config file");
DEFINE_bool(dump_config, false, "Dump config to stdout and exit");

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


    /*
    auto* config = ConfigLoader<fdgconfig::Graph>::Get();
    if (!FLAGS_config.empty()) {
        config->Load(FLAGS_config);
    } else {
    }
    if (FLAGS_dump_config) {
        puts(config->config().DebugString().c_str());
        exit(0);
    }
    */

    ld38::VolcanoApp app("Angry Volcano God");
    app.Init();

    /*
    if (argc > 1) {
        app.Load(argv[1]);
    }
    */
    app.Run();
    return 0;
}
