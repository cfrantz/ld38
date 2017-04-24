#ifndef Z2UTIL_APP_H
#define Z2UTIL_APP_H
#include <memory>
#include <string>

#include "game/volcano.h"
#include "imwidget/imapp.h"
#include "imwidget/glbitmap.h"
#include "imwidget/imwidget.h"

namespace ld38 {

class VolcanoApp: public ImApp {
  public:
    VolcanoApp(const std::string& name) : ImApp(name, 1280, 720, false) {}
    ~VolcanoApp() override {}

    void Init() override;
    void ProcessEvent(SDL_Event* event) override;
    void Draw() override;
    void GLDraw() override;

  private:
    void Query(DebugConsole* console, int argc, char **argv);
    bool loaded_;
    bool started_;
    std::unique_ptr<Volcano> game_;
    std::unique_ptr<GLBitmap> title_;
    ImVec2 center_;
    ImVec2 extent_;
};

}  // namespace avg
#endif // Z2UTIL_APP_H
