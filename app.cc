#include <cstdio>

#include <gflags/gflags.h>
#include "app.h"
#include "imgui.h"
#include "imwidget/error_dialog.h"
#include "util/browser.h"
#include "util/config.h"
#include "util/os.h"
#include "util/logging.h"
#include "util/imgui_impl_sdl.h"

#include "version.h"

#ifdef HAVE_NFD
#include "nfd.h"
#endif


DECLARE_string(config);

namespace ld38 {

void VolcanoApp::Init() {
    loaded_ = false;
    started_ = false;
    game_.reset(new Volcano);
    title_.reset(new GLBitmap);
    title_->Load("content/avg1.bmp");
    //center_ = ImVec2(0.0f, 90.0f);
    //extent_ = ImVec2(100.0f, 100.0f);
    center_ = ImVec2(0.0f, 30.0f);
    extent_ = ImVec2(35.0f, 35.0f);
    RegisterCommand("q", "Query an area", this, &VolcanoApp::Query);
}


void VolcanoApp::ProcessEvent(SDL_Event* event) {
    switch(event->type) {
    case SDL_KEYUP:
        if (event->key.keysym.scancode == SDL_SCANCODE_R) {
            game_->Init();
            started_ = true;
        } else if (event->key.keysym.scancode == SDL_SCANCODE_UP) {
            center_.y += 10.0f;
        } else if (event->key.keysym.scancode == SDL_SCANCODE_DOWN) {
            center_.y -= 10.0f;
        } else if (event->key.keysym.scancode == SDL_SCANCODE_LEFT) {
            center_.x += 10.0f;
        } else if (event->key.keysym.scancode == SDL_SCANCODE_RIGHT) {
            center_.x -= 10.0f;
        }
        printf("(%f,%f)+(%f,%f)\n", center_.x, center_.y, extent_.x, extent_.y);
        break;
    case SDL_MOUSEWHEEL:
        if (event->wheel.y < 0) {
            extent_.x *= 1.1f;
            extent_.y *= 1.1f;
        } else if (event->wheel.y > 0) {
            extent_.x /= 1.1f;
            extent_.y /= 1.1f;
        }
        printf("(%f,%f)+(%f,%f)\n", center_.x, center_.y, extent_.x, extent_.y);
        break;
    }
    if (started_) game_->HandleEvent(event);
}

void VolcanoApp::GLDraw() {
    ImVec4 clear_color = ImColor(0, 0, 0);
    glViewport(0, 0,
               (int)ImGui::GetIO().DisplaySize.x,
               (int)ImGui::GetIO().DisplaySize.y);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(center_.x - extent_.x, center_.x + extent_.x,
            center_.y - extent_.y, center_.y + extent_.y, 0.0f, 1.0f);
    LoadOrtho2DMatrix(center_.x - extent_.x, center_.x + extent_.x,
                      center_.y - extent_.y, center_.y + extent_.y);
    glMatrixMode(GL_MODELVIEW);
    glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
    glClear(GL_COLOR_BUFFER_BIT);
    if (started_) {
        game_->Step(1.0 / 60.0);
    } else {
        title_->DrawRaw(center_.x - extent_.x, 
                        center_.y + extent_.y,
                        extent_.x * 2.0, -extent_.y * 2.0);
    }
}

void VolcanoApp::Draw() {
    ImGui::SetNextWindowSize(ImVec2(500,300), ImGuiSetCond_FirstUseEver);
    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("File")) {
#ifdef HAVE_NFD
#endif
            ImGui::Separator();
            if (!FLAGS_config.empty()) {
                if (ImGui::MenuItem("Reload Config")) {
//                    auto* config = ConfigLoader<avg::RomInfo>::Get();
//                    config->Reload();
                }
            }
            if (ImGui::MenuItem("Quit")) {
                running_ = false;
            }
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Edit")) {
            ImGui::MenuItem("Debug Console", nullptr,
                            &console_.visible());
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("View")) {
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Help")) {
            if (ImGui::MenuItem("Online Help")) {
                Help("root");
            }
            if (ImGui::MenuItem("About")) {
                ErrorDialog::Spawn("About VolcanoApp",
                    "Angry Volcano God\n\n",
#ifdef BUILD_GIT_VERSION
                    "Version: ", BUILD_GIT_VERSION, "-", BUILD_SCM_STATUS
#else
                    "Version: Unknown"
#warning "Built without version stamp"
#endif
                    );

            }
            ImGui::EndMenu();
        }
        ImGui::EndMainMenuBar();
    }
}

void VolcanoApp::Query(DebugConsole* console, int argc, char **argv) {
    float x = strtod(argv[1], 0);
    float y = strtod(argv[2], 0);
    float r = strtod(argv[3], 0);

    game_->Coalesce(b2Vec2(x, y), r);
}


}  // namespace avg
