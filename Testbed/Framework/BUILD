package(default_visibility=["//visibility:public"])

cc_library(
    name = "Framework",
    hdrs = [
        "Arrow.h",
        "FullscreenUI.h",
        "Main.h",
        "ParticleEmitter.h",
        "ParticleParameter.h",
        "Render.h",
        "Test.h",
    ],
    deps = [
        "//Box2D:Box2D",
    ],
)

cc_binary(
    name = "demo",
    srcs = [
        "Arrow.cpp",
        "FullscreenUI.cpp",
        "Main.cpp",
        "ParticleParameter.cpp",
        "Render.cpp",
        "Test.cpp",
    ],
    deps = [
        ":Framework",
        "//Testbed/Tests:Tests",
        "//Box2D:Box2D",
        "//freeglut:freeglut",
        "//glui:glui",
    ],
)
