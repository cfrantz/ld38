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
    srcs = [
        "Arrow.cpp",
        "FullscreenUI.cpp",
        "ParticleParameter.cpp",
        "Render.cpp",
        "Test.cpp",
    ],
    deps = [
        "//Box2D:Box2D",
        "//freeglut:freeglut",
        "//glui:glui",
    ],
)

filegroup(
    name = "main",
    srcs = ["Main.cpp"],
)

cc_binary(
    name = "demo",
    srcs = [":main"],
    deps = [
        ":Framework",
        "//Testbed/Tests:Tests",
    ],
)
