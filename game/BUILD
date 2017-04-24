package(default_visibility=["//visibility:public"])

cc_library(
    name = "render",
    hdrs = ["render.h"],
    srcs = ["render.cc"],
    deps = [
        "//Box2D:Box2D",
        "//external:imgui",
    ],
)

cc_library(
    name = "volcano",
    hdrs = ["volcano.h"],
    srcs = [
        "ParticleEmitter.h",
        "volcano.cc"
    ],
    deps = [
        ":render",
        "//Box2D:Box2D",
        "//external:gflags",
    ],
)
