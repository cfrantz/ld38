package(default_visibility=["//visibility:public"])

cc_library(
    name = "interface",
    hdrs = [
        "b2Particle.h",
        "b2ParticleSystem.h",
    ],
    deps = [
        "//Box2D/Common:Common",
    ],
)

cc_library(
    name = "Particle",
    hdrs = [
        "b2ParticleAssembly.h",
        "b2ParticleGroup.h",
        "b2StackQueue.h",
        "b2VoronoiDiagram.h",
    ],
    srcs = [
        "b2ParticleAssembly.cpp",
        "b2Particle.cpp",
        "b2ParticleGroup.cpp",
        "b2ParticleSystem.cpp",
        "b2VoronoiDiagram.cpp",
    ],
    deps = [
        ":interface",
        "//Box2D/Common:Common",
        "//Box2D/Common:Draw",
        "//Box2D/Collision:Collision",
        "//Box2D/Dynamics:interface",
    ],
)
