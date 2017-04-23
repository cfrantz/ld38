package(default_visibility=["//visibility:public"])

cc_library(
    name = "Common",
    hdrs = [
        "b2BlockAllocator.h",
        "b2FreeList.h",
        "b2GrowableBuffer.h",
        "b2GrowableStack.h",
        "b2IntrusiveList.h",
        "b2Math.h",
        "b2Settings.h",
        "b2SlabAllocator.h",
        "b2StackAllocator.h",
        "b2Stat.h",
        "b2Timer.h",
        "b2TrackedBlock.h",
    ],
    srcs = [
        "b2BlockAllocator.cpp",
        "b2FreeList.cpp",
        "b2Math.cpp",
        "b2Settings.cpp",
        "b2StackAllocator.cpp",
        "b2Stat.cpp",
        "b2Timer.cpp",
        "b2TrackedBlock.cpp",
    ],
    deps = [
    ],
)

cc_library(
    name = "Draw",
    hdrs = [
        "b2Draw.h",
    ],
    srcs = [
        "b2Draw.cpp",
    ],
    deps = [
        ":Common",
        "//Box2D/Particle:interface",
    ],
)
