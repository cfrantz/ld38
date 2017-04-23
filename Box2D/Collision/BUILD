package(default_visibility=["//visibility:public"])

cc_library(
    name = "Collision",
    hdrs = [
        "b2BroadPhase.h",
        "b2Collision.h",
        "b2Distance.h",
        "b2DynamicTree.h",
        "b2TimeOfImpact.h",
        "Shapes/b2ChainShape.h",
        "Shapes/b2CircleShape.h",
        "Shapes/b2EdgeShape.h",
        "Shapes/b2PolygonShape.h",
        "Shapes/b2Shape.h",
    ],
    srcs = [
        "b2BroadPhase.cpp",
        "b2Collision.cpp",
        "b2CollideCircle.cpp",
        "b2CollideEdge.cpp",
        "b2CollidePolygon.cpp",
        "b2Distance.cpp",
        "b2DynamicTree.cpp",
        "b2TimeOfImpact.cpp",
        "Shapes/b2ChainShape.cpp",
        "Shapes/b2CircleShape.cpp",
        "Shapes/b2EdgeShape.cpp",
        "Shapes/b2PolygonShape.cpp",
    ],
    deps = [
        "//Box2D/Common:Common",
    ],
)



