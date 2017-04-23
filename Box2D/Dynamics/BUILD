package(default_visibility=["//visibility:public"])

cc_library(
    name = "interface",
    hdrs = [
        "b2TimeStep.h",
        "b2World.h",
        "b2WorldCallbacks.h",
        "b2ContactManager.h",
        "b2Body.h",
        "b2Fixture.h",
    ],
    deps = [
        "//Box2D/Common:Common",
        "//Box2D/Particle:interface",
    ],
)
    

cc_library(
    name = "Dynamics",
    hdrs = [
        "b2Island.h",
        "Joints/b2RopeJoint.h",
        "Joints/b2MotorJoint.h",
        "Joints/b2WeldJoint.h",
        "Joints/b2GearJoint.h",
        "Joints/b2PrismaticJoint.h",
        "Joints/b2DistanceJoint.h",
        "Joints/b2FrictionJoint.h",
        "Joints/b2Joint.h",
        "Joints/b2RevoluteJoint.h",
        "Joints/b2WheelJoint.h",
        "Joints/b2MouseJoint.h",
        "Joints/b2PulleyJoint.h",
        "Contacts/b2ChainAndPolygonContact.h",
        "Contacts/b2EdgeAndPolygonContact.h",
        "Contacts/b2PolygonAndCircleContact.h",
        "Contacts/b2EdgeAndCircleContact.h",
        "Contacts/b2ContactSolver.h",
        "Contacts/b2CircleContact.h",
        "Contacts/b2PolygonContact.h",
        "Contacts/b2ChainAndCircleContact.h",
        "Contacts/b2Contact.h",

    ],
    srcs = [
        "b2Body.cpp",
        "b2ContactManager.cpp",
        "b2Fixture.cpp",
        "b2WorldCallbacks.cpp",
        "b2Island.cpp",
        "b2World.cpp",
        "Joints/b2PrismaticJoint.cpp",
        "Joints/b2FrictionJoint.cpp",
        "Joints/b2DistanceJoint.cpp",
        "Joints/b2MouseJoint.cpp",
        "Joints/b2Joint.cpp",
        "Joints/b2GearJoint.cpp",
        "Joints/b2MotorJoint.cpp",
        "Joints/b2WeldJoint.cpp",
        "Joints/b2WheelJoint.cpp",
        "Joints/b2RevoluteJoint.cpp",
        "Joints/b2RopeJoint.cpp",
        "Joints/b2PulleyJoint.cpp",
        "Contacts/b2EdgeAndCircleContact.cpp",
        "Contacts/b2CircleContact.cpp",
        "Contacts/b2PolygonAndCircleContact.cpp",
        "Contacts/b2ContactSolver.cpp",
        "Contacts/b2ChainAndPolygonContact.cpp",
        "Contacts/b2EdgeAndPolygonContact.cpp",
        "Contacts/b2Contact.cpp",
        "Contacts/b2ChainAndCircleContact.cpp",
        "Contacts/b2PolygonContact.cpp",

    ],
    deps = [
        ":interface",
        "//Box2D/Common:Common",
        "//Box2D/Common:Draw",
        "//Box2D/Collision:Collision",
    ],
)




