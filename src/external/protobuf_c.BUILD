# Description:
#   This library provides us with a 2D physics engine
#   https://github.com/erincatto/Box2D
#   https://box2d.org/

cc_library(
    name = "protobuf",
    srcs = [
        "protobuf-c/protobuf-c.c",
    ],
    hdrs = [
        "protobuf-c/protobuf-c.h",
    ],
    includes = [
        "protobuf-c/",
    ],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "protoc",
    srcs = ["protoc-c/main.cc"],
    deps = [":protoc_lib"],
    includes = [
        "protoc-c",
    ],
)

cc_library(
    name = "protoc_lib",
    srcs = glob(["protoc-c/*.cc"], exclude = ["protoc-c/main.cc"]),
    hdrs = glob(["protoc-c/*.h"]),
    includes = [
        "protoc-c",
    ],
)

# TODO
#cc_binary(
#    name = "protoc"
#)
