cc_binary(
    name = "main_cc",
    srcs = ["main.cpp"],
    deps = [
        "//software/backend",
        "//software/backend:radio_backend",
        "//software/backend:grsim_backend",
        "//software/ai/primitive:direct_wheels_primitive",
        "//software/ai/primitive:move_primitive",
        "@boost//:interprocess",
        "//software/logger",
        "@g3log",
    ],
)
