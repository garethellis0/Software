load("@//bazel:message_generation.bzl", "generate_messages")

# Implicitly creates msgs_py and msg_cc targets in the current scope
generate_messages(
    srcs = [
        "msg/RobotState.msg",
        "msg/WheelSpeeds.msg",
    ],
    ros_package_name = "mpc_messages",
    deps = [
        "@std_msgs//:msgs",
    ],
)

cc_binary(
    name = "main_cc",
    srcs = ["main.cpp"],
    defines = [
        "_LIBCPP_ENABLE_CXX17_REMOVED_AUTO_PTR",
    ],
    deps = [
        ":msgs_cc",
        "//software/backend",
        "@roscpp",
    ],
)
