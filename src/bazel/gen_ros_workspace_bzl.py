#!/usr/bin/env python

"""
Parses the CMake files in a ROS installation directory (eg /opt/ros/melodic)
and produces bazel files to import each catkin package with the right
dependency information.
"""

import argparse
import glob
import os
import re

def _read_file_lines(fp):
    with open(fp) as f:
        lines = [l.strip() for l in f.readlines()]
    return [l for l in lines if l]

def _get_list_var_by_name(lines, var_name):
    for l in lines:
        if l.startswith("set({} ".format(var_name)):
            tag_contents = l.split('"')[1]
            return tag_contents.split(";")
    return []

def _format_str_list_for_build_file(lst, indent=8):
    out = ""
    lst.sort()
    for l in lst:
        out += " " * indent + '"{}",\n'.format(l)
    # Strip final newline
    if out:
        out = out[:-1]
    return out

def parse_cmake_config_file(fp, ros_path):
    package_name = os.path.basename(fp).replace("Config.cmake", "")

    config_lines = _read_file_lines(fp)

    depends = _get_list_var_by_name(config_lines, "depends")
    depends = [d for d in depends if d]

    libraries = _get_list_var_by_name(config_lines, "libraries")
    libraries = [l for l in libraries if l]

    blacklisted_libs = {"pthread"}
    sanitized_libs = [
        "lib/lib{}.so".format(lib)
        for lib in libraries
        # We don't care about system libraries starting with "/", they'll be dragged in
        # by the linker anyway.
        if not lib.startswith("/") and not lib in blacklisted_libs
    ]

    # Also parse extras file if it exists (has the list of .msg files to
    # generate).
    msg_extras_path = os.path.join(os.path.dirname(fp), os.path.basename(fp).replace("Config.cmake", "-msg-extras.cmake"))
    if os.path.exists(msg_extras_path):
        msg_extras_lines = _read_file_lines(msg_extras_path)
        message_files = _get_list_var_by_name(msg_extras_lines, package_name + "_MESSAGE_FILES")

        # Also open file with message dependencies
        # Unlike the others, this file has not quotes, so we need custom parsing code.
        msg_paths_path = msg_extras_path.replace("-msg-extras.cmake", "-msg-paths.cmake")
        msg_paths_lines = _read_file_lines(msg_paths_path)
        for l in msg_paths_lines:
            if package_name + "_MSG_DEPENDENCIES" in l:
                message_deps = l[l.find(" ")+1:l.find(")")].split(";")
        message_deps = [m for m in message_deps if m]
    else:
        message_files = []
        message_deps = []

    build_file_contents = """\
# Autogenerated by {}\n# from {}

package(default_visibility = ["//visibility:public"])
""".format(os.path.basename(__file__), fp)

    if message_files:
        build_file_contents += """
load("@//bazel:message_generation.bzl", "generate_messages")
"""

    build_file_contents += """
cc_library(
    name = "{name}",
    srcs = [
{srcs}
    ],
    deps = [
{deps}
    ],
    includes = [
        "include",
    ],
    hdrs = glob([
        "include/**",
    ]),
)\n""".format(
        name=package_name,
        deps=_format_str_list_for_build_file([
            "@" + dep_name for dep_name in depends
        ] + ([":msgs_cc"] if message_files else [])),
        srcs=_format_str_list_for_build_file(sanitized_libs),
    )

    # Add message generation if needed
    if message_files:
        build_file_contents += """
generate_messages(
    ros_package_name = "{name}",
    srcs = [
{srcs}
    ],
    deps = [
{deps}
    ],
)""".format(
        name=package_name,
        srcs=_format_str_list_for_build_file([
            os.path.join("share", package_name, p)
            for p in message_files
        ]),
        deps=_format_str_list_for_build_file([
            "@{}//:msgs".format(p)
            for p in message_deps
        ]),
    )

    if not build_file_contents.endswith("\n"):
        build_file_contents += "\n"

    return package_name, build_file_contents


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ws-name", required=True)
    parser.add_argument("--ros-path", required=True)
    parser.add_argument("--out-dir", required=True)
    args = parser.parse_args()

    cmake_config_files = list(sorted(glob.glob(os.path.join(args.ros_path, "share/*/cmake/*Config.cmake"))))
    assert cmake_config_files

    if not os.path.exists(args.out_dir):
        os.makedirs(args.out_dir)

    package_names = []

    for ccf in cmake_config_files:
        package_name, build_file_contents = parse_cmake_config_file(ccf, args.ros_path)
        build_file_path = os.path.join(args.out_dir, "{}.BUILD".format(package_name))
        with open(build_file_path, "w") as f:
            f.write(build_file_contents)
        package_names.append(package_name)

    # Write workspace.bzl
    workspace_contents = """\
# Autogenerated by {}
""".format(os.path.basename(__file__))

    workspace_contents += """
def ros_repositories():
"""

    for p in package_names:
        workspace_contents += """
    native.new_local_repository(
        name = "{name}",
        path = "{path}",
        build_file = "@{ws_name}//:{name}.BUILD",
    )\n""".format(
        name=p,
        path=args.ros_path,
        ws_name=args.ws_name,
    )

    with open(os.path.join(args.out_dir, "workspace.bzl"), "w") as f:
        f.write(workspace_contents)

if __name__ == "__main__":
    main()
