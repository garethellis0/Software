load("@bazel_tools//tools/cpp:toolchain_utils.bzl", "find_cpp_toolchain")

# TODO: remove all debug print statements

# TODO: Do we need this?
MyCCompileInfo = provider(doc = "", fields = ["object"])

# TODO: comment here
SUPPORTED_PROCESSORS = [
    "STM32H743xx",
    "STM32H753xx",
    "STM32H750xx",
    "STM32H742xx",
    "STM32H745xx",
    "STM32H755xx",
    "STM32H747xx",
    "STM32H757xx",
]

# We got this URL by running wireshark and watching what STM32CubeMX tried to download
STM32_H7_DRIVERS_AND_MIDDLEWARE_DOWNLOAD_URL = \
    "http://www.ebuc23.com/s3/stm_test/software/firmware/stm32cube_fw_h7_v150.zip"

# TODO: comment here
def _filter_none(input_list):
    filtered_list = []
    for element in input_list:
        if element != None:
            filtered_list.append(element)
    return filtered_list

""" Rule to Build A HAL Library With Specific Configuration """

def _cc_stm32h7_hal_library_impl(ctx):
    # Begin by copying all the requested files into the current directory
    # TODO: better comment here

    # TODO: rename "fw" to firmware..........

    fw_zip_name = "fw.zip"
    fw_zip = ctx.actions.declare_file(fw_zip_name)
    ctx.actions.run_shell(
        outputs = [fw_zip],
        command = """
        curl -L {} -o {}
        """.format(STM32_H7_DRIVERS_AND_MIDDLEWARE_DOWNLOAD_URL, fw_zip.path),
        progress_message = "Downloading firmware from {}".format(STM32_H7_DRIVERS_AND_MIDDLEWARE_DOWNLOAD_URL),
    )

    # TODO: unzipping should be with downloading above, or in it's own step, to minimize the number of time it has to be run?

    # TODO: the whole "external_deps" thing is a _bit_ convoluted, we can maybe simplify things a bit?
    drivers_and_middleware_hdrs = [
        ctx.actions.declare_file("external_deps/{}".format(filename))
        for filename in ctx.attr.drivers_and_middleware_hdrs
    ]
    drivers_and_middleware_srcs = [
        ctx.actions.declare_file("external_deps/{}".format(filename))
        for filename in ctx.attr.drivers_and_middleware_srcs
    ]

    external_deps_folder = ctx.actions.declare_directory("external_deps")
    print(external_deps_folder.path)
    ctx.actions.run_shell(
        inputs = [fw_zip],
        outputs = [external_deps_folder] + drivers_and_middleware_hdrs + drivers_and_middleware_srcs,
        command = """
        unzip -qo {} 'STM32Cube_FW_H7_V1.5.0/Drivers/*' 'STM32Cube_FW_H7_V1.5.0/Middlewares/*'
        rm -rf {external_deps_path}
        mv STM32Cube_FW_H7_V1.5.0 {external_deps_path}
        """.format(fw_zip.path, external_deps_path = external_deps_folder.path),
        progress_message = "Unzipping firmware",
    )

    cc_toolchain = find_cpp_toolchain(ctx)
    feature_configuration = cc_common.configure_features(
        ctx = ctx,
        cc_toolchain = cc_toolchain,
        requested_features = ctx.features,
        unsupported_features = ctx.disabled_features,
    )
    compilation_contexts = []
    linking_contexts = []

    hal_config_hdr_includes = [file.dirname for file in ctx.files.hal_config_hdrs]
    drivers_and_middleware_includes = [file.dirname for file in drivers_and_middleware_hdrs]

    (compilation_context, compilation_outputs) = cc_common.compile(
        name = ctx.label.name,
        actions = ctx.actions,
        feature_configuration = feature_configuration,
        cc_toolchain = cc_toolchain,
        public_hdrs =
            drivers_and_middleware_hdrs +
            ctx.files.hal_config_hdrs +
            ctx.files.free_rtos_config_hdrs,
        srcs = drivers_and_middleware_srcs,
        includes = drivers_and_middleware_includes + hal_config_hdr_includes,
        user_compile_flags = ["-D{}".format(ctx.attr.processor), "-DUSE_HAL_DRIVER"],
        compilation_contexts = compilation_contexts,
    )
    (linking_context, linking_outputs) = cc_common.create_linking_context_from_compilation_outputs(
        name = ctx.label.name,
        actions = ctx.actions,
        feature_configuration = feature_configuration,
        cc_toolchain = cc_toolchain,
        language = "c++",
        compilation_outputs = compilation_outputs,
        linking_contexts = linking_contexts,
    )
    library = linking_outputs.library_to_link
    files = []
    files.extend(compilation_outputs.objects)
    files.extend(compilation_outputs.pic_objects)
    files.append(library.pic_static_library)
    files.append(library.static_library)
    files.append(library.dynamic_library)

    return [
        DefaultInfo(
            files = depset(_filter_none(files)),
        ),
        CcInfo(
            compilation_context = compilation_context,
            linking_context = linking_context,
        ),
    ]

cc_stm32h7_hal_library = rule(
    implementation = _cc_stm32h7_hal_library_impl,
    attrs = {
        "hal_config_hdrs": attr.label_list(allow_files = [".h"]),
        "free_rtos_config_hdrs": attr.label_list(allow_files = [".h"]),
        "drivers_and_middleware_hdrs": attr.string_list(),
        "drivers_and_middleware_srcs": attr.string_list(),
        "processor": attr.string(mandatory = True, values = SUPPORTED_PROCESSORS),
        "_cc_toolchain": attr.label(default = "@bazel_tools//tools/cpp:current_cc_toolchain"),
        # TODO: check if using this means we don't need to specify a cpu/toolchain on the command line
        # TODO: is this doing anything?
        "_compiler": attr.label(
            default = Label("//tools/cc_toolchain:gcc-stm32h7"),
            allow_single_file = True,
            #executable = True,
        ),
    },
    fragments = ["cpp"],
)

""" Rule to build a binary for the stm32h7 series of MCU's """

def _cc_stm32h7_binary_impl(ctx):
    cc_toolchain = find_cpp_toolchain(ctx)
    feature_configuration = cc_common.configure_features(
        ctx = ctx,
        cc_toolchain = cc_toolchain,
        requested_features = ctx.features,
        unsupported_features = ctx.disabled_features,
    )
    compilation_contexts = []
    linking_contexts = []
    for dep in ctx.attr.deps:
        if CcInfo in dep:
            compilation_contexts.append(dep[CcInfo].compilation_context)
            linking_contexts.append(dep[CcInfo].linking_context)

    (_compilation_context, compilation_outputs) = cc_common.compile(
        name = ctx.label.name,
        actions = ctx.actions,
        feature_configuration = feature_configuration,
        cc_toolchain = cc_toolchain,
        srcs = ctx.files.srcs,
        private_hdrs = ctx.files.hdrs,
        compilation_contexts = compilation_contexts,
        user_compile_flags = ctx.attr.copts + ["-D{}".format(ctx.attr.processor), "-DUSE_HAL_DRIVER"],
    )

    linkopts = []
    for linkopt in ctx.attr.linkopts:
        linkopts.append(ctx.expand_location(linkopt, targets = [ctx.attr.linker_script]))

    print(ctx.attr)

    print(linkopts)
    print(ctx.attr.linker_script)

    linkopts += [
        "-T{}".format(ctx.file.linker_script.path),
        # TODO: why do we need  "-specs=nano.specs"? What does it do?
        "-specs=nano.specs",
        "-lc",
        "-lm",
        "-lnosys",
    ]

    linking_outputs = cc_common.link(
        name = "{}.elf".format(ctx.label.name),
        actions = ctx.actions,
        feature_configuration = feature_configuration,
        cc_toolchain = cc_toolchain,
        language = "c++",
        compilation_outputs = compilation_outputs,
        linking_contexts = linking_contexts,
        user_link_flags = linkopts,
        link_deps_statically = True,
        additional_inputs = [ctx.file.linker_script],
        output_type = "executable",
    )

    elf_file = linking_outputs.executable

    # Create the .bin from the .elf
    bin_file = ctx.actions.declare_file("{}.bin".format(ctx.label.name))
    ctx.actions.run_shell(
        inputs = [elf_file],
        outputs = [bin_file],
        tools = cc_toolchain.all_files,
        command = "{objcopy} -O binary {elf_out} {cc_bin}".format(
            objcopy=cc_toolchain.objcopy_executable(),
            elf_out=elf_file.path,
            cc_bin=bin_file.path,
        )
    )

    return [
        DefaultInfo(
            files = depset(_filter_none([elf_file, bin_file])),
        ),
    ]

cc_stm32h7_binary = rule(
    implementation = _cc_stm32h7_binary_impl,
    attrs = {
        "srcs": attr.label_list(allow_files = [".c"]),
        # TODO: Should not allow headers here
        "hdrs": attr.label_list(allow_files = [".h"]),
        "linker_script": attr.label(
            mandatory = True,
            allow_single_file = True,
        ),
        "deps": attr.label_list(
            allow_empty = True,
            providers = [CcInfo],
        ),
        "data": attr.label_list(
            default = [],
            allow_files = True,
        ),
        "linkopts": attr.string_list(),
        "copts": attr.string_list(),
        "_cc_toolchain": attr.label(default = "@bazel_tools//tools/cpp:current_cc_toolchain"),
        "processor": attr.string(mandatory = True, values = SUPPORTED_PROCESSORS),
        # TODO: is this doing anything?
        "_compiler": attr.label(
            default = Label("//tools/cc_toolchain:gcc-stm32h7"),
            allow_single_file = True,
            #executable = True,
        ),
    },
    fragments = ["cpp"],
)