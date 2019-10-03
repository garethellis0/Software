# contents of a file you create named: copy_filegroups.bzl
# known to work in bazel version 0.22.0
def _copy_filegroup_impl(ctx):
    all_input_files = [
        f
        for t in ctx.attr.targeted_filegroups
        for f in t.files.to_list()
    ]

    all_outputs = []
    for f in all_input_files:
        out = ctx.actions.declare_file(f.basename)
        all_outputs += [out]
        ctx.actions.run_shell(
            outputs = [out],
            inputs = depset([f]),
            arguments = [f.path, out.path],
            # This is what we're all about here. Just a simple 'cp' command.
            # Copy the input to CWD/f.basename, where CWD is the package where
            # the copy_filegroups_to_this_package rule is invoked.
            # (To be clear, the files aren't copied right to where your BUILD
            # file sits in source control. They are copied to the 'shadow tree'
            # parallel location under `bazel info bazel-bin`)
            command = "cp $1 $2",
        )

    # Small sanity check
    if len(all_input_files) != len(all_outputs):
        fail("Output count should be 1-to-1 with input count.")

    return [
        DefaultInfo(
            files = depset(all_outputs),
            runfiles = ctx.runfiles(files = all_outputs),
        ),
    ]

copy_filegroups_to_this_package = rule(
    implementation = _copy_filegroup_impl,
    attrs = {
        "targeted_filegroups": attr.label_list(),
    },
)
