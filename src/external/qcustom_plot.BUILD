# Description:
#   This library provides us with plotting abilities in QT
#   https://www.qcustomplot.com/index.php
package(default_visibility = ["//visibility:public"])

# TODO: comment here
genrule(
    name = "patch_qt_includes",
    outs = ["patched/qcustomplot.h"],
    srcs = ["qcustomplot.h"],
    cmd = "\n".join([
        # First we copy the header to the expected output location
        "cp $< $@",
        # Now we just patch the header by modifying it in-place
        "sed -i 's/#include\s<qmath.h>/#include <QtCore\/qmath.h>/g' $@",
        "sed -i 's/#.*include\s<QtNumeric>/#include <QtCore\/QtNumeric>/g' $@",
    ]),
)

cc_library(
    name = "qcustom_plot",
    srcs = ["qcustomplot.cpp"],
    hdrs = ["patched/qcustomplot.h"],
    includes = [
        "patched",
        "QtCore",
    ],
    strip_include_prefix = "patched",
    visibility = ["//visibility:public"],
    deps = [
        "@qt//:qt_widgets",
        "@qt//:qt_core",
        "@qt//:qt_printsupport",
    ],
    defines = [
        "QCUSTOMPLOT_COMPILE_LIBRARY",
    ],
)
