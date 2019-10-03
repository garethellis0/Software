load("//firmware_new:rules.bzl", "cc_stm32_h7_binary")

cc_stm32_h7_binary(
    name = "frankie_v1",
    srcs = glob([
        "*.c",
        "*.h",
    ]),
    free_rtos_config_hdrs = [
        "FreeRTOSConfig.h",
        "intrinsics.h",
    ],
    hal_config_hdrs = ["stm32h7xx_hal_conf.h"],
    processor = "STM32H743xx",
)
