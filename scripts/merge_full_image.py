# Post-build step for ESP32 envs: merge bootloader, partition table, boot_app0
# and the app into one image flashable at 0x0 (firmware-full.bin), for first-
# time USB installs from a release download. firmware.bin is only read, so it
# remains the app-only image OTA flashes. The offsets come from the same FLASH_EXTRA_IMAGES list
# `pio run -t upload` flashes, so the two can't drift apart.
Import("env")


def merge_full_image(source, target, env):
    images = []
    for offset, image in env.get("FLASH_EXTRA_IMAGES", []):
        images += [offset, env.subst(image)]
    images += [env.subst("$ESP32_APP_OFFSET"), str(target[0])]
    env.Execute(" ".join([
        '"$OBJCOPY"',
        "--chip", env.BoardConfig().get("build.mcu"),
        "merge_bin", "-o", '"$BUILD_DIR/firmware-full.bin"',
    ] + ['"%s"' % i for i in images]))


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", merge_full_image)
