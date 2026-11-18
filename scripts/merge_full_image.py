# Post-build step: emit firmware-full.bin, an image flashable at 0x0 for
# first-time USB installs from a release download, next to firmware.bin (the
# app-only image OTA flashes, which is only read here).
#
# ESP32: merge bootloader, partition table, boot_app0 and the app. The offsets
# come from the same FLASH_EXTRA_IMAGES list `pio run -t upload` flashes, so
# the two can't drift apart.
# ESP8266: the app image already embeds the bootloader, so it is the full
# image as is.
import shutil

Import("env")


def full_image(source, target, env):
    app = str(target[0])
    out = env.subst("$BUILD_DIR/firmware-full.bin")
    if env.get("PIOPLATFORM") == "espressif8266":
        shutil.copyfile(app, out)
        return
    images = []
    for offset, image in env.get("FLASH_EXTRA_IMAGES", []):
        images += [offset, env.subst(image)]
    images += [env.subst("$ESP32_APP_OFFSET"), app]
    env.Execute(" ".join([
        '"$OBJCOPY"',
        "--chip", env.BoardConfig().get("build.mcu"),
        "merge-bin", "-o", '"%s"' % out,
    ] + ['"%s"' % i for i in images]))


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", full_image)
