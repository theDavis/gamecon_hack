# Gamecon Hack

Slightly modified [gamecon gpio driver](https://github.com/RetroPie/RetroPie-Setup/wiki/GPIO-Modules#gamecon_gpio_rpi), mostly to learn about driver/kernel module. I haven't actually touched this in four years, but found it on an old laptop and figured I would keep it for posterity.

## Cross-compiled Build

1. Get the [raspberry pi tools](https://github.com/raspberrypi/tools)
2. Get the [linux source](https://www.kernel.org/) (or [clone from github](https://github.com/torvalds/linux))
3. set CCPREFIX to the correct arch (e.g. `/home/user/rpi_tools/arm-bcm2708/arm-bcm2708-linux-gnueabi/bin/arm-bcm2708-linux-gnueabi-`)
4. set KERNEL_SRC (e.g. `/home/user/kernel_src/`)
5. run make
