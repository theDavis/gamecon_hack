#Gamecon Hack

Slightly modified [gamecon gpio driver](https://github.com/RetroPie/RetroPie-Setup/wiki/GPIO-Modules#gamecon_gpio_rpi), mostly to learn about driver/kernel module. I haven't actually touched this in four years, but found it on an old laptop and figured I would keep it for posterity.

## Cross-compiled Build

l. Get the [raspberry pi tools](https://github.com/raspberrypi/tools)
l. Get the [linux source](https://www.kernel.org/) (or [clone from github](https://github.com/torvalds/linux))
l. set CCPREFIX to the correct arch (e.g. `/home/user/rpi_tools/arm-bcm2708/arm-bcm2708-linux-gnueabi/bin/arm-bcm2708-linux-gnueabi-`)
l. set KERNEL_SRC (e.g. `/home/user/kernel_src/`)
l. run make
