define Device/a3002ru
  DEVICE_TITLE := TOTOLINK A3002RU
  BOARDNAME = A3002RU
  CONSOLE = ttyS0,38400
  MTDPARTS = spi0.0:24k(bootloader)ro,8k(hw)ro,32k(compds)ro,64k(compcs)ro,-@0x20000(firmware)
  CVIMG_KERNEL_BURN_ADDR := 0x20000
endef

TARGET_DEVICES += a3002ru
