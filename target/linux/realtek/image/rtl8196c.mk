define Device/au-home-spot-cube
  DEVICE_TITLE := AU HOME SPOT CUBE
  BOARDNAME := AU-HOME-SPOT-CUBE
  CVIMG_KERNEL_BURN_ADDR := 0x9e000
  CONSOLE = ttyS0,38400
  MTDPARTS := spi0.0:32k(bootloader)ro,-@0x9e000(firmware)
endef
TARGET_DEVICES += au-home-spot-cube
