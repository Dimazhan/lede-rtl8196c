define Device/au-home-spot-cube
  DEVICE_TITLE := AU HOME SPOT CUBE
  BOARDNAME := AU-HOME-SPOT-CUBE
  CVIMG_KERNEL_BURN_ADDR := 0x9e000
  CVIMG_ROOTFS_BURN_ADDR := 0
  CONSOLE = ttyS0,38400
  MTDPARTS := spi0.0:32k(bootloader)ro,-@0x9e000(firmware)
  IMAGE/factory.bin = append-kernel | append-fake-rootfs | append-rootfs | cvimg-fw | cvimg-pad-rootfs
  IMAGE/sysupgrade.bin = append-kernel | append-fake-rootfs | append-rootfs | cvimg-fw | pad-rootfs
endef
TARGET_DEVICES += au-home-spot-cube
