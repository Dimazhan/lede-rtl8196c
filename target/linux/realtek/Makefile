#
# Copyright (C) 2006-2013 OpenWrt.org
# Copyright (C) 2017 Weijie Gao <hackpascal@gmail.com>
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#
include $(TOPDIR)/rules.mk

ARCH:=mips
BOARD:=realtek
BOARDNAME:=Realtek RTL819X
FEATURES:=squashfs
MAINTAINER:=Weijie Gao <hackpascal@gmail.com>
SUBTARGETS:=rtl8196c rtl819xd

KERNEL_PATCHVER:=4.14

include $(INCLUDE_DIR)/target.mk

DEFAULT_PACKAGES += swconfig kmod-gpio-button-hotplug
	#wpad-mini kmod-rtl8192ce kmod-rtl8192de kmod-rtl8192se

$(eval $(call BuildTarget))
