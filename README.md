# OpenWrt for RTL8196C

为 RTL8196C 适配的 OpenWrt

代码基于 [hackpascal/lede-rtl8196c](https://github.com/hackpascal/lede-rtl8196c) 修改，修复了其不能支持 musl 的错误（其实就是加了个编译选项...）
<br>
并且参考了 [nekromant/linux-rlx-upstream](https://github.com/rlx-router/linux-rlx-upstream) [rlx-router/linux-rlx](https://github.com/rlx-router/linux-rlx)
<br>
适配了 binutils-2.30, gcc-7.3, linux_kernel-4.14.53
<br>
目前支持一个 mach: AU_HOME_SPOT_CUBE 除了没有无线驱动其它正常

开发者正在努力添加无线～


================================================================================



================================================================================

This is the buildsystem for the OpenWrt Linux distribution.

Please use "make menuconfig" to choose your preferred
<br>
configuration for the toolchain and firmware.

You need to have installed gcc, binutils, bzip2, flex, python, perl, make,
<br>
find, grep, diff, unzip, gawk, getopt, subversion, libz-dev and libc headers.

Run "./scripts/feeds update -a" to get all the latest package definitions
<br>
defined in feeds.conf / feeds.conf.default respectively
<br>
and "./scripts/feeds install -a" to install symlinks of all of them into
<br>
package/feeds/.

Use "make menuconfig" to configure your image.

Simply running "make" will build your firmware.
<br>
It will download all sources, build the cross-compile toolchain, 
<br>
the kernel and all choosen applications.

To build your own firmware you need to have access to a Linux, BSD or MacOSX system
<br>
(case-sensitive filesystem required). Cygwin will not be supported because of
<br>
the lack of case sensitiveness in the file system.


Sunshine!
<br>
　　Your OpenWrt Community
<br>
　　http://www.openwrt.org


