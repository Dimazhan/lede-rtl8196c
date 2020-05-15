# OpenWrt for RTL819x

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


