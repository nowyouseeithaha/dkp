#!/bin/bash -e

[[ -f arch/arm/configs/cyanogen_d2_defconfig ]] || \
	{ echo "Where am I?"; exit 1; }

b="$(sed -n '/^DKP_NAME/{s/[^=]*=\W*//;p}' Makefile)"
grep -vf <( \
	sed -ne 's/# \([^ ]*\).*/\1/; s/=.*//; T o; p; : o' \
	arch/arm/configs/cyanogen_d2[a-z]* \
	arch/arm/configs/m2selinux_defconfig \
) "../build/kbuild-$b-d2spr/.config" > arch/arm/configs/cyanogen_d2_defconfig
