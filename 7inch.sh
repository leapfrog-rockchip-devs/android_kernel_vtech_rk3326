#!/bin/bash

    echo "make kernel.img"
	make ARCH=arm64 rockchip_defconfig android-10.config rk3326.config 
	make ARCH=arm64 BOOT_IMG=../rockdev/Image-rk3326_qgo/boot.img rk3326-7inch.img -j12
#cd ../u-boot/
#./scripts/pack_resource.sh ../kernel/resource.img
#cp resource.img ../kernel/resource.img
echo " end make make kernel.img"
