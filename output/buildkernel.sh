#!/bin/bash

VER="1.0.1"
ZIP_VER="NV08B_KERNEL_"$VER

OUTPUT_DIR=./output
MODULES_DIR=./output/installer/system/lib/modules/

#build kernel

cd ..
export PATH=$PATH:~/Android/linaro4.9/bin
export ARCH=arm
export SUBARCH=arm
export CROSS_COMPILE=arm-eabi-
arm-eabi-gcc --version
make dreamtab_defconfig
make -j4 -o3 > ./output/build.log 2>&1
echo "zImage ready"

cp ./arch/arm/boot/zImage ./output/work/kernel
find $KERNEL_DIR -name "*.ko" -exec cp {} $MODULES_DIR \;
cd output
./mkboot work boot.img
cp boot.img ./installer/boot.img

cd installer
zip -r9 ../$ZIP_VER'.zip' *
#cleanup
cd ..
rm boot.img work/kernel installer/boot.img installer/system/lib/modules/*.ko
echo "Complete."
