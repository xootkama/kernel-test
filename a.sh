#!/usr/bin/bash 

if [[ -d "out" ]]
then 
cd out && make clean && make distclean && make mrproper && cd  ..
else 
  mkdir -p out
  fi
  
  make O=out ARCH=arm64 KharaMe_defconfig
sattire=`find / -type d -name "11.2.0" 2>/dev/null`

PATH="$sattire/bin:$PATH" \
 
  make                O=out \
                      ARCH=arm64 \
                      CC=clang \
                      CROSS_COMPILE=aarch64-linux-gnu- \
                      CROSS_COMPILE_ARM32=arm-linux-gnueabi- \
                      CONFIG_DEBUG_SECTION_MISMATCH=y -j8
                      
                      
