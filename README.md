# linux-4.18-rt  

#configure  
make ARCH=arm64  CROSS_COMPILE=aarch64-linux-gnu-  defconfig  
make ARCH=arm64  CROSS_COMPILE=aarch64-linux-gnu-  menuconfig  

#compile  
make ARCH=arm64  CROSS_COMPILE=aarch64-linux-gnu-  -j4 Image  
  
#compile modules  
make ARCH=arm64  CROSS_COMPILE=aarch64-linux-gnu-  modules  
  
#compile dtb
make ARCH=arm64  CROSS_COMPILE=aarch64-linux-gnu-  dtbs  

#compile appoint modules, Notice:make in linux root dir   
make ARCH=arm64  CROSS_COMPILE=aarch64-linux-gnu-  M=$(modules_dir)  
example:  
make ARCH=arm64  CROSS_COMPILE=aarch64-linux-gnu-  M=drivers/i2c/busess  



