# linux-4.18-rt  

#configure  
make ARCH=arm64  CROSS_COMPILE=aarch64-linux-gnu-  defconfig  
make ARCH=arm64  CROSS_COMPILE=aarch64-linux-gnu-  menuconfig  

#compile  
make ARCH=arm64  CROSS_COMPILE=aarch64-linux-gnu-  -j4 Image  
  
#compile modules  
make ARCH=arm64  CROSS_COMPILE=aarch64-linux-gnu-  modules  


 



