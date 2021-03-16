# Intel GPIO universal SysFS driver
**Intel GPIO SysFS driver** is universal kernel driver module which supports multiple instances and can be used for general **FPGA** logic control. 

## Driver features:  
- tested on **Cyclone® V SOC FPGA** and **Arria® 10 SOC FPGA** 
- Linux kernel 4.x and 5.x tree supported (tested with 4.12, 4.17, 4.20 and 5.4) 
- configuration via **Device Tree Overlay** (registers)
- multiple instances supported
- **non root access** support

## Build
### Option 1] - cross compile 
- set `KERNEL_SRC_DIR` in Makefile (e.g. `KERNEL_SRC_DIR ?= $(HOME)/build_sockit_arm/socfpga-kernel-dev-3/KERNEL`)
- for cross compilation is necessary to export **CC variable** (gcc)
- tested with: **gcc-linaro-7.2.1-2017.11-x86_64_arm-linux-gnueabihf** 
- e.g. `export CC=`pwd`/socfpga-kernel-dev-3/dl/gcc-linaro-7.2.1-2017.11-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-`
- test cross compiler with `${CC}gcc --version`
- `make clean && make` 

### Option 2] -  compilation on target embedded Linux system
- install appropriate kernel headers
- repair headers install (see: https://github.com/armbian/build/issues/74; run `make scripts`; for Linux 5.x run `sudo make M=scripts/mod` ) 
- `make clean && make` 

## Device Tree Overlay
 >Cyclone® V SOC FPGA
- full FPGA reconfiguration is supported (use your FPGA bitstream and set firmware-name="*.rbf")
- for **Cyclone® V** target-path is `"/soc/fpga-region0"`
- modify reg. settings for your requireme
```
/dts-v1/; /plugin/;
/ {
        fragment@0 {
        target-path="/soc/fpga-region0";
        #address-cells=<1>;
        #size-cells=<1>;
        __overlay__ {
                firmware-name="altera_msgdma_socfpga.rbf";
                #address-cells=<1>;
                #size-cells=<1>;
		
                altera_pio: gpio1@ff210040 {
                            compatible = "altr,altera-gpio-1.0";
                            reg = <0xff210040 0x0000000f>;
                            reg-names = "csr";
                        };

                
                };
        };
};

```
>Arria® 10 SOC FPGA 
- full reconfiguration is not allowed on Arria 10  - use **u-boot**
- for **Arria ® 10** target-path is `"/soc"`
nts 
 ```
/dts-v1/; /plugin/;
/ {
        fragment@0 {
        /* target-path="/soc/base_fpga_region"; */ 
        target-path="/soc";
        #address-cells=<1>;
        #size-cells=<1>;
        __overlay__ {
                /*
                 
                https://forum.rocketboards.org/t/problems-loading-device-tree-overlays-on-arria10-socfpga-kernel-4-14-126/2145/3
                https://www.intel.co.jp/content/www/jp/ja/programmable/documentation/pne1482303525167.html
                https://forum.rocketboards.org/t/arria-10-fpga-manager-driver-doesnt-support-full-reconfiguration/711/3
                */
                /* --> Full reconfiguration is not allowed on Arria 10:  firmware-name="ghrd_10as066n2.core.rbf"; */
               
                #address-cells=<1>;
                #size-cells=<1>;
              
                altera_pio: gpio1@ff200120 {
                            compatible = "altr,altera-gpio-1.0";
                            reg = <0xff200120 0x0000000f>;
                            reg-names = "csr";
                        };
        
                };
        };
};
 ```
> registers ranges are from Quartus QSys/Platform Designer

![Quartus](https://github.com/pavelfpl/altera_gpio/blob/master/qsys_platform_designer_gpio.png)

## MISC
- **DTC compilator** is used for compilation of **Device Tree Blob** (use newer DTC like 1.4.6/1.4.7) 
- run: `dtc -O dtb -o altera_gpio_rbf.dtbo -b 0 -@ altera_gpio_rbf.dts`
- **non root access:** `DEVICE_ATTR(altera_gpio, S_IWUGO | S_IRUGO /*0644*/, altera_gpio_show, altera_gpio_store);`

## How to use 
- **BASH access**
 ```
#!/bin/bash
GPIO_NUM=$1
echo $GPIO_NUM > /sys/bus/platform/drivers/altera_gpio/ff200120.gpio1/altera_gpio
 ```
 - **C language access**
  ```
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>

int main(void){

	  int valuefd = open("/sys/bus/platform/drivers/altera_gpio/ff210040.gpio1/altera_gpio", O_RDWR);
  
	  if (valuefd < 0){
	      printf("Cannot open GPIO to export it ...\n");
	      return(1);
	  }
	  
	  write(valuefd,"1", 2);
	  close(valuefd);
	  return(0);
}
 ```



