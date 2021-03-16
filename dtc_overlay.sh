#!/bin/bash
rm -i altera_gpio_rbf.dtbo
# System install - use recent version (1.4.0 is OLD !!!) ...
# ----------------------------------------------------------
dtc -O dtb -o altera_gpio_rbf.dtbo -b 0 -@ altera_gpio_rbf.dts
# Local install ... 
# -----------------
# /usr/src/linux-headers-4.20.0-socfpga-r0/scripts/dtc/dtc -O dtb -o altera_gpio_rbf.dtbo -b 0 -@ altera_gpio_rbf.dts
