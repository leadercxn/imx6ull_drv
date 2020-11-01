KERNELDIR := /home/cxn/linux/kernel/linux-imx-rel_imx_4.1.15_2.1.0_ga

CURRENT_PATH := $(shell pwd)

EXTRA_CFLAGS += \
	-I$(KERNELDIR)/my_dev/common	\
	-I$(KERNELDIR)/my_dev/drv_i2c_ap3216c	\

obj-m := drv_i2c_ap3216c.o

build: kernel_modules	#指定编译目标

kernel_modules:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) modules

clean:
	@rm .*cmd *.mod* *.o *.order *.sym* .tmp* -rf

copy:
	@cp drv_i2c_ap3216c.ko  /home/cxn/linux/nfs/rootfs/home/dev_app/imx6ull_app







