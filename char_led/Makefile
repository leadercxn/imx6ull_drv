KERNELDIR := /home/cxn/linux/kernel/linux-imx-rel_imx_4.1.15_2.1.0_ga

CURRENT_PATH := $(shell pwd)

obj-m := driver_led.o


build: kernel_modules	#指定编译目标

kernel_modules:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) modules

clean:
	@rm .*cmd *.mod* *.o *.order *.sym* .tmp* -rf

copy:
	@cp driver_led.ko /home/cxn/linux/nfs/rootfs/home/dev_app/drv_chr_led







