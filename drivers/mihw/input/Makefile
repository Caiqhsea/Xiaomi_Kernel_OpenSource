# MIUI ADD: Performance_TurboSched
obj-$(CONFIG_MI_SPEED_TOUCH) += speed_touch.o

PWD := $(CURDIR)

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
# END Performance_TurboSched