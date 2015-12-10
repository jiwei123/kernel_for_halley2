ifdef CONFIG_WATCH_AW808
BOARD_DIR = aw808
endif

ifdef CONFIG_WATCH_SOLAR_SAMSUNG
BOARD_DIR = solar_samsung
endif


CFLAGS_KERNEL	+= -I$(srctree)/arch/mips/xburst/soc-m200/chip-m200/$(CONFIG_PRODUCT_NAME)/$(BOARD_DIR)

obj-y		+= common/
obj-$(CONFIG_WATCH_AW808)		+= $(BOARD_DIR)/
obj-$(CONFIG_WATCH_SOLAR_SAMSUNG)	+= $(BOARD_DIR)/
