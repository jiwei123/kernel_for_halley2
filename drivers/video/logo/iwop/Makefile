# Makefile for the Linux logos

obj-y		+= logo_iwop_clut224.o \
				logo_low_battery_clut224.o \
				charge_logo_1_clut224.o \
				charge_logo_2_clut224.o \
				charge_logo_3_clut224.o \
				charge_logo_4_clut224.o \
				charge_logo_5_clut224.o \
				charge_logo_6_clut224.o \

# How to generate logo's

# Use logo-cfiles to retrieve list of .c files to be built
logo-cfiles = $(notdir $(patsubst %.$(2), %.c, \
              $(wildcard $(srctree)/$(src)/*$(1).$(2))))

# Mono logos
extra-y += $(call logo-cfiles,_mono,pbm)

# VGA16 logos
extra-y += $(call logo-cfiles,_vga16,ppm)

# 224 Logos
extra-y += $(call logo-cfiles,_clut224,ppm)

# Gray 256
extra-y += $(call logo-cfiles,_gray256,pgm)

pnmtologo := scripts/pnmtologo

# Create commands like "pnmtologo -t mono -n logo_mac_mono -o ..."
quiet_cmd_logo = LOGO    $@
	cmd_logo = $(pnmtologo) \
			-t $(patsubst $*_%,%,$(notdir $(basename $<))) \
			-n $(notdir $(basename $<)) -o $@ $<

$(obj)/%_mono.c: $(src)/%_mono.pbm $(pnmtologo) FORCE
	$(call if_changed,logo)

$(obj)/%_vga16.c: $(src)/%_vga16.ppm $(pnmtologo) FORCE
	$(call if_changed,logo)

$(obj)/%_clut224.c: $(src)/%_clut224.ppm $(pnmtologo) FORCE
	$(call if_changed,logo)

$(obj)/%_gray256.c: $(src)/%_gray256.pgm $(pnmtologo) FORCE
	$(call if_changed,logo)

# Files generated that shall be removed upon make clean
clean-files := *.o *_mono.c *_vga16.c *_clut224.c *_gray256.c
