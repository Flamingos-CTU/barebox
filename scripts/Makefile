###
# scripts contains sources for various helper programs used throughout
# barebox for the build process.
# ---------------------------------------------------------------------------
# kallsyms:      Find all symbols in barebox

hostprogs-y                      += bin2c
hostprogs-y                      += mkimage
hostprogs-y                      += fix_size
hostprogs-y                      += bareboxenv
hostprogs-y                      += bareboxcrc32
hostprogs-y                      += kernel-install
hostprogs-$(CONFIG_IMD)          += bareboximd
hostprogs-$(CONFIG_KALLSYMS)     += kallsyms
hostprogs-$(CONFIG_ARCH_MVEBU)   += kwbimage kwboot
hostprogs-$(CONFIG_ARCH_NETX)    += gen_netx_image
hostprogs-$(CONFIG_ARCH_OMAP)    += omap_signGP mk-omap-image
hostprogs-$(CONFIG_ARCH_S5PCxx)  += s5p_cksum
hostprogs-$(CONFIG_ARCH_DAVINCI) += mkublheader
hostprogs-$(CONFIG_ARCH_ZYNQ)	 += zynq_mkimage
hostprogs-$(CONFIG_ARCH_SOCFPGA) += socfpga_mkimage
hostprogs-$(CONFIG_ARCH_MXS)     += mxsimage mxsboot
HOSTLOADLIBES_mxsimage  = `pkg-config --libs openssl`
HOSTCFLAGS_mxs-usb-loader.o = `pkg-config --cflags libusb-1.0`
HOSTLOADLIBES_mxs-usb-loader  = `pkg-config --libs libusb-1.0`
hostprogs-$(CONFIG_ARCH_MXS_USBLOADER)  += mxs-usb-loader
HOSTCFLAGS_omap3-usb-loader.o = `pkg-config --cflags libusb-1.0`
HOSTLOADLIBES_omap3-usb-loader  = `pkg-config --libs libusb-1.0`
hostprogs-$(CONFIG_OMAP3_USB_LOADER)  += omap3-usb-loader

subdir-y			+= mod
subdir-$(CONFIG_OMAP4_USBBOOT)	+= omap4_usbboot
subdir-$(CONFIG_ARCH_IMX)	+= imx
subdir-$(CONFIG_X86)		+= setupmbr
subdir-$(CONFIG_DTC)		+= dtc
subdir-$(CONFIG_ARCH_TEGRA)	+= tegra

targetprogs-$(CONFIG_BAREBOXENV_TARGET) += bareboxenv-target
targetprogs-$(CONFIG_KERNEL_INSTALL_TARGET) += kernel-install-target
targetprogs-$(CONFIG_BAREBOXCRC32_TARGET) += bareboxcrc32-target
targetprogs-$(CONFIG_IMD_TARGET) += bareboximd-target

# Let clean descend into subdirs
subdir-	+= basic kconfig setupmbr

quiet_cmd_csingle	= CC      $@
      cmd_csingle	= $(CC) -Wp,-MD,$(depfile) $(CFLAGS) -o $@ $<

__targetprogs	:= $(sort $(targetprogs-y) $(targetprogs-m))
target-csingle	:= $(foreach m,$(__targetprogs),$(if $($(m)-objs),,$(m)))
__targetprogs	:= $(addprefix $(obj)/,$(__targetprogs))
target-csingle	:= $(addprefix $(obj)/,$(target-csingle))

always		:= $(hostprogs-y) $(hostprogs-m) $(targetprogs-y)

$(target-csingle): %-target: %.c FORCE
	$(call if_changed_dep,csingle)
