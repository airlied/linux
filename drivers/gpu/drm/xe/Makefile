# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the drm device driver.  This driver provides support for the
# Direct Rendering Infrastructure (DRI) in XFree86 4.1.0 and higher.

# Add a set of useful warning flags and enable -Werror for CI to prevent
# trivial mistakes from creeping in. We have to do this piecemeal as we reject
# any patch that isn't warning clean, so turning on -Wall -Wextra (or W=1) we
# need to filter out dubious warnings.  Still it is our interest
# to keep running locally with W=1 C=1 until we are completely clean.
#
# Note the danger in using -Wall -Wextra is that when CI updates gcc we
# will most likely get a sudden build breakage... Hopefully we will fix
# new warnings before CI updates!
subdir-ccflags-y := -Wall -Wextra
subdir-ccflags-y += $(call cc-disable-warning, unused-parameter)
subdir-ccflags-y += $(call cc-disable-warning, type-limits)
subdir-ccflags-y += $(call cc-disable-warning, missing-field-initializers)
subdir-ccflags-y += $(call cc-disable-warning, unused-but-set-variable)
# clang warnings
subdir-ccflags-y += $(call cc-disable-warning, sign-compare)
subdir-ccflags-y += $(call cc-disable-warning, sometimes-uninitialized)
subdir-ccflags-y += $(call cc-disable-warning, initializer-overrides)
subdir-ccflags-y += $(call cc-disable-warning, frame-address)
subdir-ccflags-$(CONFIG_DRM_I915_WERROR) += -Werror

# Fine grained warnings disable
CFLAGS_xe_pci.o = $(call cc-disable-warning, override-init)

subdir-ccflags-y += -I$(srctree)/$(src)

# Please keep these build lists sorted!

# core driver code
xe-y += xe_bo.o \
	xe_device.o \
	xe_engine.o \
	xe_irq.o \
	xe_mmio.o \
	xe_module.o \
	xe_pci.o \
	xe_vm.o

obj-$(CONFIG_DRM_XE) += xe.o
