/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2021 Intel Corporation
 */

#include "xe_mmio.h"

#include <drm/xe_drm.h>

#include "../i915/i915_reg.h"

static int xe_set_dma_info(struct xe_device *xe)
{
	unsigned int mask_size = 39; /* TODO */
	int err;

	/*
	 * We don't have a max segment size, so set it to the max so sg's
	 * debugging layer doesn't complain
	 */
	dma_set_max_seg_size(xe->drm.dev, UINT_MAX);

	err = dma_set_mask(xe->drm.dev, DMA_BIT_MASK(mask_size));
	if (err)
		goto mask_err;

	err = dma_set_coherent_mask(xe->drm.dev, DMA_BIT_MASK(mask_size));
	if (err)
		goto mask_err;

	return 0;

mask_err:
	drm_err(&xe->drm, "Can't set DMA mask/consistent mask (%d)\n", err);
	return err;
}

static void xe_mmio_probe_vram(struct xe_device *xe)
{
	if (!IS_DGFX(xe))
		return;

	xe->vram.size = xe_mmio_read64(xe, GEN12_GSMBASE.reg);
	xe->vram.io_start = pci_resource_start(to_pci_dev(xe->drm.dev), 2);

	drm_info(&xe->drm, "VRAM: %pa\n", &xe->vram.size);
}

int xe_mmio_init(struct xe_device *xe)
{
	const int mmio_bar = 0;
	int err;

	xe->mmio.size = IS_DGFX(xe) ? SZ_4M : SZ_2M;
	xe->mmio.regs = pci_iomap(to_pci_dev(xe->drm.dev), mmio_bar,
				  xe->mmio.size);
	if (xe->mmio.regs == NULL) {
		drm_err(&xe->drm, "failed to map registers\n");
		return -EIO;
	}

	/*
	 * The boot firmware initializes local memory and assesses its health.
	 * If memory training fails, the punit will have been instructed to
	 * keep the GT powered down; we won't be able to communicate with it
	 * and we should not continue with driver initialization.
	 */
	if (IS_DGFX(i915) && !(xe_mmio_read32(xe, GU_CNTL.reg) & LMEM_INIT)) {
		drm_err(&xe->drm, "LMEM not initialized by firmware\n");
		return -ENODEV;
	}

	err = xe_set_dma_info(xe);
	if (err)
		return err;

	xe_mmio_probe_vram(xe);
	return 0;
}

void xe_mmio_finish(struct xe_device *xe)
{
	pci_iounmap(to_pci_dev(xe->drm.dev), xe->mmio.regs);
}

#define VALID_MMIO_FLAGS (\
	DRM_XE_MMIO_BITS_MASK |\
	DRM_XE_MMIO_READ |\
	DRM_XE_MMIO_WRITE)

int xe_mmio_ioctl(struct drm_device *dev, void *data,
		  struct drm_file *file)
{
	struct xe_device *xe = to_xe_device(dev);
	struct drm_xe_mmio *args = data;
	unsigned int bits_flag, bytes;

	if (XE_IOCTL_ERR(xe, !capable(CAP_SYS_ADMIN)))
		return -EPERM;

	if (XE_IOCTL_ERR(xe, args->extensions))
		return -EINVAL;

	if (XE_IOCTL_ERR(xe, args->flags & ~VALID_MMIO_FLAGS))
		return -EINVAL;

	if (XE_IOCTL_ERR(xe, !(args->flags & DRM_XE_MMIO_WRITE) && args->value))
		return -EINVAL;

	bits_flag = args->flags & DRM_XE_MMIO_BITS_MASK;
	bytes = 1 << bits_flag;
	if (XE_IOCTL_ERR(xe, args->addr + bytes > xe->mmio.size))
		return -EINVAL;

	if (args->flags & DRM_XE_MMIO_WRITE) {
		switch (bits_flag) {
		case DRM_XE_MMIO_8BIT:
			return -EINVAL; /* TODO */
		case DRM_XE_MMIO_16BIT:
			return -EINVAL; /* TODO */
		case DRM_XE_MMIO_32BIT:
			if (XE_IOCTL_ERR(xe, args->value > U32_MAX))
				return -EINVAL;
			xe_mmio_write32(xe, args->addr, args->value);
			break;
		case DRM_XE_MMIO_64BIT:
			return -EINVAL; /* TODO */
		default:
			WARN(1, "Invalid MMIO bit size");
			return -EINVAL;
		}
	}

	if (args->flags & DRM_XE_MMIO_READ) {
		switch (bits_flag) {
		case DRM_XE_MMIO_8BIT:
			return -EINVAL; /* TODO */
		case DRM_XE_MMIO_16BIT:
			return -EINVAL; /* TODO */
		case DRM_XE_MMIO_32BIT:
			args->value = xe_mmio_read32(xe, args->addr);
			break;
		case DRM_XE_MMIO_64BIT:
			args->value = xe_mmio_read64(xe, args->addr);
			break;
		default:
			WARN(1, "Invalid MMIO bit size");
			return -EINVAL;
		}
	}

	return 0;
}
