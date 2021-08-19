/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2021 Intel Corporation
 */

#include "xe_mmio.h"

#include "i915_reg.h"

#define IS_DGFX(x) true

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

int xe_mmio_init(struct xe_device *xe)
{
	const int mmio_bar = 0;
	const int mmio_size = IS_DGFX(xe) ? SZ_4M : SZ_2M;
	int err;

	xe->mmio.regs = pci_iomap(to_pci_dev(xe->drm.dev), mmio_bar, mmio_size);
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

	return 0;
}

void xe_mmio_finish(struct xe_device *xe)
{
	pci_iounmap(to_pci_dev(xe->drm.dev), xe->mmio.regs);
}
