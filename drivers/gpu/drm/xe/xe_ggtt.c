/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2021 Intel Corporation
 */

#include "xe_ggtt.h"

#include <linux/sizes.h>
#include <drm/i915_drm.h>

#include "xe_bo.h"

#define PTE_READ_ONLY	BIT(0)
#define PTE_LM		BIT(1)

#define GEN8_PTE_SHIFT 12
#define GEN8_PAGE_SIZE (1 << GEN8_PTE_SHIFT)
#define GEN8_PTE_MASK (GEN8_PAGE_SIZE - 1)

#define GEN12_PPGTT_PTE_LM	BIT_ULL(11)

static uint64_t gen8_pte_encode(struct xe_bo *bo, uint64_t bo_offset)
{
	uint64_t pte = xe_bo_addr(bo, bo_offset, GEN8_PAGE_SIZE);

	pte |= _PAGE_PRESENT;

	if (xe_bo_is_in_lmem(bo))
		pte |= GEN12_PPGTT_PTE_LM;

	return pte;
}

static unsigned int probe_gsm_size(struct pci_dev *pdev)
{
	uint16_t gmch_ctl, ggms;

	pci_read_config_word(pdev, SNB_GMCH_CTRL, &gmch_ctl);
	ggms = (gmch_ctl >> BDW_GMCH_GGMS_SHIFT) & BDW_GMCH_GGMS_MASK;
	return ggms ? SZ_1M << ggms : 0;
}

static void xe_ggtt_set_pte(struct xe_ggtt *ggtt, uint64_t addr, uint64_t pte)
{
	XE_BUG_ON(addr & GEN8_PTE_MASK);
	XE_BUG_ON(addr > ggtt->size);

	writeq(pte, &ggtt->gsm[addr >> GEN8_PTE_SHIFT]);
}

static void xe_ggtt_clear(struct xe_ggtt *ggtt, uint64_t start, uint64_t size)
{
	uint64_t end = start + size - 1;

	XE_BUG_ON(start >= end);

	while (start < end) {
		xe_ggtt_set_pte(ggtt, start, 0);
		start += GEN8_PAGE_SIZE;
	}
}

int xe_ggtt_init(struct xe_device *xe, struct xe_ggtt *ggtt)
{
	struct pci_dev *pdev = to_pci_dev(xe->drm.dev);
	unsigned int gsm_size;
	phys_addr_t phys_addr;

	gsm_size = probe_gsm_size(pdev);
	if (gsm_size == 0) {
		drm_err(&xe->drm, "Hardware reported no preallocated GSM\n");
		return -ENOMEM;
	}

	/* For Modern GENs the PTEs and register space are split in the BAR */
	phys_addr = pci_resource_start(pdev, 0) + pci_resource_len(pdev, 0) / 2;
	ggtt->gsm = ioremap(phys_addr, gsm_size);
	if (!ggtt->gsm) {
		drm_err(&xe->drm, "Failed to map the ggtt page table\n");
		return -ENOMEM;
	}

	ggtt->size = (gsm_size / 8) * (uint64_t)GEN8_PAGE_SIZE;
	xe_ggtt_clear(ggtt, 0, ggtt->size - 1);

	/* 8B per entry, each points to a 4KB page */
	drm_mm_init(&ggtt->mm, GEN8_PAGE_SIZE, ggtt->size - GEN8_PAGE_SIZE);
	mutex_init(&ggtt->lock);

	return 0;
}

void xe_ggtt_finish(struct xe_ggtt *ggtt)
{
	mutex_destroy(&ggtt->lock);
	drm_mm_takedown(&ggtt->mm);
	iounmap(ggtt->gsm);
}

int xe_ggtt_insert_bo(struct xe_ggtt *ggtt, struct xe_bo *bo)
{
	uint64_t offset, pte;
	int err;

	if (XE_WARN_ON(bo->ggtt_node.size > 0)) {
		/* Someone's already inserted this BO in the GGTT */
		XE_BUG_ON(bo->ggtt_node.size != bo->size);
		return 0;
	}

	err = xe_bo_populate(bo);
	if (err)
		return err;

	mutex_lock(&ggtt->lock);

	err = drm_mm_insert_node(&ggtt->mm, &bo->ggtt_node, bo->size);
	if (!err) {
		uint64_t start = bo->ggtt_node.start;

		for (offset = 0; offset < bo->size; offset += GEN8_PAGE_SIZE) {
			pte = gen8_pte_encode(bo, offset);
			xe_ggtt_set_pte(ggtt, start + offset, pte);
		}
	}

	mutex_unlock(&ggtt->lock);

	return 0;
}

void xe_ggtt_remove_bo(struct xe_ggtt *ggtt, struct xe_bo *bo)
{
	if (XE_WARN_ON(bo->ggtt_node.size)) {
		/* This BO is not currently in the GGTT */
		XE_BUG_ON(bo->ggtt_node.size != bo->size);
		return;
	}

	mutex_lock(&ggtt->lock);

	xe_ggtt_clear(ggtt, bo->ggtt_node.start, bo->ggtt_node.size);
	drm_mm_remove_node(&bo->ggtt_node);
	bo->ggtt_node.size = 0;

	mutex_unlock(&ggtt->lock);
}
