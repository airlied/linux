// SPDX-License-Identifier: MIT
/*
 * Copyright © 2019 Intel Corporation
 */

#include "i915_drv.h"
#include "intel_memory_region.h"
#include "gem/i915_gem_lmem.h"
#include "gem/i915_gem_region.h"
#include "intel_region_lmem.h"

static void
region_lmem_release(struct intel_memory_region *mem)
{
	io_mapping_fini(&mem->iomap);
	intel_memory_region_release_buddy(mem);
}

static int
region_lmem_init(struct intel_memory_region *mem)
{
	int ret;

	if (!io_mapping_init_wc(&mem->iomap,
				mem->io_start,
				resource_size(&mem->region)))
		return -EIO;

	ret = intel_memory_region_init_buddy(mem);
	if (ret)
		io_mapping_fini(&mem->iomap);

	intel_memory_region_set_name(mem, "local");

	return ret;
}

const struct intel_memory_region_ops intel_region_lmem_ops = {
	.init = region_lmem_init,
	.release = region_lmem_release,
	.create_object = __i915_gem_lmem_object_create,
};

static void get_legacy_lowmem_region(struct intel_uncore *uncore,
				     u64 *start, u32 *size)
{
	*start = 0;
	*size = 0;

	if (!IS_DG1_REVID(uncore->i915, DG1_REVID_A0, DG1_REVID_A0))
		return;

	*size = SZ_1M;

	DRM_DEBUG_DRIVER("LMEM: reserved legacy low-memory [0x%llx-0x%llx]\n",
			 *start, *start + *size);
}

static int reserve_lowmem_region(struct intel_uncore *uncore,
				 struct intel_memory_region *mem)
{
	u64 reserve_start;
	u64 reserve_end;
	u64 region_start;
	u32 region_size;
	int ret;

	get_legacy_lowmem_region(uncore, &region_start, &region_size);
	reserve_start = region_start;
	reserve_end = region_start + region_size;

	if (!reserve_end)
		return 0;

	DRM_INFO("LMEM: reserving low-memory region [0x%llx-0x%llx]\n",
		 reserve_start, reserve_end);
	ret = i915_buddy_alloc_range(&mem->mm, &mem->reserved,
				     reserve_start,
				     reserve_end - reserve_start);
	if (ret)
		DRM_ERROR("LMEM: reserving low memory region failed\n");

	return ret;
}

static struct intel_memory_region *
setup_lmem(struct drm_i915_private *dev_priv)
{
	struct intel_uncore *uncore = &dev_priv->uncore;
	struct pci_dev *pdev = dev_priv->drm.pdev;
	struct intel_memory_region *mem;
	resource_size_t io_start;
	resource_size_t lmem_size;

	/* Enables Local Memory functionality in GAM */
	I915_WRITE(GEN12_LMEM_CFG_ADDR, I915_READ(GEN12_LMEM_CFG_ADDR) | LMEM_ENABLE);

	/* Stolen starts from GSMBASE on DG1 */
	lmem_size = intel_uncore_read64(uncore, GEN12_GSMBASE);

	io_start = pci_resource_start(pdev, 2);

	mem = intel_memory_region_create(dev_priv,
					 0,
					 lmem_size,
					 I915_GTT_PAGE_SIZE_4K,
					 io_start,
					 &intel_region_lmem_ops);
	if (!IS_ERR(mem)) {
		int err;

		err = reserve_lowmem_region(uncore, mem);
		if (err) {
			intel_memory_region_put(mem);
			return ERR_PTR(err);
		}
	}

	if (!IS_ERR(mem)) {
		DRM_INFO("Intel graphics LMEM: %pR\n", &mem->region);
		DRM_INFO("Intel graphics LMEM IO start: %llx\n",
			 (u64)mem->io_start);
		DRM_INFO("Intel graphics LMEM size: %llx\n",
			 (u64)lmem_size);

		/* this is real device memory */
		mem->is_devmem = true;
	}

	return mem;
}

struct intel_memory_region *
i915_gem_setup_lmem(struct drm_i915_private *i915)
{
	return setup_lmem(i915);
}

