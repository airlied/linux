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
		DRM_INFO("Intel graphics LMEM: %pR\n", &mem->region);
		DRM_INFO("Intel graphics LMEM IO start: %llx\n",
			 (u64)mem->io_start);
		DRM_INFO("Intel graphics LMEM size: %llx\n",
			 (u64)lmem_size);
	}

	return mem;
}

struct intel_memory_region *
i915_gem_setup_lmem(struct drm_i915_private *i915)
{
	return setup_lmem(i915);
}

