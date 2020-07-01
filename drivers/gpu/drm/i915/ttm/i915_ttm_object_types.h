#ifndef I915_TTM_OBJECT_TYPES_H
#define I915_TTM_OBJECT_TYPES_H

#include <drm/ttm/ttm_bo_api.h>
#include <drm/ttm/ttm_bo_driver.h>
#include <drm/ttm/ttm_placement.h>

#include "i915_vma_types.h"

#define I915_TTM_BO_MAX_PLACEMENTS	3

#define I915_TTM_CREATE_VRAM_CONTIGUOUS (1 << 0)
#define I915_TTM_CREATE_CPU_GTT_USWC (1 << 1)

struct i915_ttm_mman {
	struct ttm_bo_device bdev;
	bool mem_global_referenced;
	bool initialized;
	void __iomem *aper_base_kaddr;
};


struct i915_ttm_bo_param {
	unsigned long			size;
	int				byte_align;
	u32 region;
	u32 preferred_region;
	u64				flags;
	enum ttm_bo_type		type;
	bool				no_wait_gpu;
	struct dma_resv	*resv;
};

struct i915_ttm_bo {
	u32 preferred_regions;
	u32 allowed_regions;
	struct ttm_buffer_object tbo;
	struct ttm_place placements[I915_TTM_BO_MAX_PLACEMENTS];
	struct ttm_placement placement;
	struct ttm_bo_kmap_obj kmap;
	u64 flags;
	unsigned pin_count;
	unsigned prime_shared_count;

	struct intel_frontbuffer __rcu *frontbuffer;

	struct i915_object_vmas vma;
};

static inline struct i915_ttm_bo *ttm_to_i915_bo(struct ttm_buffer_object *tbo)
{
	return container_of(tbo, struct i915_ttm_bo, tbo);
}

static inline struct i915_ttm_bo *ttm_gem_to_i915_bo(struct drm_gem_object *gem)
{
	return container_of(gem, struct i915_ttm_bo, tbo.base);
}
#endif
