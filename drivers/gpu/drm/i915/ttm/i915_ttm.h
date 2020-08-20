#ifndef I915_TTM_H
#define I915_TTM_H

/* Some notes on i915 TTM implementation.
 * This is meant for discrete GPU use only.
 * No relocations are to be supported.
 * EXEC_OBJECT_PINNED is assumed to be true.
 */

#include "i915_ttm_object_types.h"
#include "i915_ttm_bo_list.h"
#define I915_TTM_BO_INVALID_OFFSET     LONG_MAX

struct drm_syncobj;

int i915_ttm_init(struct drm_i915_private *i915);
void i915_ttm_fini(struct drm_i915_private *i915);

#include "i915_ttm_object.h"

static inline unsigned long i915_ttm_bo_size(struct i915_ttm_bo *bo)
{
	return bo->tbo.num_pages << PAGE_SHIFT;
}


/**
 * i915_ttm_bo_mmap_offset - return mmap offset of bo
 * @bo:	i915_ttm object for which we query the offset
 *
 * Returns mmap offset of the object.
 */
static inline u64 i915_ttm_bo_mmap_offset(struct i915_ttm_bo *bo)
{
	return drm_vma_node_offset_addr(&bo->tbo.base.vma_node);
}

static inline int i915_ttm_bo_reserve(struct i915_ttm_bo *bo, bool no_intr)
{
	struct drm_i915_private *i915 = to_i915_ttm_dev(bo->tbo.bdev);
	int r;

	r = ttm_bo_reserve(&bo->tbo, !no_intr, false, NULL);
	if (unlikely(r != 0)) {
		if (r != -ERESTARTSYS)
			dev_err(i915->drm.dev, "%p reserve failed\n", bo);
		return r;
	}
	return 0;
}

static inline void i915_ttm_bo_unreserve(struct i915_ttm_bo *bo)
{
	ttm_bo_unreserve(&bo->tbo);
}

/**
 * i915_ttm_mem_type_to_region - return region corresponding to mem_type
 * @mem_type:	ttm memory type
 *
 * Returns corresponding region of the ttm mem_type
 */
static inline unsigned i915_ttm_mem_type_to_region(u32 mem_type)
{
	switch (mem_type) {
	case TTM_PL_VRAM:
		return REGION_LMEM;
	case TTM_PL_TT:
		return REGION_SMEM;
	case TTM_PL_SYSTEM:
		return 0;
	case I915_TTM_PL_STOLEN:
		return REGION_STOLEN_SMEM;
	default:
		break;
	}
	return 0;
}

int i915_ttm_bo_create_reserved(struct drm_i915_private *i915,
				unsigned long size, int align,
				u32 region, struct i915_ttm_bo **bo_ptr,
				u64 *gpu_addr, void **cpu_addr);

uint32_t i915_ttm_bo_get_preferred_pin_region(struct drm_i915_private *i915,
					      uint32_t region);

int i915_ttm_bo_create_kernel(struct drm_i915_private *i915,
			      unsigned long size, int align,
			      u32 region, struct i915_ttm_bo **bo_ptr,
			      u64 *gpu_addr, void **cpu_addr);
int i915_ttm_gem_object_create(struct drm_i915_private *i915, unsigned long size,
			       int alignment, u32 initial_region,
			       u64 flags, enum ttm_bo_type type,
			       struct dma_resv *resv,
			       struct drm_gem_object **obj);
void i915_ttm_gem_object_close(struct drm_gem_object *gem, struct drm_file *file);
void i915_ttm_gem_object_free(struct drm_gem_object *gobj);
int
i915_ttm_dumb_mmap_offset(struct drm_file *file,
			  u32 handle,
			  u64 *offset);

int
i915_ttm_mmap_offset_ioctl(struct drm_file *file,
			   u32 handle,
			   enum i915_mmap_type mmap_type,
			   u64 *offset);
int
i915_ttm_do_execbuffer(struct drm_device *dev,
		       struct drm_file *file,
		       struct drm_i915_gem_execbuffer2 *args,
		       struct drm_i915_gem_exec_object2 *exec);

int i915_ttm_create_bo_pages(struct i915_ttm_bo *bo);

int i915_ttm_vram_mgr_init(struct drm_i915_private *i915);
void i915_ttm_vram_mgr_fini(struct drm_i915_private *i915);

int i915_ttm_vram_get_pages(struct i915_ttm_bo *bo);

int i915_ttm_gtt_mgr_init(struct drm_i915_private *i915);
void i915_ttm_gtt_mgr_fini(struct drm_i915_private *i915);

int i915_ttm_stolen_mgr_init(struct drm_i915_private *i915);
void i915_ttm_stolen_mgr_fini(struct drm_i915_private *i915);

int i915_ttm_stolen_get_pages(struct i915_ttm_bo *bo);
#endif
