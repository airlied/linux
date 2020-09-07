#ifndef I915_TTM_H
#define I915_TTM_H

/* Some notes on i915 TTM implementation.
 * This is meant for discrete GPU use only.
 * No relocations are to be supported.
 * EXEC_OBJECT_PINNED is assumed to be true.
 */
#include "drm/ttm/ttm_bo_driver.h"
#include "drm/ttm/ttm_execbuf_util.h"
#include "i915_ttm_object_types.h"
#include "i915_ttm_bo_list.h"
#define I915_TTM_BO_INVALID_OFFSET     LONG_MAX

struct drm_syncobj;
int i915_ttm_early_init(struct drm_i915_private *i915);
int i915_ttm_init(struct drm_i915_private *i915);
void i915_ttm_fini(struct drm_i915_private *i915);

#include "i915_ttm_object.h"

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

uint32_t i915_ttm_bo_get_preferred_pin_region(struct drm_i915_private *i915,
					      uint32_t region);

struct drm_i915_gem_object *i915_ttm_object_create_internal(struct drm_i915_private *i915, unsigned long size);

int
i915_ttm_do_execbuffer(struct drm_device *dev,
		       struct drm_file *file,
		       struct drm_i915_gem_execbuffer2 *args,
		       struct drm_i915_gem_exec_object2 *exec);

int i915_ttm_create_bo_pages(struct drm_i915_gem_object *obj);

int i915_ttm_vram_mgr_init(struct drm_i915_private *i915);
void i915_ttm_vram_mgr_fini(struct drm_i915_private *i915);

int i915_ttm_vram_get_pages(struct drm_i915_gem_object *obj);

int i915_ttm_gtt_mgr_init(struct drm_i915_private *i915);
void i915_ttm_gtt_mgr_fini(struct drm_i915_private *i915);

int i915_ttm_stolen_mgr_init(struct drm_i915_private *i915);
void i915_ttm_stolen_mgr_fini(struct drm_i915_private *i915);

int i915_ttm_stolen_get_pages(struct drm_i915_gem_object *obj);


struct drm_i915_gem_object *i915_ttm_object_create_region(struct intel_memory_region **placements,
							  int n_placements,
							  enum ttm_bo_type type,
							  unsigned long size);

static inline struct drm_i915_gem_object *ttm_to_i915_gem(struct ttm_buffer_object *tbo)
{
	return container_of(tbo, struct drm_i915_gem_object, base);
}

void __iomem *i915_ttm_pin_iomap(struct drm_i915_gem_object *obj);
#endif
