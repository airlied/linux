#ifndef I915_TTM_OBJECT_H
#define I915_TTM_OBJECT_H

int i915_ttm_bo_kmap(struct drm_i915_gem_object *obj, void **ptr);
void *i915_ttm_bo_kptr(struct drm_i915_gem_object *obj);
void i915_ttm_bo_kunmap(struct drm_i915_gem_object *obj);
void i915_ttm_bo_placement_from_region(struct drm_i915_gem_object *obj, u32 region);
int i915_ttm_bo_unpin(struct drm_i915_gem_object *obj);
bool i915_ttm_bo_is_i915_ttm_bo(struct ttm_buffer_object *bo);

int i915_ttm_alloc_gtt(struct ttm_buffer_object *tbo);

int i915_ttm_bo_pin(struct drm_i915_gem_object *obj, u32 region);

bool i915_ttm_gtt_mgr_has_gart_addr(struct ttm_resource *mem);

unsigned long i915_ttm_vram_obj_get_gtt_offset(struct ttm_resource *mem);
bool i915_ttm_vram_obj_premap_allowed(struct ttm_resource *mem);
#endif
