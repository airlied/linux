#ifndef I915_TTM_OBJECT_H
#define I915_TTM_OBJECT_H

struct i915_bo;

struct i915_ttm_bo *i915_ttm_bo_ref(struct i915_ttm_bo *bo);
void i915_ttm_bo_unref(struct i915_ttm_bo **bo);
int i915_ttm_bo_kmap(struct i915_ttm_bo *bo, void **ptr);
void *i915_ttm_bo_kptr(struct i915_ttm_bo *bo);
void i915_ttm_bo_kunmap(struct i915_ttm_bo *bo);
u64 i915_ttm_bo_gpu_offset(struct i915_ttm_bo *bo);
void i915_ttm_bo_placement_from_region(struct i915_ttm_bo *bo, u32 region);
int i915_ttm_bo_pin(struct i915_ttm_bo *bo, u32 region);
int i915_ttm_bo_unpin(struct i915_ttm_bo *bo);
bool i915_ttm_bo_is_i915_ttm_bo(struct ttm_buffer_object *bo);

int i915_ttm_alloc_gtt(struct ttm_buffer_object *tbo);
int i915_ttm_set_tiling(struct i915_ttm_bo *bo, unsigned int tiling,
			unsigned int stride);


bool i915_ttm_gtt_mgr_has_gart_addr(struct ttm_resource *mem);
#endif
