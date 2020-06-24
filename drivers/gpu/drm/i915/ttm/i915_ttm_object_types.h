#ifndef I915_TTM_OBJECT_TYPES_H
#define I915_TTM_OBJECT_TYPES_H

struct i915_ttm_mman {
	struct ttm_bo_device bdev;
	bool mem_global_referenced;
	bool initialized;
};

struct i915_ttm_object {
	struct ttm_buffer_object base;
	struct ttm_placement placement;
};

static inline struct i915_ttm_object *ttm_to_i915_object(struct ttm_buffer_object *tbo)
{
	return container_of(tbo, struct i915_ttm_object, base);
}
#endif
