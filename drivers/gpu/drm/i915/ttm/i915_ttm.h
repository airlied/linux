#ifndef I915_TTM_H
#define I915_TTM_H

#include "i915_ttm_object_types.h"

int i915_ttm_init(struct drm_i915_private *i915);
void i915_ttm_fini(struct drm_i915_private *i915);
extern const struct ttm_mem_type_manager_func i915_ttm_gtt_mgr_func;
extern const struct ttm_mem_type_manager_func i915_ttm_vram_mgr_func;

#endif
