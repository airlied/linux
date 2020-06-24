
#include "i915_drv.h"

static int i915_ttm_gtt_mgr_init(struct ttm_mem_type_manager *man,
				  unsigned long p_size)
{
	struct drm_i915_private *i915 = to_i915_ttm_dev(man->bdev);

	return 0;
}

static int i915_ttm_gtt_mgr_fini(struct ttm_mem_type_manager *man)
{
	struct drm_i915_private *i915 = to_i915_ttm_dev(man->bdev);

	return 0;
}
				  
const struct ttm_mem_type_manager_func i915_ttm_gtt_mgr_func = {
	.init = i915_ttm_gtt_mgr_init,
	.takedown = i915_ttm_gtt_mgr_fini,
};
