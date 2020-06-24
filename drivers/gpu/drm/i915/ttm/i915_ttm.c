
#include <linux/dma-mapping.h>
#include <linux/iommu.h>
#include <linux/hmm.h>
#include <linux/pagemap.h>
#include <linux/sched/task.h>
#include <linux/sched/mm.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/swap.h>
#include <linux/swiotlb.h>
#include <linux/dma-buf.h>
#include <linux/sizes.h>

#include <drm/ttm/ttm_bo_api.h>
#include <drm/ttm/ttm_bo_driver.h>
#include <drm/ttm/ttm_placement.h>
#include <drm/ttm/ttm_module.h>
#include <drm/ttm/ttm_page_alloc.h>

#include <drm/drm_debugfs.h>

#include "i915_drv.h"
#include "i915_ttm.h"

static int i915_ttm_init_lmem(struct drm_i915_private *i915)
{
	struct ttm_mem_type_manager *man = &i915->ttm_mman.bdev.man[TTM_PL_VRAM];

	man->func = &i915_ttm_vram_mgr_func;
	man->available_caching = TTM_PL_FLAG_UNCACHED | TTM_PL_FLAG_WC;
	man->default_caching = TTM_PL_FLAG_WC;
	return ttm_bo_init_mm(&i915->ttm_mman.bdev, TTM_PL_VRAM,
			      8);
}

struct i915_ttm_tt {
	struct ttm_dma_tt ttm;
	struct drm_gem_object *gobj;
	u64 offset;
};

static int i915_ttm_backend_bind(struct ttm_tt *ttm,
				 struct ttm_mem_reg *bo_mem)
{
	return 0;
}

static void i915_ttm_backend_unbind(struct ttm_tt *ttm)
{
}

static void i915_ttm_backend_destroy(struct ttm_tt *ttm)
{

	struct i915_ttm_tt *gtt = (void *)ttm;

	ttm_dma_tt_fini(&gtt->ttm);
	kfree(gtt);
}

static struct ttm_backend_func i915_ttm_backend_func = {
	.bind = &i915_ttm_backend_bind,
	.unbind = &i915_ttm_backend_unbind,
	.destroy = &i915_ttm_backend_destroy,
};

static struct ttm_tt *i915_ttm_tt_create(struct ttm_buffer_object *bo,
					 uint32_t page_flags)
{
	struct i915_ttm_tt *gtt;

	gtt = kzalloc(sizeof(struct i915_ttm_tt), GFP_KERNEL);
	if (gtt == NULL)
		return NULL;

	gtt->ttm.ttm.func = &i915_ttm_backend_func;
	gtt->gobj = &bo->base;

	if (ttm_sg_tt_init(&gtt->ttm, bo, page_flags)) {
		kfree(gtt);
		return NULL;
	}
	return &gtt->ttm.ttm;
}

static int i915_ttm_tt_populate(struct ttm_tt *ttm,
				struct ttm_operation_ctx *ctx)
{
	return 0;
}

static void i915_ttm_tt_unpopulate(struct ttm_tt *ttm)
{

}

static struct ttm_bo_driver i915_ttm_bo_driver = {
	.ttm_tt_create = &i915_ttm_tt_create,
	.ttm_tt_populate = &i915_ttm_tt_populate,
	.ttm_tt_unpopulate = &i915_ttm_tt_unpopulate,
#if 0
	.eviction_valuable = amdgpu_ttm_bo_eviction_valuable,
	.evict_flags = &amdgpu_evict_flags,
	.move = &amdgpu_bo_move,
	.verify_access = &amdgpu_verify_access,
	.move_notify = &amdgpu_bo_move_notify,
	.release_notify = &amdgpu_bo_release_notify,
	.fault_reserve_notify = &amdgpu_bo_fault_reserve_notify,
	.io_mem_reserve = &amdgpu_ttm_io_mem_reserve,
	.io_mem_free = &amdgpu_ttm_io_mem_free,
	.io_mem_pfn = amdgpu_ttm_io_mem_pfn,
	.access_memory = &amdgpu_ttm_access_memory,
	.del_from_lru_notify = &amdgpu_vm_del_from_lru_notify
#endif
};

int i915_ttm_init(struct drm_i915_private *i915)
{
	int r;
	r = ttm_bo_device_init(&i915->ttm_mman.bdev,
			       &i915_ttm_bo_driver,
			       i915->drm.anon_inode->i_mapping,
			       i915->drm.vma_offset_manager,
			       false);

	if (r) {
		DRM_ERROR("failed initialiseing TTM(%d)\n", r);
		return r;
	}
	i915->ttm_mman.initialized = true;

	r = i915_ttm_init_lmem(i915);
	if (r) {
		DRM_ERROR("failed to init TTM heap.\n");
		return r;
	}

	return 0;
}

void i915_ttm_fini(struct drm_i915_private *i915)
{
	if (!i915->ttm_mman.initialized)
		return;

	ttm_bo_clean_mm(&i915->ttm_mman.bdev, TTM_PL_VRAM);
	ttm_bo_clean_mm(&i915->ttm_mman.bdev, TTM_PL_TT);

	ttm_bo_device_release(&i915->ttm_mman.bdev);
	i915->ttm_mman.initialized = false;
	DRM_INFO("i915: ttm finialized\n");
}
