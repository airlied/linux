
#include "i915_drv.h"

struct i915_ttm_vram_mgr {
	struct drm_mm mm;
	spinlock_t lock;
	atomic64_t usage;
};
	
static int i915_ttm_vram_mgr_init(struct ttm_mem_type_manager *man,
				  unsigned long p_size)
{
	struct drm_i915_private *i915 = to_i915_ttm_dev(man->bdev);
	struct i915_ttm_vram_mgr *mgr;

	int ret;

	mgr = kzalloc(sizeof(*mgr), GFP_KERNEL);
	if (!mgr)
		return -ENOMEM;

	drm_mm_init(&mgr->mm, 0, p_size);
	spin_lock_init(&mgr->lock);
	man->priv = mgr;

	return 0;
}

static int i915_ttm_vram_mgr_fini(struct ttm_mem_type_manager *man)
{
	struct drm_i915_private *i915 = to_i915_ttm_dev(man->bdev);
	struct i915_ttm_vram_mgr *mgr = man->priv;
	spin_lock(&mgr->lock);
	drm_mm_takedown(&mgr->mm);
	spin_unlock(&mgr->lock);
	kfree(mgr);
	man->priv = NULL;
	return 0;
}

static int i915_ttm_vram_mgr_new(struct ttm_mem_type_manager *man,
				 struct ttm_buffer_object *tbo,
				 const struct ttm_place *place,
				 struct ttm_mem_reg *mem)
{

	struct drm_i915_private *i915 = to_i915_ttm_dev(man->bdev);
	struct i915_ttm_vram_mgr *mgr = man->priv;
	struct drm_mm *mm = &mgr->mm;
	struct drm_mm_node *nodes;
	enum drm_mm_insert_mode mode;
	unsigned long lpfn, num_nodes, pages_per_node, pages_left;
	uint64_t mem_bytes, max_bytes;
	unsigned i;
	int r;

	lpfn = place->lpfn;
	if (!lpfn)
		lpfn = man->size;


	if (place->flags & TTM_PL_FLAG_CONTIGUOUS) {
		num_nodes = 1;
		pages_per_node = ~0ul;
	} else {

		pages_per_node = (2UL << (20UL - PAGE_SHIFT));

		pages_per_node = max((uint32_t)pages_per_node, mem->page_alignment);
		num_nodes = DIV_ROUND_UP(mem->num_pages, pages_per_node);
	}

	nodes = kvmalloc_array((uint32_t)num_nodes, sizeof(*nodes),
			       GFP_KERNEL | __GFP_ZERO);
	if (!nodes) {
		atomic64_sub(mem_bytes, &mgr->usage);
		return -ENOMEM;
	}
	mode = DRM_MM_INSERT_BEST;
	if (place->flags & TTM_PL_FLAG_TOPDOWN)
		mode = DRM_MM_INSERT_HIGH;

	mem->start = 0;
	pages_left = mem->num_pages;

	spin_lock(&mgr->lock);
	for (i = 0; pages_left >= pages_per_node; ++i) {
		unsigned long pages = rounddown_pow_of_two(pages_left);

		r = drm_mm_insert_node_in_range(mm, &nodes[i], pages,
						pages_per_node, 0,
						place->fpfn, lpfn,
						mode);
		if (unlikely(r))
			break;

//		vis_usage += amdgpu_vram_mgr_vis_size(adev, &nodes[i]);
//		amdgpu_vram_mgr_virt_start(mem, &nodes[i]);
		pages_left -= pages;
	}
	
	for (; pages_left; ++i) {
		unsigned long pages = min(pages_left, pages_per_node);
		uint32_t alignment = mem->page_alignment;

		if (pages == pages_per_node)
			alignment = pages_per_node;

		r = drm_mm_insert_node_in_range(mm, &nodes[i],
						pages, alignment, 0,
						place->fpfn, lpfn,
						mode);
		if (unlikely(r))
			goto error;

//		vis_usage += amdgpu_vram_mgr_vis_size(adev, &nodes[i]);
//		amdgpu_vram_mgr_virt_start(mem, &nodes[i]);
		pages_left -= pages;
	}
	spin_unlock(&mgr->lock);

//	atomic64_add(vis_usage, &mgr->vis_usage);

	mem->mm_node = nodes;
	
	
	return 0;

error:
	while (i--)
		drm_mm_remove_node(&nodes[i]);
	spin_unlock(&mgr->lock);
	atomic64_sub(mem->num_pages << PAGE_SHIFT, &mgr->usage);

	kvfree(nodes);
	return r == -ENOSPC ? 0 : r;
}

static void i915_ttm_vram_mgr_del(struct ttm_mem_type_manager *man,
				  struct ttm_mem_reg *mem)
{
	struct drm_i915_private *i915 = to_i915_ttm_dev(man->bdev);
	struct i915_ttm_vram_mgr *mgr = man->priv;
	struct drm_mm_node *nodes = mem->mm_node;
	unsigned pages = mem->num_pages;
	uint64_t usage = 0;
	if (!mem->mm_node)
		return;

	spin_lock(&mgr->lock);
	while (pages) {
		pages -= nodes->size;
		drm_mm_remove_node(nodes);
		usage += nodes->size << PAGE_SHIFT;
		++nodes;
	}
	spin_unlock(&mgr->lock);
	kvfree(mem->mm_node);
	mem->mm_node = NULL;
}
				  
const struct ttm_mem_type_manager_func i915_ttm_vram_mgr_func = {
	.init = i915_ttm_vram_mgr_init,
	.takedown = i915_ttm_vram_mgr_fini,
	.get_node = i915_ttm_vram_mgr_new,
	.put_node = i915_ttm_vram_mgr_del,
	.debug = NULL,
};
