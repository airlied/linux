
#include "i915_drv.h"

static inline struct i915_ttm_vram_mgr *to_vram_mgr(struct ttm_resource_manager *man)
{
	return container_of(man, struct i915_ttm_vram_mgr, manager);
}

/**
 * i915_ttm_vram_mgr_virt_start - update virtual start address
 *
 * @mem: ttm_resource to update
 * @node: just allocated node
 *
 * Calculate a virtual BO start address to easily check if everything is CPU
 * accessible.
 */
static void i915_ttm_vram_mgr_virt_start(struct ttm_resource *mem,
					 struct drm_mm_node *node)
{
	unsigned long start;

	start = node->start + node->size;
	if (start > mem->num_pages)
		start -= mem->num_pages;
	else
		start = 0;
	mem->start = max(mem->start, start);
}

static const struct ttm_resource_manager_func i915_ttm_vram_mgr_func;
int i915_ttm_vram_mgr_init(struct drm_i915_private *i915)
{
	struct i915_ttm_vram_mgr *mgr;
	struct ttm_resource_manager *man;
	unsigned long vram_size;
	int ret;

	if (!HAS_LMEM(i915))
		return 0;

	vram_size = i915->mm.regions[INTEL_REGION_LMEM]->total >> PAGE_SHIFT;

	mgr = &i915->ttm_mman.vram_mgr;
	man = &mgr->manager;

	man->available_caching = TTM_PL_FLAG_UNCACHED | TTM_PL_FLAG_WC;
	man->default_caching = TTM_PL_FLAG_WC;
	man->func = &i915_ttm_vram_mgr_func;

	ttm_resource_manager_init(man, vram_size);
	drm_mm_init(&mgr->mm, 0, man->size);
	spin_lock_init(&mgr->lock);

	ttm_set_driver_manager(&i915->ttm_mman.bdev, TTM_PL_VRAM, &mgr->manager);
	ttm_resource_manager_set_used(man, true);
	return 0;
}

void i915_ttm_vram_mgr_fini(struct drm_i915_private *i915)
{
	struct i915_ttm_vram_mgr *mgr = &i915->ttm_mman.vram_mgr;
	struct ttm_resource_manager *man = &mgr->manager;
	int ret;

	ttm_resource_manager_set_used(man, false);

	ret = ttm_resource_manager_force_list_clean(&i915->ttm_mman.bdev, man);
	if (ret)
		return;
	spin_lock(&mgr->lock);
	drm_mm_takedown(&mgr->mm);
	spin_unlock(&mgr->lock);

	ttm_resource_manager_cleanup(man);
	ttm_set_driver_manager(&i915->ttm_mman.bdev, TTM_PL_VRAM, NULL);
}

static int i915_ttm_vram_mgr_new(struct ttm_resource_manager *man,
				 struct ttm_buffer_object *tbo,
				 const struct ttm_place *place,
				 struct ttm_resource *mem)
{
	struct i915_ttm_vram_mgr *mgr = to_vram_mgr(man);
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

//		vis_usage += i915_ttm_vram_mgr_vis_size(adev, &nodes[i]);
		i915_ttm_vram_mgr_virt_start(mem, &nodes[i]);
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

//		vis_usage += i915_ttm_vram_mgr_vis_size(adev, &nodes[i]);
		i915_ttm_vram_mgr_virt_start(mem, &nodes[i]);
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

static void i915_ttm_vram_mgr_del(struct ttm_resource_manager *man,
				  struct ttm_resource *mem)
{
	struct i915_ttm_vram_mgr *mgr = to_vram_mgr(man);
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

static const struct ttm_resource_manager_func i915_ttm_vram_mgr_func = {
	.get_node = i915_ttm_vram_mgr_new,
	.put_node = i915_ttm_vram_mgr_del,
};

int i915_ttm_vram_get_pages(struct i915_ttm_bo *bo)
{
	struct drm_i915_private *i915 = to_i915_ttm_dev(bo->tbo.bdev);
	struct sg_table *st;
	struct scatterlist *sg;
	struct ttm_resource *mem = &bo->tbo.mem;
	struct drm_mm_node *nodes = mem->mm_node;
	unsigned pages = mem->num_pages;

	st = kmalloc(sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	/* wtf rewrite */
	if (sg_alloc_table(st, (bo->tbo.num_pages * PAGE_SIZE) >> ilog2(2*1024*1024), GFP_KERNEL)) {
		kfree(st);
		return -ENOMEM;
	}


	sg = st->sgl;
	st->nents = 0;

	while (pages) {
		pages -= nodes->size;

		sg_dma_address(sg) = i915->mm.regions[INTEL_MEMORY_LOCAL]->io_start + nodes->start;

		sg_dma_len(sg) = nodes->size;
		sg->length = nodes->size;
		st->nents++;
		++nodes;
	}
	sg_mark_end(sg);
	bo->pages = st;	
	return 0;
}
