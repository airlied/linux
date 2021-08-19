#include <drm/ttm/ttm_range_manager.h>
#include <drm/ttm/ttm_placement.h>
#include "xe_bo.h"
#include "xe_device.h"

static inline struct xe_ttm_vram_mgr *
to_vram_mgr(struct ttm_resource_manager *man)
{
	return container_of(man, struct xe_ttm_vram_mgr, manager);
}

static inline struct xe_device *
mgr_to_xe_device(struct xe_ttm_vram_mgr *mgr)
{
	return container_of(mgr, struct xe_device, vram_mgr);
}

static void xe_ttm_vram_mgr_virt_start(struct ttm_resource *mem,
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

static int xe_ttm_vram_mgr_new(struct ttm_resource_manager *man,
			       struct ttm_buffer_object *tbo,
			       const struct ttm_place *place,
			       struct ttm_resource **res)
{
	unsigned long lpfn, num_nodes, pages_per_node, pages_left, pages;
	struct xe_ttm_vram_mgr *mgr = to_vram_mgr(man);
	struct xe_device *xe = mgr_to_xe_device(mgr);
	uint64_t mem_bytes, max_bytes;
	struct ttm_range_mgr_node *node;
	struct drm_mm *mm = &mgr->mm;
	enum drm_mm_insert_mode mode;
	unsigned i;
	int r;

	lpfn = place->lpfn;
	if (!lpfn)
		lpfn = man->size;

	max_bytes = xe->vram.size;

	/* bail out quickly if there's likely not enough VRAM for this BO */
	mem_bytes = tbo->base.size;
	if (atomic64_add_return(mem_bytes, &mgr->usage) > max_bytes) {
		r = -ENOSPC;
		goto error_sub;
	}

	if (place->flags & TTM_PL_FLAG_CONTIGUOUS) {
		pages_per_node = ~0ul;
		num_nodes = 1;
	} else {
		/* default to 2MB */
		pages_per_node = 2UL << (20UL - PAGE_SHIFT);
		pages_per_node = max_t(uint32_t, pages_per_node,
				       tbo->page_alignment);
		num_nodes = DIV_ROUND_UP_ULL(PFN_UP(mem_bytes), pages_per_node);
	}

	node = kvmalloc(struct_size(node, mm_nodes, num_nodes),
			GFP_KERNEL | __GFP_ZERO);
	if (!node) {
		r = -ENOMEM;
		goto error_sub;
	}

	ttm_resource_init(tbo, place, &node->base);

	mode = DRM_MM_INSERT_BEST;
	if (place->flags & TTM_PL_FLAG_TOPDOWN)
		mode = DRM_MM_INSERT_HIGH;

	pages_left = node->base.num_pages;

	/* Limit maximum size to 2GB due to SG table limitations */
	pages = min(pages_left, 2UL << (30 - PAGE_SHIFT));

	i = 0;
	spin_lock(&mgr->lock);
	while (pages_left) {
		uint32_t alignment = tbo->page_alignment;

		if (pages >= pages_per_node)
			alignment = pages_per_node;

		r = drm_mm_insert_node_in_range(mm, &node->mm_nodes[i], pages,
						alignment, 0, place->fpfn,
						lpfn, mode);
		if (unlikely(r)) {
			if (pages > pages_per_node) {
				if (is_power_of_2(pages))
					pages = pages / 2;
				else
					pages = rounddown_pow_of_two(pages);
				continue;
			}
			goto error_free;
		}

		xe_ttm_vram_mgr_virt_start(&node->base, &node->mm_nodes[i]);
		pages_left -= pages;
		++i;

		if (pages > pages_left)
			pages = pages_left;
	}
	spin_unlock(&mgr->lock);

	if (i == 1)
		node->base.placement |= TTM_PL_FLAG_CONTIGUOUS;

	*res = &node->base;
	return 0;

error_free:
	while (i--)
		drm_mm_remove_node(&node->mm_nodes[i]);
	spin_unlock(&mgr->lock);
	kvfree(node);
	return r;

error_sub:
	atomic64_sub(mem_bytes, &mgr->usage);
	return r;
}

static void xe_ttm_vram_mgr_del(struct ttm_resource_manager *man,
				struct ttm_resource *res)
{
	struct ttm_range_mgr_node *node = to_ttm_range_mgr_node(res);
	struct xe_ttm_vram_mgr *mgr = to_vram_mgr(man);
	uint64_t usage = 0;
	int i, pages;

	spin_lock(&mgr->lock);
	for (i = 0, pages = res->num_pages; pages;
	     pages -= node->mm_nodes[i].size, ++i) {
		struct drm_mm_node *mm = &node->mm_nodes[i];

		drm_mm_remove_node(mm);
		usage += mm->size << PAGE_SHIFT;
	}

	spin_unlock(&mgr->lock);

	atomic64_sub(usage, &mgr->usage);

	kvfree(node);
}

static void xe_ttm_vram_mgr_debug(struct ttm_resource_manager *man,
				  struct drm_printer *printer)
{
	struct xe_ttm_vram_mgr *mgr = to_vram_mgr(man);
	spin_lock(&mgr->lock);
	drm_mm_print(&mgr->mm, printer);
	spin_unlock(&mgr->lock);

	drm_printf(printer, "man size:%llu pages\n",
		   man->size);
}

static const struct ttm_resource_manager_func xe_ttm_vram_mgr_func = {
	.alloc	= xe_ttm_vram_mgr_new,
	.free	= xe_ttm_vram_mgr_del,
	.debug	= xe_ttm_vram_mgr_debug
};

int xe_ttm_vram_mgr_init(struct xe_device *xe)
{
	struct xe_ttm_vram_mgr *mgr = &xe->vram_mgr;
	struct ttm_resource_manager *man = &mgr->manager;

	man->func = &xe_ttm_vram_mgr_func;

	ttm_resource_manager_init(man, xe->vram.size >> PAGE_SHIFT);

	drm_mm_init(&mgr->mm, 0, man->size);
	spin_lock_init(&mgr->lock);
	ttm_set_driver_manager(&xe->ttm, TTM_PL_VRAM, &mgr->manager);
	ttm_resource_manager_set_used(man, true);
	return 0;
}

void xe_ttm_vram_mgr_fini(struct xe_device *xe)
{
	struct xe_ttm_vram_mgr *mgr = &xe->vram_mgr;
	struct ttm_resource_manager *man = &mgr->manager;
	int err;

	ttm_resource_manager_set_used(man, false);

	err = ttm_resource_manager_evict_all(&xe->ttm, man);
	if (err)
		return;

	spin_lock(&mgr->lock);
	drm_mm_takedown(&mgr->mm);
	spin_unlock(&mgr->lock);

	ttm_resource_manager_cleanup(man);
	ttm_set_driver_manager(&xe->ttm, TTM_PL_VRAM, NULL);
}
