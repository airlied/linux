// SPDX-License-Identifier: MIT
/*
 * Copyright @ 2020 Red Hat.
 */
/* stolen memory is a bunch of pages we can only see via GTT mappings of those pages. */
/* Required to allocate a linear chunk of pages, followed by a linear chunk of GTT to map them into */
/* populate should come from the linear page chunk. */
#include "i915_drv.h"
#include "i915_ttm.h"

static inline struct i915_ttm_stolen_mgr *to_stolen_mgr(struct ttm_resource_manager *man)
{
	return container_of(man, struct i915_ttm_stolen_mgr, manager);
}

struct i915_ttm_stolen_node {
	struct drm_mm_node node;
	struct ttm_resource gtt_res;
};

static inline struct drm_i915_private *stolen_mgr_to_i915(struct ttm_resource_manager *man)
{
	return container_of(man, struct drm_i915_private, ttm_mman.stolen_mgr.manager);
}

static inline struct i915_ttm_stolen_node *to_stolen_node(struct drm_mm_node *node)
{
	return (struct i915_ttm_stolen_node *)node;
}

static const struct ttm_resource_manager_func i915_ttm_stolen_mgr_func;

int i915_ttm_stolen_mgr_init(struct drm_i915_private *i915)
{
	struct i915_ttm_stolen_mgr *mgr = &i915->ttm_mman.stolen_mgr;
	struct ttm_resource_manager *man = &mgr->manager;
	unsigned long p_size = i915->stolen_usable_size >> PAGE_SHIFT;

	man->use_tt = true;
	man->func = &i915_ttm_stolen_mgr_func;
	man->available_caching = TTM_PL_MASK_CACHING;
	man->default_caching = TTM_PL_FLAG_CACHED;

	ttm_resource_manager_init(man, p_size);
	drm_mm_init(&mgr->mm, 0, p_size);
	spin_lock_init(&mgr->lock);
	atomic64_set(&mgr->available, p_size);

	ttm_set_driver_manager(&i915->ttm_mman.bdev, I915_TTM_PL_STOLEN, &mgr->manager);
	ttm_resource_manager_set_used(man, true);

	return 0;
}

void i915_ttm_stolen_mgr_fini(struct drm_i915_private *i915)
{
	struct i915_ttm_stolen_mgr *mgr = &i915->ttm_mman.stolen_mgr;
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
	ttm_set_driver_manager(&i915->ttm_mman.bdev, TTM_PL_TT, NULL);
}

/**
 * i915_ttm_stolen_mgr_has_gart_addr - Check if mem has address space
 *
 * @mem: the mem object to check
 *
 * Check if a mem object has already address space allocated.
 */
bool i915_ttm_stolen_mgr_has_gart_addr(struct ttm_resource *mem)
{
	return mem->mm_node != NULL;
}

static int i915_ttm_stolen_mgr_new(struct ttm_resource_manager *man,
				 struct ttm_buffer_object *tbo,
				 const struct ttm_place *place,
				 struct ttm_resource *mem)
{
	struct i915_ttm_stolen_mgr *mgr = to_stolen_mgr(man);
	struct i915_ttm_stolen_node *node;
	struct drm_i915_private *i915 = stolen_mgr_to_i915(man);
	struct ttm_resource_manager *gtt_mgr = ttm_manager_type(tbo->bdev, TTM_PL_TT);
	unsigned long lpfn;
	int r;

	spin_lock(&mgr->lock);
	if ((&tbo->mem == mem || tbo->mem.mem_type != TTM_PL_TT) &&
	    atomic64_read(&mgr->available) < mem->num_pages) {
		spin_unlock(&mgr->lock);
		return 0;
	}
	atomic64_sub(mem->num_pages, &mgr->available);
	spin_unlock(&mgr->lock);

	lpfn = place->lpfn;
	if (!lpfn)
		lpfn = man->size;
	node = kzalloc(sizeof(*node), GFP_KERNEL);
	if (!node) {
		r = -ENOMEM;
		goto err_out;
	}

	/* find space in stolen address space first */
	spin_lock(&mgr->lock);
	r = drm_mm_insert_node_in_range(&mgr->mm, &node->node, mem->num_pages,
					mem->page_alignment, 0, place->fpfn,
					lpfn, DRM_MM_INSERT_BEST);
	spin_unlock(&mgr->lock);
	if (unlikely(r)) {
		goto err_free;
	}

	{
		struct ttm_place gtt_place = *place;
		gtt_place.fpfn = 0;
		/* force gtt mgr to give us a node */
		gtt_place.lpfn = i915->ggtt.vm.total >> PAGE_SHIFT;
		node->gtt_res = *mem;
		node->gtt_res.mm_node = NULL;
		/* allocate a gtt node as well */
		r = gtt_mgr->func->alloc(gtt_mgr, tbo, &gtt_place, &node->gtt_res);
		if (unlikely(r)) {
			goto err_free;
		}
	}
	mem->mm_node = node;
	mem->start = node->node.start;

	return 0;
err_free:
	kfree(node);
err_out:
	atomic64_add(mem->num_pages, &mgr->available);
	return r;
}

static void i915_ttm_stolen_mgr_del(struct ttm_resource_manager *man,
				  struct ttm_resource *mem)
{
	struct i915_ttm_stolen_mgr *mgr = to_stolen_mgr(man);
	struct i915_ttm_stolen_node *node = mem->mm_node;
	struct drm_i915_private *i915 = stolen_mgr_to_i915(man);
	struct ttm_resource_manager *gtt_mgr = ttm_manager_type(&i915->ttm_mman.bdev, TTM_PL_TT);
	if (!node)
		return;

	spin_lock(&mgr->lock);
	drm_mm_remove_node(&node->node);
	spin_unlock(&mgr->lock);
	atomic64_add(mem->num_pages, &mgr->available);
	gtt_mgr->func->free(gtt_mgr, &node->gtt_res);
	kfree(node);
	mem->mm_node = NULL;
}

static const struct ttm_resource_manager_func i915_ttm_stolen_mgr_func = {
	.alloc = i915_ttm_stolen_mgr_new,
	.free = i915_ttm_stolen_mgr_del,
};

int i915_ttm_stolen_get_pages(struct drm_i915_gem_object *obj)
{
	struct drm_i915_private *i915 = obj_to_i915(obj);
	struct sg_table *st;
	struct scatterlist *sg;
	struct ttm_resource *mem = &obj->base.mem;
	struct drm_mm_node *node = mem->mm_node;

	st = kmalloc(sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	if (sg_alloc_table(st, 1, GFP_KERNEL)) {
		kfree(st);
		return -ENOMEM;
	}

	sg = st->sgl;
	sg->offset = 0;
	sg->length = obj->base.num_pages * PAGE_SIZE;

	sg_dma_address(sg) = (dma_addr_t)i915->dsm.start + node->start;
	sg_dma_len(sg) = sg->length;

	obj->mm.pages = st;
	obj->mm.page_sizes.phys = PAGE_SIZE;
	obj->mm.page_sizes.sg = PAGE_SIZE;
	obj->mm.page_sizes.gtt = PAGE_SIZE;
	return 0;
}

unsigned long i915_ttm_stolen_obj_get_gtt_offset(struct ttm_resource *mem)
{
	struct i915_ttm_stolen_node *node = mem->mm_node;

	if (!mem) {
		WARN_ON(1);
		return 0;
	}
	if (!node)
	    return 0;
	return node->gtt_res.start;
}
