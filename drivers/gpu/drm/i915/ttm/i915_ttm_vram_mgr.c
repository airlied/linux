// SPDX-License-Identifier: MIT
/*
 * Copyright @ 2020 Red Hat.
 */
#include "i915_drv.h"
#include "gem/i915_gem_object_types.h"
#include "ttm/i915_ttm.h"
static inline struct i915_ttm_vram_mgr *to_vram_mgr(struct ttm_resource_manager *man)
{
	return container_of(man, struct i915_ttm_vram_mgr, manager);
}

struct i915_ttm_vram_node {
	struct drm_mm_node *nodes;
	struct ttm_resource gtt_res;
};

static inline struct drm_i915_private *vram_mgr_to_i915(struct ttm_resource_manager *man)
{
	return container_of(man, struct drm_i915_private, ttm_mman.vram_mgr.manager);
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

	if (!HAS_LMEM(i915))
		return 0;

	vram_size = i915->mm.regions[INTEL_REGION_LMEM]->total >> PAGE_SHIFT;

	mgr = &i915->ttm_mman.vram_mgr;
	man = &mgr->manager;

	man->use_tt = true;
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
	uint64_t mem_bytes;
	struct i915_ttm_vram_node *node;
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

	node = kzalloc(sizeof(*node), GFP_KERNEL);
	if (!node) {
		r = -ENOMEM;
		goto error;
	}

	node->nodes = kvmalloc_array((uint32_t)num_nodes, sizeof(*nodes),
				     GFP_KERNEL | __GFP_ZERO);
	if (!node->nodes) {
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

		r = drm_mm_insert_node_in_range(mm, &node->nodes[i], pages,
						pages_per_node, 0,
						place->fpfn, lpfn,
						mode);
		if (unlikely(r))
			break;

//		vis_usage += i915_ttm_vram_mgr_vis_size(adev, &nodes[i]);
		i915_ttm_vram_mgr_virt_start(mem, &node->nodes[i]);
		pages_left -= pages;
	}

	for (; pages_left; ++i) {
		unsigned long pages = min(pages_left, pages_per_node);
		uint32_t alignment = mem->page_alignment;

		if (pages == pages_per_node)
			alignment = pages_per_node;

		r = drm_mm_insert_node_in_range(mm, &node->nodes[i],
						pages, alignment, 0,
						place->fpfn, lpfn,
						mode);
		if (unlikely(r))
			goto error;

//		vis_usage += i915_ttm_vram_mgr_vis_size(adev, &nodes[i]);
		i915_ttm_vram_mgr_virt_start(mem, &node->nodes[i]);
		pages_left -= pages;
	}
	spin_unlock(&mgr->lock);

	node->gtt_res.start = I915_TTM_BO_INVALID_OFFSET;
	mem->mm_node = node;
	mem->start = node->nodes[0].start;
	return 0;

error:
	while (i--)
		drm_mm_remove_node(&node->nodes[i]);
	spin_unlock(&mgr->lock);
	atomic64_sub(mem->num_pages << PAGE_SHIFT, &mgr->usage);

	kvfree(node->nodes);
	kfree(node);

	return r == -ENOSPC ? 0 : r;
}

static void i915_ttm_vram_mgr_del(struct ttm_resource_manager *man,
				  struct ttm_resource *mem)
{
	struct i915_ttm_vram_mgr *mgr = to_vram_mgr(man);
	struct i915_ttm_vram_node *node = mem->mm_node;
	struct drm_mm_node *nodes;
	unsigned pages = mem->num_pages;
	uint64_t usage = 0;
	struct drm_i915_private *i915 = vram_mgr_to_i915(man);
	struct ttm_resource_manager *gtt_mgr = ttm_manager_type(&i915->ttm_mman.bdev, TTM_PL_TT);

	if (!node)
		return;

	nodes = node->nodes;
	spin_lock(&mgr->lock);
	while (pages) {
		pages -= nodes->size;
		drm_mm_remove_node(nodes);
		usage += nodes->size << PAGE_SHIFT;
		++nodes;
	}
	spin_unlock(&mgr->lock);
	gtt_mgr->func->free(gtt_mgr, &node->gtt_res);

	kvfree(node->nodes);
	kfree(node);
	mem->mm_node = NULL;
}

static const struct ttm_resource_manager_func i915_ttm_vram_mgr_func = {
	.alloc = i915_ttm_vram_mgr_new,
	.free = i915_ttm_vram_mgr_del,
};

int i915_ttm_vram_get_pages(struct drm_i915_gem_object *obj)
{
	struct sg_table *st;
	struct scatterlist *sg;
	struct ttm_resource *mem = &obj->base.mem;
	struct i915_ttm_vram_node *node = mem->mm_node;
	struct drm_mm_node *nodes = node->nodes;
	unsigned pages = mem->num_pages;
	int num_entries = 0;

	if (!mem->mm_node)
		return -EINVAL;

	st = kmalloc(sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	for (pages = mem->num_pages, nodes = node->nodes;
	     pages; pages -= nodes->size, ++nodes)
		++num_entries;

	pages = mem->num_pages;
	nodes = node->nodes;
	/* wtf rewrite */
	if (sg_alloc_table(st, num_entries, GFP_KERNEL)) {
		kfree(st);
		return -ENOMEM;
	}


	sg = st->sgl;
	st->nents = 0;

	while (pages) {
		pages -= nodes->size;

		sg_dma_address(sg) = nodes->start << PAGE_SHIFT;

		sg_dma_len(sg) = nodes->size << PAGE_SHIFT;
		sg->length = nodes->size << PAGE_SHIFT;
		sg++;
		st->nents++;
		++nodes;
	}
	sg_mark_end(sg);
	obj->mm.pages = st;
	obj->mm.page_sizes.phys = PAGE_SIZE;
	obj->mm.page_sizes.sg = PAGE_SIZE;
	return 0;
}

unsigned long i915_ttm_vram_obj_gtt_offset(struct drm_i915_gem_object *obj,
					   struct ttm_resource *mem)
{
	struct drm_i915_private *i915 = to_i915_ttm_dev(obj->base.bdev);
	struct i915_ttm_vram_node *node = mem->mm_node;
	struct ttm_resource_manager *gtt_mgr = ttm_manager_type(obj->base.bdev, TTM_PL_TT);
	int r;
	if (!mem || !node) {
		WARN_ON(1);
		return 0;
	}
	if (node->gtt_res.start == I915_TTM_BO_INVALID_OFFSET) {
		struct ttm_place gtt_place = obj->ttm.placements[0];
		gtt_place.fpfn = 0;
		/* force gtt mgr to give us a node */
		gtt_place.lpfn = i915->ggtt.vm.total >> PAGE_SHIFT;
		node->gtt_res = *mem;
		node->gtt_res.mm_node = NULL;
		/* allocate a gtt node as well */
		r = gtt_mgr->func->alloc(gtt_mgr, &obj->base, &gtt_place, &node->gtt_res);
		if (unlikely(r)) {
			node->gtt_res.start = I915_TTM_BO_INVALID_OFFSET;
		}
	}
	return node->gtt_res.start;
}

bool i915_ttm_vram_obj_premap_allowed(struct ttm_resource *mem)
{
	struct i915_ttm_vram_node *node = mem->mm_node;

	return node->nodes[0].size == mem->num_pages;
}

struct drm_mm_node *i915_ttm_vram_find_node(struct ttm_buffer_object *bo, u64 *offset)
{
	struct i915_ttm_vram_node *node = bo->mem.mm_node;
	struct drm_mm_node *mm_node = node->nodes;

	while (*offset >= (mm_node->size << PAGE_SHIFT)) {
		*offset -= (mm_node->size << PAGE_SHIFT);
		++mm_node;
	}
	return mm_node;
}

static int i915_ttm_copy_io_page(void *dst, void *src, unsigned long page)
{
	uint32_t *dstP =
	    (uint32_t *) ((unsigned long)dst + (page << PAGE_SHIFT));
	uint32_t *srcP =
	    (uint32_t *) ((unsigned long)src + (page << PAGE_SHIFT));

	int i;
	for (i = 0; i < PAGE_SIZE / sizeof(uint32_t); ++i)
		iowrite32(ioread32(srcP++), dstP++);
	return 0;
}

int i915_ttm_vram_vram_copy(struct ttm_buffer_object *tbo,
			    struct ttm_resource *new_mem)
{
	struct ttm_resource *old_mem = &tbo->mem;
	struct drm_i915_private *i915 = to_i915_ttm_dev(tbo->bdev);
	struct i915_ttm_vram_node *old_node = old_mem->mm_node;
	struct ttm_resource old_copy = *old_mem;
	struct drm_mm_node *old_nodes = old_node->nodes;
	unsigned pages = new_mem->num_pages;
	void *dst_addr;
	void *src_addr;
	void *vram_base = i915->ttm_mman.aper_base_kaddr;

	dst_addr = vram_base + (new_mem->start << PAGE_SHIFT);

	while (pages) {
		unsigned x;

		src_addr = vram_base + (old_nodes->start << PAGE_SHIFT);
		for (x = 0; x < old_nodes->size; x++)
			i915_ttm_copy_io_page(dst_addr, src_addr, x);

		dst_addr += old_nodes->size << PAGE_SHIFT;
		pages -= old_nodes->size;
		++old_nodes;
	}

	old_copy = *old_mem;
	*old_mem = *new_mem;
	new_mem->mm_node = NULL;

	ttm_resource_free(tbo, &old_copy);

	return 0;
}
