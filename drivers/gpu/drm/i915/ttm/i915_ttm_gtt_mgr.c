
#include "i915_drv.h"
#include "i915_ttm.h"

static inline struct i915_ttm_gtt_mgr *to_gtt_mgr(struct ttm_resource_manager *man)
{
	return container_of(man, struct i915_ttm_gtt_mgr, manager);
}

struct i915_ttm_gtt_node {
	struct drm_mm_node node;
};
static const struct ttm_resource_manager_func i915_ttm_gtt_mgr_func;

int i915_ttm_gtt_mgr_init(struct drm_i915_private *i915)
{
	struct ttm_resource_manager *man;
	struct i915_ttm_gtt_mgr *mgr;
	unsigned long gtt_pages = i915->ggtt.vm.total >> PAGE_SHIFT;

	mgr = &i915->ttm_mman.gtt_mgr;
	man = &mgr->manager;

	printk("creating GTT size %llu\n", i915->ggtt.vm.total);
	man->use_tt = true;
	man->func = &i915_ttm_gtt_mgr_func;
	man->available_caching = TTM_PL_MASK_CACHING;
	man->default_caching = TTM_PL_FLAG_CACHED;

	ttm_resource_manager_init(man, gtt_pages);
	drm_mm_init(&mgr->mm, 0, gtt_pages);
	spin_lock_init(&mgr->lock);
	atomic64_set(&mgr->available, gtt_pages);

	ttm_set_driver_manager(&i915->ttm_mman.bdev, TTM_PL_TT, &mgr->manager);
	ttm_resource_manager_set_used(man, true);
	return 0;
}

void i915_ttm_gtt_mgr_fini(struct drm_i915_private *i915)
{
	struct i915_ttm_gtt_mgr *mgr = &i915->ttm_mman.gtt_mgr;
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
 * i915_ttm_gtt_mgr_has_gart_addr - Check if mem has address space
 *
 * @mem: the mem object to check
 *
 * Check if a mem object has already address space allocated.
 */
bool i915_ttm_gtt_mgr_has_gart_addr(struct ttm_resource *mem)
{
	return (mem->mm_node != NULL);
}

static int i915_ttm_gtt_mgr_new(struct ttm_resource_manager *man,
				 struct ttm_buffer_object *tbo,
				 const struct ttm_place *place,
				 struct ttm_resource *mem)
{
	struct i915_ttm_gtt_mgr *mgr = to_gtt_mgr(man);
	struct i915_ttm_gtt_node *node;
	int r;

	spin_lock(&mgr->lock);
	if ((&tbo->mem == mem || (tbo->mem.mem_type != TTM_PL_TT && tbo->mem.mem_type != I915_TTM_PL_STOLEN)) &&
	    atomic64_read(&mgr->available) < mem->num_pages) {
		spin_unlock(&mgr->lock);
		return 0;
	}
	atomic64_sub(mem->num_pages, &mgr->available);
	spin_unlock(&mgr->lock);

	if (!place->lpfn) {
		mem->mm_node = NULL;
		mem->start = I915_TTM_BO_INVALID_OFFSET;
		return 0;
	}


	node = kzalloc(sizeof(*node), GFP_KERNEL);
	if (!node) {
		r = -ENOMEM;
		goto err_out;
	}

	spin_lock(&mgr->lock);
	r = drm_mm_insert_node_in_range(&mgr->mm, &node->node, mem->num_pages,
					mem->page_alignment, 0, place->fpfn,
					place->lpfn, DRM_MM_INSERT_BEST);
	spin_unlock(&mgr->lock);

	if (unlikely(r))
		goto err_free;
	mem->mm_node = node;
	mem->start = node->node.start;
	printk(KERN_ERR "gtt mgr node start %lx\n", mem->start);
	return 0;
err_free:
	kfree(node);
err_out:
	atomic64_add(mem->num_pages, &mgr->available);
	return r;
}

static void i915_ttm_gtt_mgr_del(struct ttm_resource_manager *man,
				  struct ttm_resource *mem)
{
	struct i915_ttm_gtt_mgr *mgr = to_gtt_mgr(man);
	struct i915_ttm_gtt_node *node = mem->mm_node;

	if (node) {
		spin_lock(&mgr->lock);
		drm_mm_remove_node(&node->node);
		spin_unlock(&mgr->lock);
		kfree(node);
	}
	atomic64_add(mem->num_pages, &mgr->available);
}

static const struct ttm_resource_manager_func i915_ttm_gtt_mgr_func = {
	.get_node = i915_ttm_gtt_mgr_new,
	.put_node = i915_ttm_gtt_mgr_del,
};
