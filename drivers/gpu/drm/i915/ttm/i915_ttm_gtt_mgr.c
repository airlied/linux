
#include "i915_drv.h"
#include "i915_ttm.h"
struct i915_ttm_gtt_mgr {
	struct drm_mm mm;
	spinlock_t lock;
	atomic64_t available;
};

struct i915_ttm_gtt_node {
	struct drm_mm_node node;
	struct ttm_buffer_object *tbo;
};
static int i915_ttm_gtt_mgr_init(struct ttm_mem_type_manager *man,
				  unsigned long p_size)
{
	struct drm_i915_private *i915 = to_i915_ttm_dev(man->bdev);
	struct i915_ttm_gtt_mgr *mgr;

	int ret;

	mgr = kzalloc(sizeof(*mgr), GFP_KERNEL);
	if (!mgr)
		return -ENOMEM;

	drm_mm_init(&mgr->mm, 0, p_size);
	spin_lock_init(&mgr->lock);
	man->priv = mgr;

	return 0;
}

static int i915_ttm_gtt_mgr_fini(struct ttm_mem_type_manager *man)
{
	struct drm_i915_private *i915 = to_i915_ttm_dev(man->bdev);
	struct i915_ttm_gtt_mgr *mgr = man->priv;
	spin_lock(&mgr->lock);
	drm_mm_takedown(&mgr->mm);
	spin_unlock(&mgr->lock);
	kfree(mgr);
	man->priv = NULL;
	return 0;
}

/**
 * i915_ttm_gtt_mgr_has_gart_addr - Check if mem has address space
 *
 * @mem: the mem object to check
 *
 * Check if a mem object has already address space allocated.
 */
bool i915_ttm_gtt_mgr_has_gart_addr(struct ttm_mem_reg *mem)
{
	struct i915_ttm_gtt_node *node = mem->mm_node;

	return (node->node.start != I915_TTM_BO_INVALID_OFFSET);
}

static int i915_ttm_gtt_mgr_alloc(struct ttm_mem_type_manager *man,
				  struct ttm_buffer_object *tbo,
				  const struct ttm_place *place,
				  struct ttm_mem_reg *mem)
{
	struct drm_i915_private *i915 = to_i915_ttm_dev(man->bdev);
	struct i915_ttm_gtt_mgr *mgr = man->priv;
	struct i915_ttm_gtt_node *node = mem->mm_node;
	enum drm_mm_insert_mode mode;
	unsigned long fpfn, lpfn;
	int r;

	if (i915_ttm_gtt_mgr_has_gart_addr(mem))
		return 0;

	if (place)
		fpfn = place->fpfn;
	else
		fpfn = 0;

	if (place && place->lpfn)
		lpfn = place->lpfn;
	else
		lpfn = 0; //TODO

	mode = DRM_MM_INSERT_BEST;
	if (place && place->flags & TTM_PL_FLAG_TOPDOWN)
		mode = DRM_MM_INSERT_HIGH;

	spin_lock(&mgr->lock);
	r = drm_mm_insert_node_in_range(&mgr->mm, &node->node, mem->num_pages,
					mem->page_alignment, 0, fpfn, lpfn,
					mode);
	spin_unlock(&mgr->lock);
	if (r)
		mem->start = node->node.start;
	return r;
}
				  
static int i915_ttm_gtt_mgr_new(struct ttm_mem_type_manager *man,
				 struct ttm_buffer_object *tbo,
				 const struct ttm_place *place,
				 struct ttm_mem_reg *mem)
{
	struct drm_i915_private *i915 = to_i915_ttm_dev(man->bdev);
	struct i915_ttm_gtt_mgr *mgr = man->priv;
	struct drm_mm *mm = &mgr->mm;
	struct i915_ttm_gtt_node *node;
	int r;

	spin_lock(&mgr->lock);
	if ((&tbo->mem == mem || tbo->mem.mem_type != TTM_PL_TT) &&
	    atomic64_read(&mgr->available) < mem->num_pages) {
		spin_unlock(&mgr->lock);
		return 0;
	}
	atomic64_sub(mem->num_pages, &mgr->available);
	spin_unlock(&mgr->lock);

	node = kzalloc(sizeof(*node), GFP_KERNEL);
	if (!node) {
		r = -ENOMEM;
		goto err_out;
	}
	node->node.start = I915_TTM_BO_INVALID_OFFSET;
	node->node.size = mem->num_pages;
	node->tbo = tbo;
	mem->mm_node = node;

	if (place->fpfn || place->lpfn || place->flags & TTM_PL_FLAG_TOPDOWN) {
		r = i915_ttm_gtt_mgr_alloc(man, tbo, place, mem);
		if (unlikely(r)) {
			kfree(node);
			mem->mm_node = NULL;
			r = 0;
			goto err_out;
		}
	} else {
		mem->start = node->node.start;
	}
	return 0;
err_out:
	atomic64_add(mem->num_pages, &mgr->available);
	return r;
}

static void i915_ttm_gtt_mgr_del(struct ttm_mem_type_manager *man,
				  struct ttm_mem_reg *mem)
{
	struct drm_i915_private *i915 = to_i915_ttm_dev(man->bdev);
	struct i915_ttm_gtt_mgr *mgr = man->priv;
	struct i915_ttm_gtt_node *node = mem->mm_node;

	if (!node)
		return;

	spin_lock(&mgr->lock);
	if (node->node.start != I915_TTM_BO_INVALID_OFFSET)
		drm_mm_remove_node(&node->node);
	spin_unlock(&mgr->lock);
	atomic64_add(mem->num_pages, &mgr->available);

	kfree(node);
	mem->mm_node = NULL;
}
				  
const struct ttm_mem_type_manager_func i915_ttm_gtt_mgr_func = {
	.init = i915_ttm_gtt_mgr_init,
	.takedown = i915_ttm_gtt_mgr_fini,
	.get_node = i915_ttm_gtt_mgr_new,
	.put_node = i915_ttm_gtt_mgr_del,
	.debug = NULL,
};
