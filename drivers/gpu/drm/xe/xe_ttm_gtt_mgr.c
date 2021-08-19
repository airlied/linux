#include <drm/ttm/ttm_range_manager.h>
#include <drm/ttm/ttm_placement.h>
#include <drm/ttm/ttm_bo_driver.h>
#include "xe_device.h"

struct xe_ttm_gtt_node {
	struct ttm_buffer_object *tbo;
	struct ttm_range_mgr_node base;
};

static inline struct xe_ttm_gtt_mgr *
to_gtt_mgr(struct ttm_resource_manager *man)
{
	return container_of(man, struct xe_ttm_gtt_mgr, manager);
}

static inline struct xe_ttm_gtt_node *
to_xe_ttm_gtt_node(struct ttm_resource *res)
{
	return container_of(res, struct xe_ttm_gtt_node, base.base);
}

static int xe_ttm_gtt_mgr_new(struct ttm_resource_manager *man,
			      struct ttm_buffer_object *tbo,
			      const struct ttm_place *place,
			      struct ttm_resource **res)
{
	struct xe_ttm_gtt_mgr *mgr = to_gtt_mgr(man);
	uint32_t num_pages = PFN_UP(tbo->base.size);
	struct xe_ttm_gtt_node *node;
	int r;

	if (!(place->flags & TTM_PL_FLAG_TEMPORARY) &&
	    atomic64_add_return(num_pages, &mgr->used) > man->size) {
		atomic64_sub(num_pages, &mgr->used);
		return -ENOSPC;
	}

	node = kzalloc(struct_size(node, base.mm_nodes, 1), GFP_KERNEL);
	if (!node) {
		r = -ENOMEM;
		goto err_out;
	}

	node->tbo = tbo;
	ttm_resource_init(tbo, place, &node->base.base);

	node->base.mm_nodes[0].start = 0;
	node->base.mm_nodes[0].size = node->base.base.num_pages;
	node->base.base.start = XE_BO_INVALID_OFFSET;

	*res = &node->base.base;

	return 0;
err_out:
	if (!(place->flags & TTM_PL_FLAG_TEMPORARY))
		atomic64_sub(num_pages, &mgr->used);
	return r;
}

static void xe_ttm_gtt_mgr_del(struct ttm_resource_manager *man,
			       struct ttm_resource *res)
{
	struct xe_ttm_gtt_node *node = to_xe_ttm_gtt_node(res);
	struct xe_ttm_gtt_mgr *mgr = to_gtt_mgr(man);
	
	if (!(res->placement & TTM_PL_FLAG_TEMPORARY))
		atomic64_sub(res->num_pages, &mgr->used);

	kfree(node);
}

static void xe_ttm_gtt_mgr_debug(struct ttm_resource_manager *man,
				 struct drm_printer *printer)
{

}

static const struct ttm_resource_manager_func xe_ttm_gtt_mgr_func = {
	.alloc = xe_ttm_gtt_mgr_new,
	.free = xe_ttm_gtt_mgr_del,
	.debug = xe_ttm_gtt_mgr_debug
};

int xe_ttm_gtt_mgr_init(struct xe_device *xe, uint64_t gtt_size)
{
	struct xe_ttm_gtt_mgr *mgr = &xe->gtt_mgr;
	struct ttm_resource_manager *man = &mgr->manager;

	man->use_tt = true;
	man->func = &xe_ttm_gtt_mgr_func;

	ttm_resource_manager_init(man, gtt_size >> PAGE_SHIFT);

	atomic64_set(&mgr->used, 0);
	ttm_set_driver_manager(&xe->ttm, TTM_PL_TT, &mgr->manager);
	ttm_resource_manager_set_used(man, true);
	return 0;
}

void xe_ttm_gtt_mgr_fini(struct xe_device *xe)
{
	struct xe_ttm_gtt_mgr *mgr = &xe->gtt_mgr;
	struct ttm_resource_manager *man = &mgr->manager;
	int err;

	ttm_resource_manager_set_used(man, false);

	err = ttm_resource_manager_evict_all(&xe->ttm, man);
	if (err)
		return;

	ttm_resource_manager_cleanup(man);
	ttm_set_driver_manager(&xe->ttm, TTM_PL_TT, NULL);
}
	
