
#ifndef _TTM_SHRINKER_H_
#define _TTM_SHRINKER_H_

#include <drm/ttm/ttm_placement.h>

struct ttm_buffer_object;
struct ttm_operation_ctx;

struct ttm_shrinker {
	rwlock_t lock;
	long shrinkable_pages;
	long purgeable_pages;
	struct shrinker *shrink;
};

void ttm_shrinker_mod_pages(struct ttm_shrinker *shrinker, struct ttm_tt *tt, bool subtract);

unsigned long ttm_shrinker_count(struct ttm_shrinker *shrinker, struct shrink_control *sc);

static inline bool ttm_shrinker_purgeable(struct ttm_shrinker *shrinker) {
	bool purgeable;
	read_lock(&shrinker->lock);
	purgeable = !!shrinker->purgeable_pages;
	read_unlock(&shrinker->lock);
	return purgeable;
}

static inline long ttm_shrinker_nr_purgeable(struct ttm_shrinker *shrinker) {
	long purgeable_pages;
	read_lock(&shrinker->lock);
	purgeable_pages = shrinker->purgeable_pages;
	read_unlock(&shrinker->lock);
	return purgeable_pages;
}

void ttm_shrinker_init(struct ttm_shrinker *shrinker);
void ttm_shrinker_fini(struct ttm_shrinker *shrinker);

typedef long (*ttm_bo_shrink_func)(struct ttm_operation_ctx *ctx,
				   struct ttm_buffer_object *bo,
				   const struct ttm_bo_shrink_flags flags,
				   unsigned long *scanned);

s64 ttm_shrinker_walk(struct ttm_device *bdev,
		      struct ttm_operation_ctx *ctx,
		      const struct ttm_bo_shrink_flags flags,
		      ttm_bo_shrink_func shrink_bo,
		      unsigned long to_scan, unsigned long *scanned);

static inline bool ttm_shrinker_bo_busy(struct ttm_buffer_object *bo,
					const struct ttm_bo_shrink_flags flags)
{
	struct ttm_tt *tt = bo->ttm;
	struct ttm_place place = {.mem_type = bo->resource->mem_type};
	if (!(tt->page_flags & TTM_TT_FLAG_EXTERNAL_MAPPABLE) ||
	    (flags.purge && !ttm_tt_is_purgeable(tt)))
		return true;

	if (!ttm_bo_eviction_valuable(bo, &place))
		return true;
	return false;
}

#endif
