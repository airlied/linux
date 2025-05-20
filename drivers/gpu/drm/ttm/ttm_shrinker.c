#include <linux/shrinker.h>

#include <drm/ttm/ttm_backup.h>
#include <drm/ttm/ttm_device.h>
#include <drm/ttm/ttm_placement.h>
#include <drm/ttm/ttm_bo.h>
#include <drm/ttm/ttm_tt.h>
#include <drm/ttm/ttm_shrinker.h>

/**
 * ttm_shrinker_mod_pages() - Modify shrinker page accounting
 * @shrinker: Pointer to the struct xe_shrinker.
 * @tt: tt to use as page counter
 *
 * Modifies the shrinkable and purgeable pages accounting.
 */
void ttm_shrinker_mod_pages(struct ttm_shrinker *shrinker, struct ttm_tt *tt, bool subtract)
{
	long value = tt->num_pages * (subtract ? -1 : 1);
	write_lock(&shrinker->lock);
	if (tt->page_flags & TTM_TT_FLAG_PURGEABLE)
		shrinker->purgeable_pages += value;
	else
		shrinker->shrinkable_pages += value;
	write_unlock(&shrinker->lock);
}
EXPORT_SYMBOL_GPL(ttm_shrinker_mod_pages);

s64 ttm_shrinker_walk(struct ttm_device *bdev,
		      struct ttm_operation_ctx *ctx,
		      const struct ttm_bo_shrink_flags flags,
		      ttm_bo_shrink_func shrink_bo,
		      unsigned long to_scan, unsigned long *scanned)
{
	unsigned int mem_type;
	s64 freed = 0, lret;

	for (mem_type = TTM_PL_SYSTEM; mem_type <= TTM_PL_TT; ++mem_type) {
		struct ttm_resource_manager *man = ttm_manager_type(bdev, mem_type);
		struct ttm_bo_lru_cursor curs;
		struct ttm_buffer_object *ttm_bo;

		if (!man || !man->use_tt)
			continue;

		ttm_bo_lru_for_each_reserved_guarded(&curs, man, ctx, ttm_bo) {
			if (!ttm_bo_shrink_suitable(ttm_bo, ctx))
				continue;

			lret = (*shrink_bo)(ctx, ttm_bo, flags, scanned);
			if (lret < 0)
				return lret;

			freed += lret;
			if (*scanned >= to_scan)
				break;
		}
	}

	return freed;
}
EXPORT_SYMBOL_GPL(ttm_shrinker_walk);

unsigned long
ttm_shrinker_count(struct ttm_shrinker *shrinker, struct shrink_control *sc)
{
	unsigned long num_pages;
	bool can_backup = !!(sc->gfp_mask & __GFP_FS);

	num_pages = ttm_backup_bytes_avail() >> PAGE_SHIFT;
	read_lock(&shrinker->lock);

	if (can_backup)
		num_pages = min_t(unsigned long, num_pages, shrinker->shrinkable_pages);
	else
		num_pages = 0;

	num_pages += shrinker->purgeable_pages;
	read_unlock(&shrinker->lock);

	return num_pages ? num_pages : SHRINK_EMPTY;
}
EXPORT_SYMBOL_GPL(ttm_shrinker_count);
void ttm_shrinker_fini(struct ttm_shrinker *shrinker)
{

}
