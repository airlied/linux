// SPDX-License-Identifier: MIT
/*
 * Copyright © 2024 Intel Corporation
 */

#include <linux/shrinker.h>

#include <drm/ttm/ttm_backup.h>
#include <drm/ttm/ttm_bo.h>
#include <drm/ttm/ttm_tt.h>
#include <drm/ttm/ttm_shrinker.h>
#include "xe_bo.h"
#include "xe_pm.h"
#include "xe_shrinker.h"

static struct xe_shrinker *to_xe_shrinker(struct shrinker *shrink)
{
	return shrink->private_data;
}

static unsigned long
xe_shrinker_count(struct shrinker *shrink, struct shrink_control *sc)
{
	struct xe_shrinker *shrinker = to_xe_shrinker(shrink);

	return ttm_shrinker_count(&shrinker->base, sc);
}

/*
 * Check if we need runtime pm, and if so try to grab a reference if
 * already active. If grabbing a reference fails, queue a worker that
 * does it for us outside of reclaim, but don't wait for it to complete.
 * If bo shrinking needs an rpm reference and we don't have it (yet),
 * that bo will be skipped anyway.
 */
static bool xe_shrinker_runtime_pm_get(struct xe_shrinker *shrinker, bool force,
				       unsigned long nr_to_scan, bool can_backup)
{
	struct xe_device *xe = shrinker->xe;

	if (IS_DGFX(xe) || !xe_device_has_flat_ccs(xe) ||
	    !ttm_backup_bytes_avail())
		return false;

	if (!force) {
		long purgeable_pages = ttm_shrinker_nr_purgeable(&shrinker->base);
		force = (nr_to_scan > purgeable_pages && can_backup);
		if (!force)
			return false;
	}

	if (!xe_pm_runtime_get_if_active(xe)) {
		if (xe_rpm_reclaim_safe(xe) && !ttm_bo_shrink_avoid_wait()) {
			xe_pm_runtime_get(xe);
			return true;
		}
		queue_work(xe->unordered_wq, &shrinker->pm_worker);
		return false;
	}

	return true;
}

static void xe_shrinker_runtime_pm_put(struct xe_shrinker *shrinker, bool runtime_pm)
{
	if (runtime_pm)
		xe_pm_runtime_put(shrinker->xe);
}

static unsigned long xe_shrinker_scan(struct shrinker *shrink, struct shrink_control *sc)
{
	struct xe_shrinker *shrinker = to_xe_shrinker(shrink);
	struct ttm_operation_ctx ctx = {
		.interruptible = false,
		.no_wait_gpu = ttm_bo_shrink_avoid_wait(),
	};
	unsigned long nr_to_scan, nr_scanned = 0, freed = 0;
	struct ttm_bo_shrink_flags shrink_flags = {
		.purge = true,
		/* Don't request writeback without __GFP_IO. */
		.writeback = !ctx.no_wait_gpu && (sc->gfp_mask & __GFP_IO),
	};
	bool runtime_pm;
	bool purgeable;
	bool can_backup = !!(sc->gfp_mask & __GFP_FS);
	s64 lret;

	nr_to_scan = sc->nr_to_scan;

	purgeable = ttm_shrinker_purgeable(&shrinker->base);

	/* Might need runtime PM. Try to wake early if it looks like it. */
	runtime_pm = xe_shrinker_runtime_pm_get(shrinker, false, nr_to_scan, can_backup);

	if (purgeable && nr_scanned < nr_to_scan) {
		lret = ttm_shrinker_walk(&shrinker->xe->ttm, &ctx, shrink_flags,
					 xe_bo_shrink,
					 nr_to_scan, &nr_scanned);
		if (lret >= 0)
			freed += lret;
	}

	sc->nr_scanned = nr_scanned;
	if (nr_scanned >= nr_to_scan || !can_backup)
		goto out;

	/* If we didn't wake before, try to do it now if needed. */
	if (!runtime_pm)
		runtime_pm = xe_shrinker_runtime_pm_get(shrinker, true, 0, can_backup);

	shrink_flags.purge = false;
	lret = ttm_shrinker_walk(&shrinker->xe->ttm, &ctx, shrink_flags,
				 xe_bo_shrink,
				 nr_to_scan, &nr_scanned);
	if (lret >= 0)
		freed += lret;

	sc->nr_scanned = nr_scanned;
out:
	xe_shrinker_runtime_pm_put(shrinker, runtime_pm);
	return nr_scanned ? freed : SHRINK_STOP;
}

/* Wake up the device for shrinking. */
static void xe_shrinker_pm(struct work_struct *work)
{
	struct xe_shrinker *shrinker =
		container_of(work, typeof(*shrinker), pm_worker);

	xe_pm_runtime_get(shrinker->xe);
	xe_pm_runtime_put(shrinker->xe);
}

/**
 * xe_shrinker_create() - Create an xe per-device shrinker
 * @xe: Pointer to the xe device.
 *
 * Returns: A pointer to the created shrinker on success,
 * Negative error code on failure.
 */
struct xe_shrinker *xe_shrinker_create(struct xe_device *xe)
{
	struct xe_shrinker *shrinker = kzalloc(sizeof(*shrinker), GFP_KERNEL);

	if (!shrinker)
		return ERR_PTR(-ENOMEM);

	shrinker->base.shrink = shrinker_alloc(0, "xe system shrinker");
	if (!shrinker->base.shrink) {
		kfree(shrinker);
		return ERR_PTR(-ENOMEM);
	}

	INIT_WORK(&shrinker->pm_worker, xe_shrinker_pm);
	shrinker->xe = xe;
	rwlock_init(&shrinker->base.lock);
	shrinker->base.shrink->count_objects = xe_shrinker_count;
	shrinker->base.shrink->scan_objects = xe_shrinker_scan;
	shrinker->base.shrink->private_data = shrinker;
	shrinker_register(shrinker->base.shrink);

	return shrinker;
}

/**
 * xe_shrinker_destroy() - Destroy an xe per-device shrinker
 * @shrinker: Pointer to the shrinker to destroy.
 */
void xe_shrinker_destroy(struct xe_shrinker *shrinker)
{
	xe_assert(shrinker->xe, !shrinker->base.shrinkable_pages);
	xe_assert(shrinker->xe, !shrinker->base.purgeable_pages);
	shrinker_free(shrinker->base.shrink);
	flush_work(&shrinker->pm_worker);
	kfree(shrinker);
}
