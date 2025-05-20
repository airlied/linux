/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2024 Intel Corporation
 */

#ifndef _XE_SHRINKER_H_
#define _XE_SHRINKER_H_

#include <drm/ttm/ttm_shrinker.h>
struct xe_shrinker;
struct xe_device;

/**
 * struct xe_shrinker - per-device shrinker
 * @xe: Back pointer to the device.
 * @lock: Lock protecting accounting.
 * @shrinkable_pages: Number of pages that are currently shrinkable.
 * @purgeable_pages: Number of pages that are currently purgeable.
 * @shrink: Pointer to the mm shrinker.
 * @pm_worker: Worker to wake up the device if required.
 */
struct xe_shrinker {
	struct ttm_shrinker base;
	struct xe_device *xe;
	struct work_struct pm_worker;
};

void xe_shrinker_mod_pages(struct xe_shrinker *shrinker, long shrinkable, long purgeable);

struct xe_shrinker *xe_shrinker_create(struct xe_device *xe);

void xe_shrinker_destroy(struct xe_shrinker *shrinker);

#endif
