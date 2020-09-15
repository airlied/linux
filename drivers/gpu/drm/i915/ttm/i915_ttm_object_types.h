// SPDX-License-Identifier: MIT
/*
 * Copyright @ 2020 Red Hat.
 */
#ifndef I915_TTM_OBJECT_TYPES_H
#define I915_TTM_OBJECT_TYPES_H

#include <drm/ttm/ttm_bo_api.h>
#include <drm/ttm/ttm_bo_driver.h>
#include <drm/ttm/ttm_placement.h>

#include "i915_vma_types.h"

#define I915_TTM_PL_STOLEN (TTM_PL_PRIV + 0)

#define I915_TTM_PL_FLAG_STOLEN (TTM_PL_FLAG_PRIV << 0)

#define I915_TTM_CREATE_VRAM_CONTIGUOUS (1 << 0)
#define I915_TTM_CREATE_CPU_GTT_USWC (1 << 1)

struct i915_ttm_vram_mgr {
	struct ttm_resource_manager manager;
	struct drm_mm mm;
	spinlock_t lock;
	atomic64_t usage;
};

struct i915_ttm_gtt_mgr {
	struct ttm_resource_manager manager;
	struct drm_mm mm;
	spinlock_t lock;
	atomic64_t available;
};

struct i915_ttm_stolen_mgr {
	struct ttm_resource_manager manager;
	struct drm_mm mm;
	spinlock_t lock;
	atomic64_t available;
};

struct i915_ttm_mman {
	struct ttm_bo_device bdev;
	bool initialized;
	struct i915_ttm_vram_mgr vram_mgr;
	struct i915_ttm_gtt_mgr gtt_mgr;
	struct i915_ttm_stolen_mgr stolen_mgr;
	void __iomem *aper_base_kaddr;
	struct mutex notifier_lock;
};

#endif

