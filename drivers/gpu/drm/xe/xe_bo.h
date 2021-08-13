/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2021 Intel Corporation
 */

#ifndef _XE_BO_H_
#define _XE_BO_H_

#include <drm/ttm/ttm_bo_api.h>
#include <drm/ttm/ttm_device.h>

#include "xe_vm.h"

struct xe_bo {
	struct ttm_buffer_object ttm;

	struct xe_vm *vm;
};

static inline struct xe_bo *ttm_to_xe_bo(const struct ttm_buffer_object *bo)
{
	return container_of(bo, struct xe_bo, ttm);
}

static inline struct xe_bo *gem_to_xe_bo(const struct drm_gem_object *obj)
{
	return container_of(obj, struct xe_bo, ttm.base);
}

static inline struct xe_bo *xe_bo_get(struct xe_bo *bo)
{
	ttm_bo_get(&bo->ttm);
	return bo;
}

static inline void xe_bo_put(struct xe_bo *bo)
{
	ttm_bo_put(&bo->ttm);
}

#define xe_bo_assert_held(bo) dma_resv_assert_held((bo)->ttm.base.resv)

static inline void xe_bo_lock_vm_held(struct xe_bo *bo, struct ww_acquire_ctx *ctx)
{
	XE_BUG_ON(bo->vm && bo->ttm.base.resv != &bo->vm->resv);
	if (bo->vm)
		xe_vm_assert_held(bo->vm);
	else
		dma_resv_lock(bo->ttm.base.resv, ctx);
}

static inline void xe_bo_unlock_vm_held(struct xe_bo *bo)
{
	XE_BUG_ON(bo->vm && bo->ttm.base.resv != &bo->vm->resv);
	if (bo->vm)
		xe_vm_assert_held(bo->vm);
	else
		dma_resv_unlock(bo->ttm.base.resv);
}

static inline void xe_bo_or_vm_lock(struct xe_bo *bo, struct ww_acquire_ctx *ctx)
{
	XE_BUG_ON(bo->vm && bo->ttm.base.resv != &bo->vm->resv);
	dma_resv_lock(bo->ttm.base.resv, ctx);
}

static inline void xe_bo_or_vm_unlock(struct xe_bo *bo)
{
	XE_BUG_ON(bo->vm && bo->ttm.base.resv != &bo->vm->resv);
	dma_resv_unlock(bo->ttm.base.resv);
}

extern struct ttm_device_funcs xe_ttm_funcs;

int xe_gem_create_ioctl(struct drm_device *dev, void *data,
			struct drm_file *file);
int xe_gem_mmap_offset_ioctl(struct drm_device *dev, void *data,
			     struct drm_file *file);

#endif /* _XE_BO_H_ */
