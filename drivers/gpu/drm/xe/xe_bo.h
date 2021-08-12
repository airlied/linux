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

static inline struct xe_bo *xe_bo_get(struct xe_bo *bo)
{
	drm_gem_object_get(&bo->ttm.base);
	return bo;
}

static inline void xe_bo_put(struct xe_bo *bo)
{
	drm_gem_object_put(&bo->ttm.base);
}

extern struct ttm_device_funcs xe_ttm_funcs;

int xe_gem_create_ioctl(struct drm_device *dev, void *data,
			struct drm_file *file);

#endif /* _XE_BO_H_ */
