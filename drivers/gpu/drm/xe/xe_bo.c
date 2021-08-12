/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2021 Intel Corporation
 */

#include <drm/xe_drm.h>
#include <drm/ttm/ttm_placement.h>

#include "xe_bo.h"
#include "xe_device.h"

static const struct ttm_place sys_placement_flags = {
	.fpfn = 0,
	.lpfn = 0,
	.mem_type = TTM_PL_SYSTEM,
	.flags = 0,
};

static struct ttm_placement sys_placement = {
	.num_placement = 1,
	.placement = &sys_placement_flags,
	.num_busy_placement = 1,
	.busy_placement = &sys_placement_flags,
};

struct ttm_device_funcs xe_ttm_funcs = {
};

static void xe_ttm_bo_destroy(struct ttm_buffer_object *ttm_bo)
{
	struct xe_bo *bo = ttm_to_xe_bo(ttm_bo);

	if (bo->vm)
		xe_vm_put(bo->vm);
}

struct xe_bo *xe_bo_create(struct xe_device *xe, size_t size,
			   struct xe_vm *vm, u32 flags)
{
	struct xe_bo *bo;
	struct ttm_operation_ctx ctx = {
		.interruptible = true,
		.no_wait_gpu = false,
	};
	int err;

	bo = kzalloc(sizeof(*bo), GFP_KERNEL);
	if (!bo)
		return ERR_PTR(-ENOMEM);

	err = ttm_bo_init_reserved(&xe->ttm, &bo->ttm, size, ttm_bo_type_device,
				   &sys_placement, SZ_64K >> PAGE_SHIFT,
				   &ctx, NULL, NULL, xe_ttm_bo_destroy);
	if (err) {
		kfree(bo);
		return ERR_PTR(err);
	}

	if (vm)
		bo->vm = vm;

	return bo;
}

#define ALL_DRM_XE_GEM_CREATE_FLAGS (\
	DRM_XE_GEM_CREATE_SYSTEM)

int xe_gem_create_ioctl(struct drm_device *dev, void *data,
			struct drm_file *file)
{
	struct xe_device *xe = to_xe_device(dev);
	struct xe_file *xef = to_xe_file(file);
	struct drm_xe_gem_create *args = data;
	struct xe_vm *vm = NULL;
	struct xe_bo *bo;
	u32 handle;
	int err;

	if (args->extensions)
		return -EINVAL;

	if (args->flags & ~ALL_DRM_XE_GEM_CREATE_FLAGS)
		return -EINVAL;

	if (args->handle)
		return -EINVAL;

	if (sizeof(size_t) < 8 && args->size >= (256 << sizeof(size_t)))
		return -EINVAL;

	if (args->vm_id) {
		vm = xe_vm_lookup(xef, args->vm_id);
		if (!vm)
			return -ENOENT;
	}

	bo = xe_bo_create(xe, args->size, vm, args->flags);
	if (IS_ERR(bo)) {
		if (vm)
			xe_vm_put(vm);
		return PTR_ERR(bo);
	}

	err = drm_gem_handle_create(file, &bo->ttm.base, &handle);
	/* drop reference from allocate - handle holds it now */
	xe_bo_put(bo);
	if (err)
		return err;

	args->handle = handle;
	return 0;
}
