/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2021 Intel Corporation
 */

#include <drm/drm_gem_ttm_helper.h>
#include <drm/ttm/ttm_device.h>
#include <drm/ttm/ttm_placement.h>
#include <drm/ttm/ttm_tt.h>
#include <drm/xe_drm.h>

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

struct xe_ttm_tt {
	struct ttm_tt ttm;
};

static struct ttm_tt *xe_ttm_tt_create(struct ttm_buffer_object *ttm_bo,
				       uint32_t page_flags)
{
	struct xe_bo *bo = ttm_to_xe_bo(ttm_bo);
	struct xe_ttm_tt *tt;
	int err;

	tt = kzalloc(sizeof(*tt), GFP_KERNEL);
	if (!tt)
		return NULL;

	/* TODO: We only need to do this for user allocated BOs */
	page_flags |= TTM_PAGE_FLAG_ZERO_ALLOC;

	/* TODO: Select caching mode */
	err = ttm_sg_tt_init(&tt->ttm, &bo->ttm, page_flags, ttm_cached);
	if (err) {
		kfree(tt);
		return NULL;
	}

	return &tt->ttm;
}

static void xe_ttm_tt_destroy(struct ttm_device *ttm_dev, struct ttm_tt *tt)
{
	ttm_tt_destroy_common(ttm_dev, tt);
	ttm_tt_fini(tt);
	kfree(tt);
}

struct ttm_device_funcs xe_ttm_funcs = {
	.ttm_tt_create = xe_ttm_tt_create,
	.ttm_tt_destroy = xe_ttm_tt_destroy,
};

static void xe_ttm_bo_destroy(struct ttm_buffer_object *ttm_bo)
{
	struct xe_bo *bo = ttm_to_xe_bo(ttm_bo);
	struct xe_vma *vma, *next;

	if (!list_empty(&bo->vmas)) {
		if (bo->vm) {
			xe_vm_lock(bo->vm, NULL);
			list_for_each_entry_safe(vma, next, &bo->vmas, bo_link) {
				XE_BUG_ON(vma->vm != bo->vm);
				__xe_vma_unbind(vma);
			}
			xe_vm_unlock(bo->vm);
		} else {
			list_for_each_entry_safe(vma, next, &bo->vmas, bo_link) {
				xe_vm_lock(vma->vm, NULL);
				__xe_vma_unbind(vma);
				xe_vm_unlock(vma->vm);
			}
		}
	}

	kfree(bo);
}

static void xe_gem_object_free(struct drm_gem_object *obj)
{
	/* Our BO reference counting scheme works as follows:
	 *
	 * The ttm_buffer_object and the drm_gem_object each have their own
	 * kref.  We treat the ttm_buffer_object.kref as the "real" reference
	 * count.  The drm_gem_object implicitly owns a reference to the
	 * ttm_buffer_object and, when drm_gem_object.refcount hits zero, we
	 * drop that reference here.  When ttm_buffer_object.kref hits zero,
	 * xe_ttm_bo_destroy() is invoked to do the actual free.
	 */
	xe_bo_put(gem_to_xe_bo(obj));
}

static const struct drm_gem_object_funcs xe_gem_object_funcs = {
	.free = xe_gem_object_free,
	.mmap = drm_gem_ttm_mmap,
};

struct xe_bo *xe_bo_create(struct xe_device *xe, size_t size,
			   struct xe_vm *vm, enum ttm_bo_type type, u32 flags)
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

	bo->size = size;
	bo->ttm.base.funcs = &xe_gem_object_funcs;

	drm_gem_private_object_init(&xe->drm, &bo->ttm.base, size);

	if (vm) {
		xe_vm_lock(vm, NULL);
		bo->vm = xe_vm_get(vm);
	}

	err = ttm_bo_init_reserved(&xe->ttm, &bo->ttm, size, ttm_bo_type_device,
				   &sys_placement, SZ_64K >> PAGE_SHIFT,
				   &ctx, NULL, vm ? &vm->resv : NULL,
				   xe_ttm_bo_destroy);
	if (err)
		return ERR_PTR(err);

	INIT_LIST_HEAD(&bo->vmas);

	xe_bo_or_vm_unlock(bo);

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

	bo = xe_bo_create(xe, args->size, vm, ttm_bo_type_device, args->flags);
	if (IS_ERR(bo)) {
		if (vm)
			xe_vm_put(vm);
		return PTR_ERR(bo);
	}

	err = drm_gem_handle_create(file, &bo->ttm.base, &handle);
	drm_gem_object_put(&bo->ttm.base);
	if (err)
		return err;

	args->handle = handle;
	return 0;
}

int xe_gem_mmap_offset_ioctl(struct drm_device *dev, void *data,
			     struct drm_file *file)
{
	struct drm_xe_gem_mmap_offset *args = data;
	struct drm_gem_object *gem_obj;

	if (args->extensions)
		return -EINVAL;

	if (args->flags)
		return -EINVAL;

	gem_obj = drm_gem_object_lookup(file, args->handle);
	if (!gem_obj)
		return -ENOENT;

	/* The mmap offset was set up at BO allocation time. */
	args->offset = drm_vma_node_offset_addr(&gem_obj->vma_node);

	drm_gem_object_put(gem_obj);
	return 0;
}
