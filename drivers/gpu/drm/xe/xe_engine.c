/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2021 Intel Corporation
 */

#include "xe_engine.h"

#include <drm/xe_drm.h>

#include "xe_bo.h"
#include "xe_device.h"
#include "xe_execlist.h"
#include "xe_vm.h"

struct xe_engine *xe_engine_create(struct xe_device *xe, struct xe_vm *vm,
				   struct xe_hw_engine *hwe)
{
	struct xe_engine *e;
	int err;

	e = kzalloc(sizeof(*e), GFP_KERNEL);
	if (!e)
		return ERR_PTR(-ENOMEM);

	if (hwe->exl_port) {
		e->execlist = xe_execlist_create(xe, hwe);
		if (IS_ERR(e->execlist)) {
			err = PTR_ERR(e->execlist);
			goto err_free;
		}
	}

	e->context = xe_bo_create(xe, vm, xe_hw_engine_context_size(hwe),
				  ttm_bo_type_kernel, XE_BO_CREATE_SYSTEM_BIT);
	if (IS_ERR(e->context)) {
		err = PTR_ERR(e->context);
		goto err_execlist;
	}

	kref_init(&e->refcount);
	e->vm = vm;

	return e;

err_execlist:
	if (e->execlist)
		xe_execlist_destroy(e->execlist);
err_free:
	kfree(e);
	return ERR_PTR(err);
}

void xe_engine_free(struct kref *ref)
{
	struct xe_engine *e = container_of(ref, struct xe_engine, refcount);

	xe_vm_put(e->vm);
	xe_bo_put(e->context);
	if (e->execlist)
		xe_execlist_destroy(e->execlist);

	kfree(e);
}

struct xe_engine *xe_engine_lookup(struct xe_file *xef, u32 id)
{
	struct xe_engine *e;

	mutex_lock(&xef->engine_lock);
	e = xa_load(&xef->engine_xa, id);
	mutex_unlock(&xef->engine_lock);

	if (e)
		xe_engine_get(e);

	return e;
}

int xe_engine_create_ioctl(struct drm_device *dev, void *data,
			   struct drm_file *file)
{
	struct xe_device *xe = to_xe_device(dev);
	struct xe_file *xef = to_xe_file(file);
	struct drm_xe_engine_create *args = data;
	struct xe_hw_engine *hwe;
	struct xe_vm *vm;
	struct xe_engine *e;
	u32 id;
	int err;

	if (args->extensions)
		return -EINVAL;

	if (args->flags)
		return -EINVAL;

	vm = xe_vm_lookup(xef, args->vm_id);
	if (!vm)
		return -ENOENT;

	if (args->instance.engine_class != DRM_XE_ENGINE_CLASS_RENDER)
		return -EINVAL;

	if (args->instance.engine_instance != 0)
		return -EINVAL;

	hwe = &xe->hw_engines[XE_HW_ENGINE_RCS0];

	e = xe_engine_create(xe, vm, hwe);
	if (IS_ERR(e)) {
		xe_vm_put(vm);
		return PTR_ERR(e);
	}

	mutex_lock(&xef->engine_lock);
	err = xa_alloc(&xef->engine_xa, &id, e, xa_limit_32b, GFP_KERNEL);
	mutex_unlock(&xef->engine_lock);
	if (err) {
		xe_engine_put(e);
		return err;
	}

	args->engine_id = id;

	return 0;
}

int xe_engine_destroy_ioctl(struct drm_device *dev, void *data,
			    struct drm_file *file)
{
	struct xe_file *xef = to_xe_file(file);
	struct drm_xe_engine_destroy *args = data;
	struct xe_engine *e;

	if (args->pad)
		return -EINVAL;

	mutex_lock(&xef->engine_lock);
	e = xa_erase(&xef->engine_xa, args->engine_id);
	mutex_unlock(&xef->engine_lock);
	if (!e)
		return -ENOENT;

	xe_engine_put(e);

	return 0;
}
