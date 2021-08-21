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

static struct xe_engine *__xe_engine_create(struct xe_device *xe,
					    struct xe_vm *vm,
					    struct xe_hw_engine *hwe)
{
	struct xe_engine *e;
	int err;

	e = kzalloc(sizeof(*e), GFP_KERNEL);
	if (!e)
		return ERR_PTR(-ENOMEM);

	e->hwe = hwe;
	kref_init(&e->refcount);
	e->vm = xe_vm_get(vm);

	err = xe_lrc_init(&e->lrc, hwe, vm, SZ_16K);
	if (err)
		goto err_kfree;

	if (hwe->exl_port) {
		e->execlist = xe_execlist_create(e);
		if (IS_ERR(e->execlist)) {
			err = PTR_ERR(e->execlist);
			goto err_lrc;
		}
		e->entity = &e->execlist->entity;
	}

	return e;

err_lrc:
	xe_lrc_finish(&e->lrc);
err_kfree:
	kfree(e);
	return ERR_PTR(err);
}

struct xe_engine *xe_engine_create(struct xe_device *xe, struct xe_vm *vm,
				   struct xe_hw_engine *hwe)
{
	struct xe_engine *e;

	xe_vm_lock(vm, NULL);
	e = __xe_engine_create(xe, vm, hwe);
	xe_vm_unlock(vm);

	return e;
}

void xe_engine_free(struct kref *ref)
{
	struct xe_engine *e = container_of(ref, struct xe_engine, refcount);

	if (e->execlist)
		xe_execlist_destroy(e->execlist);

	xe_lrc_finish(&e->lrc);
	xe_vm_put(e->vm);

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

static const enum xe_engine_class user_to_xe_engine_class[] = {
	[DRM_XE_ENGINE_CLASS_RENDER] = XE_ENGINE_CLASS_RENDER,
};

static struct xe_hw_engine *
find_hw_engine(struct xe_device *xe,
	       struct drm_xe_engine_class_instance eci)
{
	enum xe_engine_class class;
	unsigned int instance;
	int i;

	if (eci.engine_class > ARRAY_SIZE(user_to_xe_engine_class))
		return NULL;

	class = user_to_xe_engine_class[eci.engine_class];
	instance = eci.engine_instance;

	for (i = 0; i < ARRAY_SIZE(xe->hw_engines); i++) {
		if (xe->hw_engines[i].xe &&
		    xe->hw_engines[i].class == class &&
		    xe->hw_engines[i].instance == instance)
			return &xe->hw_engines[i];
	}

	return NULL;
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

	if (XE_IOCTL_ERR(xe, args->extensions))
		return -EINVAL;

	if (XE_IOCTL_ERR(xe, args->flags))
		return -EINVAL;

	hwe = find_hw_engine(xe, args->instance);
	if (XE_IOCTL_ERR(xe, !hwe))
		return -EINVAL;

	vm = xe_vm_lookup(xef, args->vm_id);
	if (XE_IOCTL_ERR(xe, !vm))
		return -ENOENT;

	e = xe_engine_create(xe, vm, hwe);
	xe_vm_put(vm);
	if (IS_ERR(e))
		return PTR_ERR(e);

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
	struct xe_device *xe = to_xe_device(dev);
	struct xe_file *xef = to_xe_file(file);
	struct drm_xe_engine_destroy *args = data;
	struct xe_engine *e;

	if (XE_IOCTL_ERR(xe, args->pad))
		return -EINVAL;

	mutex_lock(&xef->engine_lock);
	e = xa_erase(&xef->engine_xa, args->engine_id);
	mutex_unlock(&xef->engine_lock);
	if (XE_IOCTL_ERR(xe, !e))
		return -ENOENT;

	xe_engine_put(e);

	return 0;
}
