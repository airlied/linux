/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2021 Intel Corporation
 */

#include <drm/xe_drm.h>

#include "xe_vm.h"

struct xe_vm *xe_vm_create(struct xe_device *xe)
{
	struct xe_vm *vm;

	vm = kzalloc(sizeof(*vm), GFP_KERNEL);
	if (!vm)
		return ERR_PTR(-ENOMEM);

	kref_init(&vm->refcount);

	return vm;
}

void xe_vm_free(struct kref *ref)
{
	struct xe_vm *vm = container_of(ref, struct xe_vm, refcount);

	kfree(vm);
}

struct xe_vm *xe_vm_lookup(struct xe_file *xef, u32 id)
{
	struct xe_vm *vm;

	mutex_lock(&xef->vm_lock);
	vm = xa_load(&xef->vm_xa, id);
	mutex_unlock(&xef->vm_lock);

	if (vm)
		xe_vm_get(vm);

	return vm;
}

int xe_vm_create_ioctl(struct drm_device *dev, void *data,
		       struct drm_file *file)
{
	struct xe_device *xe = to_xe_device(dev);
	struct xe_file *xef = to_xe_file(file);
	struct drm_xe_vm_create *args = data;
	struct xe_vm *vm;
	u32 id;
	int err;

	if (args->extensions)
		return -EINVAL;

	if (args->flags)
		return -EINVAL;

	vm = xe_vm_create(xe);
	if (IS_ERR(vm))
		return PTR_ERR(vm);

	mutex_lock(&xef->vm_lock);
	err = xa_alloc(&xef->vm_xa, &id, vm, xa_limit_32b, GFP_KERNEL);
	mutex_unlock(&xef->vm_lock);
	if (err) {
		xe_vm_put(vm);
		return err;
	}

	args->vm_id = id;

	return 0;
}

int xe_vm_destroy_ioctl(struct drm_device *dev, void *data,
			struct drm_file *file)
{
	struct xe_file *xef = to_xe_file(file);
	struct drm_xe_vm_destroy *args = data;
	struct xe_vm *vm;

	if (args->pad)
		return -EINVAL;

	mutex_lock(&xef->vm_lock);
	vm = xa_erase(&xef->vm_xa, args->vm_id);
	mutex_unlock(&xef->vm_lock);
	if (!vm)
		return -ENOENT;

	xe_vm_put(vm);

	return 0;
}
