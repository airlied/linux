/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2021 Intel Corporation
 */

#include <drm/xe_drm.h>

#include "xe_bo.h"
#include "xe_vm.h"

static struct xe_vma *xe_vma_create(struct xe_vm *vm,
				    struct xe_bo *bo, uint64_t bo_offset,
				    uint64_t start, uint64_t end)
{
	struct xe_vma *vma;

	XE_BUG_ON(start >= end);
	XE_BUG_ON(end >= vm->size);

	vma = kzalloc(sizeof(*vma), GFP_KERNEL);
	if (!vma)
		return NULL;

	vma->vm = vm;
	vma->start = start;
	vma->end = end;

	if (bo) {
		xe_bo_assert_held(bo);

		vma->bo = bo;
		vma->bo_offset = bo_offset;
		list_add_tail(&vma->bo_link, &bo->vmas);
	}

	return vma;
}

static struct xe_vma *xe_vma_clone(struct xe_vma *old)
{
	return xe_vma_create(old->vm, old->bo, old->bo_offset,
			     old->start, old->end);
}

static void xe_vma_make_empty(struct xe_vma *vma)
{
	if (!vma->bo)
		return;

	vma->bo = NULL;
	vma->bo_offset = 0;
	list_del(&vma->bo_link);
}

static void xe_vma_destroy(struct xe_vma *vma)
{
	xe_vma_make_empty(vma);
	kfree(vma);
}

static struct xe_vma *to_xe_vma(const struct rb_node *node)
{
	BUILD_BUG_ON(offsetof(struct xe_vma, vm_node) != 0);
	return (struct xe_vma *)node;
}

static struct xe_vma *xe_vma_next(const struct xe_vma *vma)
{
	return to_xe_vma(rb_next(&vma->vm_node));
}

static void xe_vma_trim_start(struct xe_vma *vma, uint64_t new_start)
{
	XE_BUG_ON(new_start <= vma->start);
	XE_BUG_ON(new_start >= vma->end);

	if (vma->bo)
		vma->bo_offset += new_start - vma->start;
	vma->start = new_start;
}

static void xe_vma_trim_end(struct xe_vma *vma, uint64_t new_end)
{
	XE_BUG_ON(new_end <= vma->start);
	XE_BUG_ON(new_end >= vma->end);

	vma->end = new_end;
}

static int xe_vma_cmp(const struct xe_vma *a, const struct xe_vma *b)
{
	if (a->end < b->start) {
		return -1;
	} else if (b->end < a->start) {
		return 1;
	} else {
		return 0;
	}
}

static int xe_vma_cmp_addr(uint64_t addr, const struct xe_vma *vma)
{
	if (addr < vma->start)
		return -1;
	else if (addr > vma->end)
		return 1;
	else
		return 0;
}

static bool xe_vma_less_cb(struct rb_node *a, const struct rb_node *b)
{
	return xe_vma_cmp(to_xe_vma(a), to_xe_vma(b)) < 0;
}

static int xe_vma_cmp_addr_cb(const void *key, const struct rb_node *node)
{
	return xe_vma_cmp_addr(*(uint64_t *)key, to_xe_vma(node));
}

static struct xe_vma *xe_vm_find_vma(struct xe_vm *vm, uint64_t addr)
{
	struct rb_node *node;

	XE_BUG_ON(addr >= vm->size);

	node = rb_find(&addr, &vm->vmas, xe_vma_cmp_addr_cb);
	XE_BUG_ON(!node);

	return to_xe_vma(node);
}

static void xe_vm_insert_vma(struct xe_vm *vm, struct xe_vma *vma)
{
	XE_BUG_ON(vma->vm != vm);

	rb_add(&vma->vm_node, &vm->vmas, xe_vma_less_cb);
}

static void xe_vm_remove_vma(struct xe_vm *vm, struct xe_vma *vma)
{
	XE_BUG_ON(vma->vm != vm);

	rb_erase(&vma->vm_node, &vm->vmas);
}

static void xe_vm_replace_vma(struct xe_vm *vm, struct xe_vma *old,
			      struct xe_vma *new)
{
	XE_BUG_ON(old->vm != vm || new->vm != vm);
	XE_BUG_ON(old == new);

	rb_replace_node(&old->vm_node, &new->vm_node, &vm->vmas);
}

struct xe_vm *xe_vm_create(struct xe_device *xe)
{
	struct xe_vm *vm;
	struct xe_vma *vma;
	int err;

	vm = kzalloc(sizeof(*vm), GFP_KERNEL);
	if (!vm)
		return ERR_PTR(-ENOMEM);

	kref_init(&vm->refcount);
	dma_resv_init(&vm->resv);

	vm->size = 1ull << 48;

	vm->vmas = RB_ROOT;
	vma = xe_vma_create(vm, NULL, 0, 0, vm->size - 1);
	if (!vma) {
		err = -ENOMEM;
		goto err_resv;
	}
	xe_vm_insert_vma(vm, vma);

	return vm;

err_resv:
	dma_resv_fini(&vm->resv);
	kfree(vm);
	return ERR_PTR(err);
}

void xe_vm_free(struct kref *ref)
{
	struct xe_vm *vm = container_of(ref, struct xe_vm, refcount);

	dma_resv_fini(&vm->resv);

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

static void
__xe_vm_trim_later_vmas(struct xe_vm *vm, struct xe_vma *vma,
			struct xe_vma *later)
{
	while (1) {
		XE_BUG_ON(!later);
		XE_BUG_ON(later->start < vma->start);

		if (later->end <= vma->end) {
			struct xe_vma *next = NULL;

			if (later->end < vma->end)
				next = xe_vma_next(later);

			xe_vm_remove_vma(vm, later);
			xe_vma_destroy(later);

			if (!next)
				return;

			later = next;
		} else {
			xe_vma_trim_start(later, vma->end + 1);
			return;
		}
	}
}

static int __xe_vm_bind_vma(struct xe_vm *vm, struct xe_vma *vma)
{
	struct xe_vma *prev, *next;

	prev = xe_vm_find_vma(vm, vma->start);
	XE_BUG_ON(prev->start > vma->start);

	if (prev->start == vma->start && prev->end == vma->end) {
		xe_vm_replace_vma(vm, prev, vma);
		xe_vma_destroy(prev);
	} else if (prev->start < vma->start && vma->end < prev->end) {
		/* vma is strictly contained in prev.  In this case, we
		 * have to split prev.
		 */
		next = xe_vma_clone(prev);
		if (!next)
			return -ENOMEM;

		xe_vma_trim_end(prev, vma->start - 1);
		xe_vma_trim_end(next, vma->end + 1);
		xe_vm_insert_vma(vm, vma);
		xe_vm_insert_vma(vm, next);
	} else if (prev->start < vma->start) {
		prev->end = vma->start - 1;
		xe_vm_insert_vma(vm, vma);
	} else {
		XE_BUG_ON(prev->start != vma->start);
		__xe_vm_trim_later_vmas(vm, vma, prev);
		xe_vm_insert_vma(vm, vma);
	}

	return 0;
}

static int __xe_vm_bind(struct xe_vm *vm, struct xe_bo *bo, uint64_t bo_offset,
			uint64_t range, uint64_t addr)
{
	struct xe_vma *vma;
	int err;

	xe_vm_assert_held(vm);

	vma = xe_vma_create(vm, bo, bo_offset, addr, addr + range);
	err = __xe_vm_bind_vma(vm, vma);
	if (err)
		xe_vma_destroy(vma);

	return err;
}

void __xe_vma_unbind(struct xe_vma *vma)
{
	xe_vm_assert_held(vma->vm);
	xe_vma_make_empty(vma);
}

static int xe_vm_bind(struct xe_vm *vm, struct xe_bo *bo, uint64_t offset,
		      uint64_t range, uint64_t addr)
{
	int err;

	/* TODO: Allow binding shared BOs */
	if (bo->vm != vm)
		return -EINVAL;

	if (range == 0)
		return -EINVAL;

	if (addr >= vm->size || range >= vm->size - addr)
		return -EINVAL;


	xe_vm_lock(vm, NULL);
	err = __xe_vm_bind(vm, bo, offset, range, addr);
	xe_vm_unlock(vm);

	return err;
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

int xe_vm_bind_ioctl(struct drm_device *dev, void *data,
		     struct drm_file *file)
{
	struct xe_file *xef = to_xe_file(file);
	struct drm_xe_vm_bind *args = data;
	struct drm_gem_object *gem_obj;
	struct xe_vm *vm;
	int err = 0;

	if (args->extensions)
		return -EINVAL;

	vm = xe_vm_lookup(xef, args->vm_id);
	if (!vm)
		return -ENOENT;

	gem_obj = drm_gem_object_lookup(file, args->obj);
	if (!gem_obj) {
		err = -ENOENT;
		goto put_vm;
	}

	err = xe_vm_bind(vm, gem_to_xe_bo(gem_obj), args->offset,
			 args->range, args->addr);

	drm_gem_object_put(gem_obj);
put_vm:
	xe_vm_put(vm);
	return err;
}
