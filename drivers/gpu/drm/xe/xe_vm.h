/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2021 Intel Corporation
 */

#ifndef _XE_VM_H_
#define _XE_VM_H_

#include <linux/kref.h>
#include <linux/dma-resv.h>

#include <drm/drm_device.h>
#include <drm/drm_file.h>

struct xe_bo;
struct xe_device;
struct xe_file;
struct xe_pt;
struct xe_vm;

struct xe_vma {
	struct rb_node vm_node;
	struct xe_vm *vm;

	uint64_t start;
	uint64_t end;

	struct xe_bo *bo;
	uint64_t bo_offset;
	struct list_head bo_link;
};

void __xe_vma_unbind(struct xe_vma *vma);

struct xe_vm {
	struct xe_device *xe;

	struct kref refcount;

	struct dma_resv resv;

	uint64_t size;
	struct rb_root vmas;

	struct xe_pt *pt_root;
};

struct xe_vm *xe_vm_create(struct xe_device *xe);
void xe_vm_free(struct kref *ref);

struct xe_vm *xe_vm_lookup(struct xe_file *xef, u32 id);

static inline struct xe_vm *xe_vm_get(struct xe_vm *vm)
{
	kref_get(&vm->refcount);
	return vm;
}

static inline void xe_vm_put(struct xe_vm *vm)
{
	kref_put(&vm->refcount, xe_vm_free);
}

static inline void xe_vm_lock(struct xe_vm *vm, struct ww_acquire_ctx *ctx)
{
	dma_resv_lock(&vm->resv, ctx);
}

static inline void xe_vm_unlock(struct xe_vm *vm)
{
	dma_resv_unlock(&vm->resv);
}

#define xe_vm_assert_held(vm) dma_resv_assert_held(&(vm)->resv)

int xe_vm_create_ioctl(struct drm_device *dev, void *data,
		       struct drm_file *file);
int xe_vm_destroy_ioctl(struct drm_device *dev, void *data,
			struct drm_file *file);
int xe_vm_bind_ioctl(struct drm_device *dev, void *data,
		     struct drm_file *file);

extern struct ttm_device_funcs xe_ttm_funcs;

#endif /* _XE_VM_H_ */
