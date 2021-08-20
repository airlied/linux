/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2021 Intel Corporation
 */

#ifndef _XE_BO_H_
#define _XE_BO_H_

#include <drm/drm_mm.h>
#include <drm/ttm/ttm_bo_api.h>
#include <drm/ttm/ttm_device.h>
#include <drm/ttm/ttm_placement.h>
#include "xe_device.h"
#include "xe_vm.h"

#define XE_DEFAULT_GTT_SIZE_MB          3072ULL /* 3GB by default */
#define XE_BO_MAX_PLACEMENTS	3

struct xe_bo {
	struct ttm_buffer_object ttm;

	size_t size;

	uint32_t flags;

	struct xe_vm *vm;

	struct list_head vmas;

	struct ttm_place placements[XE_BO_MAX_PLACEMENTS];
	struct ttm_placement placement;

	struct drm_mm_node ggtt_node;
};

#define XE_BO_CREATE_USER_BIT BIT(1)
#define XE_BO_CREATE_SYSTEM_BIT BIT(2)

struct xe_bo *xe_bo_create(struct xe_device *xe, struct xe_vm *vm, size_t size,
			   enum ttm_bo_type type, uint32_t flags);

static inline struct xe_bo *ttm_to_xe_bo(const struct ttm_buffer_object *bo)
{
	return container_of(bo, struct xe_bo, ttm);
}

static inline struct xe_bo *gem_to_xe_bo(const struct drm_gem_object *obj)
{
	return container_of(obj, struct xe_bo, ttm.base);
}

static inline struct xe_device *xe_bo_device(struct xe_bo *bo)
{
	return ttm_to_xe_device(bo->ttm.bdev);
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

int xe_bo_populate(struct xe_bo *bo);
bool xe_bo_is_xe_bo(struct ttm_buffer_object *bo);
dma_addr_t xe_bo_addr(struct xe_bo *bo, uint64_t offset, size_t page_size);

void *xe_bo_kmap(struct xe_bo *bo, unsigned long offset, unsigned long range,
		 struct ttm_bo_kmap_obj *map);
static inline void xe_bo_kunmap(struct ttm_bo_kmap_obj *map)
{
	ttm_bo_kunmap(map);
}

static inline bool
xe_bo_is_in_lmem(struct xe_bo *bo)
{
	xe_bo_assert_held(bo);
	return bo->ttm.resource->mem_type == TTM_PL_VRAM;
}

static inline uint32_t
xe_bo_ggtt_addr(struct xe_bo *bo)
{
	XE_BUG_ON(bo->ggtt_node.size != bo->size);
	XE_BUG_ON(bo->ggtt_node.start + bo->ggtt_node.size > (1ull << 32));
	return bo->ggtt_node.start;
}

extern struct ttm_device_funcs xe_ttm_funcs;

int xe_gem_create_ioctl(struct drm_device *dev, void *data,
			struct drm_file *file);
int xe_gem_mmap_offset_ioctl(struct drm_device *dev, void *data,
			     struct drm_file *file);

#endif /* _XE_BO_H_ */
