/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2021 Intel Corporation
 */

#include "xe_vm.h"

#include <drm/ttm/ttm_tt.h>
#include <drm/xe_drm.h>
#include <linux/mm.h>

#include "xe_bo.h"

enum xe_cache_level {
	XE_CACHE_NONE,
	XE_CACHE_WT,
	XE_CACHE_WB,
};

#define PTE_READ_ONLY	BIT(0)
#define PTE_LM		BIT(1)

#define PPAT_UNCACHED			(_PAGE_PWT | _PAGE_PCD)
#define PPAT_CACHED_PDE			0 /* WB LLC */
#define PPAT_CACHED			_PAGE_PAT /* WB LLCeLLC */
#define PPAT_DISPLAY_ELLC		_PAGE_PCD /* WT eLLC */

#define XE_PTES(pte_len)		((unsigned int)(PAGE_SIZE / (pte_len)))
#define XE_PTE_MASK(pte_len)		(XE_PTES(pte_len) - 1)
#define XE_PDES				512
#define XE_PDE_MASK			(XE_PDES - 1)

#define GEN12_PPGTT_PTE_LM	BIT_ULL(11)

#define GEN8_PDE_EMPTY 0
#define GEN8_PTE_EMPTY 0

static uint64_t gen8_pde_encode(const dma_addr_t addr,
				const enum xe_cache_level level)
{
	uint64_t pde = addr | _PAGE_PRESENT | _PAGE_RW;

	if (level != XE_CACHE_NONE)
		pde |= PPAT_CACHED_PDE;
	else
		pde |= PPAT_UNCACHED;

	return pde;
}

static uint64_t gen8_pte_encode(dma_addr_t addr,
				enum xe_cache_level level,
				uint32_t flags)
{
	uint64_t pte = addr | _PAGE_PRESENT | _PAGE_RW;

	if (unlikely(flags & PTE_READ_ONLY))
		pte &= ~_PAGE_RW;

	if (flags & PTE_LM)
		pte |= GEN12_PPGTT_PTE_LM;

	switch (level) {
	case XE_CACHE_NONE:
		pte |= PPAT_UNCACHED;
		break;
	case XE_CACHE_WT:
		pte |= PPAT_DISPLAY_ELLC;
		break;
	default:
		pte |= PPAT_CACHED;
		break;
	}

	return pte;
}

static dma_addr_t addr_for_bo(struct xe_bo *bo, uint64_t offset)
{
	return bo->ttm.ttm->dma_address[0];
}

#define GEN8_PTE_SHIFT 12
#define GEN8_PAGE_SIZE (1 << GEN8_PTE_SHIFT)
#define GEN8_PTE_MASK (GEN8_PAGE_SIZE - 1)
#define GEN8_PDE_SHIFT (GEN8_PTE_SHIFT - 3)
#define GEN8_PDES (1 << GEN8_PDE_SHIFT)
#define GEN8_PDE_MASK (GEN8_PDES - 1)

struct xe_pt {
	struct xe_bo *bo;
	unsigned int level;
	unsigned int num_live;

	struct ttm_bo_kmap_obj map;
};

struct xe_pt_dir {
	struct xe_pt pt;
	struct xe_pt *entries[GEN8_PDES];
};

struct xe_pt_0 {
	struct xe_pt pt;
	uint32_t live[GEN8_PDES / 32];
};

static struct xe_pt_dir *as_xe_pt_dir(struct xe_pt *pt)
{
	return container_of(pt, struct xe_pt_dir, pt);
}

static struct xe_pt_0 *as_xe_pt_0(struct xe_pt *pt)
{
	return container_of(pt, struct xe_pt_0, pt);
}

static bool xe_pt_0_is_live(struct xe_pt_0 *pt, unsigned int idx)
{
	return pt->live[idx / 32] & (1u << (idx % 32));
}

static bool xe_pt_0_set_live(struct xe_pt_0 *pt, unsigned int idx)
{
	return pt->live[idx / 32] |= (1u << (idx % 32));
}

static bool xe_pt_0_clear_live(struct xe_pt_0 *pt, unsigned int idx)
{
	return pt->live[idx / 32] &= ~(1u << (idx % 32));
}

struct xe_pt *xe_pt_create(struct xe_vm *vm, unsigned int level)
{
	struct xe_pt *pt;
	struct xe_bo *bo;
	size_t size;
	int err;

	size = level ? sizeof(struct xe_pt_dir) : sizeof(struct xe_pt_0);
	pt = kzalloc(size, GFP_KERNEL);
	if (!pt)
		return NULL;

	bo = xe_bo_create(vm->xe, vm, SZ_4K, ttm_bo_type_kernel, 0);
	if (IS_ERR(bo)) {
		err = PTR_ERR(bo);
		goto err_kfree;
	}
	pt->bo = bo;

	err = ttm_bo_kmap(&pt->bo->ttm, 0, 1, &pt->map);
	if (err)
		goto err_bo;

	pt->level = level;
	pt->num_live = 0;

	return pt;

err_bo:
	xe_bo_put(bo);
err_kfree:
	kfree(pt);
	return ERR_PTR(err);
}

static void xe_pt_destroy(struct xe_pt *pt)
{
	int i;

	ttm_bo_kunmap(&pt->map);

	XE_BUG_ON(!list_empty(&pt->bo->vmas));
	xe_bo_put(pt->bo);

	if (pt->level > 0) {
		struct xe_pt_dir *pt_dir = as_xe_pt_dir(pt);

		for (i = 0; i < XE_PDES; i++) {
			if (pt_dir->entries[i])
				xe_pt_destroy(pt_dir->entries[i]);
		}
	}
	kfree(pt);
}

static void xe_pt_write(struct xe_pt *pt, unsigned int idx, uint64_t data)
{
	bool is_iomem;
	uint64_t *map;

	map = ttm_kmap_obj_virtual(&pt->map, &is_iomem);
	*map = data;
}

static unsigned int xe_pt_shift(unsigned int level)
{
	return GEN8_PTE_SHIFT + GEN8_PDE_SHIFT * level;
}

static unsigned int xe_pt_idx(uint64_t addr, unsigned int level)
{
	return (addr >> xe_pt_shift(level)) & GEN8_PDE_MASK;
}

static uint64_t xe_pt_next_start(uint64_t start, unsigned int level)
{
	uint64_t pt_range = 1ull << xe_pt_shift(level);

	return (start + pt_range) & ~(pt_range - 1);
}

static uint64_t __xe_pt_clear(struct xe_pt *pt, unsigned int level,
			      uint64_t start, uint64_t end,
			      bool depopulate)
{
	uint64_t next_pt_start = xe_pt_next_start(start, level);

	XE_BUG_ON(start >= end);
	XE_BUG_ON(start & GEN8_PTE_MASK);

	if (!pt)
		return next_pt_start;

	if (level == 0) {
		struct xe_pt_0 *pt_0 = as_xe_pt_0(pt);

		while (start < end && start < next_pt_start) {
			unsigned int i = xe_pt_idx(start, 0);

			start += GEN8_PAGE_SIZE;
			if (!xe_pt_0_is_live(pt_0, i))
				continue;

			xe_pt_write(pt, i, GEN8_PTE_EMPTY);
			xe_pt_0_clear_live(pt_0, i);
			pt->num_live--;
		}
	} else {
		struct xe_pt_dir *pt_dir = as_xe_pt_dir(pt);

		while (start < end && start < next_pt_start) {
			unsigned int i = xe_pt_idx(start, level);
			struct xe_pt *entry = pt_dir->entries[i];

			start = __xe_pt_clear(entry, level - 1, start, end,
					      depopulate);
			if (entry && !entry->num_live && depopulate) {
				xe_pt_write(pt, i, GEN8_PDE_EMPTY);
				xe_pt_destroy(entry);
				pt_dir->entries[i] = NULL;
				pt->num_live--;
			}
		}
	}

	return start;
}

static void xe_pt_clear(struct xe_pt *pt, uint64_t start, uint64_t end,
			bool depopulate)
{
	__xe_pt_clear(pt, pt->level, start, end, depopulate);
}

static int __xe_pt_populate(struct xe_vm *vm, struct xe_pt *pt,
			    unsigned int level,
			    uint64_t *start, uint64_t end)
{
	uint64_t next_pt_start = xe_pt_next_start(*start, level);
	struct xe_pt_dir *pt_dir;
	int err;

	XE_BUG_ON(*start >= end);
	XE_BUG_ON(*start & GEN8_PTE_MASK);
	XE_BUG_ON(end >= (1ull << 63));

	if (level == 0) {
		*start = next_pt_start;
		return 0;
	}

	pt_dir = as_xe_pt_dir(pt);

	while (*start < end && *start < next_pt_start) {
		unsigned int i = xe_pt_idx(*start, level);

		if (!pt_dir->entries[i]) {
			struct xe_pt *entry;
			uint64_t pde;

			entry = xe_pt_create(vm, level - 1);
			if (IS_ERR(entry))
				return PTR_ERR(entry);

			pde = gen8_pde_encode(addr_for_bo(entry->bo, 0),
					      XE_CACHE_WB);
			xe_pt_write(pt, i, pde);
			pt_dir->entries[i] = entry;
			pt->num_live--;
		}
		err = __xe_pt_populate(vm, pt_dir->entries[i],
				       level - 1, start, end);
		if (err < 0)
			return err;
	}

	return 0;
}

static int xe_pt_populate(struct xe_vm *vm, struct xe_pt *pt,
			  uint64_t start, uint64_t end)
{
	return __xe_pt_populate(vm, pt, pt->level, &start, end);
}

static void xe_pt_set_pte(struct xe_pt *pt, uint64_t addr, uint64_t pte)
{
	unsigned int i = xe_pt_idx(addr, pt->level);

	if (pt->level > 0) {
		struct xe_pt_dir *pt_dir = as_xe_pt_dir(pt);

		XE_BUG_ON(!pt_dir->entries[i]);
		xe_pt_set_pte(pt_dir->entries[i], addr, pte);
	} else {
		struct xe_pt_0 *pt_0 = as_xe_pt_0(pt);

		xe_pt_write(pt, i, pte);
		if (!xe_pt_0_is_live(pt_0, i)) {
			xe_pt_0_set_live(pt_0, i);
			pt->num_live++;
		}
	}
}

static int xe_pt_fill(struct xe_pt *pt, struct xe_bo *bo, uint64_t bo_offset,
		      uint64_t start, uint64_t end)
{
	uint64_t pte;
	uint32_t flags = 0;
	XE_BUG_ON(end - start + bo_offset > bo->size);

	if (bo->ttm.resource->mem_type == TTM_PL_VRAM)
		flags |= PTE_LM;

	while (start < end) {
		pte = gen8_pte_encode(addr_for_bo(bo, bo_offset),
				      XE_CACHE_WB, flags);
		xe_pt_set_pte(pt, start, pte);

		start += GEN8_PAGE_SIZE;
		bo_offset += GEN8_PAGE_SIZE;
	}

	return 0;
}

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

	vm->xe = xe;
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

	xe_vm_lock(vm, NULL);
	vm->pt_root = xe_pt_create(vm, 3);
	xe_vm_unlock(vm);
	if (IS_ERR(vm->pt_root)) {
		err = PTR_ERR(vm->pt_root);
		goto err_vma;
	}

	return vm;

err_vma:
	kfree(vma);
err_resv:
	dma_resv_fini(&vm->resv);
	kfree(vm);
	return ERR_PTR(err);
}

void xe_vm_free(struct kref *ref)
{
	struct xe_vm *vm = container_of(ref, struct xe_vm, refcount);

	dma_resv_fini(&vm->resv);

	while (vm->vmas.rb_node) {
		struct rb_node *node = vm->vmas.rb_node;

		rb_erase(node, &vm->vmas);
		xe_vma_destroy(to_xe_vma(node));
	}

	xe_pt_destroy(vm->pt_root);

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

static int __xe_vm_insert_vma(struct xe_vm *vm, struct xe_vma *vma)
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

	err = xe_bo_populate(bo);
	if (err)
		return err;

	err = xe_pt_populate(vm, vm->pt_root, addr, addr + range);
	if (err)
		return err;

	vma = xe_vma_create(vm, bo, bo_offset, addr, addr + range);
	err = __xe_vm_insert_vma(vm, vma);
	if (err)
		xe_vma_destroy(vma);

	xe_pt_fill(vm->pt_root, bo, bo_offset, addr, addr + range);

	return err;
}

void __xe_vma_unbind(struct xe_vma *vma)
{
	xe_vm_assert_held(vma->vm);
	xe_vma_make_empty(vma);
	xe_pt_clear(vma->vm->pt_root, vma->start, vma->end, true);
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

	if (range >= vm->size || addr >= vm->size - range)
		return -EINVAL;

	if (range > bo->size || offset > bo->size - range)
		return -EINVAL;

	xe_vm_lock(vm, NULL);
	xe_bo_lock_vm_held(bo, NULL);
	err = __xe_vm_bind(vm, bo, offset, range, addr);
	xe_bo_unlock_vm_held(bo);
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
