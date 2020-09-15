/*
 * Copyright 2014 Advanced Micro Devices, Inc.
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sub license, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDERS, AUTHORS AND/OR ITS SUPPLIERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
 * USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 */
/*
 * Authors:
 *    Christian König <christian.koenig@amd.com>
 */

/**
 * DOC: MMU Notifier
 *
 * For coherent userptr handling registers an MMU notifier to inform the driver
 * about updates on the page tables of a process.
 *
 * When somebody tries to invalidate the page tables we block the update until
 * all operations on the pages in question are completed, then those pages are
 * marked as accessed and also dirty if it wasn't a read only access.
 *
 * New command submissions using the userptrs in question are delayed until all
 * page table invalidation are completed and we once more see a coherent process
 * address space.
 */

#include <linux/firmware.h>
#include <linux/module.h>
#include <drm/drm.h>

#include "i915_drv.h"
#include "ttm/i915_ttm.h"

/**
 * i915_ttm_mn_invalidate_gfx - callback to notify about mm change
 *
 * @mni: the range (mm) is about to update
 * @range: details on the invalidation
 * @cur_seq: Value to pass to mmu_interval_set_seq()
 *
 * Block for operations on BOs to finish and mark pages as accessed and
 * potentially dirty.
 */
static bool i915_ttm_mn_invalidate_gfx(struct mmu_interval_notifier *mni,
				     const struct mmu_notifier_range *range,
				     unsigned long cur_seq)
{
	struct drm_i915_gem_object *obj = container_of(mni, struct drm_i915_gem_object, ttm.notifier);
	struct drm_i915_private *i915 = obj_to_i915(obj);
	long r;

	if (!mmu_notifier_range_blockable(range))
		return false;

	mutex_lock(&i915->ttm_mman.notifier_lock);

	mmu_interval_set_seq(mni, cur_seq);

	r = dma_resv_wait_timeout_rcu(obj->base.base.resv, true, false,
				      MAX_SCHEDULE_TIMEOUT);
	mutex_unlock(&i915->ttm_mman.notifier_lock);
	if (r <= 0)
		DRM_ERROR("(%ld) failed to wait for user bo\n", r);
	return true;
}

static const struct mmu_interval_notifier_ops i915_ttm_mn_gfx_ops = {
	.invalidate = i915_ttm_mn_invalidate_gfx,
};

/**
 * i915_ttm_mn_register - register a BO for notifier updates
 *
 * @bo: amdgpu buffer object
 * @addr: userptr addr we should monitor
 *
 * Registers a mmu_notifier for the given BO at the specified address.
 * Returns 0 on success, -ERRNO if anything goes wrong.
 */
int i915_ttm_mn_register(struct drm_i915_gem_object *obj, unsigned long addr)
{
	return mmu_interval_notifier_insert(&obj->ttm.notifier, current->mm, addr,
					    i915_gem_object_size(obj),
					    &i915_ttm_mn_gfx_ops);
}

/**
 * i915_ttm_mn_unregister - unregister a BO for notifier updates
 *
 * @bo: amdgpu buffer object
 *
 * Remove any registration of mmu notifier updates from the buffer object.
 */
void i915_ttm_mn_unregister(struct drm_i915_gem_object *obj)
{
	if (!obj->ttm.notifier.mm)
		return;
	mmu_interval_notifier_remove(&obj->ttm.notifier);
	obj->ttm.notifier.mm = NULL;
}
