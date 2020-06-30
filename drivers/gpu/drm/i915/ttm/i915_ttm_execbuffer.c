/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2008,2010 Intel Corporation
 */

#include <linux/intel-iommu.h>
#include <linux/dma-resv.h>
#include <linux/sync_file.h>
#include <linux/uaccess.h>

#include "i915_drv.h"
#include "gt/intel_context.h"
#include "gt/intel_ring.h"
#include "gem/i915_gem_context.h"
#include "ttm/i915_ttm.h"

struct i915_ttm_execbuffer {
	struct drm_i915_private *i915;
	struct drm_file *file;

	struct intel_engine_cs *engine;
	struct intel_context *context;
	struct i915_gem_context *gem_context; /** caller's context */

	struct drm_i915_gem_execbuffer2 *args;
	struct i915_ttm_bo_list *bo_list;
	struct list_head validated;
	struct ww_acquire_ctx ticket;

	struct i915_request *request; /** our request to build */
	u32 batch_start_offset; /** Location within object of batch */
	u32 batch_len; /** Length of batch within object */
	u32 batch_flags; /** Flags composed for emit_bb_start() */
};

static struct i915_request *eb_throttle(struct intel_context *ce)
{
	struct intel_ring *ring = ce->ring;
	struct intel_timeline *tl = ce->timeline;
	struct i915_request *rq;

	/*
	 * Completely unscientific finger-in-the-air estimates for suitable
	 * maximum user request size (to avoid blocking) and then backoff.
	 */
	if (intel_ring_update_space(ring) >= PAGE_SIZE)
		return NULL;

	/*
	 * Find a request that after waiting upon, there will be at least half
	 * the ring available. The hysteresis allows us to compete for the
	 * shared ring and should mean that we sleep less often prior to
	 * claiming our resources, but not so long that the ring completely
	 * drains before we can submit our next request.
	 */
	list_for_each_entry(rq, &tl->requests, link) {
		if (rq->ring != ring)
			continue;

		if (__intel_ring_space(rq->postfix,
				       ring->emit, ring->size) > ring->size / 2)
			break;
	}
	if (&rq->link == &tl->requests)
		return NULL; /* weird, we will check again later for real */

	return i915_request_get(rq);
}

static int __eb_pin_engine(struct i915_ttm_execbuffer *eb, struct intel_context *ce)
{
	struct intel_timeline *tl;
	struct i915_request *rq;
	int err;

	/*
	 * ABI: Before userspace accesses the GPU (e.g. execbuffer), report
	 * EIO if the GPU is already wedged.
	 */
	err = intel_gt_terminally_wedged(ce->engine->gt);
	if (err)
		return err;

	if (unlikely(intel_context_is_banned(ce)))
		return -EIO;

	/*
	 * Pinning the contexts may generate requests in order to acquire
	 * GGTT space, so do this first before we reserve a seqno for
	 * ourselves.
	 */
	err = intel_context_pin(ce);
	if (err)
		return err;

	/*
	 * Take a local wakeref for preparing to dispatch the execbuf as
	 * we expect to access the hardware fairly frequently in the
	 * process, and require the engine to be kept awake between accesses.
	 * Upon dispatch, we acquire another prolonged wakeref that we hold
	 * until the timeline is idle, which in turn releases the wakeref
	 * taken on the engine, and the parent device.
	 */
	tl = intel_context_timeline_lock(ce);
	if (IS_ERR(tl)) {
		err = PTR_ERR(tl);
		goto err_unpin;
	}

	intel_context_enter(ce);
	rq = eb_throttle(ce);

	intel_context_timeline_unlock(tl);

	if (rq) {
		bool nonblock = eb->file->filp->f_flags & O_NONBLOCK;
		long timeout;

		timeout = MAX_SCHEDULE_TIMEOUT;
		if (nonblock)
			timeout = 0;

		timeout = i915_request_wait(rq,
					    I915_WAIT_INTERRUPTIBLE,
					    timeout);
		i915_request_put(rq);

		if (timeout < 0) {
			err = nonblock ? -EWOULDBLOCK : timeout;
			goto err_exit;
		}
	}

	eb->engine = ce->engine;
	eb->context = ce;
	return 0;

err_exit:
	mutex_lock(&tl->mutex);
	intel_context_exit(ce);
	intel_context_timeline_unlock(tl);
err_unpin:
	intel_context_unpin(ce);
	return err;
}

static void eb_unpin_engine(struct i915_ttm_execbuffer *eb)
{
	struct intel_context *ce = eb->context;
	struct intel_timeline *tl = ce->timeline;

	mutex_lock(&tl->mutex);
	intel_context_exit(ce);
	mutex_unlock(&tl->mutex);

	intel_context_unpin(ce);
}

static int num_vcs_engines(const struct drm_i915_private *i915)
{
	return hweight64(INTEL_INFO(i915)->engine_mask &
			 GENMASK_ULL(VCS0 + I915_MAX_VCS - 1, VCS0));
}

/*
 * Find one BSD ring to dispatch the corresponding BSD command.
 * The engine index is returned.
 */
static unsigned int
gen8_dispatch_bsd_engine(struct drm_i915_private *dev_priv,
			 struct drm_file *file)
{
	struct drm_i915_file_private *file_priv = file->driver_priv;

	/* Check whether the file_priv has already selected one ring. */
	if ((int)file_priv->bsd_engine < 0)
		file_priv->bsd_engine =
			get_random_int() % num_vcs_engines(dev_priv);

	return file_priv->bsd_engine;
}

static const enum intel_engine_id user_ring_map[] = {
	[I915_EXEC_DEFAULT]	= RCS0,
	[I915_EXEC_RENDER]	= RCS0,
	[I915_EXEC_BLT]		= BCS0,
	[I915_EXEC_BSD]		= VCS0,
	[I915_EXEC_VEBOX]	= VECS0
};

static unsigned int
eb_select_legacy_ring(struct i915_ttm_execbuffer *eb,
		      struct drm_file *file,
		      struct drm_i915_gem_execbuffer2 *args)
{
	struct drm_i915_private *i915 = eb->i915;
	unsigned int user_ring_id = args->flags & I915_EXEC_RING_MASK;

	if (user_ring_id != I915_EXEC_BSD &&
	    (args->flags & I915_EXEC_BSD_MASK)) {
		drm_dbg(&i915->drm,
			"execbuf with non bsd ring but with invalid "
			"bsd dispatch flags: %d\n", (int)(args->flags));
		return -1;
	}

	if (user_ring_id == I915_EXEC_BSD && num_vcs_engines(i915) > 1) {
		unsigned int bsd_idx = args->flags & I915_EXEC_BSD_MASK;

		if (bsd_idx == I915_EXEC_BSD_DEFAULT) {
			bsd_idx = gen8_dispatch_bsd_engine(i915, file);
		} else if (bsd_idx >= I915_EXEC_BSD_RING1 &&
			   bsd_idx <= I915_EXEC_BSD_RING2) {
			bsd_idx >>= I915_EXEC_BSD_SHIFT;
			bsd_idx--;
		} else {
			drm_dbg(&i915->drm,
				"execbuf with unknown bsd ring: %u\n",
				bsd_idx);
			return -1;
		}

		return _VCS(bsd_idx);
	}

	if (user_ring_id >= ARRAY_SIZE(user_ring_map)) {
		drm_dbg(&i915->drm, "execbuf with unknown ring: %u\n",
			user_ring_id);
		return -1;
	}

	return user_ring_map[user_ring_id];
}

static int
eb_pin_engine(struct i915_ttm_execbuffer *eb,
	      struct drm_file *file,
	      struct drm_i915_gem_execbuffer2 *args)
{
	struct intel_context *ce;
	unsigned int idx;
	int err;

	if (i915_gem_context_user_engines(eb->gem_context))
		idx = args->flags & I915_EXEC_RING_MASK;
	else
		idx = eb_select_legacy_ring(eb, file, args);

	ce = i915_gem_context_get_engine(eb->gem_context, idx);
	if (IS_ERR(ce))
		return PTR_ERR(ce);

	err = __eb_pin_engine(eb, ce);
	intel_context_put(ce);

	return err;
}

static int eb_select_context(struct i915_ttm_execbuffer *eb)
{
	struct i915_gem_context *ctx;

	ctx = i915_gem_context_lookup(eb->file->driver_priv, eb->args->rsvd1);
	if (unlikely(!ctx))
		return -ENOENT;

	eb->gem_context = ctx;
//	if (rcu_access_pointer(ctx->vm))
//		eb->invalid_flags |= EXEC_OBJECT_NEEDS_GTT;

//	eb->context_flags = 0;
//	if (test_bit(UCONTEXT_NO_ZEROMAP, &ctx->user_flags))
//		eb->context_flags |= __EXEC_OBJECT_NEEDS_BIAS;

	return 0;
}

static int i915_ttm_cs_bo_validate(struct i915_ttm_execbuffer *eb,
				   struct i915_ttm_bo *bo)
{
	struct ttm_operation_ctx ctx = {
		.interruptible = true,
		.no_wait_gpu = false,
		.resv = bo->tbo.base.resv,
		.flags = 0
	};
	uint32_t region;
	int r;

	if (bo->pin_count)
		return 0;

	region = bo->allowed_regions;

retry:
	i915_ttm_bo_placement_from_region(bo, region);
	r = ttm_bo_validate(&bo->tbo, &bo->placement, &ctx);
	if (unlikely(r == -ENOMEM) && region != bo->allowed_regions) {
		region = bo->allowed_regions;
		goto retry;
	}
	return r;
}

static int i915_ttm_list_validate(struct i915_ttm_execbuffer *eb,
				     struct list_head *validated)
{
	struct ttm_operation_ctx ctx = { true, false };
	struct i915_ttm_bo_list_entry *lobj;
	int r;

	list_for_each_entry(lobj, validated, tv.head) {
		struct i915_ttm_bo *bo = ttm_to_i915_bo(lobj->tv.bo);

		r = i915_ttm_cs_bo_validate(eb, bo);
		if (r)
			return r;
	}
	return 0;
}

static void
i915_ttm_execbuffer_fini(struct i915_ttm_execbuffer *eb, int error,
			 bool backoff)
{
	if (error && backoff)
		ttm_eu_backoff_reservation(&eb->ticket, &eb->validated);

	if (eb->bo_list)
		i915_ttm_bo_list_put(eb->bo_list);
}

static int
eb_submit(struct i915_ttm_execbuffer *eb)
{
	int err;
	/*
	 * After we completed waiting for other engines (using HW semaphores)
	 * then we can signal that this request/batch is ready to run. This
	 * allows us to determine if the batch is still waiting on the GPU
	 * or actually running by checking the breadcrumb.
	 */
	if (eb->engine->emit_init_breadcrumb) {
		err = eb->engine->emit_init_breadcrumb(eb->request);
		if (err)
			return err;
	}

	err = eb->engine->emit_bb_start(eb->request,
//					batch->node.start +
					eb->batch_start_offset,
					eb->batch_len,
					eb->batch_flags);
	if (err)
		return err;
	if (intel_context_nopreempt(eb->context))
		__set_bit(I915_FENCE_FLAG_NOPREEMPT, &eb->request->fence.flags);

	return 0;
}

static void retire_requests(struct intel_timeline *tl, struct i915_request *end)
{
	struct i915_request *rq, *rn;

	list_for_each_entry_safe(rq, rn, &tl->requests, link)
		if (rq == end || !i915_request_retire(rq))
			break;
}

static void eb_request_add(struct i915_ttm_execbuffer *eb)
{
	struct i915_request *rq = eb->request;
	struct intel_timeline * const tl = i915_request_timeline(rq);
	struct i915_sched_attr attr = {};
	struct i915_request *prev;

	lockdep_assert_held(&tl->mutex);
	lockdep_unpin_lock(&tl->mutex, rq->cookie);

	prev = __i915_request_commit(rq);

	/* Check that the context wasn't destroyed before submission */
	if (likely(!intel_context_is_closed(eb->context))) {
		attr = eb->gem_context->sched;
	} else {
		/* Serialise with context_close via the add_to_timeline */
		i915_request_set_error_once(rq, -ENOENT);
		__i915_request_skip(rq);
	}

	__i915_request_queue(rq, &attr);

	/* Try to clean up the client's timeline after submitting the request */
	if (prev)
		retire_requests(tl, prev);

	mutex_unlock(&tl->mutex);
}

int
i915_ttm_do_execbuffer(struct drm_device *dev,
		       struct drm_file *file,
		       struct drm_i915_gem_execbuffer2 *args,
		       struct drm_i915_gem_exec_object2 *exec,
		       struct drm_syncobj **fences)
{
	struct drm_i915_private *i915 = to_i915(dev);
	struct drm_i915_gem_exec_object2 *exec2_list;
	struct i915_ttm_execbuffer eb = {};
	unsigned int i;
	int r;
	bool reserved_buffers = false;
	struct i915_ttm_bo_list_entry *e;
	eb.i915 = i915;
	eb.file = file;
	eb.args = args;

	eb.batch_start_offset = args->batch_start_offset;
	eb.batch_len = args->batch_len;
	eb.batch_flags = 0;

	r = eb_select_context(&eb);
	if (unlikely(r))
		return -EINVAL;
	
	r = eb_pin_engine(&eb, file, args);

	eb.request = i915_request_create(eb.context);
	
	r = i915_ttm_bo_list_create(i915, file, exec, args->buffer_count, &eb.bo_list);
	if (r)
		return r;

	i915_ttm_bo_list_get_list(eb.bo_list, &eb.validated);
	r = ttm_eu_reserve_buffers(&eb.ticket, &eb.validated, true, NULL);
	if (r != 0) {
		if (r != -ERESTARTSYS)
			DRM_ERROR("ttm_eu_reserve_buffers failed.\n");
		goto out;
	}

	r = i915_ttm_list_validate(&eb, &eb.validated);

	i915_ttm_bo_list_for_each_entry(e, eb.bo_list) {
		struct i915_ttm_bo *bo = ttm_to_i915_bo(e->tv.bo);

		/* find VMA */
	}

	reserved_buffers = true;
	
out:
	i915_ttm_execbuffer_fini(&eb, r, reserved_buffers);
	return r;
}
