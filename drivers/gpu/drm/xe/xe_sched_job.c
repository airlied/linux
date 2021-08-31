/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2021 Intel Corporation
 */

#include "xe_sched_job.h"

#include <linux/slab.h>

#include "xe_device.h"
#include "xe_engine.h"
#include "xe_hw_engine.h"

static struct xe_sched_job *__dma_fence_to_xe_sched_job(struct dma_fence *fence)
{
	return container_of(fence, struct xe_sched_job, fence);
}

static const char *xe_sched_fence_get_driver_name(struct dma_fence *fence)
{
	struct xe_sched_job *job = __dma_fence_to_xe_sched_job(fence);

	return dev_name(job->engine->hwe->xe->drm.dev);
}

static const char *xe_sched_fence_get_timeline_name(struct dma_fence *fence)
{
	struct xe_sched_job *job = __dma_fence_to_xe_sched_job(fence);

	/* TODO: This is supposed to be a timeline name, not the HW engine */
	return job->engine->hwe->name;
}

static bool xe_sched_fence_enable_signaling(struct dma_fence *fence)
{
	struct xe_sched_job *job = __dma_fence_to_xe_sched_job(fence);

	list_add(&job->signal_link, &job->engine->hwe->signal_jobs);

	return true;
}

static bool xe_sched_fence_signaled(struct dma_fence *fence)
{
	if (test_bit(DMA_FENCE_FLAG_SIGNALED_BIT, &fence->flags))
		return true;

	return xe_sched_job_complete(__dma_fence_to_xe_sched_job(fence));
}

static void xe_sched_fence_cleanup(struct dma_fence *fence)
{
	struct xe_sched_job *job = __dma_fence_to_xe_sched_job(fence);
	unsigned long flags;

	spin_lock_irqsave(fence->lock, flags);
	if (test_bit(DMA_FENCE_FLAG_ENABLE_SIGNAL_BIT, &fence->flags) &&
	    !test_bit(DMA_FENCE_FLAG_SIGNALED_BIT, &fence->flags)) {
		list_del(&job->signal_link);
	}
	spin_unlock_irqrestore(fence->lock, flags);
}

static void xe_sched_job_destroy(struct xe_sched_job *job);

static void xe_sched_fence_release(struct dma_fence *job_fence)
{
	xe_sched_job_destroy(__dma_fence_to_xe_sched_job(job_fence));
}

static const struct dma_fence_ops sched_job_fence_ops = {
	.get_driver_name = xe_sched_fence_get_driver_name,
	.get_timeline_name = xe_sched_fence_get_timeline_name,
	.enable_signaling = xe_sched_fence_enable_signaling,
	.signaled = xe_sched_fence_signaled,
	.release = xe_sched_fence_release,
};

struct xe_sched_job *dma_fence_to_xe_sched_job(struct dma_fence *fence)
{
	if (fence->ops != &sched_job_fence_ops)
		return NULL;

	return __dma_fence_to_xe_sched_job(fence);
}

struct xe_sched_job *xe_sched_job_create(struct xe_engine *e,
					 uint64_t user_batch_addr)
{
	struct xe_sched_job *job;
	int err;

	xe_engine_assert_held(e);

	job = kzalloc(sizeof(*job), GFP_KERNEL);
	if (!job)
		return ERR_PTR(-ENOMEM);

	err = drm_sched_job_init(&job->drm, e->entity, NULL);
	if (err)
		goto err_free;

	job->engine = e;

	dma_fence_init(&job->fence, &sched_job_fence_ops, &e->hwe->fence_lock,
		       e->fence_ctx, e->next_seqno++);

	job->user_batch_addr = user_batch_addr;
	
	return job;

err_free:
	kfree(job);
	return ERR_PTR(err);
}

static void xe_sched_job_destroy(struct xe_sched_job *job)
{
	xe_sched_fence_cleanup(&job->fence);

	drm_sched_job_cleanup(&job->drm);

	/* It contains a fence so we need an RCU free */
	kfree_rcu(job, fence.rcu);
}

bool xe_sched_job_complete(struct xe_sched_job *job)
{
	uint32_t last_lrc_seqno = xe_lrc_last_seqno(&job->engine->lrc);

	return (int32_t)(job->fence.seqno - last_lrc_seqno) <= 0;
}

void xe_drm_sched_job_free(struct drm_sched_job *drm_job)
{
	xe_sched_job_put(to_xe_sched_job(drm_job));
}
