/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2021 Intel Corporation
 */

#include "xe_sched_job.h"

#include <linux/slab.h>

#include "xe_engine.h"

struct xe_sched_job *
xe_sched_job_create(struct xe_engine *e)
{
	struct xe_sched_job *job;
	int err;

	job = kzalloc(sizeof(*job), GFP_KERNEL);
	if (!job)
		return ERR_PTR(-ENOMEM);

	err = drm_sched_job_init(&job->drm, e->entity, NULL);
	if (err)
		goto err_free;

	xa_init_flags(&job->deps, XA_FLAGS_ALLOC);
	job->engine = e;
	job->first_dep = 0;
	
	return job;

err_free:
	kfree(job);
	return ERR_PTR(err);
}

void xe_sched_job_destroy(struct xe_sched_job *job)
{
	struct dma_fence *f;
	unsigned long i;

	xa_for_each_start(&job->deps, i, f, job->first_dep)
		dma_fence_put(f);
	xa_destroy(&job->deps);

	drm_sched_job_cleanup(&job->drm);
	kfree(job);
}

struct dma_fence *xe_drm_sched_job_dependency(struct drm_sched_job *drm_job,
					      struct drm_sched_entity *entity)
{
	struct xe_sched_job *job = to_xe_sched_job(drm_job);

	if (!xa_empty(&job->deps))
		return xa_erase(&job->deps, job->first_dep++);

	return NULL;
}

void xe_drm_sched_job_free(struct drm_sched_job *drm_job)
{
	xe_sched_job_destroy(to_xe_sched_job(drm_job));
}
