/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2021 Intel Corporation
 */

#ifndef _XE_SCHED_JOB_H_
#define _XE_SCHED_JOB_H_

#include <linux/xarray.h>

#include <drm/gpu_scheduler.h>

#define XE_SCHED_HANG_LIMIT 1
#define XE_SCHED_JOB_TIMEOUT LONG_MAX

struct xe_engine;

struct xe_sched_job {
	struct drm_sched_job drm;

	struct xe_engine *engine;

	struct xarray deps;
	unsigned long first_dep;
};

static inline struct xe_sched_job *
to_xe_sched_job(struct drm_sched_job *drm)
{
	return container_of(drm, struct xe_sched_job, drm);
}

struct xe_sched_job *xe_sched_job_create(struct xe_engine *e);
void xe_sched_job_destroy(struct xe_sched_job *);

struct dma_fence *xe_drm_sched_job_dependency(struct drm_sched_job *drm_job,
					      struct drm_sched_entity *entity);
void xe_drm_sched_job_free(struct drm_sched_job *drm_job);

#endif /* _XE_SCHED_JOB_H_ */
