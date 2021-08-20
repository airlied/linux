/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2021 Intel Corporation
 */

#include "xe_execlist.h"

#include "xe_device.h"
#include "xe_engine.h"
#include "xe_sched_job.h"

#define XE_EXECLIST_HANG_LIMIT 1

struct xe_execlist_port *xe_execlist_port_create(struct xe_device *xe,
						 struct xe_hw_engine *hwe)
{
	struct xe_execlist_port *port;
	int i;

	port = kzalloc(sizeof(*port), GFP_KERNEL);
	if (!port)
		return ERR_PTR(-ENOMEM);

	spin_lock_init(&port->active_lock);
	for (i = 0; i < ARRAY_SIZE(port->active); i++)
		INIT_LIST_HEAD(&port->active[i]);

	return port;
}

void xe_execlist_port_destroy(struct xe_execlist_port *port)
{
	kfree(port);
}

#if 0
static void xe_execlist_make_active(struct xe_execlist *exl)
{
	struct xe_execlist_port *port = exl->port;
	enum drm_sched_priority priority = exl->entity.priority;

	XE_BUG_ON(exl->entity.priority == DRM_SCHED_PRIORITY_UNSET);
	XE_BUG_ON(exl->entity.priority < 0);
	XE_BUG_ON(exl->entity.priority >= ARRAY_SIZE(exl->port->active));

	spin_lock(&port->active_lock);

	if (exl->active_priority != priority &&
	    exl->active_priority != DRM_SCHED_PRIORITY_UNSET) {
		/* Priority changed, move it to the right list
		 *
		 * TODO: Force a preempt?
		 */
		list_del(&exl->active_link);
		exl->active_priority = DRM_SCHED_PRIORITY_UNSET;
	}

	if (exl->active_priority == DRM_SCHED_PRIORITY_UNSET)
		list_add_tail(&exl->active_link, &port->active[priority]);

	spin_unlock(&port->active_lock);
}

static struct dma_fence *
xe_execlist_run_job(struct drm_sched_job *drm_job)
{
	struct xe_sched_job *job = to_xe_sched_job(drm_job);
	struct xe_execlist *exl = job->engine->execlist;
}
#else
static struct dma_fence *
xe_execlist_run_job(struct drm_sched_job *drm_job)
{
	return NULL;
}
#endif

static const struct drm_sched_backend_ops drm_sched_ops = {
	.dependency = xe_drm_sched_job_dependency,
	.run_job = xe_execlist_run_job,
	.free_job = xe_drm_sched_job_free,
};

struct xe_execlist *xe_execlist_create(struct xe_engine *e)
{
	struct drm_gpu_scheduler *sched;
	struct xe_execlist *exl;
	int err;

	exl = kzalloc(sizeof(*exl), GFP_KERNEL);
	if (!exl)
		return ERR_PTR(-ENOMEM);

	exl->engine = e;

	err = drm_sched_init(&exl->sched, &drm_sched_ops, U32_MAX,
			     XE_SCHED_HANG_LIMIT, XE_SCHED_JOB_TIMEOUT,
			     NULL, NULL, e->hwe->name);
	if (err)
		goto err_free;

	sched = &exl->sched;
	err = drm_sched_entity_init(&exl->entity, DRM_SCHED_PRIORITY_NORMAL,
				    &sched, 1, NULL);
	if (err)
		goto err_sched;

	exl->port = e->hwe->exl_port;
	exl->active_priority = DRM_SCHED_PRIORITY_UNSET;

	return exl;

err_sched:
	drm_sched_fini(&exl->sched);
err_free:
	kfree(exl);
	return ERR_PTR(err);
}

void xe_execlist_destroy(struct xe_execlist *exl)
{
	drm_sched_entity_fini(&exl->entity);
	drm_sched_fini(&exl->sched);
	kfree(exl);
}
