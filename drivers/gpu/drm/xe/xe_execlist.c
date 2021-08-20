/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2021 Intel Corporation
 */

#include "xe_execlist.h"

#include "xe_bo.h"
#include "xe_device.h"
#include "xe_engine.h"
#include "xe_sched_job.h"

#include "../i915/i915_reg.h"
#include "../i915/gt/intel_lrc_reg.h"

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

//static void xe_execlist_port_wake_locked(struct xe_execlist_port *port)
//{
//}

static int xe_ring_init(struct xe_ring *r, struct xe_device *xe,
			struct xe_vm *vm, struct xe_lrc *lrc, uint32_t size)
{
	uint32_t *lrc_regs;
	int err;

	r->lrc = lrc;
	r->size = size;
	r->tail = 0;
	r->bo = xe_bo_create(xe, vm, size, ttm_bo_type_kernel,
			     XE_BO_CREATE_SYSTEM_BIT | XE_BO_CREATE_GGTT_BIT);
	if (IS_ERR(r->bo))
		return PTR_ERR(r->bo);

	XE_BUG_ON(size % PAGE_SIZE);
	err = ttm_bo_kmap(&r->bo->ttm, 0, size / PAGE_SIZE, &r->kmap);
	if (err)
		goto err_bo;

	lrc_regs = xe_lrc_regs(lrc);
	lrc_regs[CTX_RING_START] = xe_bo_ggtt_addr(r->bo);
	lrc_regs[CTX_RING_HEAD] = 0;
	lrc_regs[CTX_RING_TAIL] = r->tail;
	lrc_regs[CTX_RING_CTL] = RING_CTL_SIZE(r->size) | RING_VALID;

	return 0;

err_bo:
	xe_bo_put(r->bo);
	return err;
}

static void xe_ring_finish(struct xe_ring *r)
{
	ttm_bo_kunmap(&r->kmap);
	xe_bo_put(r->bo);
}

static uint32_t xe_ring_head(struct xe_ring *r)
{
	return xe_lrc_regs(r->lrc)[CTX_RING_HEAD];
}

static uint32_t xe_ring_space(struct xe_ring *r)
{
	return (r->tail - xe_ring_head(r)) & (r->size - 1);
}

static void xe_ring_write(struct xe_ring *r, const void *data, size_t size)
{
	bool is_iomem;
	void *map;
	size_t cpy_size;

	XE_BUG_ON(size > r->size);
	XE_WARN_ON(size > xe_ring_space(r));

	map = ttm_kmap_obj_virtual(&r->kmap, &is_iomem);
	WARN_ON_ONCE(is_iomem);

	XE_BUG_ON(r->tail >= r->size);

	cpy_size = min_t(size_t, size, r->size - r->tail);
	memcpy(map + r->tail, data, cpy_size);
	if (cpy_size < size)
		memcpy(map, data + cpy_size, size - cpy_size);

	r->tail = (r->tail + size) & (r->size - 1);
	xe_lrc_regs(r->lrc)[CTX_RING_TAIL] = r->tail;
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

	xe_execlist_port_wake_locked(exl->port);

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
	struct xe_device *xe = e->hwe->xe;
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

	err = xe_ring_init(&exl->ring, xe, e->vm, &e->lrc, SZ_16K);
	if (err)
		goto err_entity;

	exl->active_priority = DRM_SCHED_PRIORITY_UNSET;

	return exl;

err_entity:
	drm_sched_entity_fini(&exl->entity);
err_sched:
	drm_sched_fini(&exl->sched);
err_free:
	kfree(exl);
	return ERR_PTR(err);
}

void xe_execlist_destroy(struct xe_execlist *exl)
{
	xe_ring_finish(&exl->ring);
	drm_sched_entity_fini(&exl->entity);
	drm_sched_fini(&exl->sched);
	kfree(exl);
}
