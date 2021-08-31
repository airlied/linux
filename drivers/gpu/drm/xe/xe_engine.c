/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2021 Intel Corporation
 */

#include "xe_engine.h"

#include <drm/drm_syncobj.h>
#include <drm/xe_drm.h>

#include "xe_bo.h"
#include "xe_device.h"
#include "xe_execlist.h"
#include "xe_sched_job.h"
#include "xe_vm.h"

static struct xe_engine *__xe_engine_create(struct xe_device *xe,
					    struct xe_vm *vm,
					    struct xe_hw_engine *hwe)
{
	struct xe_engine *e;
	int err;

	e = kzalloc(sizeof(*e), GFP_KERNEL);
	if (!e)
		return ERR_PTR(-ENOMEM);

	e->hwe = hwe;
	kref_init(&e->refcount);
	e->vm = xe_vm_get(vm);

	err = xe_lrc_init(&e->lrc, hwe, vm, SZ_16K);
	if (err)
		goto err_kfree;

	if (hwe->exl_port) {
		e->execlist = xe_execlist_create(e);
		if (IS_ERR(e->execlist)) {
			err = PTR_ERR(e->execlist);
			goto err_lrc;
		}
		e->entity = &e->execlist->entity;
	}

	e->fence_ctx = dma_fence_context_alloc(1);
	e->next_seqno = 1;

	return e;

err_lrc:
	xe_lrc_finish(&e->lrc);
err_kfree:
	kfree(e);
	return ERR_PTR(err);
}

struct xe_engine *xe_engine_create(struct xe_device *xe, struct xe_vm *vm,
				   struct xe_hw_engine *hwe)
{
	struct xe_engine *e;

	xe_vm_lock(vm, NULL);
	e = __xe_engine_create(xe, vm, hwe);
	xe_vm_unlock(vm);

	return e;
}

void xe_engine_free(struct kref *ref)
{
	struct xe_engine *e = container_of(ref, struct xe_engine, refcount);

	if (e->execlist)
		xe_execlist_destroy(e->execlist);

	xe_lrc_finish(&e->lrc);
	xe_vm_put(e->vm);

	kfree(e);
}

struct xe_engine *xe_engine_lookup(struct xe_file *xef, u32 id)
{
	struct xe_engine *e;

	mutex_lock(&xef->engine_lock);
	e = xa_load(&xef->engine_xa, id);
	mutex_unlock(&xef->engine_lock);

	if (e)
		xe_engine_get(e);

	return e;
}

static const enum xe_engine_class user_to_xe_engine_class[] = {
	[DRM_XE_ENGINE_CLASS_RENDER] = XE_ENGINE_CLASS_RENDER,
	[DRM_XE_ENGINE_CLASS_COPY] = XE_ENGINE_CLASS_COPY,
};

static struct xe_hw_engine *
find_hw_engine(struct xe_device *xe,
	       struct drm_xe_engine_class_instance eci)
{
	if (eci.engine_class > ARRAY_SIZE(user_to_xe_engine_class))
		return NULL;

	return xe_device_hw_engine(xe,
		user_to_xe_engine_class[eci.engine_class], eci.engine_instance);
}

int xe_engine_create_ioctl(struct drm_device *dev, void *data,
			   struct drm_file *file)
{
	struct xe_device *xe = to_xe_device(dev);
	struct xe_file *xef = to_xe_file(file);
	struct drm_xe_engine_create *args = data;
	struct xe_hw_engine *hwe;
	struct xe_vm *vm;
	struct xe_engine *e;
	u32 id;
	int err;

	if (XE_IOCTL_ERR(xe, args->extensions))
		return -EINVAL;

	if (XE_IOCTL_ERR(xe, args->flags))
		return -EINVAL;

	hwe = find_hw_engine(xe, args->instance);
	if (XE_IOCTL_ERR(xe, !hwe))
		return -EINVAL;

	vm = xe_vm_lookup(xef, args->vm_id);
	if (XE_IOCTL_ERR(xe, !vm))
		return -ENOENT;

	e = xe_engine_create(xe, vm, hwe);
	xe_vm_put(vm);
	if (IS_ERR(e))
		return PTR_ERR(e);

	mutex_lock(&xef->engine_lock);
	err = xa_alloc(&xef->engine_xa, &id, e, xa_limit_32b, GFP_KERNEL);
	mutex_unlock(&xef->engine_lock);
	if (err) {
		xe_engine_put(e);
		return err;
	}

	args->engine_id = id;

	return 0;
}

int xe_engine_destroy_ioctl(struct drm_device *dev, void *data,
			    struct drm_file *file)
{
	struct xe_device *xe = to_xe_device(dev);
	struct xe_file *xef = to_xe_file(file);
	struct drm_xe_engine_destroy *args = data;
	struct xe_engine *e;

	if (XE_IOCTL_ERR(xe, args->pad))
		return -EINVAL;

	mutex_lock(&xef->engine_lock);
	e = xa_erase(&xef->engine_xa, args->engine_id);
	mutex_unlock(&xef->engine_lock);
	if (XE_IOCTL_ERR(xe, !e))
		return -ENOENT;

	xe_engine_put(e);

	return 0;
}

struct sync {
	struct drm_syncobj *syncobj;
	struct dma_fence *fence;
	struct dma_fence_chain *chain_fence;
	uint64_t timeline_value;
	uint32_t flags;
};

#define SYNC_FLAGS_TYPE_MASK 0x3

static int parse_sync(struct xe_device *xe, struct xe_file *xef,
		      struct sync *sync, struct drm_xe_sync __user *sync_user)
{
	struct drm_xe_sync sync_in;
	int err;

	if (__copy_from_user(&sync_in, sync_user, sizeof(*sync_user)))
		return -EFAULT;

	memset(sync, 0, sizeof(*sync));

	switch (sync_in.flags & SYNC_FLAGS_TYPE_MASK) {
	case DRM_XE_SYNC_SYNCOBJ:
		sync->syncobj = drm_syncobj_find(xef->drm, sync_in.handle);
		if (XE_IOCTL_ERR(xe, !sync->syncobj))
			return -ENOENT;

		if (!(sync_in.flags & DRM_XE_SYNC_SIGNAL)) {
			sync->fence = drm_syncobj_fence_get(sync->syncobj);
			if (XE_IOCTL_ERR(xe, !sync->fence))
				return -EINVAL;
		}
		break;

	case DRM_XE_SYNC_TIMELINE_SYNCOBJ:
		if (XE_IOCTL_ERR(xe, sync_in.timeline_value == 0))
			return -EINVAL;

		sync->syncobj = drm_syncobj_find(xef->drm, sync_in.handle);
		if (XE_IOCTL_ERR(xe, !sync->syncobj))
			return -ENOENT;

		if (sync_in.flags & DRM_XE_SYNC_SIGNAL) {
			sync->chain_fence = dma_fence_chain_alloc();
			if (!sync->chain_fence)
				return -ENOMEM;
		} else {
			sync->fence = drm_syncobj_fence_get(sync->syncobj);
			if (XE_IOCTL_ERR(xe, !sync->fence))
				return -EINVAL;

			err = dma_fence_chain_find_seqno(&sync->fence,
							 sync_in.timeline_value);
			if (err)
				return err;
		}
		break;

	case DRM_XE_SYNC_DMA_BUF:
		if (XE_IOCTL_ERR(xe, "TODO"))
			return -EINVAL;
		break;

	default:
		return -EINVAL;
	}

	sync->flags = sync_in.flags;
	sync->timeline_value = sync_in.timeline_value;

	return 0;
}

static int add_sync_deps(struct sync *sync, struct xe_sched_job *job)
{
	if (sync->fence)
		xe_sched_job_add_dependency(job, sync->fence);

	return 0;
}

static void signal_sync(struct sync *sync, struct dma_fence *fence)
{
	if (!(sync->flags & DRM_XE_SYNC_SIGNAL))
		return;

	if (sync->chain_fence) {
		drm_syncobj_add_point(sync->syncobj, sync->chain_fence,
				      fence, sync->timeline_value);
		/*
		 * The chain's ownership is transferred to the
		 * timeline.
		 */
		sync->chain_fence = NULL;
	} else if (sync->syncobj) {
		drm_syncobj_replace_fence(sync->syncobj, fence);
	}

	/* TODO: BO */
}

static void put_sync(struct sync *sync)
{
	if (sync->syncobj)
		drm_syncobj_put(sync->syncobj);
	if (sync->fence)
		dma_fence_put(sync->fence);
	if (sync->chain_fence)
		dma_fence_put(&sync->chain_fence->base);
}

int xe_exec_ioctl(struct drm_device *dev, void *data, struct drm_file *file)
{
	struct xe_device *xe = to_xe_device(dev);
	struct xe_file *xef = to_xe_file(file);
	struct drm_xe_exec *args = data;
	struct drm_xe_sync __user *syncs_user = u64_to_user_ptr(args->syncs);
	struct xe_engine *engine;
	struct sync *syncs;
	uint32_t i, num_syncs = 0;
	struct xe_sched_job *job;
	int err = 0;

	if (XE_IOCTL_ERR(xe, args->extensions))
		return -EINVAL;

	engine = xe_engine_lookup(xef, args->engine_id);
	if (XE_IOCTL_ERR(xe, !engine))
		return -ENOENT;

	syncs = kcalloc(args->num_syncs, sizeof(*syncs), GFP_KERNEL);
	if (!syncs) {
		err = -ENOMEM;
		goto err_engine;
	}

	for (i = 0; i < args->num_syncs; i++) {
		err = parse_sync(xe, xef, &syncs[num_syncs++], &syncs_user[i]);
		if (err)
			goto err_syncs;
	}

	xe_vm_lock(engine->vm, NULL);

	job = xe_sched_job_create(engine, args->address);
	if (IS_ERR(job)) {
		err = PTR_ERR(job);
		goto err_unlock_vm;
	}

	for (i = 0; i < num_syncs; i++) {
		err = add_sync_deps(&syncs[i], job);
		if (err)
			goto err_put_job;
	}

	drm_sched_entity_push_job(&job->drm);

	for (i = 0; i < num_syncs; i++)
		signal_sync(&syncs[i], &job->fence);

err_put_job:
	if (err)
		xe_sched_job_put(job);
err_unlock_vm:
	xe_vm_unlock(engine->vm);
err_syncs:
	for (i = 0; i < num_syncs; i++)
		put_sync(&syncs[i]);
	kfree(syncs);
err_engine:
	xe_engine_put(engine);

	return err;
}
