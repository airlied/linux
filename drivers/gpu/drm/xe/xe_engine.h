/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2021 Intel Corporation
 */

#ifndef _XE_ENGINE_H_
#define _XE_ENGINE_H_

#include <linux/kref.h>

#include <drm/drm_device.h>
#include <drm/drm_file.h>
#include <drm/gpu_scheduler.h>

#include "xe_lrc.h"

struct xe_device;
struct xe_execlist;
struct xe_file;

struct xe_engine {
	struct xe_hw_engine *hwe;

	struct kref refcount;

	struct xe_vm *vm;

	struct xe_execlist *execlist;

	struct drm_sched_entity *entity;

	struct xe_lrc lrc;

	uint64_t fence_ctx;
	uint32_t next_seqno;
};

struct xe_engine *xe_engine_create(struct xe_device *xe, struct xe_vm *vm,
				   struct xe_hw_engine *hw_engine);
void xe_engine_free(struct kref *ref);

struct xe_engine *xe_engine_lookup(struct xe_file *xef, u32 id);

static inline struct xe_engine *xe_engine_get(struct xe_engine *engine)
{
	kref_get(&engine->refcount);
	return engine;
}

static inline void xe_engine_put(struct xe_engine *engine)
{
	kref_put(&engine->refcount, xe_engine_free);
}

#define xe_engine_assert_held(e) xe_vm_assert_held((e)->vm)

int xe_engine_create_ioctl(struct drm_device *dev, void *data,
			   struct drm_file *file);
int xe_engine_destroy_ioctl(struct drm_device *dev, void *data,
			    struct drm_file *file);

#endif /* _XE_ENGINE_H_ */
