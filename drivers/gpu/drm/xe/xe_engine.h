/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2021 Intel Corporation
 */

#ifndef _XE_ENGINE_H_
#define _XE_ENGINE_H_

#include <linux/kref.h>

#include "xe_device.h"
#include "xe_vm.h"

struct xe_engine {
	struct kref refcount;

	struct xe_vm *vm;
};

struct xe_engine *xe_engine_create(struct xe_device *xe, struct xe_vm *vm);
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

int xe_engine_create_ioctl(struct drm_device *dev, void *data,
			   struct drm_file *file);
int xe_engine_destroy_ioctl(struct drm_device *dev, void *data,
			    struct drm_file *file);

#endif /* _XE_ENGINE_H_ */
