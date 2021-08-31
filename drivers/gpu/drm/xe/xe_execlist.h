/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2021 Intel Corporation
 */

#ifndef _XE_EXECLIST_H_
#define _XE_EXECLIST_H_

#include <linux/list.h>
#include <linux/spinlock.h>

#include <drm/gpu_scheduler.h>

#include "xe_bo.h"
#include "xe_hw_engine.h"

struct xe_execlist;

#define xe_execlist_port_assert_held(port) lockdep_assert_held(&(port)->lock);

struct xe_execlist_port {
	struct xe_hw_engine *hwe;

	spinlock_t lock;

	struct list_head active[DRM_SCHED_PRIORITY_COUNT];

	uint32_t last_ctx_id;

	struct xe_execlist *running_exl;
};

struct xe_execlist_port *xe_execlist_port_create(struct xe_device *xe,
						 struct xe_hw_engine *hwe);
void xe_execlist_port_destroy(struct xe_execlist_port *port);

struct xe_execlist {
	struct xe_engine *engine;

	struct drm_gpu_scheduler sched;

	struct drm_sched_entity entity;

	struct xe_execlist_port *port;

	bool has_run;

	enum drm_sched_priority active_priority;
	struct list_head active_link;
};

struct xe_execlist *xe_execlist_create(struct xe_engine *e);
void xe_execlist_destroy(struct xe_execlist *exl);

#endif /* _XE_EXECLIST_H_ */
