/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2021 Intel Corporation
 */

#ifndef _XE_HW_ENGINE_H_
#define _XE_HW_ENGINE_H_

#include <linux/list.h>
#include <linux/spinlock.h>

#include "xe_lrc.h"

/* See "Engine ID Definition" struct in the Icelake PRM */
enum xe_engine_class {
	XE_ENGINE_CLASS_RENDER = 0,
	XE_ENGINE_CLASS_VIDEO_DECODE = 1,
	XE_ENGINE_CLASS_VIDEO_ENHANCE = 2,
	XE_ENGINE_CLASS_COPY = 3,
	XE_ENGINE_CLASS_OTHER = 4,
	XE_ENGINE_CLASS_COMPUTE = 5,
};

enum xe_hw_engine_id {
	XE_HW_ENGINE_RCS0,
	XE_HW_ENGINE_BCS0,
	XE_HW_ENGINE_VCS0,
	XE_HW_ENGINE_VCS1,
	XE_HW_ENGINE_VCS2,
	XE_HW_ENGINE_VCS3,
	XE_HW_ENGINE_VCS4,
	XE_HW_ENGINE_VCS5,
	XE_HW_ENGINE_VCS6,
	XE_HW_ENGINE_VCS7,
	XE_HW_ENGINE_VECS0,
	XE_HW_ENGINE_VECS1,
	XE_HW_ENGINE_VECS2,
	XE_HW_ENGINE_VECS3,
	XE_NUM_HW_ENGINES,
};

struct xe_device;
struct xe_execlist_port;

struct xe_hw_engine {
	struct xe_device *xe;

	const char *name;
	enum xe_engine_class class;
	uint16_t instance;
	uint32_t mmio_base;

	struct xe_bo *hwsp;

	struct xe_lrc kernel_lrc;

	struct xe_execlist_port *exl_port;

	spinlock_t fence_lock;
	struct list_head signal_jobs;

	void (*irq_handler)(struct xe_hw_engine *, uint16_t);
};

int xe_hw_engine_init(struct xe_device *xe, struct xe_hw_engine *hwe,
		      enum xe_hw_engine_id id);
void xe_hw_engine_finish(struct xe_hw_engine *hwe);

static inline bool xe_hw_engine_is_valid(struct xe_hw_engine *hwe)
{
	return hwe->name;
}

void xe_hw_engine_handle_irq(struct xe_hw_engine *hwe, uint16_t intr_vec);

#endif /* _XE_ENGINE_H_ */
