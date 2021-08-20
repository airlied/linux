/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2021 Intel Corporation
 */

#ifndef _XE_HW_ENGINE_H_
#define _XE_HW_ENGINE_H_

#include <linux/types.h>

enum xe_engine_class {
	XE_ENGINE_CLASS_RENDER,
	XE_ENGINE_CLASS_COPY,
	XE_ENGINE_CLASS_VIDEO_DECODE,
	XE_ENGINE_CLASS_VIDEO_ENHANCE,
};

enum xe_hw_engine_id {
	XE_HW_ENGINE_RCS0,
	XE_NUM_HW_ENGINES,
};

struct xe_device;
struct xe_execlist_port;

struct xe_hw_engine {
	const char *name;
	enum xe_engine_class class;
	struct xe_execlist_port *exl_port;
};

int xe_hw_engine_init(struct xe_device *xe, struct xe_hw_engine *hwe,
		      enum xe_hw_engine_id id);
void xe_hw_engine_finish(struct xe_hw_engine *hwe);

uint32_t xe_hw_engine_context_size(struct xe_hw_engine *hwe);

#endif /* _XE_ENGINE_H_ */
