/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2021 Intel Corporation
 */

#include "xe_hw_engine.h"

#include "xe_device.h"
#include "xe_execlist.h"

static const char * const xe_hw_engine_id_to_name[XE_NUM_HW_ENGINES] = {
	[XE_HW_ENGINE_RCS0] = "rcs0",
};

static const enum xe_engine_class xe_hw_engine_id_to_class[XE_NUM_HW_ENGINES] = {
	[XE_HW_ENGINE_RCS0] = XE_ENGINE_CLASS_RENDER,
};

int xe_hw_engine_init(struct xe_device *xe, struct xe_hw_engine *hwe,
		      enum xe_hw_engine_id id)
{
	hwe->name = NULL;
	hwe->class = xe_hw_engine_id_to_class[id];

	hwe->exl_port = xe_execlist_port_create(xe, hwe);
	if (IS_ERR(hwe->exl_port))
		return PTR_ERR(hwe->exl_port);

	/* Set this last because we use it's used to detect fully set up
	 * xe_hw_engines in tear-down code.
	 */
	XE_BUG_ON(!xe_hw_engine_id_to_name[id]);
	hwe->name = xe_hw_engine_id_to_name[id];

	return 0;
}

void xe_hw_engine_finish(struct xe_hw_engine *hwe)
{
	xe_execlist_port_destroy(hwe->exl_port);
	hwe->name = NULL;
}

uint32_t
xe_hw_engine_context_size(struct xe_hw_engine *hwe)
{
	switch (hwe->class) {
	case XE_ENGINE_CLASS_RENDER:
		switch (GRAPHICS_VER(xe)) {
		case 12:
		case 11:
			return 14 * SZ_4K;
		case 9:
			return 22 * SZ_4K;
		case 8:
			return 20 * SZ_4K;
		default:
			WARN(1, "Unknown GFX version: %d", GRAPHICS_VER(xe));
			return 22 * SZ_4K;
		}
	default:
		WARN(1, "Unknown engine class: %d", hwe->class);
		fallthrough;
	case XE_ENGINE_CLASS_COPY:
	case XE_ENGINE_CLASS_VIDEO_DECODE:
	case XE_ENGINE_CLASS_VIDEO_ENHANCE:
		return 2 * SZ_4K;
	}
}
