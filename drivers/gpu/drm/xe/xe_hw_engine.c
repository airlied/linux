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

int xe_hw_engine_init(struct xe_device *xe, struct xe_hw_engine *hwe,
		      enum xe_hw_engine_id id)
{
	hwe->name = NULL;

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
