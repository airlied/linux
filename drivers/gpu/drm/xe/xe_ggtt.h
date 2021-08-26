/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2021 Intel Corporation
 */

#ifndef _XE_GGTT_H_
#define _XE_GGTT_H_

#include <drm/drm_mm.h>

struct xe_bo;
struct xe_device;

struct xe_ggtt {
	uint64_t size;

	struct xe_bo *scratch;

	struct mutex lock;

	uint64_t __iomem *gsm;

	struct drm_mm mm;
};

int xe_ggtt_init(struct xe_device *xe, struct xe_ggtt *ggtt);
void xe_ggtt_finish(struct xe_ggtt *ggtt);

int xe_ggtt_insert_bo(struct xe_ggtt *ggtt, struct xe_bo *bo);
void xe_ggtt_remove_bo(struct xe_ggtt *ggtt, struct xe_bo *bo);

#endif /* _XE_GGTT_H_ */
