/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2021 Intel Corporation
 */
#ifndef _XE_LRC_H_
#define _XE_LRC_H_

#include "xe_bo.h"
#include "xe_device.h"

struct xe_hw_engine;
struct xe_vm;

struct xe_lrc {
	struct xe_bo *bo;
	struct ttm_bo_kmap_obj kmap;
};

int xe_lrc_init(struct xe_lrc *lrc, struct xe_hw_engine *hwe, struct xe_vm *vm);
void xe_lrc_finish(struct xe_lrc *lrc);
int xe_lrc_map(struct xe_lrc *lrc);
void xe_lrc_unmap(struct xe_lrc *lrc);

static inline uint32_t *xe_lrc_pphwsp(struct xe_lrc *lrc)
{
	XE_BUG_ON(!lrc->kmap.virtual);
	return lrc->kmap.virtual;
}

static inline uint32_t *xe_lrc_regs(struct xe_lrc *lrc)
{
	XE_BUG_ON(!lrc->kmap.virtual);
	return lrc->kmap.virtual + SZ_4K;
}

#endif /* _XE_LRC_H_ */
