/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2021 Intel Corporation
 */
#ifndef _XE_LRC_H_
#define _XE_LRC_H_

#include <drm/ttm/ttm_bo_api.h>

struct xe_bo;
struct xe_hw_engine;
struct xe_vm;

struct xe_lrc {
	struct xe_bo *bo;
	struct ttm_bo_kmap_obj kmap;

	uint32_t ring_size;
	uint32_t ring_tail;

	uint64_t desc;
};

int xe_lrc_init(struct xe_lrc *lrc, struct xe_hw_engine *hwe,
		struct xe_vm *vm, uint32_t ring_size);
void xe_lrc_finish(struct xe_lrc *lrc);

int xe_lrc_map(struct xe_lrc *lrc);
void xe_lrc_unmap(struct xe_lrc *lrc);

uint32_t xe_lrc_ring_head(struct xe_lrc *lrc);
uint32_t xe_lrc_ring_space(struct xe_lrc *lrc);
void xe_lrc_write_ring(struct xe_lrc *lrc, const void *data, size_t size);

uint32_t xe_lrc_last_seqno(struct xe_lrc *lrc);
uint32_t xe_lrc_seqno_ggtt_addr(struct xe_lrc *lrc);

uint32_t xe_lrc_ggtt_addr(struct xe_lrc *lrc);
void *xe_lrc_pphwsp(struct xe_lrc *lrc);
uint32_t *xe_lrc_regs(struct xe_lrc *lrc);

uint64_t xe_lrc_descriptor(struct xe_lrc *lrc);

#endif /* _XE_LRC_H_ */
