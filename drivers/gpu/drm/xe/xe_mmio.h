/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2021 Intel Corporation
 */

#ifndef _XE_MMIO_H_
#define _XE_MMIO_H_

#include "xe_device.h"

int xe_mmio_init(struct xe_device *xe);
void xe_mmio_finish(struct xe_device *xe);

static inline void xe_mmio_write32(struct xe_device *xe,
				   uint32_t reg, uint32_t val)
{
	writel(val, xe->mmio.regs + reg);
}

static inline uint32_t xe_mmio_read32(struct xe_device *xe, uint32_t reg)
{
	return readl(xe->mmio.regs + reg);
}

static inline uint64_t xe_mmio_read64(struct xe_device *xe, uint32_t reg)
{
	return readq(xe->mmio.regs + reg);
}


#endif /* _XE_MMIO_H_ */
