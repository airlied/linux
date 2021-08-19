/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2021 Intel Corporation
 */

#include "xe_drv.h"
#include "xe_device.h"
#include "xe_mmio.h"

#include "i915_reg.h"

static inline uint32_t dg1_intr_disable(struct xe_device *xe)
{
	uint32_t val;

	/* First disable interrupts */
	xe_mmio_write32(xe, DG1_MSTR_TILE_INTR.reg, 0);

	/* Get the indication levels and ack the master unit */
	val = xe_mmio_read32(xe, DG1_MSTR_TILE_INTR.reg);
	if (unlikely(!val))
		return 0;

	xe_mmio_write32(xe, DG1_MSTR_TILE_INTR.reg, val);

	return val;
}

void gen11_gt_irq_reset(struct xe_device *xe)
{
	/* Disable RCS, BCS, VCS and VECS class engines. */
	xe_mmio_write32(xe, GEN11_RENDER_COPY_INTR_ENABLE.reg, 0);
	xe_mmio_write32(xe, GEN11_VCS_VECS_INTR_ENABLE.reg,	  0);

	/* Restore masks irqs on RCS, BCS, VCS and VECS engines. */
	xe_mmio_write32(xe, GEN11_RCS0_RSVD_INTR_MASK.reg,	~0);
	xe_mmio_write32(xe, GEN11_BCS_RSVD_INTR_MASK.reg,	~0);
	xe_mmio_write32(xe, GEN11_VCS0_VCS1_INTR_MASK.reg,	~0);
	xe_mmio_write32(xe, GEN11_VCS2_VCS3_INTR_MASK.reg,	~0);
//	if (HAS_ENGINE(gt, VCS4) || HAS_ENGINE(gt, VCS5))
//		xe_mmio_write32(xe, GEN12_VCS4_VCS5_INTR_MASK.reg,   ~0);
//	if (HAS_ENGINE(gt, VCS6) || HAS_ENGINE(gt, VCS7))
//		xe_mmio_write32(xe, GEN12_VCS6_VCS7_INTR_MASK.reg,   ~0);
	xe_mmio_write32(xe, GEN11_VECS0_VECS1_INTR_MASK.reg,	~0);
//	if (HAS_ENGINE(gt, VECS2) || HAS_ENGINE(gt, VECS3))
//		xe_mmio_write32(xe, GEN12_VECS2_VECS3_INTR_MASK.reg, ~0);

	xe_mmio_write32(xe, GEN11_GPM_WGBOXPERF_INTR_ENABLE.reg, 0);
	xe_mmio_write32(xe, GEN11_GPM_WGBOXPERF_INTR_MASK.reg,  ~0);
	xe_mmio_write32(xe, GEN11_GUC_SG_INTR_ENABLE.reg, 0);
	xe_mmio_write32(xe, GEN11_GUC_SG_INTR_MASK.reg,  ~0);
}

static void xe_irq_reset(struct xe_device *xe)
{
	dg1_intr_disable(xe);

	gen11_gt_irq_reset(xe);

	xe_mmio_write32(xe, GEN11_GU_MISC_IMR.reg, 0xffffffff);
	xe_mmio_read32(xe, GEN11_GU_MISC_IMR.reg);
	xe_mmio_write32(xe, GEN11_GU_MISC_IER.reg, 0);
	xe_mmio_write32(xe, GEN11_GU_MISC_IIR.reg, 0xffffffff);
	xe_mmio_read32(xe, GEN11_GU_MISC_IIR.reg);
	xe_mmio_write32(xe, GEN11_GU_MISC_IIR.reg, 0xffffffff);
	xe_mmio_read32(xe, GEN11_GU_MISC_IIR.reg);

	xe_mmio_write32(xe, GEN8_PCU_IMR.reg, 0xffffffff);
	xe_mmio_read32(xe, GEN8_PCU_IMR.reg);
	xe_mmio_write32(xe, GEN8_PCU_IER.reg, 0);
	xe_mmio_write32(xe, GEN8_PCU_IIR.reg, 0xffffffff);
	xe_mmio_read32(xe, GEN8_PCU_IIR.reg);
	xe_mmio_write32(xe, GEN8_PCU_IIR.reg, 0xffffffff);
	xe_mmio_read32(xe, GEN8_PCU_IIR.reg);

	/* TODO */
}

static void xe_irq_postinstall(struct xe_device *xe)
{
	/* TODO */
}

static irqreturn_t xe_irq_handler(int irq, void *arg)
{
	return IRQ_HANDLED;
}

int xe_irq_install(struct xe_device *xe)
{
	int irq = to_pci_dev(xe->drm.dev)->irq;
	int err;

	xe_irq_reset(xe);

	xe->irq_enabled = true;
	err = request_irq(irq, xe_irq_handler,
			  IRQF_SHARED, DRIVER_NAME, xe);
	if (err < 0) {
		xe->irq_enabled = false;
		return err;
	}

	xe_irq_postinstall(xe);

	return err;
}

void xe_irq_uninstall(struct xe_device *xe)
{
	int irq = to_pci_dev(xe->drm.dev)->irq;

	if (!xe->irq_enabled)
		return;

	xe->irq_enabled = false;

	xe_irq_reset(xe);

	free_irq(irq, xe);
}
