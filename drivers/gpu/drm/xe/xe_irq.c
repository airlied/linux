/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2021 Intel Corporation
 */

#include "xe_drv.h"
#include "xe_device.h"

static void xe_irq_reset(struct xe_device *xe)
{
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
	int irq = xe->pdev->irq;
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
	int irq = xe->pdev->irq;

	if (!xe->irq_enabled)
		return;

	xe->irq_enabled = false;

	xe_irq_reset(xe);
	
	free_irq(irq, xe);
}
