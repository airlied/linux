/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2021 Intel Corporation
 */

#ifndef _XE_DEVICE_H_
#define _XE_DEVICE_H_

#include <linux/pci.h>

#include <drm/drm_device.h>

struct xe_device {
	struct drm_device drm;
};

struct xe_device *xe_device_create(struct pci_dev *pdev,
				   const struct pci_device_id *ent);
void xe_device_remove(struct xe_device *xe);
void xe_device_shutdown(struct xe_device *xe);

#endif /* _XE_DEVICE_H_ */
