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

struct xe_file {
};

static inline struct xe_device *to_xe_device(const struct drm_device *dev)
{
	return container_of(dev, struct xe_device, drm);
}

static inline struct xe_device *pdev_to_xe_device(struct pci_dev *pdev)
{
	return pci_get_drvdata(pdev);
}

struct xe_device *xe_device_create(struct pci_dev *pdev,
				   const struct pci_device_id *ent);
void xe_device_remove(struct xe_device *xe);
void xe_device_shutdown(struct xe_device *xe);

#endif /* _XE_DEVICE_H_ */
