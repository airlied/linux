/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2021 Intel Corporation
 */

#ifndef _XE_DEVICE_H_
#define _XE_DEVICE_H_

#include <linux/pci.h>

#include <drm/drm_device.h>
#include <drm/drm_file.h>
#include <drm/ttm/ttm_device.h>

#define XE_EXTRA_DEBUG 1
#define XE_WARN_ON WARN_ON
#define XE_BUG_ON BUG_ON

struct xe_device {
	struct drm_device drm;

	struct ttm_device ttm;
};

struct xe_file {
	struct xarray vm_xa;
	struct mutex vm_lock;

	struct xarray engine_xa;
	struct mutex engine_lock;
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

static inline struct xe_file *to_xe_file(const struct drm_file *file)
{
	return file->driver_priv;
}

#endif /* _XE_DEVICE_H_ */
