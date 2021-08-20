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
#include <drm/drm_mm.h>
#include <drm/ttm/ttm_device.h>

#include "xe_ggtt.h"
#include "xe_hw_engine.h"

#define XE_EXTRA_DEBUG 1
#define XE_WARN_ON WARN_ON
#define XE_BUG_ON BUG_ON

#define XE_BO_INVALID_OFFSET	LONG_MAX

#define GRAPHICS_VER(xe) 12
#define GRAPHICS_VERx10(xe) 120

struct xe_ttm_vram_mgr {
	struct ttm_resource_manager manager;
	struct drm_mm mm;
	spinlock_t lock;
	atomic64_t usage;
};

struct xe_ttm_gtt_mgr {
	struct ttm_resource_manager manager;
	atomic64_t used;
};

struct xe_device {
	struct drm_device drm;

	struct ttm_device ttm;
	struct xe_ttm_vram_mgr vram_mgr;
	struct xe_ttm_gtt_mgr gtt_mgr;

	bool irq_enabled;

	struct {
		void *regs;
	} mmio;

	struct {
		resource_size_t io_start;
		resource_size_t size;
		void *__iomem mapping;
	} vram;

	struct xe_ggtt ggtt;

	struct xe_hw_engine hw_engines[XE_NUM_HW_ENGINES];
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

static inline struct xe_device *ttm_to_xe_device(struct ttm_device *ttm)
{
	return container_of(ttm, struct xe_device, ttm);
}

struct xe_device *xe_device_create(struct pci_dev *pdev,
				   const struct pci_device_id *ent);
int xe_device_probe(struct xe_device *xe);
void xe_device_remove(struct xe_device *xe);
void xe_device_shutdown(struct xe_device *xe);

static inline struct xe_file *to_xe_file(const struct drm_file *file)
{
	return file->driver_priv;
}

int xe_irq_install(struct xe_device *xe);
void xe_irq_uninstall(struct xe_device *xe);

/* TTM memory managers */
int xe_ttm_vram_mgr_init(struct xe_device *xe);
void xe_ttm_vram_mgr_fini(struct xe_device *xe);

int xe_ttm_gtt_mgr_init(struct xe_device *xe, uint64_t gtt_size);
void xe_ttm_gtt_mgr_fini(struct xe_device *xe);
#endif /* _XE_DEVICE_H_ */
