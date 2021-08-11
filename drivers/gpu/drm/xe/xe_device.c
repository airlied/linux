/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2021 Intel Corporation
 */

#include <drm/drm_file.h>
#include <drm/drm_ioctl.h>

#include "xe_bo.h"
#include "xe_device.h"
#include "xe_drv.h"

static int xe_file_open(struct drm_device *dev, struct drm_file *file)
{
	struct xe_file *xef;

	xef = kzalloc(sizeof(*xef), GFP_KERNEL);
	if (!xef)
		return -ENOMEM;

	file->driver_priv = xef;
	return 0;
}

static void xe_file_close(struct drm_device *dev, struct drm_file *file)
{
	struct xe_file *xef = file->driver_priv;

	kfree(xef);
}

static const struct drm_ioctl_desc xe_ioctls[] = {
};

static const struct file_operations xe_driver_fops = {
	.owner = THIS_MODULE,
	.open = drm_open,
	.release = drm_release_noglobal,
	.unlocked_ioctl = drm_ioctl,
//	.mmap = i915_gem_mmap,
	.poll = drm_poll,
	.read = drm_read,
//	.compat_ioctl = i915_ioc32_compat_ioctl,
	.llseek = noop_llseek,
};

static const struct drm_driver driver = {
	/* Don't use MTRRs here; the Xserver or userspace app should
	 * deal with them for Intel hardware.
	 */
	.driver_features =
	    DRIVER_GEM |
	    DRIVER_RENDER | DRIVER_MODESET | DRIVER_ATOMIC | DRIVER_SYNCOBJ |
	    DRIVER_SYNCOBJ_TIMELINE,
	.open = xe_file_open,
	.postclose = xe_file_close,

	.prime_handle_to_fd = drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle = drm_gem_prime_fd_to_handle,
//	.gem_prime_import = i915_gem_prime_import,
//
//	.dumb_create = i915_gem_dumb_create,
//	.dumb_map_offset = i915_gem_dumb_mmap_offset,

	.ioctls = xe_ioctls,
	.num_ioctls = ARRAY_SIZE(xe_ioctls),
	.fops = &xe_driver_fops,
	.name = DRIVER_NAME,
	.desc = DRIVER_DESC,
	.date = DRIVER_DATE,
	.major = DRIVER_MAJOR,
	.minor = DRIVER_MINOR,
	.patchlevel = DRIVER_PATCHLEVEL,
};

struct xe_device *
xe_device_create(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	struct xe_device *xe;
	int ret;

	xe = devm_drm_dev_alloc(&pdev->dev, &driver, struct xe_device, drm);
	if (IS_ERR(xe))
		return xe;

	ret = ttm_device_init(&xe->ttm, &xe_ttm_funcs, xe->drm.dev,
			      xe->drm.anon_inode->i_mapping,
			      xe->drm.vma_offset_manager, false, false);

	ret = drm_dev_register(&xe->drm, 0);
	if (ret)
		return ERR_PTR(ret);

	return xe;
}

void xe_device_remove(struct xe_device *xe)
{
	drm_dev_unregister(&xe->drm);
}

void xe_device_shutdown(struct xe_device *xe)
{
}
