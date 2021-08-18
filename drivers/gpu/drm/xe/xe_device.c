/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2021 Intel Corporation
 */

#include "xe_device.h"

#include <drm/drm_gem_ttm_helper.h>
#include <drm/drm_aperture.h>
#include <drm/drm_ioctl.h>
#include <drm/xe_drm.h>

#include "xe_bo.h"
#include "xe_drv.h"
#include "xe_engine.h"
#include "xe_vm.h"

static int xe_file_open(struct drm_device *dev, struct drm_file *file)
{
	struct xe_file *xef;

	xef = kzalloc(sizeof(*xef), GFP_KERNEL);
	if (!xef)
		return -ENOMEM;

	mutex_init(&xef->vm_lock);
	xa_init_flags(&xef->vm_xa, XA_FLAGS_ALLOC1);

	mutex_init(&xef->engine_lock);
	xa_init_flags(&xef->engine_xa, XA_FLAGS_ALLOC1);

	file->driver_priv = xef;
	return 0;
}

static void xe_file_close(struct drm_device *dev, struct drm_file *file)
{
	struct xe_file *xef = file->driver_priv;
	struct xe_vm *vm;
	struct xe_engine *e;
	unsigned long idx;

	xa_for_each(&xef->vm_xa, idx, vm)
		xe_vm_put(vm);
	mutex_destroy(&xef->vm_lock);

	xa_for_each(&xef->engine_xa, idx, e)
		xe_engine_put(e);
	mutex_destroy(&xef->engine_lock);

	kfree(xef);
}

static const struct drm_ioctl_desc xe_ioctls[] = {
	DRM_IOCTL_DEF_DRV(XE_GEM_CREATE, xe_gem_create_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(XE_GEM_MMAP_OFFSET, xe_gem_mmap_offset_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(XE_VM_CREATE, xe_vm_create_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(XE_VM_DESTROY, xe_vm_destroy_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(XE_VM_BIND, xe_vm_bind_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(XE_ENGINE_CREATE, xe_engine_create_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(XE_ENGINE_DESTROY, xe_engine_destroy_ioctl, DRM_RENDER_ALLOW),
};

static const struct file_operations xe_driver_fops = {
	.owner = THIS_MODULE,
	.open = drm_open,
	.release = drm_release_noglobal,
	.unlocked_ioctl = drm_ioctl,
	.mmap = drm_gem_mmap,
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
	    DRIVER_RENDER | DRIVER_SYNCOBJ |
	    DRIVER_SYNCOBJ_TIMELINE,
	.open = xe_file_open,
	.postclose = xe_file_close,

	.prime_handle_to_fd = drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle = drm_gem_prime_fd_to_handle,
//	.gem_prime_import = i915_gem_prime_import,
//
//	.dumb_create = i915_gem_dumb_create,
	.dumb_map_offset = drm_gem_ttm_dumb_map_offset,

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
	int err;

	err = drm_aperture_remove_conflicting_pci_framebuffers(pdev, &driver);
	if (err)
		return ERR_PTR(err);

	xe = devm_drm_dev_alloc(&pdev->dev, &driver, struct xe_device, drm);
	if (IS_ERR(xe))
		return xe;

	xe->pdev = pdev;

	err = pci_enable_device(pdev);
	if (err)
		return ERR_PTR(err);

	err = ttm_device_init(&xe->ttm, &xe_ttm_funcs, xe->drm.dev,
			      xe->drm.anon_inode->i_mapping,
			      xe->drm.vma_offset_manager, false, false);
	if (WARN_ON(err)) {
		drm_dev_put(&xe->drm);
		return ERR_PTR(err);
	}

	err = drm_dev_register(&xe->drm, 0);
	if (err)
		return ERR_PTR(err);

	err = xe_irq_install(xe);
	if (err)
		return ERR_PTR(err);

	return xe;
}

void xe_device_remove(struct xe_device *xe)
{
	xe_irq_uninstall(xe);
	drm_dev_unregister(&xe->drm);
	ttm_device_fini(&xe->ttm);
}

void xe_device_shutdown(struct xe_device *xe)
{
}
