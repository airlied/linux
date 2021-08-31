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
#include "xe_mmio.h"
#include "xe_vm.h"

#include "../i915/i915_reg.h"

static int xe_file_open(struct drm_device *dev, struct drm_file *file)
{
	struct xe_file *xef;

	xef = kzalloc(sizeof(*xef), GFP_KERNEL);
	if (!xef)
		return -ENOMEM;

	xef->drm = file;

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
	DRM_IOCTL_DEF_DRV(XE_EXEC, xe_exec_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(XE_MMIO, xe_mmio_ioctl, DRM_RENDER_ALLOW),
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

static void xe_driver_release(struct drm_device *dev)
{
	struct xe_device *xe = to_xe_device(dev);

	pci_set_drvdata(to_pci_dev(xe->drm.dev), NULL);
}

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
	.release = &xe_driver_release,

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

struct xe_device *xe_device_create(struct pci_dev *pdev,
				   const struct pci_device_id *ent)
{
	struct xe_device *xe;
	int err;

	err = drm_aperture_remove_conflicting_pci_framebuffers(pdev, &driver);
	if (err)
		return ERR_PTR(err);

	xe = devm_drm_dev_alloc(&pdev->dev, &driver, struct xe_device, drm);
	if (IS_ERR(xe))
		return xe;

	err = ttm_device_init(&xe->ttm, &xe_ttm_funcs, xe->drm.dev,
			      xe->drm.anon_inode->i_mapping,
			      xe->drm.vma_offset_manager, false, false);
	if (WARN_ON(err))
		goto err_put;

	spin_lock_init(&xe->gt_irq_lock);

	return xe;

err_put:
	drm_dev_put(&xe->drm);
	return ERR_PTR(err);
}

static int xe_set_dma_info(struct xe_device *xe)
{
	unsigned int mask_size = 39; /* TODO: Don't hard-code */
	int err;

	/*
	 * We don't have a max segment size, so set it to the max so sg's
	 * debugging layer doesn't complain
	 */
	dma_set_max_seg_size(xe->drm.dev, UINT_MAX);

	err = dma_set_mask(xe->drm.dev, DMA_BIT_MASK(mask_size));
	if (err)
		goto mask_err;

	err = dma_set_coherent_mask(xe->drm.dev, DMA_BIT_MASK(mask_size));
	if (err)
		goto mask_err;

	return 0;

mask_err:
	drm_err(&xe->drm, "Can't set DMA mask/consistent mask (%d)\n", err);
	return err;
}


#define CHV_PPAT_SNOOP			REG_BIT(6)
#define GEN8_PPAT_AGE(x)		((x)<<4)
#define GEN8_PPAT_LLCeLLC		(3<<2)
#define GEN8_PPAT_LLCELLC		(2<<2)
#define GEN8_PPAT_LLC			(1<<2)
#define GEN8_PPAT_WB			(3<<0)
#define GEN8_PPAT_WT			(2<<0)
#define GEN8_PPAT_WC			(1<<0)
#define GEN8_PPAT_UC			(0<<0)
#define GEN8_PPAT_ELLC_OVERRIDE		(0<<2)
#define GEN8_PPAT(i, x)			((u64)(x) << ((i) * 8))

static void tgl_setup_private_ppat(struct xe_device *xe)
{
	/* TGL doesn't support LLC or AGE settings */
	xe_mmio_write32(xe, GEN12_PAT_INDEX(0).reg, GEN8_PPAT_WB);
	xe_mmio_write32(xe, GEN12_PAT_INDEX(1).reg, GEN8_PPAT_WC);
	xe_mmio_write32(xe, GEN12_PAT_INDEX(2).reg, GEN8_PPAT_WT);
	xe_mmio_write32(xe, GEN12_PAT_INDEX(3).reg, GEN8_PPAT_UC);
	xe_mmio_write32(xe, GEN12_PAT_INDEX(4).reg, GEN8_PPAT_WB);
	xe_mmio_write32(xe, GEN12_PAT_INDEX(5).reg, GEN8_PPAT_WB);
	xe_mmio_write32(xe, GEN12_PAT_INDEX(6).reg, GEN8_PPAT_WB);
	xe_mmio_write32(xe, GEN12_PAT_INDEX(7).reg, GEN8_PPAT_WB);
}

static int xe_device_ttm_mgr_init(struct xe_device *xe)
{
	int err;
	struct sysinfo si;
	uint64_t gtt_size;

	si_meminfo(&si);
	gtt_size = (uint64_t)si.totalram * si.mem_unit * 3/4;

	if (xe->vram.size) {
		err = xe_ttm_vram_mgr_init(xe);
		if (err)
			return err;
#ifdef CONFIG_64BIT
		xe->vram.mapping = ioremap_wc(xe->vram.io_start,
					      xe->vram.size);
#endif
		gtt_size = min(max((XE_DEFAULT_GTT_SIZE_MB << 20),
				   xe->vram.size),
			       gtt_size);
	}

	err = xe_ttm_gtt_mgr_init(xe, gtt_size);
	if (err)
		goto err_vram_mgr;

	return 0;
err_vram_mgr:
	if (xe->vram.size)
		xe_ttm_vram_mgr_fini(xe);
	return err;
}

static void xe_device_ttm_mgr_fini(struct xe_device *xe)
{
	if (xe->vram.size)
		xe_ttm_vram_mgr_fini(xe);
	xe_ttm_gtt_mgr_fini(xe);
}

int xe_device_probe(struct xe_device *xe)
{
	int err, i;

	err = xe_mmio_init(xe);
	if (err)
		return err;

	err = xe_set_dma_info(xe);
	if (err)
		return err;

	tgl_setup_private_ppat(xe);

	err = xe_device_ttm_mgr_init(xe);
	if (err)
		goto err_mmio;

	err = xe_ggtt_init(xe, &xe->ggtt);
	if (err)
		goto err_ttm_mgr;

	for (i = 0; i < ARRAY_SIZE(xe->hw_engines); i++) {
		err = xe_hw_engine_init(xe, &xe->hw_engines[i], i);
		if (err)
			goto err_hw_engines;
	}

	err = xe_irq_install(xe);
	if (err)
		goto err_hw_engines;

	err = drm_dev_register(&xe->drm, 0);
	if (err)
		goto err_irq;

	return 0;

err_irq:
	xe_irq_uninstall(xe);
err_hw_engines:
	for (i = 0; i < ARRAY_SIZE(xe->hw_engines); i++) {
		if (xe_hw_engine_is_valid(&xe->hw_engines[i]))
			xe_hw_engine_finish(&xe->hw_engines[i]);
	}
	xe_ggtt_finish(&xe->ggtt);
err_ttm_mgr:
	xe_device_ttm_mgr_fini(xe);
err_mmio:
	xe_mmio_finish(xe);

	return err;
}

void xe_device_remove(struct xe_device *xe)
{
	int i;

	if (xe->vram.mapping)
		iounmap(xe->vram.mapping);
	drm_dev_unregister(&xe->drm);
	xe_irq_uninstall(xe);
	for (i = 0; i < ARRAY_SIZE(xe->hw_engines); i++) {
		if (xe_hw_engine_is_valid(&xe->hw_engines[i]))
			xe_hw_engine_finish(&xe->hw_engines[i]);
	}
	xe_ggtt_finish(&xe->ggtt);
	xe_device_ttm_mgr_fini(xe);
	xe_mmio_finish(xe);
	ttm_device_fini(&xe->ttm);
}

void xe_device_shutdown(struct xe_device *xe)
{
}

struct xe_hw_engine *xe_device_hw_engine(struct xe_device *xe,
					 enum xe_engine_class class,
					 uint16_t instance)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(xe->hw_engines); i++) {
		if (xe_hw_engine_is_valid(&xe->hw_engines[i]) &&
		    xe->hw_engines[i].class == class &&
		    xe->hw_engines[i].instance == instance)
			return &xe->hw_engines[i];
	}

	return NULL;
}
