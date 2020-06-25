
#include <linux/dma-mapping.h>
#include <linux/iommu.h>
#include <linux/hmm.h>
#include <linux/pagemap.h>
#include <linux/sched/task.h>
#include <linux/sched/mm.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/swap.h>
#include <linux/swiotlb.h>
#include <linux/dma-buf.h>
#include <linux/sizes.h>

#include <drm/ttm/ttm_bo_api.h>
#include <drm/ttm/ttm_bo_driver.h>
#include <drm/ttm/ttm_placement.h>
#include <drm/ttm/ttm_module.h>
#include <drm/ttm/ttm_page_alloc.h>

#include <drm/drm_debugfs.h>

#include "i915_drv.h"
#include "i915_ttm.h"

static int i915_ttm_init_lmem(struct drm_i915_private *i915)
{
	struct ttm_mem_type_manager *man = &i915->ttm_mman.bdev.man[TTM_PL_VRAM];

	man->func = &i915_ttm_vram_mgr_func;
	man->available_caching = TTM_PL_FLAG_UNCACHED | TTM_PL_FLAG_WC;
	man->default_caching = TTM_PL_FLAG_WC;
	return ttm_bo_init_mm(&i915->ttm_mman.bdev, TTM_PL_VRAM,
			      8);
}

struct i915_ttm_tt {
	struct ttm_dma_tt ttm;
	struct drm_gem_object *gobj;
	u64 offset;
};

static int i915_ttm_backend_bind(struct ttm_tt *ttm,
				 struct ttm_mem_reg *bo_mem)
{
	return 0;
}

static void i915_ttm_backend_unbind(struct ttm_tt *ttm)
{
}

static void i915_ttm_backend_destroy(struct ttm_tt *ttm)
{

	struct i915_ttm_tt *gtt = (void *)ttm;

	ttm_dma_tt_fini(&gtt->ttm);
	kfree(gtt);
}

static struct ttm_backend_func i915_ttm_backend_func = {
	.bind = &i915_ttm_backend_bind,
	.unbind = &i915_ttm_backend_unbind,
	.destroy = &i915_ttm_backend_destroy,
};

static struct ttm_tt *i915_ttm_tt_create(struct ttm_buffer_object *bo,
					 uint32_t page_flags)
{
	struct i915_ttm_tt *gtt;

	gtt = kzalloc(sizeof(struct i915_ttm_tt), GFP_KERNEL);
	if (gtt == NULL)
		return NULL;

	gtt->ttm.ttm.func = &i915_ttm_backend_func;
	gtt->gobj = &bo->base;

	if (ttm_sg_tt_init(&gtt->ttm, bo, page_flags)) {
		kfree(gtt);
		return NULL;
	}
	return &gtt->ttm.ttm;
}

static int i915_ttm_tt_populate(struct ttm_tt *ttm,
				struct ttm_operation_ctx *ctx)
{
	return 0;
}

static void i915_ttm_tt_unpopulate(struct ttm_tt *ttm)
{

}

static struct ttm_bo_driver i915_ttm_bo_driver = {
	.ttm_tt_create = &i915_ttm_tt_create,
	.ttm_tt_populate = &i915_ttm_tt_populate,
	.ttm_tt_unpopulate = &i915_ttm_tt_unpopulate,
#if 0
	.eviction_valuable = amdgpu_ttm_bo_eviction_valuable,
	.evict_flags = &amdgpu_evict_flags,
	.move = &amdgpu_bo_move,
	.verify_access = &amdgpu_verify_access,
	.move_notify = &amdgpu_bo_move_notify,
	.release_notify = &amdgpu_bo_release_notify,
	.fault_reserve_notify = &amdgpu_bo_fault_reserve_notify,
	.io_mem_reserve = &amdgpu_ttm_io_mem_reserve,
	.io_mem_free = &amdgpu_ttm_io_mem_free,
	.io_mem_pfn = amdgpu_ttm_io_mem_pfn,
	.access_memory = &amdgpu_ttm_access_memory,
	.del_from_lru_notify = &amdgpu_vm_del_from_lru_notify
#endif
};

int i915_ttm_init(struct drm_i915_private *i915)
{
	int r;
	r = ttm_bo_device_init(&i915->ttm_mman.bdev,
			       &i915_ttm_bo_driver,
			       i915->drm.anon_inode->i_mapping,
			       i915->drm.vma_offset_manager,
			       false);

	if (r) {
		DRM_ERROR("failed initialiseing TTM(%d)\n", r);
		return r;
	}
	i915->ttm_mman.initialized = true;

	r = i915_ttm_init_lmem(i915);
	if (r) {
		DRM_ERROR("failed to init TTM heap.\n");
		return r;
	}

	return 0;
}

void i915_ttm_fini(struct drm_i915_private *i915)
{
	if (!i915->ttm_mman.initialized)
		return;

	ttm_bo_clean_mm(&i915->ttm_mman.bdev, TTM_PL_VRAM);
	ttm_bo_clean_mm(&i915->ttm_mman.bdev, TTM_PL_TT);

	ttm_bo_device_release(&i915->ttm_mman.bdev);
	i915->ttm_mman.initialized = false;
	DRM_INFO("i915: ttm finialized\n");
}


/* Validate bo size is bit bigger then the request domain */
static bool i915_ttm_bo_validate_size(struct drm_i915_private *i915,
				      unsigned long size, u32 domain)
{
	struct ttm_mem_type_manager *man = NULL;

	/*
	 * If GTT is part of requested domains the check must succeed to
	 * allow fall back to GTT
	 */
	if (domain & REGION_SMEM) {
		man = &i915->ttm_mman.bdev.man[TTM_PL_TT];

		if (size < (man->size << PAGE_SHIFT))
			return true;
		else
			goto fail;
	}

	if (domain & (REGION_STOLEN | REGION_LMEM)) {
		man = &i915->ttm_mman.bdev.man[TTM_PL_VRAM];

		if (size < (man->size << PAGE_SHIFT))
			return true;
		else
			goto fail;
	}


	/* TODO add more domains checks, such as AMDGPU_GEM_DOMAIN_CPU */
	return true;

fail:
	DRM_DEBUG("BO size %lu > total memory in domain: %llu\n", size,
		  man->size << PAGE_SHIFT);
	return false;
}

static void i915_ttm_bo_destroy(struct ttm_buffer_object *tbo)
{
	struct drm_i915_private *i915 = to_i915_ttm_dev(tbo->bdev);
	struct i915_ttm_bo *bo = ttm_to_i915_bo(tbo);

	drm_gem_object_release(&bo->tbo.base);
	kfree(bo);
}

static int i915_ttm_bo_do_create(struct drm_i915_private *i915,
				 struct i915_ttm_bo_param *bp,
				 struct i915_ttm_bo **bo_ptr)
{
	struct ttm_operation_ctx ctx = {
		.interruptible = (bp->type != ttm_bo_type_kernel),
		.no_wait_gpu = bp->no_wait_gpu,
		.flags = bp->type != ttm_bo_type_kernel ? TTM_OPT_FLAG_ALLOW_RES_EVICT : 0
	};
	int r;

	struct i915_ttm_bo *bo;
	unsigned long page_align, size = bp->size;
	size_t acc_size;

	page_align = ALIGN(bp->byte_align, PAGE_SIZE) >> PAGE_SHIFT;
	size = ALIGN(size, PAGE_SIZE);

	if (!i915_ttm_bo_validate_size(i915, size, bp->region))
		return -ENOMEM;

	*bo_ptr = NULL;

	acc_size = ttm_bo_dma_acc_size(&i915->ttm_mman.bdev, size, sizeof(struct i915_ttm_bo));

	bo = kzalloc(sizeof(struct i915_ttm_bo), GFP_KERNEL);
	if (bo == NULL)
		return -ENOMEM;

	drm_gem_private_object_init(&i915->drm, &bo->tbo.base, size);
//if (bp->type != ttm_bo_type_kernel && bo->allowed_regions == REGION_LMEM)
		
	bo->tbo.bdev = &i915->ttm_mman.bdev;

	/* placement */

	r = ttm_bo_init_reserved(&i915->ttm_mman.bdev, &bo->tbo, size, bp->type,
				 &bo->placement, page_align, &ctx, acc_size,
				 NULL, 0, &i915_ttm_bo_destroy);

	if (unlikely(r != 0))
		return r;

	*bo_ptr = bo;
	return 0;
}


int i915_ttm_bo_create(struct drm_i915_private *i915,
		       struct i915_ttm_bo_param *bp,
		       struct i915_ttm_bo **bo_ptr)
{
	return i915_ttm_bo_do_create(i915, bp, bo_ptr);
}


/**
 * i915_ttm_bo_placement_from_region - set buffer's placement
 * @bo: &i915_ttm_bo buffer object whose placement is to be set
 * @region: requested region
 *
 * Sets buffer's placement according to requested region and the buffer's
 * flags.
 */
void i915_ttm_bo_placement_from_region(struct i915_ttm_bo *bo, u32 region)
{
	struct drm_i915_private *i915 = to_i915_ttm_dev(bo->tbo.bdev);
	struct ttm_placement *placement = &bo->placement;
	struct ttm_place *places = bo->placements;
	u64 flags = bo->flags;
	u32 c = 0;

	if (region & REGION_LMEM) {
		places[c].fpfn = 0;
		places[c].lpfn = 0;
		places[c].flags = TTM_PL_FLAG_WC | TTM_PL_FLAG_UNCACHED |
			TTM_PL_FLAG_VRAM;

		places[c].flags |= TTM_PL_FLAG_TOPDOWN;

		if (flags & I915_TTM_CREATE_VRAM_CONTIGUOUS)
			places[c].flags |= TTM_PL_FLAG_CONTIGUOUS;
		c++;
	}

	if (region & REGION_SMEM) {
		places[c].fpfn = 0;
		places[c].lpfn = 0;
		places[c].flags = TTM_PL_FLAG_TT;
		if (flags & I915_TTM_CREATE_CPU_GTT_USWC)
			places[c].flags |= TTM_PL_FLAG_WC |
				TTM_PL_FLAG_UNCACHED;
		else
			places[c].flags |= TTM_PL_FLAG_CACHED;
		c++;
	}

	if (!region) {
		places[c].fpfn = 0;
		places[c].lpfn = 0;
		places[c].flags = TTM_PL_FLAG_SYSTEM;
		if (flags & I915_TTM_CREATE_CPU_GTT_USWC)
			places[c].flags |= TTM_PL_FLAG_WC |
				TTM_PL_FLAG_UNCACHED;
		else
			places[c].flags |= TTM_PL_FLAG_CACHED;
		c++;
	}

	if (!c) {
		places[c].fpfn = 0;
		places[c].lpfn = 0;
		places[c].flags = TTM_PL_MASK_CACHING | TTM_PL_FLAG_SYSTEM;
		c++;
	}

	BUG_ON(c >= I915_TTM_BO_MAX_PLACEMENTS);

	placement->num_placement = c;
	placement->placement = places;

	placement->num_busy_placement = c;
	placement->busy_placement = places;
}

/**
 * i915_ttm_bo_create_reserved - create reserved BO for kernel use
 *
 * @i915: i915 object
 * @size: size for the new BO
 * @align: alignment for the new BO
 * @region: where to place it
 * @bo_ptr: used to initialize BOs in structures
 * @gpu_addr: GPU addr of the pinned BO
 * @cpu_addr: optional CPU address mapping
 *
 * Allocates and pins a BO for kernel internal use, and returns it still
 * reserved.
 *
 * Note: For bo_ptr new BO is only created if bo_ptr points to NULL.
 *
 * Returns:
 * 0 on success, negative error code otherwise.
 */
int i915_ttm_bo_create_reserved(struct drm_i915_private *i915,
				unsigned long size, int align,
				u32 region, struct i915_ttm_bo **bo_ptr,
				u64 *gpu_addr, void **cpu_addr)
{
	struct i915_ttm_bo_param bp;
	bool free = false;
	int r;

	if (!size) {
		i915_ttm_bo_unref(bo_ptr);
		return 0;
	}

	memset(&bp, 0, sizeof(bp));
	bp.size = size;
	bp.byte_align = align;
	bp.region = region;
	bp.flags = 0;
	bp.flags |= I915_TTM_CREATE_VRAM_CONTIGUOUS;
	bp.type = ttm_bo_type_kernel;
	bp.resv = NULL;


	if (!*bo_ptr) {
		r = i915_ttm_bo_create(i915, &bp, bo_ptr);
		if (r) {
			dev_err(i915->drm.dev, "(%d) failed to allocate kernel bo\n", r);
			return r;
		}
		free = true;
	}

	/* reserver */
	r = i915_ttm_bo_reserve(*bo_ptr, false);
	if (r) {
		dev_err(i915->drm.dev, "(%d) failed to reserve kernel bo\n", r);
		goto error_free;
	}
	
	/* pin */
	r = i915_ttm_bo_pin(*bo_ptr, region);
	if (r) {
		dev_err(i915->drm.dev, "(%d) failed to pin kernel bo\n", r);
		goto error_unreserve;
	}

	/* alloc gart? */

	/* set gpu addr */

	/* return cpu_addr */
	if (cpu_addr) {
		r = i915_ttm_bo_kmap(*bo_ptr, cpu_addr);
		if (r) {
			dev_err(i915->drm.dev, "(%d) kernel bo map failed\n", r);
			goto error_unpin;
		}
	}
	return 0;
error_unpin:
	i915_ttm_bo_unpin(*bo_ptr);
error_unreserve:
	i915_ttm_bo_unreserve(*bo_ptr);
error_free:
	if (free)
		i915_ttm_bo_unref(bo_ptr);
	return r;
}

int i915_ttm_bo_create_kernel(struct drm_i915_private *i915,
			      unsigned long size, int align,
			      u32 region, struct i915_ttm_bo **bo_ptr,
			      u64 *gpu_addr, void **cpu_addr)
{
	int r;

	r = i915_ttm_bo_create_reserved(i915, size, align, region, bo_ptr,
					gpu_addr, cpu_addr);

	if (r)
		return r;

	if (*bo_ptr)
		i915_ttm_bo_unreserve(*bo_ptr);

	return 0;
}


/**
 * i915_ttm_bo_kmap - map an &i915_ttm_bo buffer object
 * @bo: &i915_ttm_bo buffer object to be mapped
 * @ptr: kernel virtual address to be returned
 *
 * Calls ttm_bo_kmap() to set up the kernel virtual mapping; calls
 * i915_ttm_bo_kptr() to get the kernel virtual address.
 *
 * Returns:
 * 0 for success or a negative error code on failure.
 */
int i915_ttm_bo_kmap(struct i915_ttm_bo *bo, void **ptr)
{
	void *kptr;
	long r;

//	if (bo->flags & I915_TTM_GEM_CREATE_NO_CPU_ACCESS)
//		return -EPERM;

	kptr = i915_ttm_bo_kptr(bo);
	if (kptr) {
		if (ptr)
			*ptr = kptr;
		return 0;
	}

	r = dma_resv_wait_timeout_rcu(bo->tbo.base.resv, false, false,
						MAX_SCHEDULE_TIMEOUT);
	if (r < 0)
		return r;

	r = ttm_bo_kmap(&bo->tbo, 0, bo->tbo.num_pages, &bo->kmap);
	if (r)
		return r;

	if (ptr)
		*ptr = i915_ttm_bo_kptr(bo);

	return 0;
}

/**
 * i915_ttm_bo_kptr - returns a kernel virtual address of the buffer object
 * @bo: &i915_ttm_bo buffer object
 *
 * Calls ttm_kmap_obj_virtual() to get the kernel virtual address
 *
 * Returns:
 * the virtual address of a buffer object area.
 */
void *i915_ttm_bo_kptr(struct i915_ttm_bo *bo)
{
	bool is_iomem;

	return ttm_kmap_obj_virtual(&bo->kmap, &is_iomem);
}

/**
 * i915_ttm_bo_kunmap - unmap an &i915_ttm_bo buffer object
 * @bo: &i915_ttm_bo buffer object to be unmapped
 *
 * Unmaps a kernel map set up by i915_ttm_bo_kmap().
 */
void i915_ttm_bo_kunmap(struct i915_ttm_bo *bo)
{
	if (bo->kmap.bo)
		ttm_bo_kunmap(&bo->kmap);
}

/**
 * i915_ttm_bo_ref - reference an &i915_ttm_bo buffer object
 * @bo: &i915_ttm_bo buffer object
 *
 * References the contained &ttm_buffer_object.
 *
 * Returns:
 * a refcounted pointer to the &i915_ttm_bo buffer object.
 */
struct i915_ttm_bo *i915_ttm_bo_ref(struct i915_ttm_bo *bo)
{
	if (bo == NULL)
		return NULL;

	ttm_bo_get(&bo->tbo);
	return bo;
}

/**
 * i915_ttm_bo_unref - unreference an &i915_ttm_bo buffer object
 * @bo: &i915_ttm_bo buffer object
 *
 * Unreferences the contained &ttm_buffer_object and clear the pointer
 */
void i915_ttm_bo_unref(struct i915_ttm_bo **bo)
{
	struct ttm_buffer_object *tbo;

	if ((*bo) == NULL)
		return;

	tbo = &((*bo)->tbo);
	ttm_bo_put(tbo);
	*bo = NULL;
}

/**
 * i915_ttm_bo_pin_restricted - pin an &i915_ttm_bo buffer object
 * @bo: &i915_ttm_bo buffer object to be pinned
 * @region: region to be pinned to
 * @min_offset: the start of requested address range
 * @max_offset: the end of requested address range
 *
 * Pins the buffer object according to requested region and address range. If
 * the memory is unbound gart memory, binds the pages into gart table. Adjusts
 * pin_count and pin_size accordingly.
 *
 * Pinning means to lock pages in memory along with keeping them at a fixed
 * offset. It is required when a buffer can not be moved, for example, when
 * a display buffer is being scanned out.
 *
 * Compared with i915_ttm_bo_pin(), this function gives more flexibility on
 * where to pin a buffer if there are specific restrictions on where a buffer
 * must be located.
 *
 * Returns:
 * 0 for success or a negative error code on failure.
 */
int i915_ttm_bo_pin_restricted(struct i915_ttm_bo *bo, u32 region,
			       u64 min_offset, u64 max_offset)
{
	struct drm_i915_private *i915 = to_i915_ttm_dev(bo->tbo.bdev);
	struct ttm_operation_ctx ctx = { false, false };
	int r, i;

	if (WARN_ON_ONCE(min_offset > max_offset))
		return -EINVAL;

	/* A shared bo cannot be migrated to VRAM */
	if (bo->prime_shared_count) {
		if (region & REGION_SMEM)
		        region = REGION_SMEM;
		else
			return -EINVAL;
	}

	/* This assumes only APU display buffers are pinned with (VRAM|GTT).
	 * See function i915_ttm_display_supported_regions()
	 */
	region = i915_ttm_bo_get_preferred_pin_region(i915, region);

	if (bo->pin_count) {
		uint32_t mem_type = bo->tbo.mem.mem_type;

		if (!(region & i915_ttm_mem_type_to_region(mem_type)))
			return -EINVAL;

		bo->pin_count++;

		if (max_offset != 0) {
			u64 region_start = bo->tbo.bdev->man[mem_type].gpu_offset;
			WARN_ON_ONCE(max_offset <
				     (i915_ttm_bo_gpu_offset(bo) - region_start));
		}

		return 0;
	}

	if (bo->tbo.base.import_attach)
		dma_buf_pin(bo->tbo.base.import_attach);

	bo->flags |= I915_TTM_CREATE_VRAM_CONTIGUOUS;
	i915_ttm_bo_placement_from_region(bo, region);
	for (i = 0; i < bo->placement.num_placement; i++) {
		unsigned fpfn, lpfn;

		fpfn = min_offset >> PAGE_SHIFT;
		lpfn = max_offset >> PAGE_SHIFT;

		if (fpfn > bo->placements[i].fpfn)
			bo->placements[i].fpfn = fpfn;
		if (!bo->placements[i].lpfn ||
		    (lpfn && lpfn < bo->placements[i].lpfn))
			bo->placements[i].lpfn = lpfn;
		bo->placements[i].flags |= TTM_PL_FLAG_NO_EVICT;
	}

	r = ttm_bo_validate(&bo->tbo, &bo->placement, &ctx);
	if (unlikely(r)) {
		dev_err(i915->drm.dev, "%p pin failed\n", bo);
		goto error;
	}

	bo->pin_count = 1;
#if 0
	region = i915_ttm_mem_type_to_region(bo->tbo.mem.mem_type);
	if (region == I915_TTM_GEM_REGION_VRAM) {
		atomic64_add(i915_ttm_bo_size(bo), &adev->vram_pin_size);
		atomic64_add(i915_ttm_vram_mgr_bo_visible_size(bo),
			     &adev->visible_pin_size);
	} else if (region == REGION_SMEM) {
		atomic64_add(i915_ttm_bo_size(bo), &adev->gart_pin_size);
	}
#endif
error:
	return r;
}

/**
 * i915_ttm_bo_pin - pin an &i915_ttm_bo buffer object
 * @bo: &i915_ttm_bo buffer object to be pinned
 * @region: region to be pinned to.
 *
 * A simple wrapper to i915_ttm_bo_pin_restricted().
 * Provides a simpler API for buffers that do not have any strict restrictions
 * on where a buffer must be located.
 *
 * Returns:
 * 0 for success or a negative error code on failure.
 */
int i915_ttm_bo_pin(struct i915_ttm_bo *bo, u32 region)
{
	return i915_ttm_bo_pin_restricted(bo, region, 0, 0);
}

/**
 * i915_ttm_bo_unpin - unpin an &i915_ttm_bo buffer object
 * @bo: &i915_ttm_bo buffer object to be unpinned
 *
 * Decreases the pin_count, and clears the flags if pin_count reaches 0.
 * Changes placement and pin size accordingly.
 *
 * Returns:
 * 0 for success or a negative error code on failure.
 */
int i915_ttm_bo_unpin(struct i915_ttm_bo *bo)
{
	struct drm_i915_private *i915 = to_i915_ttm_dev(bo->tbo.bdev);
	struct ttm_operation_ctx ctx = { false, false };
	int r, i;

	if (WARN_ON_ONCE(!bo->pin_count)) {
		dev_warn(i915->drm.dev, "%p unpin not necessary\n", bo);
		return 0;
	}
	bo->pin_count--;
	if (bo->pin_count)
		return 0;

//	i915_ttm_bo_subtract_pin_size(bo);

//	if (bo->tbo.base.import_attach)
//		dma_buf_unpin(bo->tbo.base.import_attach);

	for (i = 0; i < bo->placement.num_placement; i++) {
		bo->placements[i].lpfn = 0;
		bo->placements[i].flags &= ~TTM_PL_FLAG_NO_EVICT;
	}
	r = ttm_bo_validate(&bo->tbo, &bo->placement, &ctx);
	if (unlikely(r))
		dev_err(i915->drm.dev, "%p validate failed for unpin\n", bo);

	return r;
}

/**
 * i915_ttm_bo_gpu_offset - return GPU offset of bo
 * @bo:	i915_ttm object for which we query the offset
 *
 * Note: object should either be pinned or reserved when calling this
 * function, it might be useful to add check for this for debugging.
 *
 * Returns:
 * current GPU offset of the object.
 */
u64 i915_ttm_bo_gpu_offset(struct i915_ttm_bo *bo)
{
	WARN_ON_ONCE(bo->tbo.mem.mem_type == TTM_PL_SYSTEM);
	WARN_ON_ONCE(!dma_resv_is_locked(bo->tbo.base.resv) &&
		     !bo->pin_count && bo->tbo.type != ttm_bo_type_kernel);
	WARN_ON_ONCE(bo->tbo.mem.start == I915_TTM_BO_INVALID_OFFSET);
	WARN_ON_ONCE(bo->tbo.mem.mem_type == TTM_PL_VRAM &&
		     !(bo->flags & I915_TTM_CREATE_VRAM_CONTIGUOUS));

	return bo->tbo.offset;
}

/**
 * i915_ttm_bo_get_preferred_pin_region - get preferred region for scanout
 * @adev: i915_ttm device object
 * @region: allowed :ref:`memory regions <i915_ttm_memory_regions>`
 *
 * Returns:
 * Which of the allowed regions is preferred for pinning the BO for scanout.
 */
uint32_t i915_ttm_bo_get_preferred_pin_region(struct drm_i915_private *i915,
					      uint32_t region)
{
	if (region == (REGION_LMEM | REGION_SMEM)) {
		region = REGION_LMEM;
//		if (adev->gmc.real_vram_size <= I915_TTM_SG_THRESHOLD)
//			region = REGION_SMEM;
	}
	return region;
}


void i915_ttm_gem_object_free(struct drm_gem_object *gobj)
{
	struct i915_ttm_bo *robj = ttm_gem_to_i915_bo(gobj);

	if (robj) {
		i915_ttm_bo_unref(&robj);
	}
}

int i915_ttm_gem_object_create(struct drm_i915_private *i915, unsigned long size,
			       int alignment, u32 initial_region,
			       u64 flags, enum ttm_bo_type type,
			       struct dma_resv *resv,
			       struct drm_gem_object **obj)
{
	struct i915_ttm_bo *bo;
	struct i915_ttm_bo_param bp;
	int r;

	memset(&bp, 0, sizeof(bp));
	*obj = NULL;

	bp.size = size;
	bp.byte_align = alignment;
	bp.type = type;
	bp.resv = resv;
	bp.preferred_region = initial_region;
retry:
	bp.flags = flags;
	bp.region = initial_region;
	r = i915_ttm_bo_create(i915, &bp, &bo);
	if (r) {
		if (r != -ERESTARTSYS) {
			if (initial_region == REGION_LMEM) { 
				initial_region |= REGION_SMEM;
				goto retry;
			}
			DRM_DEBUG("Failed to allocate GEM object (%ld, %d, %u, %d)\n",
				  size, initial_region, alignment, r);
		}
		return r;
	}
	*obj = &bo->tbo.base;

	return 0;
}
