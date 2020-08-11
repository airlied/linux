
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
#include "intel_pm.h"
#include "gt/intel_gt.h"

static void i915_ttm_evict_flags(struct ttm_buffer_object *tbo,
				struct ttm_placement *placement)
{
	struct drm_i915_private *i915 = to_i915_ttm_dev(tbo->bdev);
	struct i915_ttm_bo *bo;
	static const struct ttm_place placements = {
		.fpfn = 0,
		.lpfn = 0,
		.flags = TTM_PL_MASK_CACHING | TTM_PL_FLAG_SYSTEM
	};

	/* Don't handle scatter gather BOs */
	if (tbo->type == ttm_bo_type_sg) {
		placement->num_placement = 0;
		placement->num_busy_placement = 0;
		return;
	}

	/* Object isn't an I915_TTM object so ignore */
	if (!i915_ttm_bo_is_i915_ttm_bo(tbo)) {
		placement->placement = &placements;
		placement->busy_placement = &placements;
		placement->num_placement = 1;
		placement->num_busy_placement = 1;
		return;
	}


	bo = ttm_to_i915_bo(tbo);
	switch (tbo->mem.mem_type) {
	case TTM_PL_VRAM:
	case I915_TTM_PL_STOLEN:
	default:
		break;
	}

	*placement = bo->placement;
}


/**
 * i915_ttm_verify_access - Verify access for a mmap call
 *
 * @bo:	The buffer object to map
 * @filp: The file pointer from the process performing the mmap
 *
 * This is called by ttm_bo_mmap() to verify whether a process
 * has the right to mmap a BO to their process space.
 */
static int i915_ttm_verify_access(struct ttm_buffer_object *tbo, struct file *filp)
{
	struct i915_ttm_bo *bo = ttm_to_i915_bo(tbo);

//	if (amdgpu_ttm_tt_get_usermm(bo->ttm))
//		return -EPERM;
	return drm_vma_node_verify_access(&tbo->base.vma_node,
					  filp->private_data);
}

/**
 * i915_ttm_move_null - Register memory for a buffer object
 *
 * @bo: The bo to assign the memory to
 * @new_mem: The memory to be assigned.
 *
 * Assign the memory from new_mem to the memory of the buffer object bo.
 */
static void i915_ttm_move_null(struct ttm_buffer_object *bo,
			     struct ttm_resource *new_mem)
{
	struct ttm_resource *old_mem = &bo->mem;

	BUG_ON(old_mem->mm_node != NULL);
	*old_mem = *new_mem;
	new_mem->mm_node = NULL;
}

/**
 * i915_ttm_mm_node_addr - Compute the GPU relative offset of a GTT buffer.
 *
 * @bo: The bo to assign the memory to.
 * @mm_node: Memory manager node for drm allocator.
 * @mem: The region where the bo resides.
 *
 */
static uint64_t i915_ttm_mm_node_addr(struct ttm_buffer_object *bo,
				    struct drm_mm_node *mm_node,
				    struct ttm_resource *mem)
{
	uint64_t addr = 0;

	if (mm_node->start != I915_TTM_BO_INVALID_OFFSET) {
		addr = mm_node->start << PAGE_SHIFT;
//		addr += bo->bdev->man[mem->mem_type].gpu_offset;
	}
	return addr;
}

/**
 * i915_ttm_find_mm_node - Helper function finds the drm_mm_node corresponding to
 * @offset. It also modifies the offset to be within the drm_mm_node returned
 *
 * @mem: The region where the bo resides.
 * @offset: The offset that drm_mm_node is used for finding.
 *
 */
static struct drm_mm_node *i915_ttm_find_mm_node(struct ttm_resource *mem,
					       uint64_t *offset)
{
	struct drm_mm_node *mm_node = mem->mm_node;

	while (*offset >= (mm_node->size << PAGE_SHIFT)) {
		*offset -= (mm_node->size << PAGE_SHIFT);
		++mm_node;
	}
	return mm_node;
}

static int i915_ttm_bo_move(struct ttm_buffer_object *tbo, bool evict,
			    struct ttm_operation_ctx *ctx,
			    struct ttm_resource *new_mem)
{
	struct ttm_resource *old_mem = &tbo->mem;
	struct i915_ttm_bo *bo;
	struct drm_i915_private *i915;
	int r;

	bo = ttm_to_i915_bo(tbo);
	if (WARN_ON_ONCE(bo->pin_count > 0))
		return -EINVAL;

	i915 = to_i915_ttm_dev(tbo->bdev);

	if (old_mem->mem_type == TTM_PL_SYSTEM && tbo->ttm == NULL) {
		i915_ttm_move_null(tbo, new_mem);
		return 0;
	}

	if ((old_mem->mem_type == TTM_PL_TT &&
	     new_mem->mem_type == TTM_PL_SYSTEM) ||
	    (old_mem->mem_type == TTM_PL_SYSTEM &&
	     new_mem->mem_type == TTM_PL_TT)) {
		/* bind is enough */
		i915_ttm_move_null(tbo, new_mem);
		return 0;
	}

	r = ttm_bo_move_memcpy(tbo, ctx, new_mem);
	if (r)
		return r;

	return 0;
}

/**
 * i915_ttm_ttm_io_mem_reserve - Reserve a block of memory during a fault
 *
 * Called by ttm_mem_io_reserve() ultimately via ttm_bo_vm_fault()
 */
static int i915_ttm_io_mem_reserve(struct ttm_bo_device *bdev, struct ttm_resource *mem)
{
	struct ttm_resource_manager *man = ttm_manager_type(bdev, mem->mem_type);
	struct drm_i915_private *i915 = to_i915_ttm_dev(bdev);
	struct drm_mm_node *mm_node = mem->mm_node;

	mem->bus.addr = NULL;
	mem->bus.offset = 0;
	mem->bus.size = mem->num_pages << PAGE_SHIFT;
	mem->bus.base = 0;
	mem->bus.is_iomem = false;

	switch (mem->mem_type) {
	case TTM_PL_SYSTEM:
		/* system memory */
		return 0;
	case TTM_PL_TT:
		break;
	case I915_TTM_PL_STOLEN:
		mem->bus.is_iomem = true;
		mem->bus.offset = mem->start << PAGE_SHIFT;
		if (i915->ttm_mman.aper_base_kaddr &&
		    (mm_node->size == mem->num_pages))
			mem->bus.addr = (u8 *)i915->ttm_mman.aper_base_kaddr +
					mem->bus.offset;

		mem->bus.base = i915->ggtt.gmadr.start;
		break;
	case TTM_PL_VRAM:
		mem->bus.offset = mem->start << PAGE_SHIFT;
		/* check if it's visible */
//		if ((mem->bus.offset + mem->bus.size) > adev->gmc.visible_vram_size)
//			return -EINVAL;
		/* Only physically contiguous buffers apply. In a contiguous
		 * buffer, size of the first mm_node would match the number of
		 * pages in ttm_resource.
		 */
		if (i915->ttm_mman.aper_base_kaddr &&
		    (mm_node->size == mem->num_pages))
			mem->bus.addr = (u8 *)i915->ttm_mman.aper_base_kaddr +
					mem->bus.offset;

		mem->bus.base = i915->mm.regions[INTEL_MEMORY_LOCAL]->io_start;
		mem->bus.is_iomem = true;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static void i915_ttm_io_mem_free(struct ttm_bo_device *bdev, struct ttm_resource *mem)
{
}

static unsigned long i915_ttm_io_mem_pfn(struct ttm_buffer_object *bo,
					 unsigned long page_offset)
{
	uint64_t offset = (page_offset << PAGE_SHIFT);
	struct drm_mm_node *mm;

	mm = i915_ttm_find_mm_node(&bo->mem, &offset);
	return (bo->mem.bus.base >> PAGE_SHIFT) + mm->start +
		(offset >> PAGE_SHIFT);
}

struct i915_ttm_tt {
	struct ttm_dma_tt ttm;
	struct i915_ttm_bo *bo;
	struct drm_gem_object *gobj;
	struct i915_vma *vma;
	uint64_t offset;
};

static int i915_ttm_backend_bind(struct ttm_tt *ttm,
				 struct ttm_resource *bo_mem)
{
	struct i915_ttm_tt *gtt = (struct i915_ttm_tt *)ttm;
	int r;
	if (ttm->page_flags & TTM_PAGE_FLAG_SG)
		return 0;

	if (!i915_ttm_gtt_mgr_has_gart_addr(bo_mem)) {
		gtt->offset = I915_TTM_BO_INVALID_OFFSET;
		return 0;
	}
	r = i915_vma_pin(gtt->vma, 0, 0, PIN_MAPPABLE | PIN_OFFSET_FIXED | PIN_GLOBAL | (bo_mem->start << PAGE_SHIFT));
	return r;
}

static void i915_ttm_backend_unbind(struct ttm_tt *ttm)
{
	struct i915_ttm_tt *gtt = (struct i915_ttm_tt *)ttm;
	if (ttm->page_flags & TTM_PAGE_FLAG_SG)
		return;
	i915_vma_unpin(gtt->vma);
}

static void i915_ttm_backend_destroy(struct ttm_tt *ttm)
{

	struct i915_ttm_tt *gtt = (void *)ttm;

	i915_vma_put(gtt->vma);
	ttm_dma_tt_fini(&gtt->ttm);
	kfree(gtt);
}

static struct ttm_backend_func i915_ttm_backend_func = {
	.bind = &i915_ttm_backend_bind,
	.unbind = &i915_ttm_backend_unbind,
	.destroy = &i915_ttm_backend_destroy,
};

static struct ttm_tt *i915_ttm_tt_create(struct ttm_buffer_object *tbo,
					 uint32_t page_flags)
{
	struct i915_ttm_tt *gtt;
	struct i915_ttm_bo *bo = ttm_to_i915_bo(tbo);
	struct drm_i915_private *i915 = to_i915_ttm_dev(tbo->bdev);

	gtt = kzalloc(sizeof(struct i915_ttm_tt), GFP_KERNEL);
	if (gtt == NULL)
		return NULL;

	gtt->ttm.ttm.func = &i915_ttm_backend_func;
	gtt->bo = bo;

	gtt->vma = i915_ttm_vma_instance(bo,
					 &i915->ggtt.vm, NULL);

	/* flags to disallows mapping */
	if (bo->allowed_regions == REGION_STOLEN)
		page_flags |= TTM_PAGE_FLAG_SG;

	if (ttm_sg_tt_init(&gtt->ttm, tbo, page_flags)) {
		kfree(gtt);
		return NULL;
	}
	return &gtt->ttm.ttm;
}

static int i915_ttm_tt_populate(struct ttm_tt *ttm,
				struct ttm_operation_ctx *ctx)
{
	struct drm_i915_private *i915 = to_i915_ttm_dev(ttm->bdev);
	struct i915_ttm_tt *gtt = (void *)ttm;

	if (ttm->page_flags & TTM_PAGE_FLAG_SG) {
		return 0;
	}
	/* TODO swiotlb */
	return ttm_populate_and_map_pages(i915->drm.dev, &gtt->ttm, ctx);
}

static void i915_ttm_tt_unpopulate(struct ttm_tt *ttm)
{
	struct drm_i915_private *i915 = to_i915_ttm_dev(ttm->bdev);
	struct i915_ttm_tt *gtt = (void *)ttm;
	printk(KERN_INFO "tt unpopulate %p\n", ttm);
	ttm_unmap_and_unpopulate_pages(i915->drm.dev, &gtt->ttm);
}

static struct ttm_bo_driver i915_ttm_bo_driver = {
	.ttm_tt_create = &i915_ttm_tt_create,
	.ttm_tt_populate = &i915_ttm_tt_populate,
	.ttm_tt_unpopulate = &i915_ttm_tt_unpopulate,
	.evict_flags = &i915_ttm_evict_flags,
	.move = &i915_ttm_bo_move,

	.io_mem_reserve = &i915_ttm_io_mem_reserve,
	.io_mem_free = &i915_ttm_io_mem_free,
	.io_mem_pfn = i915_ttm_io_mem_pfn,

	.verify_access = &i915_ttm_verify_access,
#if 0
	.eviction_valuable = amdgpu_ttm_bo_eviction_valuable,


	.move_notify = &amdgpu_bo_move_notify,
	.release_notify = &amdgpu_vbo_release_notify,
	.fault_reserve_notify = &amdgpu_bo_fault_reserve_notify,

	.access_memory = &amdgpu_ttm_access_memory,
	.del_from_lru_notify = &amdgpu_vm_del_from_lru_notify
#endif
};

int i915_ttm_init(struct drm_i915_private *i915)
{
	int r;
	uint64_t vram_size = 0;

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

	if (HAS_LMEM(i915)) {
		r = i915_ttm_vram_mgr_init(i915);
		if (r) {
			DRM_ERROR("failed to init TTM heap.\n");
			return r;
		}

#ifdef CONFIG_64BIT
		i915->ttm_mman.aper_base_kaddr = ioremap_wc(i915->mm.regions[INTEL_MEMORY_LOCAL]->io_start,
							    i915->mm.regions[INTEL_MEMORY_LOCAL]->total);
#endif
	} else
#ifdef CONFIG_64BIT
		i915->ttm_mman.aper_base_kaddr = ioremap_wc(i915->ggtt.gmadr.start,
							    i915->ggtt.mappable_end);
#endif

	r = i915_ttm_gtt_mgr_init(i915);

	printk("creating stolen size %llu\n", i915->stolen_usable_size);
	if (i915->stolen_usable_size) {
		r = i915_ttm_stolen_mgr_init(i915);
	}

	intel_uc_fetch_firmwares(&i915->gt.uc);
	intel_wopcm_init(&i915->wopcm);

	intel_init_clock_gating(i915);

	r = intel_gt_init(&i915->gt);
	if (r)
		goto err_unlock;
	return 0;
err_unlock:
	return r;
}

void i915_ttm_fini(struct drm_i915_private *i915)
{
	if (!i915->ttm_mman.initialized)
		return;

	if (i915->ttm_mman.aper_base_kaddr)
		iounmap(i915->ttm_mman.aper_base_kaddr);
	i915->ttm_mman.aper_base_kaddr = NULL;

	i915_ttm_stolen_mgr_fini(i915);
	i915_ttm_gtt_mgr_fini(i915);
	i915_ttm_vram_mgr_fini(i915);

	ttm_bo_device_release(&i915->ttm_mman.bdev);
	i915->ttm_mman.initialized = false;
	DRM_INFO("i915: ttm finialized\n");
}


/* Validate bo size is bit bigger then the request domain */
static bool i915_ttm_bo_validate_size(struct drm_i915_private *i915,
				      unsigned long size, u32 domain)
{
	struct ttm_resource_manager *man = NULL;

	/*
	 * If GTT is part of requested domains the check must succeed to
	 * allow fall back to GTT
	 */
	if (domain & REGION_SMEM) {
		man = ttm_manager_type(&i915->ttm_mman.bdev, TTM_PL_TT);

		if (size < (man->size << PAGE_SHIFT))
			return true;
		else
			goto fail;
	}

	if (domain & REGION_LMEM) {
		man = ttm_manager_type(&i915->ttm_mman.bdev, TTM_PL_VRAM);

		if (size < (man->size << PAGE_SHIFT))
			return true;
		else
			goto fail;
	}


	if (domain & REGION_STOLEN) {
		man = ttm_manager_type(&i915->ttm_mman.bdev, I915_TTM_PL_STOLEN);

		if (size < (man->size << PAGE_SHIFT))
			return true;
		else
			goto fail;
	}

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

/**
 * i915_ttm_bo_is_i915_ttm_bo - check if the buffer object is an &i915_ttm_bo
 * @bo: buffer object to be checked
 *
 * Uses destroy function associated with the object to determine if this is
 * an &i915_ttm_bo.
 *
 * Returns:
 * true if the object belongs to &i915_ttm_bo, false if not.
 */
bool i915_ttm_bo_is_i915_ttm_bo(struct ttm_buffer_object *bo)
{
	if (bo->destroy == &i915_ttm_bo_destroy)
		return true;
	return false;
}

static int i915_ttm_bo_do_create(struct drm_i915_private *i915,
				 struct i915_ttm_bo_param *bp,
				 struct i915_ttm_bo **bo_ptr)
{
	struct ttm_operation_ctx ctx = {
		.interruptible = (bp->type != ttm_bo_type_kernel),
		.no_wait_gpu = bp->no_wait_gpu,
		.resv = bp->resv,
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
	bo->preferred_regions = bp->preferred_region ? bp->preferred_region : bp->region;
	bo->allowed_regions = bo->preferred_regions;

	if (bp->type != ttm_bo_type_kernel && bo->allowed_regions == REGION_LMEM)
		bo->allowed_regions |= REGION_SMEM;

	bo->flags = bp->flags;

	spin_lock_init(&bo->vma.lock);
	INIT_LIST_HEAD(&bo->vma.list);

	bo->tbo.bdev = &i915->ttm_mman.bdev;
	i915_ttm_bo_placement_from_region(bo, bp->region);

	r = ttm_bo_init_reserved(&i915->ttm_mman.bdev, &bo->tbo, size, bp->type,
				 &bo->placement, page_align, &ctx, acc_size,
				 NULL, 0, &i915_ttm_bo_destroy);

	if (unlikely(r != 0))
		return r;

	if (!bp->resv)
		i915_ttm_bo_unreserve(bo);

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

	if (region & REGION_STOLEN) {
		places[c].fpfn = 0;
		places[c].lpfn = 0;
		/* WaSkipStolenMemoryFirstPage:bdw+ */
		if (INTEL_GEN(i915) >= 8)
			places[c].fpfn = 1;
		places[c].flags = TTM_PL_FLAG_WC | TTM_PL_FLAG_UNCACHED | I915_TTM_PL_FLAG_STOLEN;

		places[c].flags |= TTM_PL_FLAG_CONTIGUOUS;
		c++;
	}

	if (region & REGION_LMEM) {
		places[c].fpfn = 0;
		places[c].lpfn = 0;
		places[c].flags = TTM_PL_FLAG_WC | TTM_PL_FLAG_UNCACHED |
			TTM_PL_FLAG_VRAM;

//		places[c].flags |= TTM_PL_FLAG_TOPDOWN;

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
//			u64 region_start = bo->tbo.bdev->man[mem_type].gpu_offset;
//			WARN_ON_ONCE(max_offset <
//				     (i915_ttm_bo_gpu_offset(bo) - region_start));
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

	return bo->tbo.mem.start << PAGE_SHIFT;
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
	if (region == (REGION_STOLEN | REGION_SMEM)) {
		region = REGION_STOLEN;
	}
	else if (region == (REGION_LMEM | REGION_SMEM)) {
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

void i915_ttm_gem_object_close(struct drm_gem_object *gem, struct drm_file *file)
{

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

int
i915_ttm_dumb_mmap_offset(struct drm_i915_private *i915,
			  struct drm_file *file,
			  u32 handle,
			  u64 *offset)
{
	struct drm_gem_object *gobj;
	struct i915_ttm_bo *bo;

	gobj = drm_gem_object_lookup(file, handle);
	if (gobj == NULL)
		return -ENOENT;

	bo = ttm_gem_to_i915_bo(gobj);

	*offset = i915_ttm_bo_mmap_offset(bo);
	drm_gem_object_put(gobj);
	return 0;
}

int
i915_ttm_mmap_offset_ioctl(struct drm_i915_private *i915,
			   struct drm_i915_gem_mmap_offset *args,
			   struct drm_file *file)
{
	return i915_ttm_dumb_mmap_offset(i915, file, args->handle, &args->offset);
}


int i915_ttm_create_bo_pages(struct i915_ttm_bo *bo)
{
	struct drm_i915_private *i915 = to_i915_ttm_dev(bo->tbo.bdev);
	/* create pages for LMEM bindings here */
	struct sg_table *st;
	struct scatterlist *sg;
	struct ttm_resource *mem = &bo->tbo.mem;
	struct drm_mm_node *nodes = mem->mm_node;
	unsigned pages = mem->num_pages;
	unsigned int sg_page_sizes;
	int ret;
	st = kmalloc(sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	if (mem->mem_type == I915_TTM_PL_STOLEN) {
		printk(KERN_ERR "stolen create bo pages\n");
		if (sg_alloc_table(st, 1, GFP_KERNEL)) {
			kfree(st);
			return -ENOMEM;
		}

		sg = st->sgl;
		sg->offset = 0;
		sg->length = bo->tbo.num_pages * PAGE_SIZE;

		sg_dma_address(sg) = (dma_addr_t)i915->dsm.start + nodes->start;
		sg_dma_len(sg) = sg->length;


	} else if (mem->mem_type == TTM_PL_TT || mem->mem_type == TTM_PL_SYSTEM) {
		struct ttm_dma_tt *ttm;
		dma_addr_t *pages_addr;
		int i;
		struct scatterlist *sgx;

		ttm = container_of(bo->tbo.ttm, struct ttm_dma_tt, ttm);
		pages_addr = ttm->dma_address;

		sg_alloc_table_from_pages(st, ttm->ttm.pages, ttm->ttm.num_pages, 0,
					  (unsigned long)ttm->ttm.num_pages << PAGE_SHIFT,
					  GFP_KERNEL);

		//TODO scatter list binding for real life
		for_each_sg(st->sgl, sgx, st->nents, i) {
			sgx->dma_address = ttm->dma_address[i];
		}
	} else {
		uint32_t region = mem->mem_type == INTEL_MEMORY_LOCAL;
		if (sg_alloc_table(st, (bo->tbo.num_pages * PAGE_SIZE) >> ilog2(2*1024*1024), GFP_KERNEL)) {
			kfree(st);
			return -ENOMEM;
		}


		sg = st->sgl;
		st->nents = 0;
		sg_page_sizes = 0;

		while (pages) {
			pages -= nodes->size;

			sg_dma_address(sg) = i915->mm.regions[region]->io_start + nodes->start;

			sg_dma_len(sg) = nodes->size;
			sg->length = nodes->size;
			st->nents++;
			++nodes;
		}
		sg_mark_end(sg);
	}

	bo->pages = st;
	bo->page_sizes.sg = sg_page_sizes;
	return 0;
}

int i915_ttm_set_tiling(struct i915_ttm_bo *bo, unsigned int tiling,
			unsigned int stride)
{
	if ((tiling | stride) == bo->tiling_and_stride)
		return 0;

	bo->tiling_and_stride = tiling | stride;
	return 0;
}

/**
 * amdgpu_ttm_alloc_gart - Allocate GART memory for buffer object
 */
int i915_ttm_alloc_gtt(struct ttm_buffer_object *tbo)
{
	struct drm_i915_private *i915 = to_i915_ttm_dev(tbo->bdev);
	struct ttm_operation_ctx ctx = { false, false };
	struct i915_ttm_tt *gtt = (void*)tbo->ttm;
	struct ttm_resource tmp;
	struct ttm_placement placement;
	struct ttm_place placements;
	uint64_t addr, flags;
	int r;

	if (tbo->mem.start != I915_TTM_BO_INVALID_OFFSET)
		return 0;

	/* allocate GART space */
	tmp = tbo->mem;
	tmp.mm_node = NULL;
	placement.num_placement = 1;
	placement.placement = &placements;
	placement.num_busy_placement = 1;
	placement.busy_placement = &placements;
	placements.fpfn = 0;
	placements.lpfn = i915->ggtt.vm.total >> PAGE_SHIFT;
	placements.flags = (tbo->mem.placement & ~TTM_PL_MASK_MEM) |
		TTM_PL_FLAG_TT;

	r = ttm_bo_mem_space(tbo, &placement, &tmp, &ctx);
	if (unlikely(r))
		return r;

	/* Bind pages */
	gtt->offset = (u64)tmp.start << PAGE_SHIFT;
	r = i915_vma_pin(gtt->vma, 0, 0, PIN_MAPPABLE | PIN_OFFSET_FIXED | PIN_GLOBAL | (tmp.start << PAGE_SHIFT));
	if (unlikely(r)) {
		ttm_bo_mem_put(tbo, &tmp);
		return r;
	}

	ttm_bo_mem_put(tbo, &tbo->mem);
	tbo->mem = tmp;

	return 0;
}
