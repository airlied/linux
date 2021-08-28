/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2021 Intel Corporation
 */

#include "xe_execlist.h"

#include "xe_bo.h"
#include "xe_device.h"
#include "xe_engine.h"
#include "xe_mmio.h"
#include "xe_sched_job.h"

#include "../i915/i915_reg.h"
#include "../i915/gt/intel_gpu_commands.h"
#include "../i915/gt/intel_lrc_reg.h"

#define XE_EXECLIST_HANG_LIMIT 1

static void xe_execlist_run(struct xe_execlist *exl)
{
	struct xe_hw_engine *hwe = exl->engine->hwe;
	struct xe_lrc *lrc = &exl->engine->lrc;
	struct xe_device *xe = hwe->xe;
	uint64_t lrc_desc;

	lrc_desc = xe_lrc_descriptor(lrc) | 0x62;

	xe_lrc_regs(lrc)[CTX_RING_TAIL] = lrc->ring_tail;

	xe_mmio_write32(xe, RING_HWS_PGA(hwe->mmio_base).reg,
			xe_bo_ggtt_addr(hwe->hwsp));
	xe_mmio_read32(xe, RING_HWS_PGA(hwe->mmio_base).reg);
	xe_mmio_write32(xe, RING_MODE_GEN7(hwe->mmio_base).reg,
			_MASKED_BIT_ENABLE(GEN11_GFX_DISABLE_LEGACY_MODE));
	xe_mmio_write32(xe, RING_EXECLIST_SQ_CONTENTS(hwe->mmio_base).reg + 0,
			lower_32_bits(lrc_desc));
	xe_mmio_write32(xe, RING_EXECLIST_SQ_CONTENTS(hwe->mmio_base).reg + 4,
			upper_32_bits(lrc_desc));
	xe_mmio_write32(xe, RING_EXECLIST_CONTROL(hwe->mmio_base).reg,
			EL_CTRL_LOAD);
}

static void xe_execlist_port_irq_handler(struct xe_hw_engine *hwe,
					 uint16_t intr_vec)
{
	printk(KERN_INFO "xe_execlist: %s interrupt received: 0x%04x",
	       hwe->name, (unsigned int)intr_vec);
}

struct xe_execlist_port *xe_execlist_port_create(struct xe_device *xe,
						 struct xe_hw_engine *hwe)
{
	struct xe_execlist_port *port;
	int i;

	port = kzalloc(sizeof(*port), GFP_KERNEL);
	if (!port)
		return ERR_PTR(-ENOMEM);

	port->hwe = hwe;

	spin_lock_init(&port->active_lock);
	for (i = 0; i < ARRAY_SIZE(port->active); i++)
		INIT_LIST_HEAD(&port->active[i]);

	hwe->irq_handler = xe_execlist_port_irq_handler;

	return port;
}

void xe_execlist_port_destroy(struct xe_execlist_port *port)
{
	/* Prevent an interrupt while we're destroying */
	spin_lock_irq(&port->hwe->xe->gt_irq_lock);
	port->hwe->irq_handler = NULL;
	spin_unlock_irq(&port->hwe->xe->gt_irq_lock);

	kfree(port);
}

#if 0
static void xe_execlist_port_wake_locked(struct xe_execlist_port *port)
{
}

static void xe_execlist_make_active(struct xe_execlist *exl)
{
	struct xe_execlist_port *port = exl->port;
	enum drm_sched_priority priority = exl->entity.priority;

	XE_BUG_ON(exl->entity.priority == DRM_SCHED_PRIORITY_UNSET);
	XE_BUG_ON(exl->entity.priority < 0);
	XE_BUG_ON(exl->entity.priority >= ARRAY_SIZE(exl->port->active));

	spin_lock(&port->active_lock);

	if (exl->active_priority != priority &&
	    exl->active_priority != DRM_SCHED_PRIORITY_UNSET) {
		/* Priority changed, move it to the right list
		 *
		 * TODO: Force a preempt?
		 */
		list_del(&exl->active_link);
		exl->active_priority = DRM_SCHED_PRIORITY_UNSET;
	}

	if (exl->active_priority == DRM_SCHED_PRIORITY_UNSET)
		list_add_tail(&exl->active_link, &port->active[priority]);

	xe_execlist_port_wake_locked(exl->port);

	spin_unlock(&port->active_lock);
}
#endif

static struct dma_fence *
xe_execlist_run_job(struct drm_sched_job *drm_job)
{
	struct xe_sched_job *job = to_xe_sched_job(drm_job);
	struct xe_execlist *exl = job->engine->execlist;
	struct xe_lrc *lrc = &job->engine->lrc;
	uint32_t dw[10], i = 0;

	dw[i++] = MI_ARB_ON_OFF | MI_ARB_ENABLE;

	dw[i++] = MI_BATCH_BUFFER_START_GEN8 | BIT(8);
	dw[i++] = lower_32_bits(job->user_batch_addr);
	dw[i++] = upper_32_bits(job->user_batch_addr);

	dw[i++] = MI_ARB_ON_OFF | MI_ARB_DISABLE;

	dw[i++] = MI_STORE_DATA_IMM | BIT(22) /* GGTT */ | 2;
	dw[i++] = xe_lrc_seqno_ggtt_addr(lrc);
	dw[i++] = 0;
	dw[i++] = job->fence.seqno;
	dw[i++] = MI_USER_INTERRUPT;

	XE_BUG_ON(i > ARRAY_SIZE(dw));

	xe_lrc_write_ring(lrc, dw, i * sizeof(*dw));
//	xe_execlist_make_active(exl);
	xe_execlist_run(exl);

	return dma_fence_get(&job->fence);
}

static const struct drm_sched_backend_ops drm_sched_ops = {
	.dependency = xe_drm_sched_job_dependency,
	.run_job = xe_execlist_run_job,
	.free_job = xe_drm_sched_job_free,
};

static inline void test(struct xe_execlist *exl)
{
	struct xe_hw_engine *hwe = exl->engine->hwe;
	struct xe_lrc *lrc = &exl->engine->lrc;
	struct xe_device *xe = hwe->xe;
	uint32_t lrc_addr = xe_lrc_ggtt_addr(lrc);
	uint32_t data_addr = lrc_addr + 1024;
	uint32_t *data_p = xe_lrc_pphwsp(lrc) + 1024, data;
	uint64_t desc;
	uint32_t dw[8], i = 0;

	dw[i++] = MI_STORE_DATA_IMM | BIT(22) /* GGTT */ | 2;
	dw[i++] = lower_32_bits(data_addr);
	dw[i++] = upper_32_bits(data_addr);
	dw[i++] = 0xdeadbeef;
	dw[i++] = MI_USER_INTERRUPT;

	xe_lrc_write_ring(lrc, dw, i * sizeof(*dw));

	desc = GEN8_CTX_VALID;
	desc |= INTEL_LEGACY_64B_CONTEXT << GEN8_CTX_ADDRESSING_MODE_SHIFT;
	/* TODO: Priority */
	/* TODO: Privilege */
	if (GRAPHICS_VER(hwe->xe) == 8)
		desc |= GEN8_CTX_L3LLC_COHERENT;

	desc |= (7ull << 37); /* TODO: Context ID */

	desc |= lrc_addr;

	xe_mmio_write32(xe, RING_HWS_PGA(hwe->mmio_base).reg,
			xe_bo_ggtt_addr(hwe->hwsp));
	xe_mmio_read32(xe, RING_HWS_PGA(hwe->mmio_base).reg);
	xe_mmio_write32(xe, RING_MODE_GEN7(hwe->mmio_base).reg,
			_MASKED_BIT_ENABLE(GEN11_GFX_DISABLE_LEGACY_MODE));
	xe_mmio_write32(xe, RING_EXECLIST_SQ_CONTENTS(hwe->mmio_base).reg + 0,
			lower_32_bits(desc));
	xe_mmio_write32(xe, RING_EXECLIST_SQ_CONTENTS(hwe->mmio_base).reg + 4,
			upper_32_bits(desc));
	xe_mmio_write32(xe, RING_EXECLIST_CONTROL(hwe->mmio_base).reg,
			EL_CTRL_LOAD);

	while (!(data = READ_ONCE(*data_p)) && i++ < (1u << 31))
		continue;

	xe_ggtt_printk(&xe->ggtt, KERN_INFO);

	printk(KERN_INFO "Attempted to execute on %s, mmio_base = 0x%05x",
	       hwe->name, hwe->mmio_base);
	printk(KERN_INFO "Execlist descriptor: 0x%08x %08x",
	       upper_32_bits(desc), lower_32_bits(desc));
	printk(KERN_INFO "Data read: 0x%08x", data);
	printk(KERN_INFO "Ring: (head, tail) = (0x%x, 0x%x)",
			 xe_lrc_ring_head(lrc), lrc->ring_tail);
	printk(KERN_INFO "EXECLIST_STATUS = 0x%08x %08x",
	       xe_mmio_read32(xe, RING_EXECLIST_STATUS_HI(hwe->mmio_base).reg),
	       xe_mmio_read32(xe, RING_EXECLIST_STATUS_LO(hwe->mmio_base).reg));
	printk(KERN_INFO "ACTHD = 0x%08x %08x",
	       xe_mmio_read32(xe, RING_ACTHD_UDW(hwe->mmio_base).reg),
	       xe_mmio_read32(xe, RING_ACTHD(hwe->mmio_base).reg));
	printk(KERN_INFO "RING_START = 0x%08x",
	       xe_mmio_read32(xe, RING_START(hwe->mmio_base).reg));
}

struct xe_execlist *xe_execlist_create(struct xe_engine *e)
{
	struct drm_gpu_scheduler *sched;
	struct xe_execlist *exl;
	int err;

	exl = kzalloc(sizeof(*exl), GFP_KERNEL);
	if (!exl)
		return ERR_PTR(-ENOMEM);

	exl->engine = e;

	err = drm_sched_init(&exl->sched, &drm_sched_ops, U32_MAX,
			     XE_SCHED_HANG_LIMIT, XE_SCHED_JOB_TIMEOUT,
			     NULL, NULL, e->hwe->name);
	if (err)
		goto err_free;

	sched = &exl->sched;
	err = drm_sched_entity_init(&exl->entity, DRM_SCHED_PRIORITY_NORMAL,
				    &sched, 1, NULL);
	if (err)
		goto err_sched;

	exl->port = e->hwe->exl_port;
	exl->active_priority = DRM_SCHED_PRIORITY_UNSET;

	return exl;

err_sched:
	drm_sched_fini(&exl->sched);
err_free:
	kfree(exl);
	return ERR_PTR(err);
}

void xe_execlist_destroy(struct xe_execlist *exl)
{
	drm_sched_entity_fini(&exl->entity);
	drm_sched_fini(&exl->sched);
	kfree(exl);
}
