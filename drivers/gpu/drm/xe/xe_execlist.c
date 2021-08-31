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

#define GEN11_SW_CTX_ID \
	GENMASK_ULL(GEN11_SW_CTX_ID_WIDTH + GEN11_SW_CTX_ID_SHIFT - 1, \
		    GEN11_SW_CTX_ID_SHIFT)

static void __start_lrc(struct xe_hw_engine *hwe, struct xe_lrc *lrc,
			uint32_t ctx_id)
{
	struct xe_device *xe = hwe->xe;
	uint64_t lrc_desc;

	printk(KERN_INFO "__start_lrc(%s, 0x%p, %u)\n", hwe->name, lrc, ctx_id);

	lrc_desc = xe_lrc_descriptor(lrc);

	XE_BUG_ON(!FIELD_FIT(GEN11_SW_CTX_ID, ctx_id));
	lrc_desc |= FIELD_PREP(GEN11_SW_CTX_ID, ctx_id);

	xe_lrc_regs(lrc)[CTX_RING_TAIL] = lrc->ring_tail;
	lrc->ring_old_tail = lrc->ring_tail;

	/*
	 * Make sure the context image is complete before we submit it to HW.
	 *
	 * Ostensibly, writes (including the WCB) should be flushed prior to
	 * an uncached write such as our mmio register access, the empirical
	 * evidence (esp. on Braswell) suggests that the WC write into memory
	 * may not be visible to the HW prior to the completion of the UC
	 * register write and that we may begin execution from the context
	 * before its image is complete leading to invalid PD chasing.
	 */
	wmb();

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

static void __xe_execlist_port_start(struct xe_execlist_port *port,
				     struct xe_execlist *exl)
{
	xe_execlist_port_assert_held(port);

	if (port->running_exl != exl || !exl->has_run) {
		port->last_ctx_id++;

		/* 0 is reserved for the kernel context */
		if (port->last_ctx_id > FIELD_MAX(GEN11_SW_CTX_ID))
			port->last_ctx_id = 1;
	}

	__start_lrc(port->hwe, &exl->engine->lrc, port->last_ctx_id);
	port->running_exl = exl;
	exl->has_run = true;
}

static void __xe_execlist_port_idle(struct xe_execlist_port *port)
{
	uint32_t noop[2] = { MI_NOOP, MI_NOOP };

	xe_execlist_port_assert_held(port);

	if (!port->running_exl)
		return;

	printk(KERN_INFO "__xe_execlist_port_idle()");

	xe_lrc_write_ring(&port->hwe->kernel_lrc, noop, sizeof(noop));
	__start_lrc(port->hwe, &port->hwe->kernel_lrc, 0);
	port->running_exl = NULL;
}

static bool xe_execlist_is_idle(struct xe_execlist *exl)
{
	struct xe_lrc *lrc = &exl->engine->lrc;

	return lrc->ring_tail == lrc->ring_old_tail;
}

static void __xe_execlist_port_start_next_active(struct xe_execlist_port *port)
{
	struct xe_execlist *exl = NULL;
	int i;

	xe_execlist_port_assert_held(port);

	for (i = ARRAY_SIZE(port->active) - 1; i >= 0; i--) {
		while (!list_empty(&port->active[i])) {
			exl = list_first_entry(&port->active[i],
					       struct xe_execlist,
					       active_link);
			list_del(&exl->active_link);

			if (xe_execlist_is_idle(exl)) {
				exl->active_priority = DRM_SCHED_PRIORITY_UNSET;
				continue;
			}

			list_add_tail(&exl->active_link, &port->active[i]);
			__xe_execlist_port_start(port, exl);
			return;
		}
	}

	__xe_execlist_port_idle(port);
}

static uint64_t read_execlist_status(struct xe_hw_engine *hwe)
{
	struct xe_device *xe = hwe->xe;
	uint32_t hi, lo;

	lo = xe_mmio_read32(xe, RING_EXECLIST_STATUS_LO(hwe->mmio_base).reg);
	hi = xe_mmio_read32(xe, RING_EXECLIST_STATUS_HI(hwe->mmio_base).reg);

	printk(KERN_INFO "EXECLIST_STATUS = 0x%08x %08x\n", hi, lo);

	return lo | (uint64_t)hi << 32;
}

static void xe_execlist_port_irq_handler_locked(struct xe_hw_engine *hwe)
{
	struct xe_execlist_port *port = hwe->exl_port;
	uint64_t status;

	xe_execlist_port_assert_held(port);

	status = read_execlist_status(hwe);
	if (status & BIT(7))
		return;

	__xe_execlist_port_start_next_active(port);
}

static void xe_execlist_port_irq_handler(struct xe_hw_engine *hwe,
					 uint16_t intr_vec)
{
	struct xe_execlist_port *port = hwe->exl_port;

	spin_lock(&port->lock);
	xe_execlist_port_irq_handler_locked(hwe);
	spin_unlock(&port->lock);
}

static void xe_execlist_port_wake_locked(struct xe_execlist_port *port,
					 enum drm_sched_priority priority)
{
	xe_execlist_port_assert_held(port);

	if (port->running_exl && port->running_exl->active_priority >= priority)
		return;

	__xe_execlist_port_start_next_active(port);
}

static void xe_execlist_make_active(struct xe_execlist *exl)
{
	struct xe_execlist_port *port = exl->port;
	enum drm_sched_priority priority = exl->entity.priority;

	XE_BUG_ON(priority == DRM_SCHED_PRIORITY_UNSET);
	XE_BUG_ON(priority < 0);
	XE_BUG_ON(priority >= ARRAY_SIZE(exl->port->active));

	spin_lock_irq(&port->lock);

	if (exl->active_priority != priority &&
	    exl->active_priority != DRM_SCHED_PRIORITY_UNSET) {
		/* Priority changed, move it to the right list */
		list_del(&exl->active_link);
		exl->active_priority = DRM_SCHED_PRIORITY_UNSET;
	}

	if (exl->active_priority == DRM_SCHED_PRIORITY_UNSET) {
		exl->active_priority = priority;
		list_add_tail(&exl->active_link, &port->active[priority]);
	}

	xe_execlist_port_wake_locked(exl->port, priority);

	spin_unlock_irq(&port->lock);
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

	spin_lock_init(&port->lock);
	for (i = 0; i < ARRAY_SIZE(port->active); i++)
		INIT_LIST_HEAD(&port->active[i]);

	port->last_ctx_id = 1;
	port->running_exl = NULL;

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

#define MAX_JOB_SIZE_DW 16
#define MAX_JOB_SIZE_BYTES (MAX_JOB_SIZE_DW * 4)

static struct dma_fence *
xe_execlist_run_job(struct drm_sched_job *drm_job)
{
	struct xe_sched_job *job = to_xe_sched_job(drm_job);
	struct xe_execlist *exl = job->engine->execlist;
	struct xe_lrc *lrc = &job->engine->lrc;
	uint32_t dw[MAX_JOB_SIZE_DW], i = 0;

	dw[i++] = MI_BATCH_BUFFER_START_GEN8 | BIT(8);
	dw[i++] = lower_32_bits(job->user_batch_addr);
	dw[i++] = upper_32_bits(job->user_batch_addr);

	dw[i++] = MI_STORE_DATA_IMM | BIT(22) /* GGTT */ | 2;
	dw[i++] = xe_lrc_seqno_ggtt_addr(lrc);
	dw[i++] = 0;
	dw[i++] = job->fence.seqno;

	dw[i++] = MI_USER_INTERRUPT;
	dw[i++] = MI_ARB_ON_OFF | MI_ARB_ENABLE;

	XE_BUG_ON(i > ARRAY_SIZE(dw));

	xe_lrc_write_ring(lrc, dw, i * sizeof(*dw));

	xe_execlist_make_active(exl);

	return dma_fence_get(&job->fence);
}

static const struct drm_sched_backend_ops drm_sched_ops = {
	.dependency = xe_drm_sched_job_dependency,
	.run_job = xe_execlist_run_job,
	.free_job = xe_drm_sched_job_free,
};

struct xe_execlist *xe_execlist_create(struct xe_engine *e)
{
	struct drm_gpu_scheduler *sched;
	struct xe_execlist *exl;
	int err;

	exl = kzalloc(sizeof(*exl), GFP_KERNEL);
	if (!exl)
		return ERR_PTR(-ENOMEM);

	exl->engine = e;

	err = drm_sched_init(&exl->sched, &drm_sched_ops,
			     e->lrc.ring_size / MAX_JOB_SIZE_BYTES,
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
	exl->has_run = false;
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
	unsigned long flags;

	spin_lock_irqsave(&exl->port->lock, flags);
	if (WARN_ON(exl->active_priority != DRM_SCHED_PRIORITY_UNSET))
		list_del(&exl->active_link);
	spin_unlock_irqrestore(&exl->port->lock, flags);

	drm_sched_entity_fini(&exl->entity);
	drm_sched_fini(&exl->sched);
	kfree(exl);
}
