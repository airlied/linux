// SPDX-License-Identifier: MIT
/*
 * Copyright © 2020 Intel Corporation
 */
#include <linux/kernel.h>
#include <linux/slab.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_plane.h>
#include <drm/drm_plane_helper.h>

#include "i915_trace.h"
#include "i915_vgpu.h"

#include "intel_atomic.h"
#include "intel_atomic_plane.h"
#include "intel_color.h"
#include "intel_crtc.h"
#include "intel_cursor.h"
#include "intel_display_debugfs.h"
#include "intel_display_types.h"
#include "intel_dpll.h"
#include "intel_dsi.h"
#include "intel_fbc.h"
#include "intel_fdi.h"
#include "intel_fifo_underrun.h"
#include "intel_pipe_crc.h"
#include "intel_pm.h"
#include "intel_psr.h"
#include "intel_sideband.h"
#include "intel_sprite.h"
#include "intel_vdsc.h"
#include "i9xx_plane.h"
#include "skl_universal_plane.h"

static void i9xx_crtc_clock_get(struct intel_crtc *crtc,
				struct intel_crtc_state *pipe_config);
static void ilk_get_pfit_config(struct intel_crtc_state *crtc_state);

static void assert_vblank_disabled(struct drm_crtc *crtc)
{
	if (I915_STATE_WARN_ON(drm_crtc_vblank_get(crtc) == 0))
		drm_crtc_vblank_put(crtc);
}

u32 intel_crtc_get_vblank_counter(struct intel_crtc *crtc)
{
	struct drm_device *dev = crtc->base.dev;
	struct drm_vblank_crtc *vblank = &dev->vblank[drm_crtc_index(&crtc->base)];

	if (!vblank->max_vblank_count)
		return (u32)drm_crtc_accurate_vblank_count(&crtc->base);

	return crtc->base.funcs->get_vblank_counter(&crtc->base);
}

u32 intel_crtc_max_vblank_count(const struct intel_crtc_state *crtc_state)
{
	struct drm_i915_private *dev_priv = to_i915(crtc_state->uapi.crtc->dev);
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	u32 mode_flags = crtc->mode_flags;

	/*
	 * From Gen 11, In case of dsi cmd mode, frame counter wouldnt
	 * have updated at the beginning of TE, if we want to use
	 * the hw counter, then we would find it updated in only
	 * the next TE, hence switching to sw counter.
	 */
	if (mode_flags & (I915_MODE_FLAG_DSI_USE_TE0 | I915_MODE_FLAG_DSI_USE_TE1))
		return 0;

	/*
	 * On i965gm the hardware frame counter reads
	 * zero when the TV encoder is enabled :(
	 */
	if (IS_I965GM(dev_priv) &&
	    (crtc_state->output_types & BIT(INTEL_OUTPUT_TVOUT)))
		return 0;

	if (INTEL_GEN(dev_priv) >= 5 || IS_G4X(dev_priv))
		return 0xffffffff; /* full 32 bit counter */
	else if (INTEL_GEN(dev_priv) >= 3)
		return 0xffffff; /* only 24 bits of frame count */
	else
		return 0; /* Gen2 doesn't have a hardware frame counter */
}

void intel_crtc_vblank_on(const struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);

	assert_vblank_disabled(&crtc->base);
	drm_crtc_set_max_vblank_count(&crtc->base,
				      intel_crtc_max_vblank_count(crtc_state));
	drm_crtc_vblank_on(&crtc->base);
}

void intel_crtc_vblank_off(const struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);

	drm_crtc_vblank_off(&crtc->base);
	assert_vblank_disabled(&crtc->base);
}

void intel_encoders_pre_pll_enable(struct intel_atomic_state *state,
				   struct intel_crtc *crtc)
{
	const struct intel_crtc_state *crtc_state =
		intel_atomic_get_new_crtc_state(state, crtc);
	const struct drm_connector_state *conn_state;
	struct drm_connector *conn;
	int i;

	for_each_new_connector_in_state(&state->base, conn, conn_state, i) {
		struct intel_encoder *encoder =
			to_intel_encoder(conn_state->best_encoder);

		if (conn_state->crtc != &crtc->base)
			continue;

		if (encoder->pre_pll_enable)
			encoder->pre_pll_enable(state, encoder,
						crtc_state, conn_state);
	}
}

static void intel_encoders_pre_enable(struct intel_atomic_state *state,
				      struct intel_crtc *crtc)
{
	const struct intel_crtc_state *crtc_state =
		intel_atomic_get_new_crtc_state(state, crtc);
	const struct drm_connector_state *conn_state;
	struct drm_connector *conn;
	int i;

	for_each_new_connector_in_state(&state->base, conn, conn_state, i) {
		struct intel_encoder *encoder =
			to_intel_encoder(conn_state->best_encoder);

		if (conn_state->crtc != &crtc->base)
			continue;

		if (encoder->pre_enable)
			encoder->pre_enable(state, encoder,
					    crtc_state, conn_state);
	}
}

static void intel_encoders_enable(struct intel_atomic_state *state,
				  struct intel_crtc *crtc)
{
	const struct intel_crtc_state *crtc_state =
		intel_atomic_get_new_crtc_state(state, crtc);
	const struct drm_connector_state *conn_state;
	struct drm_connector *conn;
	int i;

	for_each_new_connector_in_state(&state->base, conn, conn_state, i) {
		struct intel_encoder *encoder =
			to_intel_encoder(conn_state->best_encoder);

		if (conn_state->crtc != &crtc->base)
			continue;

		if (encoder->enable)
			encoder->enable(state, encoder,
					crtc_state, conn_state);
		intel_opregion_notify_encoder(encoder, true);
	}
}

static void intel_encoders_disable(struct intel_atomic_state *state,
				   struct intel_crtc *crtc)
{
	const struct intel_crtc_state *old_crtc_state =
		intel_atomic_get_old_crtc_state(state, crtc);
	const struct drm_connector_state *old_conn_state;
	struct drm_connector *conn;
	int i;

	for_each_old_connector_in_state(&state->base, conn, old_conn_state, i) {
		struct intel_encoder *encoder =
			to_intel_encoder(old_conn_state->best_encoder);

		if (old_conn_state->crtc != &crtc->base)
			continue;

		intel_opregion_notify_encoder(encoder, false);
		if (encoder->disable)
			encoder->disable(state, encoder,
					 old_crtc_state, old_conn_state);
	}
}

static void intel_encoders_post_disable(struct intel_atomic_state *state,
					struct intel_crtc *crtc)
{
	const struct intel_crtc_state *old_crtc_state =
		intel_atomic_get_old_crtc_state(state, crtc);
	const struct drm_connector_state *old_conn_state;
	struct drm_connector *conn;
	int i;

	for_each_old_connector_in_state(&state->base, conn, old_conn_state, i) {
		struct intel_encoder *encoder =
			to_intel_encoder(old_conn_state->best_encoder);

		if (old_conn_state->crtc != &crtc->base)
			continue;

		if (encoder->post_disable)
			encoder->post_disable(state, encoder,
					      old_crtc_state, old_conn_state);
	}
}

static void intel_encoders_post_pll_disable(struct intel_atomic_state *state,
					    struct intel_crtc *crtc)
{
	const struct intel_crtc_state *old_crtc_state =
		intel_atomic_get_old_crtc_state(state, crtc);
	const struct drm_connector_state *old_conn_state;
	struct drm_connector *conn;
	int i;

	for_each_old_connector_in_state(&state->base, conn, old_conn_state, i) {
		struct intel_encoder *encoder =
			to_intel_encoder(old_conn_state->best_encoder);

		if (old_conn_state->crtc != &crtc->base)
			continue;

		if (encoder->post_pll_disable)
			encoder->post_pll_disable(state, encoder,
						  old_crtc_state, old_conn_state);
	}
}

void intel_encoders_update_pipe(struct intel_atomic_state *state,
				struct intel_crtc *crtc)
{
	const struct intel_crtc_state *crtc_state =
		intel_atomic_get_new_crtc_state(state, crtc);
	const struct drm_connector_state *conn_state;
	struct drm_connector *conn;
	int i;

	for_each_new_connector_in_state(&state->base, conn, conn_state, i) {
		struct intel_encoder *encoder =
			to_intel_encoder(conn_state->best_encoder);

		if (conn_state->crtc != &crtc->base)
			continue;

		if (encoder->update_pipe)
			encoder->update_pipe(state, encoder,
					     crtc_state, conn_state);
	}
}
static void intel_disable_primary_plane(const struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct intel_plane *plane = to_intel_plane(crtc->base.primary);

	plane->disable_plane(plane, crtc_state);
}
static bool pipe_scanline_is_moving(struct drm_i915_private *dev_priv,
				    enum pipe pipe)
{
	i915_reg_t reg = PIPEDSL(pipe);
	u32 line1, line2;
	u32 line_mask;

	if (IS_GEN(dev_priv, 2))
		line_mask = DSL_LINEMASK_GEN2;
	else
		line_mask = DSL_LINEMASK_GEN3;

	line1 = intel_de_read(dev_priv, reg) & line_mask;
	msleep(5);
	line2 = intel_de_read(dev_priv, reg) & line_mask;

	return line1 != line2;
}

static void wait_for_pipe_scanline_moving(struct intel_crtc *crtc, bool state)
{
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	enum pipe pipe = crtc->pipe;

	/* Wait for the display line to settle/start moving */
	if (wait_for(pipe_scanline_is_moving(dev_priv, pipe) == state, 100))
		drm_err(&dev_priv->drm,
			"pipe %c scanline %s wait timed out\n",
			pipe_name(pipe), onoff(state));
}

void intel_wait_for_pipe_scanline_stopped(struct intel_crtc *crtc)
{
	wait_for_pipe_scanline_moving(crtc, false);
}

void intel_wait_for_pipe_scanline_moving(struct intel_crtc *crtc)
{
	wait_for_pipe_scanline_moving(crtc, true);
}

static void
intel_wait_for_pipe_off(const struct intel_crtc_state *old_crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(old_crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);

	if (INTEL_GEN(dev_priv) >= 4) {
		enum transcoder cpu_transcoder = old_crtc_state->cpu_transcoder;
		i915_reg_t reg = PIPECONF(cpu_transcoder);

		/* Wait for the Pipe State to go off */
		if (intel_de_wait_for_clear(dev_priv, reg,
					    I965_PIPECONF_ACTIVE, 100))
			drm_WARN(&dev_priv->drm, 1,
				 "pipe_off wait timed out\n");
	} else {
		intel_wait_for_pipe_scanline_stopped(crtc);
	}
}

struct intel_crtc_state *intel_crtc_state_alloc(struct intel_crtc *crtc)
{
	struct intel_crtc_state *crtc_state;

	crtc_state = kmalloc(sizeof(*crtc_state), GFP_KERNEL);

	if (crtc_state)
		intel_crtc_state_reset(crtc_state, crtc);

	return crtc_state;
}

void intel_crtc_state_reset(struct intel_crtc_state *crtc_state,
			    struct intel_crtc *crtc)
{
	memset(crtc_state, 0, sizeof(*crtc_state));

	__drm_atomic_helper_crtc_state_reset(&crtc_state->uapi, &crtc->base);

	crtc_state->cpu_transcoder = INVALID_TRANSCODER;
	crtc_state->master_transcoder = INVALID_TRANSCODER;
	crtc_state->hsw_workaround_pipe = INVALID_PIPE;
	crtc_state->output_format = INTEL_OUTPUT_FORMAT_INVALID;
	crtc_state->scaler_state.scaler_id = -1;
	crtc_state->mst_master_transcoder = INVALID_TRANSCODER;
}

static struct intel_crtc *intel_crtc_alloc(void)
{
	struct intel_crtc_state *crtc_state;
	struct intel_crtc *crtc;

	crtc = kzalloc(sizeof(*crtc), GFP_KERNEL);
	if (!crtc)
		return ERR_PTR(-ENOMEM);

	crtc_state = intel_crtc_state_alloc(crtc);
	if (!crtc_state) {
		kfree(crtc);
		return ERR_PTR(-ENOMEM);
	}

	crtc->base.state = &crtc_state->uapi;
	crtc->config = crtc_state;

	return crtc;
}

static void intel_crtc_free(struct intel_crtc *crtc)
{
	intel_crtc_destroy_state(&crtc->base, crtc->base.state);
	kfree(crtc);
}

static void intel_crtc_destroy(struct drm_crtc *crtc)
{
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);

	drm_crtc_cleanup(crtc);
	kfree(intel_crtc);
}

static int intel_crtc_late_register(struct drm_crtc *crtc)
{
	intel_crtc_debugfs_add(crtc);
	return 0;
}

#define INTEL_CRTC_FUNCS \
	.gamma_set = drm_atomic_helper_legacy_gamma_set, \
	.set_config = drm_atomic_helper_set_config, \
	.destroy = intel_crtc_destroy, \
	.page_flip = drm_atomic_helper_page_flip, \
	.atomic_duplicate_state = intel_crtc_duplicate_state, \
	.atomic_destroy_state = intel_crtc_destroy_state, \
	.set_crc_source = intel_crtc_set_crc_source, \
	.verify_crc_source = intel_crtc_verify_crc_source, \
	.get_crc_sources = intel_crtc_get_crc_sources, \
	.late_register = intel_crtc_late_register

static const struct drm_crtc_funcs bdw_crtc_funcs = {
	INTEL_CRTC_FUNCS,

	.get_vblank_counter = g4x_get_vblank_counter,
	.enable_vblank = bdw_enable_vblank,
	.disable_vblank = bdw_disable_vblank,
	.get_vblank_timestamp = intel_crtc_get_vblank_timestamp,
};

static const struct drm_crtc_funcs ilk_crtc_funcs = {
	INTEL_CRTC_FUNCS,

	.get_vblank_counter = g4x_get_vblank_counter,
	.enable_vblank = ilk_enable_vblank,
	.disable_vblank = ilk_disable_vblank,
	.get_vblank_timestamp = intel_crtc_get_vblank_timestamp,
};

static const struct drm_crtc_funcs g4x_crtc_funcs = {
	INTEL_CRTC_FUNCS,

	.get_vblank_counter = g4x_get_vblank_counter,
	.enable_vblank = i965_enable_vblank,
	.disable_vblank = i965_disable_vblank,
	.get_vblank_timestamp = intel_crtc_get_vblank_timestamp,
};

static const struct drm_crtc_funcs i965_crtc_funcs = {
	INTEL_CRTC_FUNCS,

	.get_vblank_counter = i915_get_vblank_counter,
	.enable_vblank = i965_enable_vblank,
	.disable_vblank = i965_disable_vblank,
	.get_vblank_timestamp = intel_crtc_get_vblank_timestamp,
};

static const struct drm_crtc_funcs i915gm_crtc_funcs = {
	INTEL_CRTC_FUNCS,

	.get_vblank_counter = i915_get_vblank_counter,
	.enable_vblank = i915gm_enable_vblank,
	.disable_vblank = i915gm_disable_vblank,
	.get_vblank_timestamp = intel_crtc_get_vblank_timestamp,
};

static const struct drm_crtc_funcs i915_crtc_funcs = {
	INTEL_CRTC_FUNCS,

	.get_vblank_counter = i915_get_vblank_counter,
	.enable_vblank = i8xx_enable_vblank,
	.disable_vblank = i8xx_disable_vblank,
	.get_vblank_timestamp = intel_crtc_get_vblank_timestamp,
};

static const struct drm_crtc_funcs i8xx_crtc_funcs = {
	INTEL_CRTC_FUNCS,

	/* no hw vblank counter */
	.enable_vblank = i8xx_enable_vblank,
	.disable_vblank = i8xx_disable_vblank,
	.get_vblank_timestamp = intel_crtc_get_vblank_timestamp,
};

int intel_crtc_init(struct drm_i915_private *dev_priv, enum pipe pipe)
{
	struct intel_plane *primary, *cursor;
	const struct drm_crtc_funcs *funcs;
	struct intel_crtc *crtc;
	int sprite, ret;

	crtc = intel_crtc_alloc();
	if (IS_ERR(crtc))
		return PTR_ERR(crtc);

	crtc->pipe = pipe;
	crtc->num_scalers = RUNTIME_INFO(dev_priv)->num_scalers[pipe];

	if (INTEL_GEN(dev_priv) >= 9)
		primary = skl_universal_plane_create(dev_priv, pipe,
						     PLANE_PRIMARY);
	else
		primary = intel_primary_plane_create(dev_priv, pipe);
	if (IS_ERR(primary)) {
		ret = PTR_ERR(primary);
		goto fail;
	}
	crtc->plane_ids_mask |= BIT(primary->id);

	for_each_sprite(dev_priv, pipe, sprite) {
		struct intel_plane *plane;

		if (INTEL_GEN(dev_priv) >= 9)
			plane = skl_universal_plane_create(dev_priv, pipe,
							   PLANE_SPRITE0 + sprite);
		else
			plane = intel_sprite_plane_create(dev_priv, pipe, sprite);
		if (IS_ERR(plane)) {
			ret = PTR_ERR(plane);
			goto fail;
		}
		crtc->plane_ids_mask |= BIT(plane->id);
	}

	cursor = intel_cursor_plane_create(dev_priv, pipe);
	if (IS_ERR(cursor)) {
		ret = PTR_ERR(cursor);
		goto fail;
	}
	crtc->plane_ids_mask |= BIT(cursor->id);

	if (HAS_GMCH(dev_priv)) {
		if (IS_CHERRYVIEW(dev_priv) ||
		    IS_VALLEYVIEW(dev_priv) || IS_G4X(dev_priv))
			funcs = &g4x_crtc_funcs;
		else if (IS_GEN(dev_priv, 4))
			funcs = &i965_crtc_funcs;
		else if (IS_I945GM(dev_priv) || IS_I915GM(dev_priv))
			funcs = &i915gm_crtc_funcs;
		else if (IS_GEN(dev_priv, 3))
			funcs = &i915_crtc_funcs;
		else
			funcs = &i8xx_crtc_funcs;
	} else {
		if (INTEL_GEN(dev_priv) >= 8)
			funcs = &bdw_crtc_funcs;
		else
			funcs = &ilk_crtc_funcs;
	}

	ret = drm_crtc_init_with_planes(&dev_priv->drm, &crtc->base,
					&primary->base, &cursor->base,
					funcs, "pipe %c", pipe_name(pipe));
	if (ret)
		goto fail;

	BUG_ON(pipe >= ARRAY_SIZE(dev_priv->pipe_to_crtc_mapping) ||
	       dev_priv->pipe_to_crtc_mapping[pipe] != NULL);
	dev_priv->pipe_to_crtc_mapping[pipe] = crtc;

	if (INTEL_GEN(dev_priv) < 9) {
		enum i9xx_plane_id i9xx_plane = primary->i9xx_plane;

		BUG_ON(i9xx_plane >= ARRAY_SIZE(dev_priv->plane_to_crtc_mapping) ||
		       dev_priv->plane_to_crtc_mapping[i9xx_plane] != NULL);
		dev_priv->plane_to_crtc_mapping[i9xx_plane] = crtc;
	}

	if (INTEL_GEN(dev_priv) >= 10)
		drm_crtc_create_scaling_filter_property(&crtc->base,
						BIT(DRM_SCALING_FILTER_DEFAULT) |
						BIT(DRM_SCALING_FILTER_NEAREST_NEIGHBOR));

	intel_color_init(crtc);

	intel_crtc_crc_init(crtc);

	drm_WARN_ON(&dev_priv->drm, drm_crtc_index(&crtc->base) != crtc->pipe);

	return 0;

fail:
	intel_crtc_free(crtc);

	return ret;
}

int intel_usecs_to_scanlines(const struct drm_display_mode *adjusted_mode,
			     int usecs)
{
	/* paranoia */
	if (!adjusted_mode->crtc_htotal)
		return 1;

	return DIV_ROUND_UP(usecs * adjusted_mode->crtc_clock,
			    1000 * adjusted_mode->crtc_htotal);
}

/**
 * intel_pipe_update_start() - start update of a set of display registers
 * @new_crtc_state: the new crtc state
 *
 * Mark the start of an update to pipe registers that should be updated
 * atomically regarding vblank. If the next vblank will happens within
 * the next 100 us, this function waits until the vblank passes.
 *
 * After a successful call to this function, interrupts will be disabled
 * until a subsequent call to intel_pipe_update_end(). That is done to
 * avoid random delays.
 */
void intel_pipe_update_start(const struct intel_crtc_state *new_crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(new_crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	const struct drm_display_mode *adjusted_mode = &new_crtc_state->hw.adjusted_mode;
	long timeout = msecs_to_jiffies_timeout(1);
	int scanline, min, max, vblank_start;
	wait_queue_head_t *wq = drm_crtc_vblank_waitqueue(&crtc->base);
	bool need_vlv_dsi_wa = (IS_VALLEYVIEW(dev_priv) || IS_CHERRYVIEW(dev_priv)) &&
		intel_crtc_has_type(new_crtc_state, INTEL_OUTPUT_DSI);
	DEFINE_WAIT(wait);
	u32 psr_status;

	if (new_crtc_state->uapi.async_flip)
		return;

	vblank_start = adjusted_mode->crtc_vblank_start;
	if (adjusted_mode->flags & DRM_MODE_FLAG_INTERLACE)
		vblank_start = DIV_ROUND_UP(vblank_start, 2);

	/* FIXME needs to be calibrated sensibly */
	min = vblank_start - intel_usecs_to_scanlines(adjusted_mode,
						      VBLANK_EVASION_TIME_US);
	max = vblank_start - 1;

	if (min <= 0 || max <= 0)
		goto irq_disable;

	if (drm_WARN_ON(&dev_priv->drm, drm_crtc_vblank_get(&crtc->base)))
		goto irq_disable;

	/*
	 * Wait for psr to idle out after enabling the VBL interrupts
	 * VBL interrupts will start the PSR exit and prevent a PSR
	 * re-entry as well.
	 */
	if (intel_psr_wait_for_idle(new_crtc_state, &psr_status))
		drm_err(&dev_priv->drm,
			"PSR idle timed out 0x%x, atomic update may fail\n",
			psr_status);

	local_irq_disable();

	crtc->debug.min_vbl = min;
	crtc->debug.max_vbl = max;
	trace_intel_pipe_update_start(crtc);

	for (;;) {
		/*
		 * prepare_to_wait() has a memory barrier, which guarantees
		 * other CPUs can see the task state update by the time we
		 * read the scanline.
		 */
		prepare_to_wait(wq, &wait, TASK_UNINTERRUPTIBLE);

		scanline = intel_get_crtc_scanline(crtc);
		if (scanline < min || scanline > max)
			break;

		if (!timeout) {
			drm_err(&dev_priv->drm,
				"Potential atomic update failure on pipe %c\n",
				pipe_name(crtc->pipe));
			break;
		}

		local_irq_enable();

		timeout = schedule_timeout(timeout);

		local_irq_disable();
	}

	finish_wait(wq, &wait);

	drm_crtc_vblank_put(&crtc->base);

	/*
	 * On VLV/CHV DSI the scanline counter would appear to
	 * increment approx. 1/3 of a scanline before start of vblank.
	 * The registers still get latched at start of vblank however.
	 * This means we must not write any registers on the first
	 * line of vblank (since not the whole line is actually in
	 * vblank). And unfortunately we can't use the interrupt to
	 * wait here since it will fire too soon. We could use the
	 * frame start interrupt instead since it will fire after the
	 * critical scanline, but that would require more changes
	 * in the interrupt code. So for now we'll just do the nasty
	 * thing and poll for the bad scanline to pass us by.
	 *
	 * FIXME figure out if BXT+ DSI suffers from this as well
	 */
	while (need_vlv_dsi_wa && scanline == vblank_start)
		scanline = intel_get_crtc_scanline(crtc);

	crtc->debug.scanline_start = scanline;
	crtc->debug.start_vbl_time = ktime_get();
	crtc->debug.start_vbl_count = intel_crtc_get_vblank_counter(crtc);

	trace_intel_pipe_update_vblank_evaded(crtc);
	return;

irq_disable:
	local_irq_disable();
}

#if IS_ENABLED(CONFIG_DRM_I915_DEBUG_VBLANK_EVADE)
static void dbg_vblank_evade(struct intel_crtc *crtc, ktime_t end)
{
	u64 delta = ktime_to_ns(ktime_sub(end, crtc->debug.start_vbl_time));
	unsigned int h;

	h = ilog2(delta >> 9);
	if (h >= ARRAY_SIZE(crtc->debug.vbl.times))
		h = ARRAY_SIZE(crtc->debug.vbl.times) - 1;
	crtc->debug.vbl.times[h]++;

	crtc->debug.vbl.sum += delta;
	if (!crtc->debug.vbl.min || delta < crtc->debug.vbl.min)
		crtc->debug.vbl.min = delta;
	if (delta > crtc->debug.vbl.max)
		crtc->debug.vbl.max = delta;

	if (delta > 1000 * VBLANK_EVASION_TIME_US) {
		drm_dbg_kms(crtc->base.dev,
			    "Atomic update on pipe (%c) took %lld us, max time under evasion is %u us\n",
			    pipe_name(crtc->pipe),
			    div_u64(delta, 1000),
			    VBLANK_EVASION_TIME_US);
		crtc->debug.vbl.over++;
	}
}
#else
static void dbg_vblank_evade(struct intel_crtc *crtc, ktime_t end) {}
#endif

/**
 * intel_pipe_update_end() - end update of a set of display registers
 * @new_crtc_state: the new crtc state
 *
 * Mark the end of an update started with intel_pipe_update_start(). This
 * re-enables interrupts and verifies the update was actually completed
 * before a vblank.
 */
void intel_pipe_update_end(struct intel_crtc_state *new_crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(new_crtc_state->uapi.crtc);
	enum pipe pipe = crtc->pipe;
	int scanline_end = intel_get_crtc_scanline(crtc);
	u32 end_vbl_count = intel_crtc_get_vblank_counter(crtc);
	ktime_t end_vbl_time = ktime_get();
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);

	if (new_crtc_state->uapi.async_flip)
		return;

	trace_intel_pipe_update_end(crtc, end_vbl_count, scanline_end);

	/*
	 * Incase of mipi dsi command mode, we need to set frame update
	 * request for every commit.
	 */
	if (INTEL_GEN(dev_priv) >= 11 &&
	    intel_crtc_has_type(new_crtc_state, INTEL_OUTPUT_DSI))
		icl_dsi_frame_update(new_crtc_state);

	/* We're still in the vblank-evade critical section, this can't race.
	 * Would be slightly nice to just grab the vblank count and arm the
	 * event outside of the critical section - the spinlock might spin for a
	 * while ... */
	if (new_crtc_state->uapi.event) {
		drm_WARN_ON(&dev_priv->drm,
			    drm_crtc_vblank_get(&crtc->base) != 0);

		spin_lock(&crtc->base.dev->event_lock);
		drm_crtc_arm_vblank_event(&crtc->base,
					  new_crtc_state->uapi.event);
		spin_unlock(&crtc->base.dev->event_lock);

		new_crtc_state->uapi.event = NULL;
	}

	local_irq_enable();

	if (intel_vgpu_active(dev_priv))
		return;

	if (crtc->debug.start_vbl_count &&
	    crtc->debug.start_vbl_count != end_vbl_count) {
		drm_err(&dev_priv->drm,
			"Atomic update failure on pipe %c (start=%u end=%u) time %lld us, min %d, max %d, scanline start %d, end %d\n",
			pipe_name(pipe), crtc->debug.start_vbl_count,
			end_vbl_count,
			ktime_us_delta(end_vbl_time,
				       crtc->debug.start_vbl_time),
			crtc->debug.min_vbl, crtc->debug.max_vbl,
			crtc->debug.scanline_start, scanline_end);
	}

	dbg_vblank_evade(crtc, end_vbl_time);
}

void
intel_crtc_update_active_timings(const struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	const struct drm_display_mode *adjusted_mode =
		&crtc_state->hw.adjusted_mode;

	drm_calc_timestamping_constants(&crtc->base, adjusted_mode);

	crtc->mode_flags = crtc_state->mode_flags;

	/*
	 * The scanline counter increments at the leading edge of hsync.
	 *
	 * On most platforms it starts counting from vtotal-1 on the
	 * first active line. That means the scanline counter value is
	 * always one less than what we would expect. Ie. just after
	 * start of vblank, which also occurs at start of hsync (on the
	 * last active line), the scanline counter will read vblank_start-1.
	 *
	 * On gen2 the scanline counter starts counting from 1 instead
	 * of vtotal-1, so we have to subtract one (or rather add vtotal-1
	 * to keep the value positive), instead of adding one.
	 *
	 * On HSW+ the behaviour of the scanline counter depends on the output
	 * type. For DP ports it behaves like most other platforms, but on HDMI
	 * there's an extra 1 line difference. So we need to add two instead of
	 * one to the value.
	 *
	 * On VLV/CHV DSI the scanline counter would appear to increment
	 * approx. 1/3 of a scanline before start of vblank. Unfortunately
	 * that means we can't tell whether we're in vblank or not while
	 * we're on that particular line. We must still set scanline_offset
	 * to 1 so that the vblank timestamps come out correct when we query
	 * the scanline counter from within the vblank interrupt handler.
	 * However if queried just before the start of vblank we'll get an
	 * answer that's slightly in the future.
	 */
	if (IS_GEN(dev_priv, 2)) {
		int vtotal;

		vtotal = adjusted_mode->crtc_vtotal;
		if (adjusted_mode->flags & DRM_MODE_FLAG_INTERLACE)
			vtotal /= 2;

		crtc->scanline_offset = vtotal - 1;
	} else if (HAS_DDI(dev_priv) &&
		   intel_crtc_has_type(crtc_state, INTEL_OUTPUT_HDMI)) {
		crtc->scanline_offset = 2;
	} else {
		crtc->scanline_offset = 1;
	}
}

static void intel_set_transcoder_timings(const struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	enum pipe pipe = crtc->pipe;
	enum transcoder cpu_transcoder = crtc_state->cpu_transcoder;
	const struct drm_display_mode *adjusted_mode = &crtc_state->hw.adjusted_mode;
	u32 crtc_vtotal, crtc_vblank_end;
	int vsyncshift = 0;

	/* We need to be careful not to changed the adjusted mode, for otherwise
	 * the hw state checker will get angry at the mismatch. */
	crtc_vtotal = adjusted_mode->crtc_vtotal;
	crtc_vblank_end = adjusted_mode->crtc_vblank_end;

	if (adjusted_mode->flags & DRM_MODE_FLAG_INTERLACE) {
		/* the chip adds 2 halflines automatically */
		crtc_vtotal -= 1;
		crtc_vblank_end -= 1;

		if (intel_crtc_has_type(crtc_state, INTEL_OUTPUT_SDVO))
			vsyncshift = (adjusted_mode->crtc_htotal - 1) / 2;
		else
			vsyncshift = adjusted_mode->crtc_hsync_start -
				adjusted_mode->crtc_htotal / 2;
		if (vsyncshift < 0)
			vsyncshift += adjusted_mode->crtc_htotal;
	}

	if (INTEL_GEN(dev_priv) > 3)
		intel_de_write(dev_priv, VSYNCSHIFT(cpu_transcoder),
		               vsyncshift);

	intel_de_write(dev_priv, HTOTAL(cpu_transcoder),
		       (adjusted_mode->crtc_hdisplay - 1) | ((adjusted_mode->crtc_htotal - 1) << 16));
	intel_de_write(dev_priv, HBLANK(cpu_transcoder),
		       (adjusted_mode->crtc_hblank_start - 1) | ((adjusted_mode->crtc_hblank_end - 1) << 16));
	intel_de_write(dev_priv, HSYNC(cpu_transcoder),
		       (adjusted_mode->crtc_hsync_start - 1) | ((adjusted_mode->crtc_hsync_end - 1) << 16));

	intel_de_write(dev_priv, VTOTAL(cpu_transcoder),
		       (adjusted_mode->crtc_vdisplay - 1) | ((crtc_vtotal - 1) << 16));
	intel_de_write(dev_priv, VBLANK(cpu_transcoder),
		       (adjusted_mode->crtc_vblank_start - 1) | ((crtc_vblank_end - 1) << 16));
	intel_de_write(dev_priv, VSYNC(cpu_transcoder),
		       (adjusted_mode->crtc_vsync_start - 1) | ((adjusted_mode->crtc_vsync_end - 1) << 16));

	/* Workaround: when the EDP input selection is B, the VTOTAL_B must be
	 * programmed with the VTOTAL_EDP value. Same for VTOTAL_C. This is
	 * documented on the DDI_FUNC_CTL register description, EDP Input Select
	 * bits. */
	if (IS_HASWELL(dev_priv) && cpu_transcoder == TRANSCODER_EDP &&
	    (pipe == PIPE_B || pipe == PIPE_C))
		intel_de_write(dev_priv, VTOTAL(pipe),
		               intel_de_read(dev_priv, VTOTAL(cpu_transcoder)));

}

static void intel_get_pipe_src_size(struct intel_crtc *crtc,
				    struct intel_crtc_state *pipe_config)
{
	struct drm_device *dev = crtc->base.dev;
	struct drm_i915_private *dev_priv = to_i915(dev);
	u32 tmp;

	tmp = intel_de_read(dev_priv, PIPESRC(crtc->pipe));
	pipe_config->pipe_src_h = (tmp & 0xffff) + 1;
	pipe_config->pipe_src_w = ((tmp >> 16) & 0xffff) + 1;
}

static void intel_set_pipe_src_size(const struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	enum pipe pipe = crtc->pipe;

	/* pipesrc controls the size that is scaled from, which should
	 * always be the user's requested size.
	 */
	intel_de_write(dev_priv, PIPESRC(pipe),
		       ((crtc_state->pipe_src_w - 1) << 16) | (crtc_state->pipe_src_h - 1));
}

static bool intel_pipe_is_interlaced(const struct intel_crtc_state *crtc_state)
{
	struct drm_i915_private *dev_priv = to_i915(crtc_state->uapi.crtc->dev);
	enum transcoder cpu_transcoder = crtc_state->cpu_transcoder;

	if (IS_GEN(dev_priv, 2))
		return false;

	if (INTEL_GEN(dev_priv) >= 9 ||
	    IS_BROADWELL(dev_priv) || IS_HASWELL(dev_priv))
		return intel_de_read(dev_priv, PIPECONF(cpu_transcoder)) & PIPECONF_INTERLACE_MASK_HSW;
	else
		return intel_de_read(dev_priv, PIPECONF(cpu_transcoder)) & PIPECONF_INTERLACE_MASK;
}

static void intel_get_transcoder_timings(struct intel_crtc *crtc,
					 struct intel_crtc_state *pipe_config)
{
	struct drm_device *dev = crtc->base.dev;
	struct drm_i915_private *dev_priv = to_i915(dev);
	enum transcoder cpu_transcoder = pipe_config->cpu_transcoder;
	u32 tmp;

	tmp = intel_de_read(dev_priv, HTOTAL(cpu_transcoder));
	pipe_config->hw.adjusted_mode.crtc_hdisplay = (tmp & 0xffff) + 1;
	pipe_config->hw.adjusted_mode.crtc_htotal = ((tmp >> 16) & 0xffff) + 1;

	if (!transcoder_is_dsi(cpu_transcoder)) {
		tmp = intel_de_read(dev_priv, HBLANK(cpu_transcoder));
		pipe_config->hw.adjusted_mode.crtc_hblank_start =
							(tmp & 0xffff) + 1;
		pipe_config->hw.adjusted_mode.crtc_hblank_end =
						((tmp >> 16) & 0xffff) + 1;
	}
	tmp = intel_de_read(dev_priv, HSYNC(cpu_transcoder));
	pipe_config->hw.adjusted_mode.crtc_hsync_start = (tmp & 0xffff) + 1;
	pipe_config->hw.adjusted_mode.crtc_hsync_end = ((tmp >> 16) & 0xffff) + 1;

	tmp = intel_de_read(dev_priv, VTOTAL(cpu_transcoder));
	pipe_config->hw.adjusted_mode.crtc_vdisplay = (tmp & 0xffff) + 1;
	pipe_config->hw.adjusted_mode.crtc_vtotal = ((tmp >> 16) & 0xffff) + 1;

	if (!transcoder_is_dsi(cpu_transcoder)) {
		tmp = intel_de_read(dev_priv, VBLANK(cpu_transcoder));
		pipe_config->hw.adjusted_mode.crtc_vblank_start =
							(tmp & 0xffff) + 1;
		pipe_config->hw.adjusted_mode.crtc_vblank_end =
						((tmp >> 16) & 0xffff) + 1;
	}
	tmp = intel_de_read(dev_priv, VSYNC(cpu_transcoder));
	pipe_config->hw.adjusted_mode.crtc_vsync_start = (tmp & 0xffff) + 1;
	pipe_config->hw.adjusted_mode.crtc_vsync_end = ((tmp >> 16) & 0xffff) + 1;

	if (intel_pipe_is_interlaced(pipe_config)) {
		pipe_config->hw.adjusted_mode.flags |= DRM_MODE_FLAG_INTERLACE;
		pipe_config->hw.adjusted_mode.crtc_vtotal += 1;
		pipe_config->hw.adjusted_mode.crtc_vblank_end += 1;
	}
}

static int i9xx_pll_refclk(struct drm_device *dev,
			   const struct intel_crtc_state *pipe_config)
{
	struct drm_i915_private *dev_priv = to_i915(dev);
	u32 dpll = pipe_config->dpll_hw_state.dpll;

	if ((dpll & PLL_REF_INPUT_MASK) == PLLB_REF_INPUT_SPREADSPECTRUMIN)
		return dev_priv->vbt.lvds_ssc_freq;
	else if (HAS_PCH_SPLIT(dev_priv))
		return 120000;
	else if (!IS_GEN(dev_priv, 2))
		return 96000;
	else
		return 48000;
}

/* Returns the clock of the currently programmed mode of the given pipe. */
static void i9xx_crtc_clock_get(struct intel_crtc *crtc,
				struct intel_crtc_state *pipe_config)
{
	struct drm_device *dev = crtc->base.dev;
	struct drm_i915_private *dev_priv = to_i915(dev);
	enum pipe pipe = crtc->pipe;
	u32 dpll = pipe_config->dpll_hw_state.dpll;
	u32 fp;
	struct dpll clock;
	int port_clock;
	int refclk = i9xx_pll_refclk(dev, pipe_config);

	if ((dpll & DISPLAY_RATE_SELECT_FPA1) == 0)
		fp = pipe_config->dpll_hw_state.fp0;
	else
		fp = pipe_config->dpll_hw_state.fp1;

	clock.m1 = (fp & FP_M1_DIV_MASK) >> FP_M1_DIV_SHIFT;
	if (IS_PINEVIEW(dev_priv)) {
		clock.n = ffs((fp & FP_N_PINEVIEW_DIV_MASK) >> FP_N_DIV_SHIFT) - 1;
		clock.m2 = (fp & FP_M2_PINEVIEW_DIV_MASK) >> FP_M2_DIV_SHIFT;
	} else {
		clock.n = (fp & FP_N_DIV_MASK) >> FP_N_DIV_SHIFT;
		clock.m2 = (fp & FP_M2_DIV_MASK) >> FP_M2_DIV_SHIFT;
	}

	if (!IS_GEN(dev_priv, 2)) {
		if (IS_PINEVIEW(dev_priv))
			clock.p1 = ffs((dpll & DPLL_FPA01_P1_POST_DIV_MASK_PINEVIEW) >>
				DPLL_FPA01_P1_POST_DIV_SHIFT_PINEVIEW);
		else
			clock.p1 = ffs((dpll & DPLL_FPA01_P1_POST_DIV_MASK) >>
			       DPLL_FPA01_P1_POST_DIV_SHIFT);

		switch (dpll & DPLL_MODE_MASK) {
		case DPLLB_MODE_DAC_SERIAL:
			clock.p2 = dpll & DPLL_DAC_SERIAL_P2_CLOCK_DIV_5 ?
				5 : 10;
			break;
		case DPLLB_MODE_LVDS:
			clock.p2 = dpll & DPLLB_LVDS_P2_CLOCK_DIV_7 ?
				7 : 14;
			break;
		default:
			drm_dbg_kms(&dev_priv->drm,
				    "Unknown DPLL mode %08x in programmed "
				    "mode\n", (int)(dpll & DPLL_MODE_MASK));
			return;
		}

		if (IS_PINEVIEW(dev_priv))
			port_clock = pnv_calc_dpll_params(refclk, &clock);
		else
			port_clock = i9xx_calc_dpll_params(refclk, &clock);
	} else {
		u32 lvds = IS_I830(dev_priv) ? 0 : intel_de_read(dev_priv,
								 LVDS);
		bool is_lvds = (pipe == 1) && (lvds & LVDS_PORT_EN);

		if (is_lvds) {
			clock.p1 = ffs((dpll & DPLL_FPA01_P1_POST_DIV_MASK_I830_LVDS) >>
				       DPLL_FPA01_P1_POST_DIV_SHIFT);

			if (lvds & LVDS_CLKB_POWER_UP)
				clock.p2 = 7;
			else
				clock.p2 = 14;
		} else {
			if (dpll & PLL_P1_DIVIDE_BY_TWO)
				clock.p1 = 2;
			else {
				clock.p1 = ((dpll & DPLL_FPA01_P1_POST_DIV_MASK_I830) >>
					    DPLL_FPA01_P1_POST_DIV_SHIFT) + 2;
			}
			if (dpll & PLL_P2_DIVIDE_BY_4)
				clock.p2 = 4;
			else
				clock.p2 = 2;
		}

		port_clock = i9xx_calc_dpll_params(refclk, &clock);
	}

	/*
	 * This value includes pixel_multiplier. We will use
	 * port_clock to compute adjusted_mode.crtc_clock in the
	 * encoder's get_config() function.
	 */
	pipe_config->port_clock = port_clock;
}

int intel_dotclock_calculate(int link_freq,
			     const struct intel_link_m_n *m_n)
{
	/*
	 * The calculation for the data clock is:
	 * pixel_clock = ((m/n)*(link_clock * nr_lanes))/bpp
	 * But we want to avoid losing precison if possible, so:
	 * pixel_clock = ((m * link_clock * nr_lanes)/(n*bpp))
	 *
	 * and the link clock is simpler:
	 * link_clock = (m * link_clock) / n
	 */

	if (!m_n->link_n)
		return 0;

	return div_u64(mul_u32_u32(m_n->link_m, link_freq), m_n->link_n);
}

static void ilk_pch_clock_get(struct intel_crtc *crtc,
			      struct intel_crtc_state *pipe_config)
{
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);

	/* read out port_clock from the DPLL */
	i9xx_crtc_clock_get(crtc, pipe_config);

	/*
	 * In case there is an active pipe without active ports,
	 * we may need some idea for the dotclock anyway.
	 * Calculate one based on the FDI configuration.
	 */
	pipe_config->hw.adjusted_mode.crtc_clock =
		intel_dotclock_calculate(intel_fdi_link_freq(dev_priv, pipe_config),
					 &pipe_config->fdi_m_n);
}

static bool i9xx_has_pfit(struct drm_i915_private *dev_priv)
{
	if (IS_I830(dev_priv))
		return false;

	return INTEL_GEN(dev_priv) >= 4 ||
		IS_PINEVIEW(dev_priv) || IS_MOBILE(dev_priv);
}

static void i9xx_get_pfit_config(struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	u32 tmp;

	if (!i9xx_has_pfit(dev_priv))
		return;

	tmp = intel_de_read(dev_priv, PFIT_CONTROL);
	if (!(tmp & PFIT_ENABLE))
		return;

	/* Check whether the pfit is attached to our pipe. */
	if (INTEL_GEN(dev_priv) < 4) {
		if (crtc->pipe != PIPE_B)
			return;
	} else {
		if ((tmp & PFIT_PIPE_MASK) != (crtc->pipe << PFIT_PIPE_SHIFT))
			return;
	}

	crtc_state->gmch_pfit.control = tmp;
	crtc_state->gmch_pfit.pgm_ratios =
		intel_de_read(dev_priv, PFIT_PGM_RATIOS);
}

static void vlv_crtc_clock_get(struct intel_crtc *crtc,
			       struct intel_crtc_state *pipe_config)
{
	struct drm_device *dev = crtc->base.dev;
	struct drm_i915_private *dev_priv = to_i915(dev);
	enum pipe pipe = crtc->pipe;
	struct dpll clock;
	u32 mdiv;
	int refclk = 100000;

	/* In case of DSI, DPLL will not be used */
	if ((pipe_config->dpll_hw_state.dpll & DPLL_VCO_ENABLE) == 0)
		return;

	vlv_dpio_get(dev_priv);
	mdiv = vlv_dpio_read(dev_priv, pipe, VLV_PLL_DW3(pipe));
	vlv_dpio_put(dev_priv);

	clock.m1 = (mdiv >> DPIO_M1DIV_SHIFT) & 7;
	clock.m2 = mdiv & DPIO_M2DIV_MASK;
	clock.n = (mdiv >> DPIO_N_SHIFT) & 0xf;
	clock.p1 = (mdiv >> DPIO_P1_SHIFT) & 7;
	clock.p2 = (mdiv >> DPIO_P2_SHIFT) & 0x1f;

	pipe_config->port_clock = vlv_calc_dpll_params(refclk, &clock);
}

static void chv_crtc_clock_get(struct intel_crtc *crtc,
			       struct intel_crtc_state *pipe_config)
{
	struct drm_device *dev = crtc->base.dev;
	struct drm_i915_private *dev_priv = to_i915(dev);
	enum pipe pipe = crtc->pipe;
	enum dpio_channel port = vlv_pipe_to_channel(pipe);
	struct dpll clock;
	u32 cmn_dw13, pll_dw0, pll_dw1, pll_dw2, pll_dw3;
	int refclk = 100000;

	/* In case of DSI, DPLL will not be used */
	if ((pipe_config->dpll_hw_state.dpll & DPLL_VCO_ENABLE) == 0)
		return;

	vlv_dpio_get(dev_priv);
	cmn_dw13 = vlv_dpio_read(dev_priv, pipe, CHV_CMN_DW13(port));
	pll_dw0 = vlv_dpio_read(dev_priv, pipe, CHV_PLL_DW0(port));
	pll_dw1 = vlv_dpio_read(dev_priv, pipe, CHV_PLL_DW1(port));
	pll_dw2 = vlv_dpio_read(dev_priv, pipe, CHV_PLL_DW2(port));
	pll_dw3 = vlv_dpio_read(dev_priv, pipe, CHV_PLL_DW3(port));
	vlv_dpio_put(dev_priv);

	clock.m1 = (pll_dw1 & 0x7) == DPIO_CHV_M1_DIV_BY_2 ? 2 : 0;
	clock.m2 = (pll_dw0 & 0xff) << 22;
	if (pll_dw3 & DPIO_CHV_FRAC_DIV_EN)
		clock.m2 |= pll_dw2 & 0x3fffff;
	clock.n = (pll_dw1 >> DPIO_CHV_N_DIV_SHIFT) & 0xf;
	clock.p1 = (cmn_dw13 >> DPIO_CHV_P1_DIV_SHIFT) & 0x7;
	clock.p2 = (cmn_dw13 >> DPIO_CHV_P2_DIV_SHIFT) & 0x1f;

	pipe_config->port_clock = chv_calc_dpll_params(refclk, &clock);
}

static void ilk_set_pipeconf(const struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	enum pipe pipe = crtc->pipe;
	u32 val;

	val = 0;

	switch (crtc_state->pipe_bpp) {
	case 18:
		val |= PIPECONF_6BPC;
		break;
	case 24:
		val |= PIPECONF_8BPC;
		break;
	case 30:
		val |= PIPECONF_10BPC;
		break;
	case 36:
		val |= PIPECONF_12BPC;
		break;
	default:
		/* Case prevented by intel_choose_pipe_bpp_dither. */
		BUG();
	}

	if (crtc_state->dither)
		val |= (PIPECONF_DITHER_EN | PIPECONF_DITHER_TYPE_SP);

	if (crtc_state->hw.adjusted_mode.flags & DRM_MODE_FLAG_INTERLACE)
		val |= PIPECONF_INTERLACED_ILK;
	else
		val |= PIPECONF_PROGRESSIVE;

	/*
	 * This would end up with an odd purple hue over
	 * the entire display. Make sure we don't do it.
	 */
	drm_WARN_ON(&dev_priv->drm, crtc_state->limited_color_range &&
		    crtc_state->output_format != INTEL_OUTPUT_FORMAT_RGB);

	if (crtc_state->limited_color_range &&
	    !intel_crtc_has_type(crtc_state, INTEL_OUTPUT_SDVO))
		val |= PIPECONF_COLOR_RANGE_SELECT;

	if (crtc_state->output_format != INTEL_OUTPUT_FORMAT_RGB)
		val |= PIPECONF_OUTPUT_COLORSPACE_YUV709;

	val |= PIPECONF_GAMMA_MODE(crtc_state->gamma_mode);

	val |= PIPECONF_FRAME_START_DELAY(0);

	intel_de_write(dev_priv, PIPECONF(pipe), val);
	intel_de_posting_read(dev_priv, PIPECONF(pipe));
}

static void hsw_set_pipeconf(const struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	enum transcoder cpu_transcoder = crtc_state->cpu_transcoder;
	u32 val = 0;

	if (IS_HASWELL(dev_priv) && crtc_state->dither)
		val |= (PIPECONF_DITHER_EN | PIPECONF_DITHER_TYPE_SP);

	if (crtc_state->hw.adjusted_mode.flags & DRM_MODE_FLAG_INTERLACE)
		val |= PIPECONF_INTERLACED_ILK;
	else
		val |= PIPECONF_PROGRESSIVE;

	if (IS_HASWELL(dev_priv) &&
	    crtc_state->output_format != INTEL_OUTPUT_FORMAT_RGB)
		val |= PIPECONF_OUTPUT_COLORSPACE_YUV_HSW;

	intel_de_write(dev_priv, PIPECONF(cpu_transcoder), val);
	intel_de_posting_read(dev_priv, PIPECONF(cpu_transcoder));
}

void bdw_set_pipemisc(const struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	u32 val = 0;

	switch (crtc_state->pipe_bpp) {
	case 18:
		val |= PIPEMISC_DITHER_6_BPC;
		break;
	case 24:
		val |= PIPEMISC_DITHER_8_BPC;
		break;
	case 30:
		val |= PIPEMISC_DITHER_10_BPC;
		break;
	case 36:
		val |= PIPEMISC_DITHER_12_BPC;
		break;
	default:
		MISSING_CASE(crtc_state->pipe_bpp);
		break;
	}

	if (crtc_state->dither)
		val |= PIPEMISC_DITHER_ENABLE | PIPEMISC_DITHER_TYPE_SP;

	if (crtc_state->output_format == INTEL_OUTPUT_FORMAT_YCBCR420 ||
	    crtc_state->output_format == INTEL_OUTPUT_FORMAT_YCBCR444)
		val |= PIPEMISC_OUTPUT_COLORSPACE_YUV;

	if (crtc_state->output_format == INTEL_OUTPUT_FORMAT_YCBCR420)
		val |= PIPEMISC_YUV420_ENABLE |
			PIPEMISC_YUV420_MODE_FULL_BLEND;

	if (INTEL_GEN(dev_priv) >= 11 &&
	    (crtc_state->active_planes & ~(icl_hdr_plane_mask() |
					   BIT(PLANE_CURSOR))) == 0)
		val |= PIPEMISC_HDR_MODE_PRECISION;

	if (INTEL_GEN(dev_priv) >= 12)
		val |= PIPEMISC_PIXEL_ROUNDING_TRUNC;

	intel_de_write(dev_priv, PIPEMISC(crtc->pipe), val);
}

int bdw_get_pipemisc_bpp(struct intel_crtc *crtc)
{
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	u32 tmp;

	tmp = intel_de_read(dev_priv, PIPEMISC(crtc->pipe));

	switch (tmp & PIPEMISC_DITHER_BPC_MASK) {
	case PIPEMISC_DITHER_6_BPC:
		return 18;
	case PIPEMISC_DITHER_8_BPC:
		return 24;
	case PIPEMISC_DITHER_10_BPC:
		return 30;
	case PIPEMISC_DITHER_12_BPC:
		return 36;
	default:
		MISSING_CASE(tmp);
		return 0;
	}
}

static void icl_pipe_mbus_enable(struct intel_crtc *crtc)
{
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	enum pipe pipe = crtc->pipe;
	u32 val;

	val = MBUS_DBOX_A_CREDIT(2);

	if (INTEL_GEN(dev_priv) >= 12) {
		val |= MBUS_DBOX_BW_CREDIT(2);
		val |= MBUS_DBOX_B_CREDIT(12);
	} else {
		val |= MBUS_DBOX_BW_CREDIT(1);
		val |= MBUS_DBOX_B_CREDIT(8);
	}

	intel_de_write(dev_priv, PIPE_MBUS_DBOX_CTL(pipe), val);
}

void hsw_set_linetime_wm(const struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);

	intel_de_write(dev_priv, WM_LINETIME(crtc->pipe),
		       HSW_LINETIME(crtc_state->linetime) |
		       HSW_IPS_LINETIME(crtc_state->ips_linetime));
}

static void hsw_set_frame_start_delay(const struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	i915_reg_t reg = CHICKEN_TRANS(crtc_state->cpu_transcoder);
	u32 val;

	val = intel_de_read(dev_priv, reg);
	val &= ~HSW_FRAME_START_DELAY_MASK;
	val |= HSW_FRAME_START_DELAY(0);
	intel_de_write(dev_priv, reg, val);
}

static void i9xx_set_pipeconf(const struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	u32 pipeconf;

	pipeconf = 0;

	/* we keep both pipes enabled on 830 */
	if (IS_I830(dev_priv))
		pipeconf |= intel_de_read(dev_priv, PIPECONF(crtc->pipe)) & PIPECONF_ENABLE;

	if (crtc_state->double_wide)
		pipeconf |= PIPECONF_DOUBLE_WIDE;

	/* only g4x and later have fancy bpc/dither controls */
	if (IS_G4X(dev_priv) || IS_VALLEYVIEW(dev_priv) ||
	    IS_CHERRYVIEW(dev_priv)) {
		/* Bspec claims that we can't use dithering for 30bpp pipes. */
		if (crtc_state->dither && crtc_state->pipe_bpp != 30)
			pipeconf |= PIPECONF_DITHER_EN |
				    PIPECONF_DITHER_TYPE_SP;

		switch (crtc_state->pipe_bpp) {
		case 18:
			pipeconf |= PIPECONF_6BPC;
			break;
		case 24:
			pipeconf |= PIPECONF_8BPC;
			break;
		case 30:
			pipeconf |= PIPECONF_10BPC;
			break;
		default:
			/* Case prevented by intel_choose_pipe_bpp_dither. */
			BUG();
		}
	}

	if (crtc_state->hw.adjusted_mode.flags & DRM_MODE_FLAG_INTERLACE) {
		if (INTEL_GEN(dev_priv) < 4 ||
		    intel_crtc_has_type(crtc_state, INTEL_OUTPUT_SDVO))
			pipeconf |= PIPECONF_INTERLACE_W_FIELD_INDICATION;
		else
			pipeconf |= PIPECONF_INTERLACE_W_SYNC_SHIFT;
	} else {
		pipeconf |= PIPECONF_PROGRESSIVE;
	}

	if ((IS_VALLEYVIEW(dev_priv) || IS_CHERRYVIEW(dev_priv)) &&
	     crtc_state->limited_color_range)
		pipeconf |= PIPECONF_COLOR_RANGE_SELECT;

	pipeconf |= PIPECONF_GAMMA_MODE(crtc_state->gamma_mode);

	pipeconf |= PIPECONF_FRAME_START_DELAY(0);

	intel_de_write(dev_priv, PIPECONF(crtc->pipe), pipeconf);
	intel_de_posting_read(dev_priv, PIPECONF(crtc->pipe));
}

static void icl_ddi_bigjoiner_pre_enable(struct intel_atomic_state *state,
					 const struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *master = to_intel_crtc(crtc_state->uapi.crtc);
	struct intel_crtc_state *master_crtc_state;
	struct drm_connector_state *conn_state;
	struct drm_connector *conn;
	struct intel_encoder *encoder = NULL;
	int i;

	if (crtc_state->bigjoiner_slave)
		master = crtc_state->bigjoiner_linked_crtc;

	master_crtc_state = intel_atomic_get_new_crtc_state(state, master);

	for_each_new_connector_in_state(&state->base, conn, conn_state, i) {
		if (conn_state->crtc != &master->base)
			continue;

		encoder = to_intel_encoder(conn_state->best_encoder);
		break;
	}

	if (!crtc_state->bigjoiner_slave) {
		/* need to enable VDSC, which we skipped in pre-enable */
		intel_dsc_enable(encoder, crtc_state);
	} else {
		/*
		 * Enable sequence steps 1-7 on bigjoiner master
		 */
		intel_encoders_pre_pll_enable(state, master);
		intel_enable_shared_dpll(master_crtc_state);
		intel_encoders_pre_enable(state, master);

		/* and DSC on slave */
		intel_dsc_enable(NULL, crtc_state);
	}
}

void intel_enable_crtc(struct intel_atomic_state *state,
		       struct intel_crtc *crtc)
{
	struct drm_i915_private *dev_priv = to_i915(state->base.dev);
	const struct intel_crtc_state *new_crtc_state =
		intel_atomic_get_new_crtc_state(state, crtc);

	if (!intel_crtc_needs_modeset(new_crtc_state))
		return;

	intel_crtc_update_active_timings(new_crtc_state);

	dev_priv->display.crtc_enable(state, crtc);

	if (new_crtc_state->bigjoiner_slave)
		return;

	/* vblanks work again, re-enable pipe CRC. */
	intel_crtc_enable_pipe_crc(crtc);
}


void ilk_pfit_enable(const struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	const struct drm_rect *dst = &crtc_state->pch_pfit.dst;
	enum pipe pipe = crtc->pipe;
	int width = drm_rect_width(dst);
	int height = drm_rect_height(dst);
	int x = dst->x1;
	int y = dst->y1;

	if (!crtc_state->pch_pfit.enabled)
		return;

	/* Force use of hard-coded filter coefficients
	 * as some pre-programmed values are broken,
	 * e.g. x201.
	 */
	if (IS_IVYBRIDGE(dev_priv) || IS_HASWELL(dev_priv))
		intel_de_write(dev_priv, PF_CTL(pipe), PF_ENABLE |
			       PF_FILTER_MED_3x3 | PF_PIPE_SEL_IVB(pipe));
	else
		intel_de_write(dev_priv, PF_CTL(pipe), PF_ENABLE |
			       PF_FILTER_MED_3x3);
	intel_de_write(dev_priv, PF_WIN_POS(pipe), x << 16 | y);
	intel_de_write(dev_priv, PF_WIN_SZ(pipe), width << 16 | height);
}

static void i9xx_pfit_enable(const struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);

	if (!crtc_state->gmch_pfit.control)
		return;

	/*
	 * The panel fitter should only be adjusted whilst the pipe is disabled,
	 * according to register description and PRM.
	 */
	drm_WARN_ON(&dev_priv->drm,
		    intel_de_read(dev_priv, PFIT_CONTROL) & PFIT_ENABLE);
	assert_pipe_disabled(dev_priv, crtc_state->cpu_transcoder);

	intel_de_write(dev_priv, PFIT_PGM_RATIOS,
		       crtc_state->gmch_pfit.pgm_ratios);
	intel_de_write(dev_priv, PFIT_CONTROL, crtc_state->gmch_pfit.control);

	/* Border color in case we don't scale up to the full screen. Black by
	 * default, change to something else for debugging. */
	intel_de_write(dev_priv, BCLRPAT(crtc->pipe), 0);
}

static void skl_detach_scaler(struct intel_crtc *intel_crtc, int id)
{
	struct drm_device *dev = intel_crtc->base.dev;
	struct drm_i915_private *dev_priv = to_i915(dev);
	unsigned long irqflags;

	spin_lock_irqsave(&dev_priv->uncore.lock, irqflags);

	intel_de_write_fw(dev_priv, SKL_PS_CTRL(intel_crtc->pipe, id), 0);
	intel_de_write_fw(dev_priv, SKL_PS_WIN_POS(intel_crtc->pipe, id), 0);
	intel_de_write_fw(dev_priv, SKL_PS_WIN_SZ(intel_crtc->pipe, id), 0);

	spin_unlock_irqrestore(&dev_priv->uncore.lock, irqflags);
}

/*
 * This function detaches (aka. unbinds) unused scalers in hardware
 */
static void skl_detach_scalers(const struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc_state->uapi.crtc);
	const struct intel_crtc_scaler_state *scaler_state =
		&crtc_state->scaler_state;
	int i;

	/* loop through and disable scalers that aren't in use */
	for (i = 0; i < intel_crtc->num_scalers; i++) {
		if (!scaler_state->scalers[i].in_use)
			skl_detach_scaler(intel_crtc, i);
	}
}

void skl_scaler_disable(const struct intel_crtc_state *old_crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(old_crtc_state->uapi.crtc);
	int i;

	for (i = 0; i < crtc->num_scalers; i++)
		skl_detach_scaler(crtc, i);
}

static void intel_pipe_fastset(const struct intel_crtc_state *old_crtc_state,
			       const struct intel_crtc_state *new_crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(new_crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);

	/*
	 * Update pipe size and adjust fitter if needed: the reason for this is
	 * that in compute_mode_changes we check the native mode (not the pfit
	 * mode) to see if we can flip rather than do a full mode set. In the
	 * fastboot case, we'll flip, but if we don't update the pipesrc and
	 * pfit state, we'll end up with a big fb scanned out into the wrong
	 * sized surface.
	 */
	intel_set_pipe_src_size(new_crtc_state);

	/* on skylake this is done by detaching scalers */
	if (INTEL_GEN(dev_priv) >= 9) {
		skl_detach_scalers(new_crtc_state);

		if (new_crtc_state->pch_pfit.enabled)
			skl_pfit_enable(new_crtc_state);
	} else if (HAS_PCH_SPLIT(dev_priv)) {
		if (new_crtc_state->pch_pfit.enabled)
			ilk_pfit_enable(new_crtc_state);
		else if (old_crtc_state->pch_pfit.enabled)
			ilk_pfit_disable(old_crtc_state);
	}

	/*
	 * The register is supposedly single buffered so perhaps
	 * not 100% correct to do this here. But SKL+ calculate
	 * this based on the adjust pixel rate so pfit changes do
	 * affect it and so it must be updated for fastsets.
	 * HSW/BDW only really need this here for fastboot, after
	 * that the value should not change without a full modeset.
	 */
	if (INTEL_GEN(dev_priv) >= 9 ||
	    IS_BROADWELL(dev_priv) || IS_HASWELL(dev_priv))
		hsw_set_linetime_wm(new_crtc_state);

	if (INTEL_GEN(dev_priv) >= 11)
		icl_set_pipe_chicken(crtc);
}

static void commit_pipe_config(struct intel_atomic_state *state,
			       struct intel_crtc *crtc)
{
	struct drm_i915_private *dev_priv = to_i915(state->base.dev);
	const struct intel_crtc_state *old_crtc_state =
		intel_atomic_get_old_crtc_state(state, crtc);
	const struct intel_crtc_state *new_crtc_state =
		intel_atomic_get_new_crtc_state(state, crtc);
	bool modeset = intel_crtc_needs_modeset(new_crtc_state);

	/*
	 * During modesets pipe configuration was programmed as the
	 * CRTC was enabled.
	 */
	if (!modeset) {
		if (new_crtc_state->uapi.color_mgmt_changed ||
		    new_crtc_state->update_pipe)
			intel_color_commit(new_crtc_state);

		if (INTEL_GEN(dev_priv) >= 9)
			skl_detach_scalers(new_crtc_state);

		if (INTEL_GEN(dev_priv) >= 9 || IS_BROADWELL(dev_priv))
			bdw_set_pipemisc(new_crtc_state);

		if (new_crtc_state->update_pipe)
			intel_pipe_fastset(old_crtc_state, new_crtc_state);

		intel_psr2_program_trans_man_trk_ctl(new_crtc_state);
	}

	if (dev_priv->display.atomic_update_watermarks)
		dev_priv->display.atomic_update_watermarks(state, crtc);
}

void intel_update_crtc(struct intel_atomic_state *state,
		       struct intel_crtc *crtc)
{
	struct drm_i915_private *dev_priv = to_i915(state->base.dev);
	const struct intel_crtc_state *old_crtc_state =
		intel_atomic_get_old_crtc_state(state, crtc);
	struct intel_crtc_state *new_crtc_state =
		intel_atomic_get_new_crtc_state(state, crtc);
	bool modeset = intel_crtc_needs_modeset(new_crtc_state);

	if (!modeset) {
		if (new_crtc_state->preload_luts &&
		    (new_crtc_state->uapi.color_mgmt_changed ||
		     new_crtc_state->update_pipe))
			intel_color_load_luts(new_crtc_state);

		intel_pre_plane_update(state, crtc);

		if (new_crtc_state->update_pipe)
			intel_encoders_update_pipe(state, crtc);
	}

	if (new_crtc_state->update_pipe && !new_crtc_state->enable_fbc)
		intel_fbc_disable(crtc);
	else
		intel_fbc_enable(state, crtc);

	/* Perform vblank evasion around commit operation */
	intel_pipe_update_start(new_crtc_state);

	commit_pipe_config(state, crtc);

	if (INTEL_GEN(dev_priv) >= 9)
		skl_update_planes_on_crtc(state, crtc);
	else
		i9xx_update_planes_on_crtc(state, crtc);

	intel_pipe_update_end(new_crtc_state);

	/*
	 * We usually enable FIFO underrun interrupts as part of the
	 * CRTC enable sequence during modesets.  But when we inherit a
	 * valid pipe configuration from the BIOS we need to take care
	 * of enabling them on the CRTC's first fastset.
	 */
	if (new_crtc_state->update_pipe && !modeset &&
	    old_crtc_state->inherited)
		intel_crtc_arm_fifo_underrun(crtc, new_crtc_state);
}

static void valleyview_crtc_enable(struct intel_atomic_state *state,
				   struct intel_crtc *crtc)
{
	const struct intel_crtc_state *new_crtc_state =
		intel_atomic_get_new_crtc_state(state, crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	enum pipe pipe = crtc->pipe;

	if (drm_WARN_ON(&dev_priv->drm, crtc->active))
		return;

	if (intel_crtc_has_dp_encoder(new_crtc_state))
		intel_dp_set_m_n(new_crtc_state, M1_N1);

	intel_set_transcoder_timings(new_crtc_state);
	intel_set_pipe_src_size(new_crtc_state);

	if (IS_CHERRYVIEW(dev_priv) && pipe == PIPE_B) {
		intel_de_write(dev_priv, CHV_BLEND(pipe), CHV_BLEND_LEGACY);
		intel_de_write(dev_priv, CHV_CANVAS(pipe), 0);
	}

	i9xx_set_pipeconf(new_crtc_state);

	crtc->active = true;

	intel_set_cpu_fifo_underrun_reporting(dev_priv, pipe, true);

	intel_encoders_pre_pll_enable(state, crtc);

	if (IS_CHERRYVIEW(dev_priv)) {
		chv_prepare_pll(crtc, new_crtc_state);
		chv_enable_pll(crtc, new_crtc_state);
	} else {
		vlv_prepare_pll(crtc, new_crtc_state);
		vlv_enable_pll(crtc, new_crtc_state);
	}

	intel_encoders_pre_enable(state, crtc);

	i9xx_pfit_enable(new_crtc_state);

	intel_color_load_luts(new_crtc_state);
	intel_color_commit(new_crtc_state);
	/* update DSPCNTR to configure gamma for pipe bottom color */
	intel_disable_primary_plane(new_crtc_state);

	dev_priv->display.initial_watermarks(state, crtc);
	intel_enable_pipe(new_crtc_state);

	intel_crtc_vblank_on(new_crtc_state);

	intel_encoders_enable(state, crtc);
}

static void i9xx_set_pll_dividers(const struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);

	intel_de_write(dev_priv, FP0(crtc->pipe),
		       crtc_state->dpll_hw_state.fp0);
	intel_de_write(dev_priv, FP1(crtc->pipe),
		       crtc_state->dpll_hw_state.fp1);
}

static enum intel_output_format
bdw_get_pipemisc_output_format(struct intel_crtc *crtc)
{
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	u32 tmp;

	tmp = intel_de_read(dev_priv, PIPEMISC(crtc->pipe));

	if (tmp & PIPEMISC_YUV420_ENABLE) {
		/* We support 4:2:0 in full blend mode only */
		drm_WARN_ON(&dev_priv->drm,
			    (tmp & PIPEMISC_YUV420_MODE_FULL_BLEND) == 0);

		return INTEL_OUTPUT_FORMAT_YCBCR420;
	} else if (tmp & PIPEMISC_OUTPUT_COLORSPACE_YUV) {
		return INTEL_OUTPUT_FORMAT_YCBCR444;
	} else {
		return INTEL_OUTPUT_FORMAT_RGB;
	}
}

static void i9xx_get_pipe_color_config(struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct intel_plane *plane = to_intel_plane(crtc->base.primary);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	enum i9xx_plane_id i9xx_plane = plane->i9xx_plane;
	u32 tmp;

	tmp = intel_de_read(dev_priv, DSPCNTR(i9xx_plane));

	if (tmp & DISPPLANE_GAMMA_ENABLE)
		crtc_state->gamma_enable = true;

	if (!HAS_GMCH(dev_priv) &&
	    tmp & DISPPLANE_PIPE_CSC_ENABLE)
		crtc_state->csc_enable = true;
}

static void i9xx_crtc_enable(struct intel_atomic_state *state,
			     struct intel_crtc *crtc)
{
	const struct intel_crtc_state *new_crtc_state =
		intel_atomic_get_new_crtc_state(state, crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	enum pipe pipe = crtc->pipe;

	if (drm_WARN_ON(&dev_priv->drm, crtc->active))
		return;

	i9xx_set_pll_dividers(new_crtc_state);

	if (intel_crtc_has_dp_encoder(new_crtc_state))
		intel_dp_set_m_n(new_crtc_state, M1_N1);

	intel_set_transcoder_timings(new_crtc_state);
	intel_set_pipe_src_size(new_crtc_state);

	i9xx_set_pipeconf(new_crtc_state);

	crtc->active = true;

	if (!IS_GEN(dev_priv, 2))
		intel_set_cpu_fifo_underrun_reporting(dev_priv, pipe, true);

	intel_encoders_pre_enable(state, crtc);

	i9xx_enable_pll(crtc, new_crtc_state);

	i9xx_pfit_enable(new_crtc_state);

	intel_color_load_luts(new_crtc_state);
	intel_color_commit(new_crtc_state);
	/* update DSPCNTR to configure gamma for pipe bottom color */
	intel_disable_primary_plane(new_crtc_state);

	if (dev_priv->display.initial_watermarks)
		dev_priv->display.initial_watermarks(state, crtc);
	else
		intel_update_watermarks(crtc);
	intel_enable_pipe(new_crtc_state);

	intel_crtc_vblank_on(new_crtc_state);

	intel_encoders_enable(state, crtc);

	/* prevents spurious underruns */
	if (IS_GEN(dev_priv, 2))
		intel_wait_for_vblank(dev_priv, pipe);
}

static void ilk_pch_transcoder_set_timings(const struct intel_crtc_state *crtc_state,
					   enum pipe pch_transcoder)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	enum transcoder cpu_transcoder = crtc_state->cpu_transcoder;

	intel_de_write(dev_priv, PCH_TRANS_HTOTAL(pch_transcoder),
		       intel_de_read(dev_priv, HTOTAL(cpu_transcoder)));
	intel_de_write(dev_priv, PCH_TRANS_HBLANK(pch_transcoder),
		       intel_de_read(dev_priv, HBLANK(cpu_transcoder)));
	intel_de_write(dev_priv, PCH_TRANS_HSYNC(pch_transcoder),
		       intel_de_read(dev_priv, HSYNC(cpu_transcoder)));

	intel_de_write(dev_priv, PCH_TRANS_VTOTAL(pch_transcoder),
		       intel_de_read(dev_priv, VTOTAL(cpu_transcoder)));
	intel_de_write(dev_priv, PCH_TRANS_VBLANK(pch_transcoder),
		       intel_de_read(dev_priv, VBLANK(cpu_transcoder)));
	intel_de_write(dev_priv, PCH_TRANS_VSYNC(pch_transcoder),
		       intel_de_read(dev_priv, VSYNC(cpu_transcoder)));
	intel_de_write(dev_priv, PCH_TRANS_VSYNCSHIFT(pch_transcoder),
		       intel_de_read(dev_priv, VSYNCSHIFT(cpu_transcoder)));
}

static void cpt_set_fdi_bc_bifurcation(struct drm_i915_private *dev_priv, bool enable)
{
	u32 temp;

	temp = intel_de_read(dev_priv, SOUTH_CHICKEN1);
	if (!!(temp & FDI_BC_BIFURCATION_SELECT) == enable)
		return;

	drm_WARN_ON(&dev_priv->drm,
		    intel_de_read(dev_priv, FDI_RX_CTL(PIPE_B)) &
		    FDI_RX_ENABLE);
	drm_WARN_ON(&dev_priv->drm,
		    intel_de_read(dev_priv, FDI_RX_CTL(PIPE_C)) &
		    FDI_RX_ENABLE);

	temp &= ~FDI_BC_BIFURCATION_SELECT;
	if (enable)
		temp |= FDI_BC_BIFURCATION_SELECT;

	drm_dbg_kms(&dev_priv->drm, "%sabling fdi C rx\n",
		    enable ? "en" : "dis");
	intel_de_write(dev_priv, SOUTH_CHICKEN1, temp);
	intel_de_posting_read(dev_priv, SOUTH_CHICKEN1);
}

static void ivb_update_fdi_bc_bifurcation(const struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);

	switch (crtc->pipe) {
	case PIPE_A:
		break;
	case PIPE_B:
		if (crtc_state->fdi_lanes > 2)
			cpt_set_fdi_bc_bifurcation(dev_priv, false);
		else
			cpt_set_fdi_bc_bifurcation(dev_priv, true);

		break;
	case PIPE_C:
		cpt_set_fdi_bc_bifurcation(dev_priv, true);

		break;
	default:
		BUG();
	}
}

static void ilk_enable_pch_transcoder(const struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	enum pipe pipe = crtc->pipe;
	i915_reg_t reg;
	u32 val, pipeconf_val;

	/* Make sure PCH DPLL is enabled */
	assert_shared_dpll_enabled(dev_priv, crtc_state->shared_dpll);

	/* FDI must be feeding us bits for PCH ports */
	assert_fdi_tx_enabled(dev_priv, pipe);
	assert_fdi_rx_enabled(dev_priv, pipe);

	if (HAS_PCH_CPT(dev_priv)) {
		reg = TRANS_CHICKEN2(pipe);
		val = intel_de_read(dev_priv, reg);
		/*
		 * Workaround: Set the timing override bit
		 * before enabling the pch transcoder.
		 */
		val |= TRANS_CHICKEN2_TIMING_OVERRIDE;
		/* Configure frame start delay to match the CPU */
		val &= ~TRANS_CHICKEN2_FRAME_START_DELAY_MASK;
		val |= TRANS_CHICKEN2_FRAME_START_DELAY(0);
		intel_de_write(dev_priv, reg, val);
	}

	reg = PCH_TRANSCONF(pipe);
	val = intel_de_read(dev_priv, reg);
	pipeconf_val = intel_de_read(dev_priv, PIPECONF(pipe));

	if (HAS_PCH_IBX(dev_priv)) {
		/* Configure frame start delay to match the CPU */
		val &= ~TRANS_FRAME_START_DELAY_MASK;
		val |= TRANS_FRAME_START_DELAY(0);

		/*
		 * Make the BPC in transcoder be consistent with
		 * that in pipeconf reg. For HDMI we must use 8bpc
		 * here for both 8bpc and 12bpc.
		 */
		val &= ~PIPECONF_BPC_MASK;
		if (intel_crtc_has_type(crtc_state, INTEL_OUTPUT_HDMI))
			val |= PIPECONF_8BPC;
		else
			val |= pipeconf_val & PIPECONF_BPC_MASK;
	}

	val &= ~TRANS_INTERLACE_MASK;
	if ((pipeconf_val & PIPECONF_INTERLACE_MASK) == PIPECONF_INTERLACED_ILK) {
		if (HAS_PCH_IBX(dev_priv) &&
		    intel_crtc_has_type(crtc_state, INTEL_OUTPUT_SDVO))
			val |= TRANS_LEGACY_INTERLACED_ILK;
		else
			val |= TRANS_INTERLACED;
	} else {
		val |= TRANS_PROGRESSIVE;
	}

	intel_de_write(dev_priv, reg, val | TRANS_ENABLE);
	if (intel_de_wait_for_set(dev_priv, reg, TRANS_STATE_ENABLE, 100))
		drm_err(&dev_priv->drm, "failed to enable transcoder %c\n",
			pipe_name(pipe));
}
/*
 * Enable PCH resources required for PCH ports:
 *   - PCH PLLs
 *   - FDI training & RX/TX
 *   - update transcoder timings
 *   - DP transcoding bits
 *   - transcoder
 */
static void ilk_pch_enable(const struct intel_atomic_state *state,
			   const struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct drm_device *dev = crtc->base.dev;
	struct drm_i915_private *dev_priv = to_i915(dev);
	enum pipe pipe = crtc->pipe;
	u32 temp;

	assert_pch_transcoder_disabled(dev_priv, pipe);

	if (IS_IVYBRIDGE(dev_priv))
		ivb_update_fdi_bc_bifurcation(crtc_state);

	/* Write the TU size bits before fdi link training, so that error
	 * detection works. */
	intel_de_write(dev_priv, FDI_RX_TUSIZE1(pipe),
		       intel_de_read(dev_priv, PIPE_DATA_M1(pipe)) & TU_SIZE_MASK);

	/* For PCH output, training FDI link */
	dev_priv->display.fdi_link_train(crtc, crtc_state);

	/* We need to program the right clock selection before writing the pixel
	 * mutliplier into the DPLL. */
	if (HAS_PCH_CPT(dev_priv)) {
		u32 sel;

		temp = intel_de_read(dev_priv, PCH_DPLL_SEL);
		temp |= TRANS_DPLL_ENABLE(pipe);
		sel = TRANS_DPLLB_SEL(pipe);
		if (crtc_state->shared_dpll ==
		    intel_get_shared_dpll_by_id(dev_priv, DPLL_ID_PCH_PLL_B))
			temp |= sel;
		else
			temp &= ~sel;
		intel_de_write(dev_priv, PCH_DPLL_SEL, temp);
	}

	/* XXX: pch pll's can be enabled any time before we enable the PCH
	 * transcoder, and we actually should do this to not upset any PCH
	 * transcoder that already use the clock when we share it.
	 *
	 * Note that enable_shared_dpll tries to do the right thing, but
	 * get_shared_dpll unconditionally resets the pll - we need that to have
	 * the right LVDS enable sequence. */
	intel_enable_shared_dpll(crtc_state);

	/* set transcoder timing, panel must allow it */
	assert_panel_unlocked(dev_priv, pipe);
	ilk_pch_transcoder_set_timings(crtc_state, pipe);

	intel_fdi_normal_train(crtc);

	/* For PCH DP, enable TRANS_DP_CTL */
	if (HAS_PCH_CPT(dev_priv) &&
	    intel_crtc_has_dp_encoder(crtc_state)) {
		const struct drm_display_mode *adjusted_mode =
			&crtc_state->hw.adjusted_mode;
		u32 bpc = (intel_de_read(dev_priv, PIPECONF(pipe)) & PIPECONF_BPC_MASK) >> 5;
		i915_reg_t reg = TRANS_DP_CTL(pipe);
		enum port port;

		temp = intel_de_read(dev_priv, reg);
		temp &= ~(TRANS_DP_PORT_SEL_MASK |
			  TRANS_DP_SYNC_MASK |
			  TRANS_DP_BPC_MASK);
		temp |= TRANS_DP_OUTPUT_ENABLE;
		temp |= bpc << 9; /* same format but at 11:9 */

		if (adjusted_mode->flags & DRM_MODE_FLAG_PHSYNC)
			temp |= TRANS_DP_HSYNC_ACTIVE_HIGH;
		if (adjusted_mode->flags & DRM_MODE_FLAG_PVSYNC)
			temp |= TRANS_DP_VSYNC_ACTIVE_HIGH;

		port = intel_get_crtc_new_encoder(state, crtc_state)->port;
		drm_WARN_ON(dev, port < PORT_B || port > PORT_D);
		temp |= TRANS_DP_PORT_SEL(port);

		intel_de_write(dev_priv, reg, temp);
	}

	ilk_enable_pch_transcoder(crtc_state);
}

static void lpt_enable_pch_transcoder(struct drm_i915_private *dev_priv,
				      enum transcoder cpu_transcoder)
{
	u32 val, pipeconf_val;

	/* FDI must be feeding us bits for PCH ports */
	assert_fdi_tx_enabled(dev_priv, (enum pipe) cpu_transcoder);
	assert_fdi_rx_enabled(dev_priv, PIPE_A);

	val = intel_de_read(dev_priv, TRANS_CHICKEN2(PIPE_A));
	/* Workaround: set timing override bit. */
	val |= TRANS_CHICKEN2_TIMING_OVERRIDE;
	/* Configure frame start delay to match the CPU */
	val &= ~TRANS_CHICKEN2_FRAME_START_DELAY_MASK;
	val |= TRANS_CHICKEN2_FRAME_START_DELAY(0);
	intel_de_write(dev_priv, TRANS_CHICKEN2(PIPE_A), val);

	val = TRANS_ENABLE;
	pipeconf_val = intel_de_read(dev_priv, PIPECONF(cpu_transcoder));

	if ((pipeconf_val & PIPECONF_INTERLACE_MASK_HSW) ==
	    PIPECONF_INTERLACED_ILK)
		val |= TRANS_INTERLACED;
	else
		val |= TRANS_PROGRESSIVE;

	intel_de_write(dev_priv, LPT_TRANSCONF, val);
	if (intel_de_wait_for_set(dev_priv, LPT_TRANSCONF,
				  TRANS_STATE_ENABLE, 100))
		drm_err(&dev_priv->drm, "Failed to enable PCH transcoder\n");
}

void lpt_disable_iclkip(struct drm_i915_private *dev_priv)
{
	u32 temp;

	intel_de_write(dev_priv, PIXCLK_GATE, PIXCLK_GATE_GATE);

	mutex_lock(&dev_priv->sb_lock);

	temp = intel_sbi_read(dev_priv, SBI_SSCCTL6, SBI_ICLK);
	temp |= SBI_SSCCTL_DISABLE;
	intel_sbi_write(dev_priv, SBI_SSCCTL6, temp, SBI_ICLK);

	mutex_unlock(&dev_priv->sb_lock);
}

/* Program iCLKIP clock to the desired frequency */
static void lpt_program_iclkip(const struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	int clock = crtc_state->hw.adjusted_mode.crtc_clock;
	u32 divsel, phaseinc, auxdiv, phasedir = 0;
	u32 temp;

	lpt_disable_iclkip(dev_priv);

	/* The iCLK virtual clock root frequency is in MHz,
	 * but the adjusted_mode->crtc_clock in in KHz. To get the
	 * divisors, it is necessary to divide one by another, so we
	 * convert the virtual clock precision to KHz here for higher
	 * precision.
	 */
	for (auxdiv = 0; auxdiv < 2; auxdiv++) {
		u32 iclk_virtual_root_freq = 172800 * 1000;
		u32 iclk_pi_range = 64;
		u32 desired_divisor;

		desired_divisor = DIV_ROUND_CLOSEST(iclk_virtual_root_freq,
						    clock << auxdiv);
		divsel = (desired_divisor / iclk_pi_range) - 2;
		phaseinc = desired_divisor % iclk_pi_range;

		/*
		 * Near 20MHz is a corner case which is
		 * out of range for the 7-bit divisor
		 */
		if (divsel <= 0x7f)
			break;
	}

	/* This should not happen with any sane values */
	drm_WARN_ON(&dev_priv->drm, SBI_SSCDIVINTPHASE_DIVSEL(divsel) &
		    ~SBI_SSCDIVINTPHASE_DIVSEL_MASK);
	drm_WARN_ON(&dev_priv->drm, SBI_SSCDIVINTPHASE_DIR(phasedir) &
		    ~SBI_SSCDIVINTPHASE_INCVAL_MASK);

	drm_dbg_kms(&dev_priv->drm,
		    "iCLKIP clock: found settings for %dKHz refresh rate: auxdiv=%x, divsel=%x, phasedir=%x, phaseinc=%x\n",
		    clock, auxdiv, divsel, phasedir, phaseinc);

	mutex_lock(&dev_priv->sb_lock);

	/* Program SSCDIVINTPHASE6 */
	temp = intel_sbi_read(dev_priv, SBI_SSCDIVINTPHASE6, SBI_ICLK);
	temp &= ~SBI_SSCDIVINTPHASE_DIVSEL_MASK;
	temp |= SBI_SSCDIVINTPHASE_DIVSEL(divsel);
	temp &= ~SBI_SSCDIVINTPHASE_INCVAL_MASK;
	temp |= SBI_SSCDIVINTPHASE_INCVAL(phaseinc);
	temp |= SBI_SSCDIVINTPHASE_DIR(phasedir);
	temp |= SBI_SSCDIVINTPHASE_PROPAGATE;
	intel_sbi_write(dev_priv, SBI_SSCDIVINTPHASE6, temp, SBI_ICLK);

	/* Program SSCAUXDIV */
	temp = intel_sbi_read(dev_priv, SBI_SSCAUXDIV6, SBI_ICLK);
	temp &= ~SBI_SSCAUXDIV_FINALDIV2SEL(1);
	temp |= SBI_SSCAUXDIV_FINALDIV2SEL(auxdiv);
	intel_sbi_write(dev_priv, SBI_SSCAUXDIV6, temp, SBI_ICLK);

	/* Enable modulator and associated divider */
	temp = intel_sbi_read(dev_priv, SBI_SSCCTL6, SBI_ICLK);
	temp &= ~SBI_SSCCTL_DISABLE;
	intel_sbi_write(dev_priv, SBI_SSCCTL6, temp, SBI_ICLK);

	mutex_unlock(&dev_priv->sb_lock);

	/* Wait for initialization time */
	udelay(24);

	intel_de_write(dev_priv, PIXCLK_GATE, PIXCLK_GATE_UNGATE);
}

int lpt_get_iclkip(struct drm_i915_private *dev_priv)
{
	u32 divsel, phaseinc, auxdiv;
	u32 iclk_virtual_root_freq = 172800 * 1000;
	u32 iclk_pi_range = 64;
	u32 desired_divisor;
	u32 temp;

	if ((intel_de_read(dev_priv, PIXCLK_GATE) & PIXCLK_GATE_UNGATE) == 0)
		return 0;

	mutex_lock(&dev_priv->sb_lock);

	temp = intel_sbi_read(dev_priv, SBI_SSCCTL6, SBI_ICLK);
	if (temp & SBI_SSCCTL_DISABLE) {
		mutex_unlock(&dev_priv->sb_lock);
		return 0;
	}

	temp = intel_sbi_read(dev_priv, SBI_SSCDIVINTPHASE6, SBI_ICLK);
	divsel = (temp & SBI_SSCDIVINTPHASE_DIVSEL_MASK) >>
		SBI_SSCDIVINTPHASE_DIVSEL_SHIFT;
	phaseinc = (temp & SBI_SSCDIVINTPHASE_INCVAL_MASK) >>
		SBI_SSCDIVINTPHASE_INCVAL_SHIFT;

	temp = intel_sbi_read(dev_priv, SBI_SSCAUXDIV6, SBI_ICLK);
	auxdiv = (temp & SBI_SSCAUXDIV_FINALDIV2SEL_MASK) >>
		SBI_SSCAUXDIV_FINALDIV2SEL_SHIFT;

	mutex_unlock(&dev_priv->sb_lock);

	desired_divisor = (divsel + 2) * iclk_pi_range + phaseinc;

	return DIV_ROUND_CLOSEST(iclk_virtual_root_freq,
				 desired_divisor << auxdiv);
}

void lpt_pch_enable(const struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	enum transcoder cpu_transcoder = crtc_state->cpu_transcoder;

	assert_pch_transcoder_disabled(dev_priv, PIPE_A);

	lpt_program_iclkip(crtc_state);

	/* Set transcoder timing. */
	ilk_pch_transcoder_set_timings(crtc_state, PIPE_A);

	lpt_enable_pch_transcoder(dev_priv, cpu_transcoder);
}

static void ilk_disable_pch_transcoder(struct drm_i915_private *dev_priv,
				       enum pipe pipe)
{
	i915_reg_t reg;
	u32 val;

	/* FDI relies on the transcoder */
	assert_fdi_tx_disabled(dev_priv, pipe);
	assert_fdi_rx_disabled(dev_priv, pipe);

	/* Ports must be off as well */
	assert_pch_ports_disabled(dev_priv, pipe);

	reg = PCH_TRANSCONF(pipe);
	val = intel_de_read(dev_priv, reg);
	val &= ~TRANS_ENABLE;
	intel_de_write(dev_priv, reg, val);
	/* wait for PCH transcoder off, transcoder state */
	if (intel_de_wait_for_clear(dev_priv, reg, TRANS_STATE_ENABLE, 50))
		drm_err(&dev_priv->drm, "failed to disable transcoder %c\n",
			pipe_name(pipe));

	if (HAS_PCH_CPT(dev_priv)) {
		/* Workaround: Clear the timing override chicken bit again. */
		reg = TRANS_CHICKEN2(pipe);
		val = intel_de_read(dev_priv, reg);
		val &= ~TRANS_CHICKEN2_TIMING_OVERRIDE;
		intel_de_write(dev_priv, reg, val);
	}
}

void lpt_disable_pch_transcoder(struct drm_i915_private *dev_priv)
{
	u32 val;

	val = intel_de_read(dev_priv, LPT_TRANSCONF);
	val &= ~TRANS_ENABLE;
	intel_de_write(dev_priv, LPT_TRANSCONF, val);
	/* wait for PCH transcoder off, transcoder state */
	if (intel_de_wait_for_clear(dev_priv, LPT_TRANSCONF,
				    TRANS_STATE_ENABLE, 50))
		drm_err(&dev_priv->drm, "Failed to disable PCH transcoder\n");

	/* Workaround: clear timing override bit. */
	val = intel_de_read(dev_priv, TRANS_CHICKEN2(PIPE_A));
	val &= ~TRANS_CHICKEN2_TIMING_OVERRIDE;
	intel_de_write(dev_priv, TRANS_CHICKEN2(PIPE_A), val);
}

enum pipe intel_crtc_pch_transcoder(struct intel_crtc *crtc)
{
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);

	if (HAS_PCH_LPT(dev_priv))
		return PIPE_A;
	else
		return crtc->pipe;
}

void intel_enable_pipe(const struct intel_crtc_state *new_crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(new_crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	enum transcoder cpu_transcoder = new_crtc_state->cpu_transcoder;
	enum pipe pipe = crtc->pipe;
	i915_reg_t reg;
	u32 val;

	drm_dbg_kms(&dev_priv->drm, "enabling pipe %c\n", pipe_name(pipe));

	assert_planes_disabled(crtc);

	/*
	 * A pipe without a PLL won't actually be able to drive bits from
	 * a plane.  On ILK+ the pipe PLLs are integrated, so we don't
	 * need the check.
	 */
	if (HAS_GMCH(dev_priv)) {
		if (intel_crtc_has_type(new_crtc_state, INTEL_OUTPUT_DSI))
			assert_dsi_pll_enabled(dev_priv);
		else
			assert_pll_enabled(dev_priv, pipe);
	} else {
		if (new_crtc_state->has_pch_encoder) {
			/* if driving the PCH, we need FDI enabled */
			assert_fdi_rx_pll_enabled(dev_priv,
						  intel_crtc_pch_transcoder(crtc));
			assert_fdi_tx_pll_enabled(dev_priv,
						  (enum pipe) cpu_transcoder);
		}
		/* FIXME: assert CPU port conditions for SNB+ */
	}

	trace_intel_pipe_enable(crtc);

	reg = PIPECONF(cpu_transcoder);
	val = intel_de_read(dev_priv, reg);
	if (val & PIPECONF_ENABLE) {
		/* we keep both pipes enabled on 830 */
		drm_WARN_ON(&dev_priv->drm, !IS_I830(dev_priv));
		return;
	}

	intel_de_write(dev_priv, reg, val | PIPECONF_ENABLE);
	intel_de_posting_read(dev_priv, reg);

	/*
	 * Until the pipe starts PIPEDSL reads will return a stale value,
	 * which causes an apparent vblank timestamp jump when PIPEDSL
	 * resets to its proper value. That also messes up the frame count
	 * when it's derived from the timestamps. So let's wait for the
	 * pipe to start properly before we call drm_crtc_vblank_on()
	 */
	if (intel_crtc_max_vblank_count(new_crtc_state) == 0)
		intel_wait_for_pipe_scanline_moving(crtc);
}

void intel_disable_pipe(const struct intel_crtc_state *old_crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(old_crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	enum transcoder cpu_transcoder = old_crtc_state->cpu_transcoder;
	enum pipe pipe = crtc->pipe;
	i915_reg_t reg;
	u32 val;

	drm_dbg_kms(&dev_priv->drm, "disabling pipe %c\n", pipe_name(pipe));

	/*
	 * Make sure planes won't keep trying to pump pixels to us,
	 * or we might hang the display.
	 */
	assert_planes_disabled(crtc);

	trace_intel_pipe_disable(crtc);

	reg = PIPECONF(cpu_transcoder);
	val = intel_de_read(dev_priv, reg);
	if ((val & PIPECONF_ENABLE) == 0)
		return;

	/*
	 * Double wide has implications for planes
	 * so best keep it disabled when not needed.
	 */
	if (old_crtc_state->double_wide)
		val &= ~PIPECONF_DOUBLE_WIDE;

	/* Don't disable pipe or pipe PLLs if needed */
	if (!IS_I830(dev_priv))
		val &= ~PIPECONF_ENABLE;

	intel_de_write(dev_priv, reg, val);
	if ((val & PIPECONF_ENABLE) == 0)
		intel_wait_for_pipe_off(old_crtc_state);
}

static void cpt_verify_modeset(struct drm_i915_private *dev_priv,
			       enum pipe pipe)
{
	i915_reg_t dslreg = PIPEDSL(pipe);
	u32 temp;

	temp = intel_de_read(dev_priv, dslreg);
	udelay(500);
	if (wait_for(intel_de_read(dev_priv, dslreg) != temp, 5)) {
		if (wait_for(intel_de_read(dev_priv, dslreg) != temp, 5))
			drm_err(&dev_priv->drm,
				"mode set failed: pipe %c stuck\n",
				pipe_name(pipe));
	}
}

static void ilk_crtc_enable(struct intel_atomic_state *state,
			    struct intel_crtc *crtc)
{
	const struct intel_crtc_state *new_crtc_state =
		intel_atomic_get_new_crtc_state(state, crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	enum pipe pipe = crtc->pipe;

	if (drm_WARN_ON(&dev_priv->drm, crtc->active))
		return;

	/*
	 * Sometimes spurious CPU pipe underruns happen during FDI
	 * training, at least with VGA+HDMI cloning. Suppress them.
	 *
	 * On ILK we get an occasional spurious CPU pipe underruns
	 * between eDP port A enable and vdd enable. Also PCH port
	 * enable seems to result in the occasional CPU pipe underrun.
	 *
	 * Spurious PCH underruns also occur during PCH enabling.
	 */
	intel_set_cpu_fifo_underrun_reporting(dev_priv, pipe, false);
	intel_set_pch_fifo_underrun_reporting(dev_priv, pipe, false);

	if (new_crtc_state->has_pch_encoder)
		intel_prepare_shared_dpll(new_crtc_state);

	if (intel_crtc_has_dp_encoder(new_crtc_state))
		intel_dp_set_m_n(new_crtc_state, M1_N1);

	intel_set_transcoder_timings(new_crtc_state);
	intel_set_pipe_src_size(new_crtc_state);

	if (new_crtc_state->has_pch_encoder)
		intel_cpu_transcoder_set_m_n(new_crtc_state,
					     &new_crtc_state->fdi_m_n, NULL);

	ilk_set_pipeconf(new_crtc_state);

	crtc->active = true;

	intel_encoders_pre_enable(state, crtc);

	if (new_crtc_state->has_pch_encoder) {
		/* Note: FDI PLL enabling _must_ be done before we enable the
		 * cpu pipes, hence this is separate from all the other fdi/pch
		 * enabling. */
		ilk_fdi_pll_enable(new_crtc_state);
	} else {
		assert_fdi_tx_disabled(dev_priv, pipe);
		assert_fdi_rx_disabled(dev_priv, pipe);
	}

	ilk_pfit_enable(new_crtc_state);

	/*
	 * On ILK+ LUT must be loaded before the pipe is running but with
	 * clocks enabled
	 */
	intel_color_load_luts(new_crtc_state);
	intel_color_commit(new_crtc_state);
	/* update DSPCNTR to configure gamma for pipe bottom color */
	intel_disable_primary_plane(new_crtc_state);

	if (dev_priv->display.initial_watermarks)
		dev_priv->display.initial_watermarks(state, crtc);
	intel_enable_pipe(new_crtc_state);

	if (new_crtc_state->has_pch_encoder)
		ilk_pch_enable(state, new_crtc_state);

	intel_crtc_vblank_on(new_crtc_state);

	intel_encoders_enable(state, crtc);

	if (HAS_PCH_CPT(dev_priv))
		cpt_verify_modeset(dev_priv, pipe);

	/*
	 * Must wait for vblank to avoid spurious PCH FIFO underruns.
	 * And a second vblank wait is needed at least on ILK with
	 * some interlaced HDMI modes. Let's do the double wait always
	 * in case there are more corner cases we don't know about.
	 */
	if (new_crtc_state->has_pch_encoder) {
		intel_wait_for_vblank(dev_priv, pipe);
		intel_wait_for_vblank(dev_priv, pipe);
	}
	intel_set_cpu_fifo_underrun_reporting(dev_priv, pipe, true);
	intel_set_pch_fifo_underrun_reporting(dev_priv, pipe, true);
}

static void glk_pipe_scaler_clock_gating_wa(struct drm_i915_private *dev_priv,
					    enum pipe pipe, bool apply)
{
	u32 val = intel_de_read(dev_priv, CLKGATE_DIS_PSL(pipe));
	u32 mask = DPF_GATING_DIS | DPF_RAM_GATING_DIS | DPFR_GATING_DIS;

	if (apply)
		val |= mask;
	else
		val &= ~mask;

	intel_de_write(dev_priv, CLKGATE_DIS_PSL(pipe), val);
}

static void hsw_crtc_enable(struct intel_atomic_state *state,
			    struct intel_crtc *crtc)
{
	const struct intel_crtc_state *new_crtc_state =
		intel_atomic_get_new_crtc_state(state, crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	enum pipe pipe = crtc->pipe, hsw_workaround_pipe;
	enum transcoder cpu_transcoder = new_crtc_state->cpu_transcoder;
	bool psl_clkgate_wa;

	if (drm_WARN_ON(&dev_priv->drm, crtc->active))
		return;

	if (!new_crtc_state->bigjoiner) {
		intel_encoders_pre_pll_enable(state, crtc);

		if (new_crtc_state->shared_dpll)
			intel_enable_shared_dpll(new_crtc_state);

		intel_encoders_pre_enable(state, crtc);
	} else {
		icl_ddi_bigjoiner_pre_enable(state, new_crtc_state);
	}

	intel_set_pipe_src_size(new_crtc_state);
	if (INTEL_GEN(dev_priv) >= 9 || IS_BROADWELL(dev_priv))
		bdw_set_pipemisc(new_crtc_state);

	if (!new_crtc_state->bigjoiner_slave && !transcoder_is_dsi(cpu_transcoder)) {
		intel_set_transcoder_timings(new_crtc_state);

		if (cpu_transcoder != TRANSCODER_EDP)
			intel_de_write(dev_priv, PIPE_MULT(cpu_transcoder),
				       new_crtc_state->pixel_multiplier - 1);

		if (new_crtc_state->has_pch_encoder)
			intel_cpu_transcoder_set_m_n(new_crtc_state,
						     &new_crtc_state->fdi_m_n, NULL);

		hsw_set_frame_start_delay(new_crtc_state);
	}

	if (!transcoder_is_dsi(cpu_transcoder))
		hsw_set_pipeconf(new_crtc_state);

	crtc->active = true;

	/* Display WA #1180: WaDisableScalarClockGating: glk, cnl */
	psl_clkgate_wa = (IS_GEMINILAKE(dev_priv) || IS_CANNONLAKE(dev_priv)) &&
		new_crtc_state->pch_pfit.enabled;
	if (psl_clkgate_wa)
		glk_pipe_scaler_clock_gating_wa(dev_priv, pipe, true);

	if (INTEL_GEN(dev_priv) >= 9)
		skl_pfit_enable(new_crtc_state);
	else
		ilk_pfit_enable(new_crtc_state);

	/*
	 * On ILK+ LUT must be loaded before the pipe is running but with
	 * clocks enabled
	 */
	intel_color_load_luts(new_crtc_state);
	intel_color_commit(new_crtc_state);
	/* update DSPCNTR to configure gamma/csc for pipe bottom color */
	if (INTEL_GEN(dev_priv) < 9)
		intel_disable_primary_plane(new_crtc_state);

	hsw_set_linetime_wm(new_crtc_state);

	if (INTEL_GEN(dev_priv) >= 11)
		icl_set_pipe_chicken(crtc);

	if (dev_priv->display.initial_watermarks)
		dev_priv->display.initial_watermarks(state, crtc);

	if (INTEL_GEN(dev_priv) >= 11)
		icl_pipe_mbus_enable(crtc);

	if (new_crtc_state->bigjoiner_slave) {
		trace_intel_pipe_enable(crtc);
		intel_crtc_vblank_on(new_crtc_state);
	}

	intel_encoders_enable(state, crtc);

	if (psl_clkgate_wa) {
		intel_wait_for_vblank(dev_priv, pipe);
		glk_pipe_scaler_clock_gating_wa(dev_priv, pipe, false);
	}

	/* If we change the relative order between pipe/planes enabling, we need
	 * to change the workaround. */
	hsw_workaround_pipe = new_crtc_state->hsw_workaround_pipe;
	if (IS_HASWELL(dev_priv) && hsw_workaround_pipe != INVALID_PIPE) {
		intel_wait_for_vblank(dev_priv, hsw_workaround_pipe);
		intel_wait_for_vblank(dev_priv, hsw_workaround_pipe);
	}
}

void ilk_pfit_disable(const struct intel_crtc_state *old_crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(old_crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	enum pipe pipe = crtc->pipe;

	/* To avoid upsetting the power well on haswell only disable the pfit if
	 * it's in use. The hw state code will make sure we get this right. */
	if (!old_crtc_state->pch_pfit.enabled)
		return;

	intel_de_write(dev_priv, PF_CTL(pipe), 0);
	intel_de_write(dev_priv, PF_WIN_POS(pipe), 0);
	intel_de_write(dev_priv, PF_WIN_SZ(pipe), 0);
}

static void ilk_crtc_disable(struct intel_atomic_state *state,
			     struct intel_crtc *crtc)
{
	const struct intel_crtc_state *old_crtc_state =
		intel_atomic_get_old_crtc_state(state, crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	enum pipe pipe = crtc->pipe;

	/*
	 * Sometimes spurious CPU pipe underruns happen when the
	 * pipe is already disabled, but FDI RX/TX is still enabled.
	 * Happens at least with VGA+HDMI cloning. Suppress them.
	 */
	intel_set_cpu_fifo_underrun_reporting(dev_priv, pipe, false);
	intel_set_pch_fifo_underrun_reporting(dev_priv, pipe, false);

	intel_encoders_disable(state, crtc);

	intel_crtc_vblank_off(old_crtc_state);

	intel_disable_pipe(old_crtc_state);

	ilk_pfit_disable(old_crtc_state);

	if (old_crtc_state->has_pch_encoder)
		ilk_fdi_disable(crtc);

	intel_encoders_post_disable(state, crtc);

	if (old_crtc_state->has_pch_encoder) {
		ilk_disable_pch_transcoder(dev_priv, pipe);

		if (HAS_PCH_CPT(dev_priv)) {
			i915_reg_t reg;
			u32 temp;

			/* disable TRANS_DP_CTL */
			reg = TRANS_DP_CTL(pipe);
			temp = intel_de_read(dev_priv, reg);
			temp &= ~(TRANS_DP_OUTPUT_ENABLE |
				  TRANS_DP_PORT_SEL_MASK);
			temp |= TRANS_DP_PORT_SEL_NONE;
			intel_de_write(dev_priv, reg, temp);

			/* disable DPLL_SEL */
			temp = intel_de_read(dev_priv, PCH_DPLL_SEL);
			temp &= ~(TRANS_DPLL_ENABLE(pipe) | TRANS_DPLLB_SEL(pipe));
			intel_de_write(dev_priv, PCH_DPLL_SEL, temp);
		}

		ilk_fdi_pll_disable(crtc);
	}

	intel_set_cpu_fifo_underrun_reporting(dev_priv, pipe, true);
	intel_set_pch_fifo_underrun_reporting(dev_priv, pipe, true);
}

static void hsw_crtc_disable(struct intel_atomic_state *state,
			     struct intel_crtc *crtc)
{
	/*
	 * FIXME collapse everything to one hook.
	 * Need care with mst->ddi interactions.
	 */
	intel_encoders_disable(state, crtc);
	intel_encoders_post_disable(state, crtc);
}

static void i9xx_pfit_disable(const struct intel_crtc_state *old_crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(old_crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);

	if (!old_crtc_state->gmch_pfit.control)
		return;

	assert_pipe_disabled(dev_priv, old_crtc_state->cpu_transcoder);

	drm_dbg_kms(&dev_priv->drm, "disabling pfit, current: 0x%08x\n",
		    intel_de_read(dev_priv, PFIT_CONTROL));
	intel_de_write(dev_priv, PFIT_CONTROL, 0);
}

static void i9xx_crtc_disable(struct intel_atomic_state *state,
			      struct intel_crtc *crtc)
{
	struct intel_crtc_state *old_crtc_state =
		intel_atomic_get_old_crtc_state(state, crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	enum pipe pipe = crtc->pipe;

	/*
	 * On gen2 planes are double buffered but the pipe isn't, so we must
	 * wait for planes to fully turn off before disabling the pipe.
	 */
	if (IS_GEN(dev_priv, 2))
		intel_wait_for_vblank(dev_priv, pipe);

	intel_encoders_disable(state, crtc);

	intel_crtc_vblank_off(old_crtc_state);

	intel_disable_pipe(old_crtc_state);

	i9xx_pfit_disable(old_crtc_state);

	intel_encoders_post_disable(state, crtc);

	if (!intel_crtc_has_type(old_crtc_state, INTEL_OUTPUT_DSI)) {
		if (IS_CHERRYVIEW(dev_priv))
			chv_disable_pll(dev_priv, pipe);
		else if (IS_VALLEYVIEW(dev_priv))
			vlv_disable_pll(dev_priv, pipe);
		else
			i9xx_disable_pll(old_crtc_state);
	}

	intel_encoders_post_pll_disable(state, crtc);

	if (!IS_GEN(dev_priv, 2))
		intel_set_cpu_fifo_underrun_reporting(dev_priv, pipe, false);

	if (!dev_priv->display.initial_watermarks)
		intel_update_watermarks(crtc);

	/* clock the pipe down to 640x480@60 to potentially save power */
	if (IS_I830(dev_priv))
		i830_enable_pipe(dev_priv, pipe);
}

static void ilk_get_fdi_m_n_config(struct intel_crtc *crtc,
				   struct intel_crtc_state *pipe_config)
{
	intel_cpu_transcoder_get_m_n(crtc, pipe_config->cpu_transcoder,
				     &pipe_config->fdi_m_n, NULL);
}

static bool ilk_get_pipe_config(struct intel_crtc *crtc,
				struct intel_crtc_state *pipe_config)
{
	struct drm_device *dev = crtc->base.dev;
	struct drm_i915_private *dev_priv = to_i915(dev);
	enum intel_display_power_domain power_domain;
	intel_wakeref_t wakeref;
	u32 tmp;
	bool ret;

	power_domain = POWER_DOMAIN_PIPE(crtc->pipe);
	wakeref = intel_display_power_get_if_enabled(dev_priv, power_domain);
	if (!wakeref)
		return false;

	pipe_config->cpu_transcoder = (enum transcoder) crtc->pipe;
	pipe_config->shared_dpll = NULL;

	ret = false;
	tmp = intel_de_read(dev_priv, PIPECONF(crtc->pipe));
	if (!(tmp & PIPECONF_ENABLE))
		goto out;

	switch (tmp & PIPECONF_BPC_MASK) {
	case PIPECONF_6BPC:
		pipe_config->pipe_bpp = 18;
		break;
	case PIPECONF_8BPC:
		pipe_config->pipe_bpp = 24;
		break;
	case PIPECONF_10BPC:
		pipe_config->pipe_bpp = 30;
		break;
	case PIPECONF_12BPC:
		pipe_config->pipe_bpp = 36;
		break;
	default:
		break;
	}

	if (tmp & PIPECONF_COLOR_RANGE_SELECT)
		pipe_config->limited_color_range = true;

	switch (tmp & PIPECONF_OUTPUT_COLORSPACE_MASK) {
	case PIPECONF_OUTPUT_COLORSPACE_YUV601:
	case PIPECONF_OUTPUT_COLORSPACE_YUV709:
		pipe_config->output_format = INTEL_OUTPUT_FORMAT_YCBCR444;
		break;
	default:
		pipe_config->output_format = INTEL_OUTPUT_FORMAT_RGB;
		break;
	}

	pipe_config->gamma_mode = (tmp & PIPECONF_GAMMA_MODE_MASK_ILK) >>
		PIPECONF_GAMMA_MODE_SHIFT;

	pipe_config->csc_mode = intel_de_read(dev_priv,
					      PIPE_CSC_MODE(crtc->pipe));

	i9xx_get_pipe_color_config(pipe_config);
	intel_color_get_config(pipe_config);

	if (intel_de_read(dev_priv, PCH_TRANSCONF(crtc->pipe)) & TRANS_ENABLE) {
		struct intel_shared_dpll *pll;
		enum intel_dpll_id pll_id;
		bool pll_active;

		pipe_config->has_pch_encoder = true;

		tmp = intel_de_read(dev_priv, FDI_RX_CTL(crtc->pipe));
		pipe_config->fdi_lanes = ((FDI_DP_PORT_WIDTH_MASK & tmp) >>
					  FDI_DP_PORT_WIDTH_SHIFT) + 1;

		ilk_get_fdi_m_n_config(crtc, pipe_config);

		if (HAS_PCH_IBX(dev_priv)) {
			/*
			 * The pipe->pch transcoder and pch transcoder->pll
			 * mapping is fixed.
			 */
			pll_id = (enum intel_dpll_id) crtc->pipe;
		} else {
			tmp = intel_de_read(dev_priv, PCH_DPLL_SEL);
			if (tmp & TRANS_DPLLB_SEL(crtc->pipe))
				pll_id = DPLL_ID_PCH_PLL_B;
			else
				pll_id= DPLL_ID_PCH_PLL_A;
		}

		pipe_config->shared_dpll =
			intel_get_shared_dpll_by_id(dev_priv, pll_id);
		pll = pipe_config->shared_dpll;

		pll_active = intel_dpll_get_hw_state(dev_priv, pll,
						     &pipe_config->dpll_hw_state);
		drm_WARN_ON(dev, !pll_active);

		tmp = pipe_config->dpll_hw_state.dpll;
		pipe_config->pixel_multiplier =
			((tmp & PLL_REF_SDVO_HDMI_MULTIPLIER_MASK)
			 >> PLL_REF_SDVO_HDMI_MULTIPLIER_SHIFT) + 1;

		ilk_pch_clock_get(crtc, pipe_config);
	} else {
		pipe_config->pixel_multiplier = 1;
	}

	intel_get_transcoder_timings(crtc, pipe_config);
	intel_get_pipe_src_size(crtc, pipe_config);

	ilk_get_pfit_config(pipe_config);

	ret = true;

out:
	intel_display_power_put(dev_priv, power_domain, wakeref);

	return ret;
}

static bool i9xx_get_pipe_config(struct intel_crtc *crtc,
				 struct intel_crtc_state *pipe_config)
{
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	enum intel_display_power_domain power_domain;
	intel_wakeref_t wakeref;
	u32 tmp;
	bool ret;

	power_domain = POWER_DOMAIN_PIPE(crtc->pipe);
	wakeref = intel_display_power_get_if_enabled(dev_priv, power_domain);
	if (!wakeref)
		return false;

	pipe_config->output_format = INTEL_OUTPUT_FORMAT_RGB;
	pipe_config->cpu_transcoder = (enum transcoder) crtc->pipe;
	pipe_config->shared_dpll = NULL;

	ret = false;

	tmp = intel_de_read(dev_priv, PIPECONF(crtc->pipe));
	if (!(tmp & PIPECONF_ENABLE))
		goto out;

	if (IS_G4X(dev_priv) || IS_VALLEYVIEW(dev_priv) ||
	    IS_CHERRYVIEW(dev_priv)) {
		switch (tmp & PIPECONF_BPC_MASK) {
		case PIPECONF_6BPC:
			pipe_config->pipe_bpp = 18;
			break;
		case PIPECONF_8BPC:
			pipe_config->pipe_bpp = 24;
			break;
		case PIPECONF_10BPC:
			pipe_config->pipe_bpp = 30;
			break;
		default:
			break;
		}
	}

	if ((IS_VALLEYVIEW(dev_priv) || IS_CHERRYVIEW(dev_priv)) &&
	    (tmp & PIPECONF_COLOR_RANGE_SELECT))
		pipe_config->limited_color_range = true;

	pipe_config->gamma_mode = (tmp & PIPECONF_GAMMA_MODE_MASK_I9XX) >>
		PIPECONF_GAMMA_MODE_SHIFT;

	if (IS_CHERRYVIEW(dev_priv))
		pipe_config->cgm_mode = intel_de_read(dev_priv,
						      CGM_PIPE_MODE(crtc->pipe));

	i9xx_get_pipe_color_config(pipe_config);
	intel_color_get_config(pipe_config);

	if (INTEL_GEN(dev_priv) < 4)
		pipe_config->double_wide = tmp & PIPECONF_DOUBLE_WIDE;

	intel_get_transcoder_timings(crtc, pipe_config);
	intel_get_pipe_src_size(crtc, pipe_config);

	i9xx_get_pfit_config(pipe_config);

	if (INTEL_GEN(dev_priv) >= 4) {
		/* No way to read it out on pipes B and C */
		if (IS_CHERRYVIEW(dev_priv) && crtc->pipe != PIPE_A)
			tmp = dev_priv->chv_dpll_md[crtc->pipe];
		else
			tmp = intel_de_read(dev_priv, DPLL_MD(crtc->pipe));
		pipe_config->pixel_multiplier =
			((tmp & DPLL_MD_UDI_MULTIPLIER_MASK)
			 >> DPLL_MD_UDI_MULTIPLIER_SHIFT) + 1;
		pipe_config->dpll_hw_state.dpll_md = tmp;
	} else if (IS_I945G(dev_priv) || IS_I945GM(dev_priv) ||
		   IS_G33(dev_priv) || IS_PINEVIEW(dev_priv)) {
		tmp = intel_de_read(dev_priv, DPLL(crtc->pipe));
		pipe_config->pixel_multiplier =
			((tmp & SDVO_MULTIPLIER_MASK)
			 >> SDVO_MULTIPLIER_SHIFT_HIRES) + 1;
	} else {
		/* Note that on i915G/GM the pixel multiplier is in the sdvo
		 * port and will be fixed up in the encoder->get_config
		 * function. */
		pipe_config->pixel_multiplier = 1;
	}
	pipe_config->dpll_hw_state.dpll = intel_de_read(dev_priv,
							DPLL(crtc->pipe));
	if (!IS_VALLEYVIEW(dev_priv) && !IS_CHERRYVIEW(dev_priv)) {
		pipe_config->dpll_hw_state.fp0 = intel_de_read(dev_priv,
							       FP0(crtc->pipe));
		pipe_config->dpll_hw_state.fp1 = intel_de_read(dev_priv,
							       FP1(crtc->pipe));
	} else {
		/* Mask out read-only status bits. */
		pipe_config->dpll_hw_state.dpll &= ~(DPLL_LOCK_VLV |
						     DPLL_PORTC_READY_MASK |
						     DPLL_PORTB_READY_MASK);
	}

	if (IS_CHERRYVIEW(dev_priv))
		chv_crtc_clock_get(crtc, pipe_config);
	else if (IS_VALLEYVIEW(dev_priv))
		vlv_crtc_clock_get(crtc, pipe_config);
	else
		i9xx_crtc_clock_get(crtc, pipe_config);

	/*
	 * Normally the dotclock is filled in by the encoder .get_config()
	 * but in case the pipe is enabled w/o any ports we need a sane
	 * default.
	 */
	pipe_config->hw.adjusted_mode.crtc_clock =
		pipe_config->port_clock / pipe_config->pixel_multiplier;

	ret = true;

out:
	intel_display_power_put(dev_priv, power_domain, wakeref);

	return ret;
}

static void dg1_get_ddi_pll(struct drm_i915_private *dev_priv, enum port port,
			    struct intel_crtc_state *pipe_config)
{
	enum icl_port_dpll_id port_dpll_id = ICL_PORT_DPLL_DEFAULT;
	enum phy phy = intel_port_to_phy(dev_priv, port);
	struct icl_port_dpll *port_dpll;
	struct intel_shared_dpll *pll;
	enum intel_dpll_id id;
	bool pll_active;
	u32 clk_sel;

	clk_sel = intel_de_read(dev_priv, DG1_DPCLKA_CFGCR0(phy)) & DG1_DPCLKA_CFGCR0_DDI_CLK_SEL_MASK(phy);
	id = DG1_DPCLKA_CFGCR0_DDI_CLK_SEL_DPLL_MAP(clk_sel, phy);

	if (WARN_ON(id > DPLL_ID_DG1_DPLL3))
		return;

	pll = intel_get_shared_dpll_by_id(dev_priv, id);
	port_dpll = &pipe_config->icl_port_dplls[port_dpll_id];

	port_dpll->pll = pll;
	pll_active = intel_dpll_get_hw_state(dev_priv, pll,
					     &port_dpll->hw_state);
	drm_WARN_ON(&dev_priv->drm, !pll_active);

	icl_set_active_port_dpll(pipe_config, port_dpll_id);
}

static void icl_get_ddi_pll(struct drm_i915_private *dev_priv, enum port port,
			    struct intel_crtc_state *pipe_config)
{
	enum phy phy = intel_port_to_phy(dev_priv, port);
	enum icl_port_dpll_id port_dpll_id;
	struct icl_port_dpll *port_dpll;
	struct intel_shared_dpll *pll;
	enum intel_dpll_id id;
	bool pll_active;
	u32 temp;

	if (intel_phy_is_combo(dev_priv, phy)) {
		u32 mask, shift;

		if (IS_ROCKETLAKE(dev_priv)) {
			mask = RKL_DPCLKA_CFGCR0_DDI_CLK_SEL_MASK(phy);
			shift = RKL_DPCLKA_CFGCR0_DDI_CLK_SEL_SHIFT(phy);
		} else {
			mask = ICL_DPCLKA_CFGCR0_DDI_CLK_SEL_MASK(phy);
			shift = ICL_DPCLKA_CFGCR0_DDI_CLK_SEL_SHIFT(phy);
		}

		temp = intel_de_read(dev_priv, ICL_DPCLKA_CFGCR0) & mask;
		id = temp >> shift;
		port_dpll_id = ICL_PORT_DPLL_DEFAULT;
	} else if (intel_phy_is_tc(dev_priv, phy)) {
		u32 clk_sel = intel_de_read(dev_priv, DDI_CLK_SEL(port)) & DDI_CLK_SEL_MASK;

		if (clk_sel == DDI_CLK_SEL_MG) {
			id = icl_tc_port_to_pll_id(intel_port_to_tc(dev_priv,
								    port));
			port_dpll_id = ICL_PORT_DPLL_MG_PHY;
		} else {
			drm_WARN_ON(&dev_priv->drm,
				    clk_sel < DDI_CLK_SEL_TBT_162);
			id = DPLL_ID_ICL_TBTPLL;
			port_dpll_id = ICL_PORT_DPLL_DEFAULT;
		}
	} else {
		drm_WARN(&dev_priv->drm, 1, "Invalid port %x\n", port);
		return;
	}

	pll = intel_get_shared_dpll_by_id(dev_priv, id);
	port_dpll = &pipe_config->icl_port_dplls[port_dpll_id];

	port_dpll->pll = pll;
	pll_active = intel_dpll_get_hw_state(dev_priv, pll,
					     &port_dpll->hw_state);
	drm_WARN_ON(&dev_priv->drm, !pll_active);

	icl_set_active_port_dpll(pipe_config, port_dpll_id);
}

static void cnl_get_ddi_pll(struct drm_i915_private *dev_priv, enum port port,
			    struct intel_crtc_state *pipe_config)
{
	struct intel_shared_dpll *pll;
	enum intel_dpll_id id;
	bool pll_active;
	u32 temp;

	temp = intel_de_read(dev_priv, DPCLKA_CFGCR0) & DPCLKA_CFGCR0_DDI_CLK_SEL_MASK(port);
	id = temp >> DPCLKA_CFGCR0_DDI_CLK_SEL_SHIFT(port);

	if (drm_WARN_ON(&dev_priv->drm, id < SKL_DPLL0 || id > SKL_DPLL2))
		return;

	pll = intel_get_shared_dpll_by_id(dev_priv, id);

	pipe_config->shared_dpll = pll;
	pll_active = intel_dpll_get_hw_state(dev_priv, pll,
					     &pipe_config->dpll_hw_state);
	drm_WARN_ON(&dev_priv->drm, !pll_active);
}

static void bxt_get_ddi_pll(struct drm_i915_private *dev_priv,
				enum port port,
				struct intel_crtc_state *pipe_config)
{
	struct intel_shared_dpll *pll;
	enum intel_dpll_id id;
	bool pll_active;

	switch (port) {
	case PORT_A:
		id = DPLL_ID_SKL_DPLL0;
		break;
	case PORT_B:
		id = DPLL_ID_SKL_DPLL1;
		break;
	case PORT_C:
		id = DPLL_ID_SKL_DPLL2;
		break;
	default:
		drm_err(&dev_priv->drm, "Incorrect port type\n");
		return;
	}

	pll = intel_get_shared_dpll_by_id(dev_priv, id);

	pipe_config->shared_dpll = pll;
	pll_active = intel_dpll_get_hw_state(dev_priv, pll,
					     &pipe_config->dpll_hw_state);
	drm_WARN_ON(&dev_priv->drm, !pll_active);
}

static void skl_get_ddi_pll(struct drm_i915_private *dev_priv, enum port port,
			    struct intel_crtc_state *pipe_config)
{
	struct intel_shared_dpll *pll;
	enum intel_dpll_id id;
	bool pll_active;
	u32 temp;

	temp = intel_de_read(dev_priv, DPLL_CTRL2) & DPLL_CTRL2_DDI_CLK_SEL_MASK(port);
	id = temp >> (port * 3 + 1);

	if (drm_WARN_ON(&dev_priv->drm, id < SKL_DPLL0 || id > SKL_DPLL3))
		return;

	pll = intel_get_shared_dpll_by_id(dev_priv, id);

	pipe_config->shared_dpll = pll;
	pll_active = intel_dpll_get_hw_state(dev_priv, pll,
					     &pipe_config->dpll_hw_state);
	drm_WARN_ON(&dev_priv->drm, !pll_active);
}

static void hsw_get_ddi_pll(struct drm_i915_private *dev_priv, enum port port,
			    struct intel_crtc_state *pipe_config)
{
	struct intel_shared_dpll *pll;
	enum intel_dpll_id id;
	u32 ddi_pll_sel = intel_de_read(dev_priv, PORT_CLK_SEL(port));
	bool pll_active;

	switch (ddi_pll_sel) {
	case PORT_CLK_SEL_WRPLL1:
		id = DPLL_ID_WRPLL1;
		break;
	case PORT_CLK_SEL_WRPLL2:
		id = DPLL_ID_WRPLL2;
		break;
	case PORT_CLK_SEL_SPLL:
		id = DPLL_ID_SPLL;
		break;
	case PORT_CLK_SEL_LCPLL_810:
		id = DPLL_ID_LCPLL_810;
		break;
	case PORT_CLK_SEL_LCPLL_1350:
		id = DPLL_ID_LCPLL_1350;
		break;
	case PORT_CLK_SEL_LCPLL_2700:
		id = DPLL_ID_LCPLL_2700;
		break;
	default:
		MISSING_CASE(ddi_pll_sel);
		fallthrough;
	case PORT_CLK_SEL_NONE:
		return;
	}

	pll = intel_get_shared_dpll_by_id(dev_priv, id);

	pipe_config->shared_dpll = pll;
	pll_active = intel_dpll_get_hw_state(dev_priv, pll,
					     &pipe_config->dpll_hw_state);
	drm_WARN_ON(&dev_priv->drm, !pll_active);
}

static bool hsw_get_transcoder_state(struct intel_crtc *crtc,
				     struct intel_crtc_state *pipe_config,
				     struct intel_display_power_domain_set *power_domain_set)
{
	struct drm_device *dev = crtc->base.dev;
	struct drm_i915_private *dev_priv = to_i915(dev);
	unsigned long panel_transcoder_mask = BIT(TRANSCODER_EDP);
	unsigned long enabled_panel_transcoders = 0;
	enum transcoder panel_transcoder;
	u32 tmp;

	if (INTEL_GEN(dev_priv) >= 11)
		panel_transcoder_mask |=
			BIT(TRANSCODER_DSI_0) | BIT(TRANSCODER_DSI_1);

	/*
	 * The pipe->transcoder mapping is fixed with the exception of the eDP
	 * and DSI transcoders handled below.
	 */
	pipe_config->cpu_transcoder = (enum transcoder) crtc->pipe;

	/*
	 * XXX: Do intel_display_power_get_if_enabled before reading this (for
	 * consistency and less surprising code; it's in always on power).
	 */
	for_each_cpu_transcoder_masked(dev_priv, panel_transcoder,
				       panel_transcoder_mask) {
		bool force_thru = false;
		enum pipe trans_pipe;

		tmp = intel_de_read(dev_priv,
				    TRANS_DDI_FUNC_CTL(panel_transcoder));
		if (!(tmp & TRANS_DDI_FUNC_ENABLE))
			continue;

		/*
		 * Log all enabled ones, only use the first one.
		 *
		 * FIXME: This won't work for two separate DSI displays.
		 */
		enabled_panel_transcoders |= BIT(panel_transcoder);
		if (enabled_panel_transcoders != BIT(panel_transcoder))
			continue;

		switch (tmp & TRANS_DDI_EDP_INPUT_MASK) {
		default:
			drm_WARN(dev, 1,
				 "unknown pipe linked to transcoder %s\n",
				 transcoder_name(panel_transcoder));
			fallthrough;
		case TRANS_DDI_EDP_INPUT_A_ONOFF:
			force_thru = true;
			fallthrough;
		case TRANS_DDI_EDP_INPUT_A_ON:
			trans_pipe = PIPE_A;
			break;
		case TRANS_DDI_EDP_INPUT_B_ONOFF:
			trans_pipe = PIPE_B;
			break;
		case TRANS_DDI_EDP_INPUT_C_ONOFF:
			trans_pipe = PIPE_C;
			break;
		case TRANS_DDI_EDP_INPUT_D_ONOFF:
			trans_pipe = PIPE_D;
			break;
		}

		if (trans_pipe == crtc->pipe) {
			pipe_config->cpu_transcoder = panel_transcoder;
			pipe_config->pch_pfit.force_thru = force_thru;
		}
	}

	/*
	 * Valid combos: none, eDP, DSI0, DSI1, DSI0+DSI1
	 */
	drm_WARN_ON(dev, (enabled_panel_transcoders & BIT(TRANSCODER_EDP)) &&
		    enabled_panel_transcoders != BIT(TRANSCODER_EDP));

	if (!intel_display_power_get_in_set_if_enabled(dev_priv, power_domain_set,
						       POWER_DOMAIN_TRANSCODER(pipe_config->cpu_transcoder)))
		return false;

	tmp = intel_de_read(dev_priv, PIPECONF(pipe_config->cpu_transcoder));

	return tmp & PIPECONF_ENABLE;
}

static bool bxt_get_dsi_transcoder_state(struct intel_crtc *crtc,
					 struct intel_crtc_state *pipe_config,
					 struct intel_display_power_domain_set *power_domain_set)
{
	struct drm_device *dev = crtc->base.dev;
	struct drm_i915_private *dev_priv = to_i915(dev);
	enum transcoder cpu_transcoder;
	enum port port;
	u32 tmp;

	for_each_port_masked(port, BIT(PORT_A) | BIT(PORT_C)) {
		if (port == PORT_A)
			cpu_transcoder = TRANSCODER_DSI_A;
		else
			cpu_transcoder = TRANSCODER_DSI_C;

		if (!intel_display_power_get_in_set_if_enabled(dev_priv, power_domain_set,
							       POWER_DOMAIN_TRANSCODER(cpu_transcoder)))
			continue;

		/*
		 * The PLL needs to be enabled with a valid divider
		 * configuration, otherwise accessing DSI registers will hang
		 * the machine. See BSpec North Display Engine
		 * registers/MIPI[BXT]. We can break out here early, since we
		 * need the same DSI PLL to be enabled for both DSI ports.
		 */
		if (!bxt_dsi_pll_is_enabled(dev_priv))
			break;

		/* XXX: this works for video mode only */
		tmp = intel_de_read(dev_priv, BXT_MIPI_PORT_CTRL(port));
		if (!(tmp & DPI_ENABLE))
			continue;

		tmp = intel_de_read(dev_priv, MIPI_CTRL(port));
		if ((tmp & BXT_PIPE_SELECT_MASK) != BXT_PIPE_SELECT(crtc->pipe))
			continue;

		pipe_config->cpu_transcoder = cpu_transcoder;
		break;
	}

	return transcoder_is_dsi(pipe_config->cpu_transcoder);
}

static void hsw_get_ddi_port_state(struct intel_crtc *crtc,
				   struct intel_crtc_state *pipe_config)
{
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	enum transcoder cpu_transcoder = pipe_config->cpu_transcoder;
	enum port port;
	u32 tmp;

	if (transcoder_is_dsi(cpu_transcoder)) {
		port = (cpu_transcoder == TRANSCODER_DSI_A) ?
						PORT_A : PORT_B;
	} else {
		tmp = intel_de_read(dev_priv,
				    TRANS_DDI_FUNC_CTL(cpu_transcoder));
		if (!(tmp & TRANS_DDI_FUNC_ENABLE))
			return;
		if (INTEL_GEN(dev_priv) >= 12)
			port = TGL_TRANS_DDI_FUNC_CTL_VAL_TO_PORT(tmp);
		else
			port = TRANS_DDI_FUNC_CTL_VAL_TO_PORT(tmp);
	}

	if (IS_DG1(dev_priv))
		dg1_get_ddi_pll(dev_priv, port, pipe_config);
	else if (INTEL_GEN(dev_priv) >= 11)
		icl_get_ddi_pll(dev_priv, port, pipe_config);
	else if (IS_CANNONLAKE(dev_priv))
		cnl_get_ddi_pll(dev_priv, port, pipe_config);
	else if (IS_GEN9_LP(dev_priv))
		bxt_get_ddi_pll(dev_priv, port, pipe_config);
	else if (IS_GEN9_BC(dev_priv))
		skl_get_ddi_pll(dev_priv, port, pipe_config);
	else
		hsw_get_ddi_pll(dev_priv, port, pipe_config);

	/*
	 * Haswell has only FDI/PCH transcoder A. It is which is connected to
	 * DDI E. So just check whether this pipe is wired to DDI E and whether
	 * the PCH transcoder is on.
	 */
	if (INTEL_GEN(dev_priv) < 9 &&
	    (port == PORT_E) && intel_de_read(dev_priv, LPT_TRANSCONF) & TRANS_ENABLE) {
		pipe_config->has_pch_encoder = true;

		tmp = intel_de_read(dev_priv, FDI_RX_CTL(PIPE_A));
		pipe_config->fdi_lanes = ((FDI_DP_PORT_WIDTH_MASK & tmp) >>
					  FDI_DP_PORT_WIDTH_SHIFT) + 1;

		ilk_get_fdi_m_n_config(crtc, pipe_config);
	}
}

static void ilk_get_pfit_pos_size(struct intel_crtc_state *crtc_state,
				  u32 pos, u32 size)
{
	drm_rect_init(&crtc_state->pch_pfit.dst,
		      pos >> 16, pos & 0xffff,
		      size >> 16, size & 0xffff);
}

static void skl_get_pfit_config(struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	struct intel_crtc_scaler_state *scaler_state = &crtc_state->scaler_state;
	int id = -1;
	int i;

	/* find scaler attached to this pipe */
	for (i = 0; i < crtc->num_scalers; i++) {
		u32 ctl, pos, size;

		ctl = intel_de_read(dev_priv, SKL_PS_CTRL(crtc->pipe, i));
		if ((ctl & (PS_SCALER_EN | PS_PLANE_SEL_MASK)) != PS_SCALER_EN)
			continue;

		id = i;
		crtc_state->pch_pfit.enabled = true;

		pos = intel_de_read(dev_priv, SKL_PS_WIN_POS(crtc->pipe, i));
		size = intel_de_read(dev_priv, SKL_PS_WIN_SZ(crtc->pipe, i));

		ilk_get_pfit_pos_size(crtc_state, pos, size);

		scaler_state->scalers[i].in_use = true;
		break;
	}

	scaler_state->scaler_id = id;
	if (id >= 0)
		scaler_state->scaler_users |= (1 << SKL_CRTC_INDEX);
	else
		scaler_state->scaler_users &= ~(1 << SKL_CRTC_INDEX);
}

static void ilk_get_pfit_config(struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	u32 ctl, pos, size;

	ctl = intel_de_read(dev_priv, PF_CTL(crtc->pipe));
	if ((ctl & PF_ENABLE) == 0)
		return;

	crtc_state->pch_pfit.enabled = true;

	pos = intel_de_read(dev_priv, PF_WIN_POS(crtc->pipe));
	size = intel_de_read(dev_priv, PF_WIN_SZ(crtc->pipe));

	ilk_get_pfit_pos_size(crtc_state, pos, size);

	/*
	 * We currently do not free assignements of panel fitters on
	 * ivb/hsw (since we don't use the higher upscaling modes which
	 * differentiates them) so just WARN about this case for now.
	 */
	drm_WARN_ON(&dev_priv->drm, IS_GEN(dev_priv, 7) &&
		    (ctl & PF_PIPE_SEL_MASK_IVB) != PF_PIPE_SEL_IVB(crtc->pipe));
}

static bool hsw_get_pipe_config(struct intel_crtc *crtc,
				struct intel_crtc_state *pipe_config)
{
	struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);
	struct intel_display_power_domain_set power_domain_set = { };
	bool active;
	u32 tmp;

	pipe_config->master_transcoder = INVALID_TRANSCODER;

	if (!intel_display_power_get_in_set_if_enabled(dev_priv, &power_domain_set,
						       POWER_DOMAIN_PIPE(crtc->pipe)))
		return false;

	pipe_config->shared_dpll = NULL;

	active = hsw_get_transcoder_state(crtc, pipe_config, &power_domain_set);

	if (IS_GEN9_LP(dev_priv) &&
	    bxt_get_dsi_transcoder_state(crtc, pipe_config, &power_domain_set)) {
		drm_WARN_ON(&dev_priv->drm, active);
		active = true;
	}

	intel_dsc_get_config(pipe_config);

	if (!active) {
		/* bigjoiner slave doesn't enable transcoder */
		if (!pipe_config->bigjoiner_slave)
			goto out;

		active = true;
		pipe_config->pixel_multiplier = 1;

		/* we cannot read out most state, so don't bother.. */
		pipe_config->quirks |= PIPE_CONFIG_QUIRK_BIGJOINER_SLAVE;
	} else if (!transcoder_is_dsi(pipe_config->cpu_transcoder) ||
	    INTEL_GEN(dev_priv) >= 11) {
		hsw_get_ddi_port_state(crtc, pipe_config);
		intel_get_transcoder_timings(crtc, pipe_config);
	}

	intel_get_pipe_src_size(crtc, pipe_config);

	if (IS_HASWELL(dev_priv)) {
		u32 tmp = intel_de_read(dev_priv,
					PIPECONF(pipe_config->cpu_transcoder));

		if (tmp & PIPECONF_OUTPUT_COLORSPACE_YUV_HSW)
			pipe_config->output_format = INTEL_OUTPUT_FORMAT_YCBCR444;
		else
			pipe_config->output_format = INTEL_OUTPUT_FORMAT_RGB;
	} else {
		pipe_config->output_format =
			bdw_get_pipemisc_output_format(crtc);
	}

	pipe_config->gamma_mode = intel_de_read(dev_priv,
						GAMMA_MODE(crtc->pipe));

	pipe_config->csc_mode = intel_de_read(dev_priv,
					      PIPE_CSC_MODE(crtc->pipe));

	if (INTEL_GEN(dev_priv) >= 9) {
		tmp = intel_de_read(dev_priv, SKL_BOTTOM_COLOR(crtc->pipe));

		if (tmp & SKL_BOTTOM_COLOR_GAMMA_ENABLE)
			pipe_config->gamma_enable = true;

		if (tmp & SKL_BOTTOM_COLOR_CSC_ENABLE)
			pipe_config->csc_enable = true;
	} else {
		i9xx_get_pipe_color_config(pipe_config);
	}

	intel_color_get_config(pipe_config);

	tmp = intel_de_read(dev_priv, WM_LINETIME(crtc->pipe));
	pipe_config->linetime = REG_FIELD_GET(HSW_LINETIME_MASK, tmp);
	if (IS_BROADWELL(dev_priv) || IS_HASWELL(dev_priv))
		pipe_config->ips_linetime =
			REG_FIELD_GET(HSW_IPS_LINETIME_MASK, tmp);

	if (intel_display_power_get_in_set_if_enabled(dev_priv, &power_domain_set,
						      POWER_DOMAIN_PIPE_PANEL_FITTER(crtc->pipe))) {
		if (INTEL_GEN(dev_priv) >= 9)
			skl_get_pfit_config(pipe_config);
		else
			ilk_get_pfit_config(pipe_config);
	}

	if (hsw_crtc_supports_ips(crtc)) {
		if (IS_HASWELL(dev_priv))
			pipe_config->ips_enabled = intel_de_read(dev_priv,
								 IPS_CTL) & IPS_ENABLE;
		else {
			/*
			 * We cannot readout IPS state on broadwell, set to
			 * true so we can set it to a defined state on first
			 * commit.
			 */
			pipe_config->ips_enabled = true;
		}
	}

	if (pipe_config->bigjoiner_slave) {
		/* Cannot be read out as a slave, set to 0. */
		pipe_config->pixel_multiplier = 0;
	} else if (pipe_config->cpu_transcoder != TRANSCODER_EDP &&
	    !transcoder_is_dsi(pipe_config->cpu_transcoder)) {
		pipe_config->pixel_multiplier =
			intel_de_read(dev_priv,
				      PIPE_MULT(pipe_config->cpu_transcoder)) + 1;
	} else {
		pipe_config->pixel_multiplier = 1;
	}

out:
	intel_display_power_put_all_in_set(dev_priv, &power_domain_set);

	return active;
}

bool intel_crtc_get_pipe_config(struct intel_crtc_state *crtc_state)
{
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->uapi.crtc);
	struct drm_i915_private *i915 = to_i915(crtc->base.dev);

	if (!i915->display.get_pipe_config(crtc, crtc_state))
		return false;

	crtc_state->hw.active = true;

	intel_crtc_readout_derived_state(crtc_state);

	return true;
}

void
intel_crtc_init_hooks(struct drm_i915_private *dev_priv)
{
	if (INTEL_GEN(dev_priv) >= 9) {
		dev_priv->display.get_pipe_config = hsw_get_pipe_config;
		dev_priv->display.crtc_enable = hsw_crtc_enable;
		dev_priv->display.crtc_disable = hsw_crtc_disable;
	} else if (HAS_DDI(dev_priv)) {
		dev_priv->display.get_pipe_config = hsw_get_pipe_config;
		dev_priv->display.crtc_enable = hsw_crtc_enable;
		dev_priv->display.crtc_disable = hsw_crtc_disable;
	} else if (HAS_PCH_SPLIT(dev_priv)) {
		dev_priv->display.get_pipe_config = ilk_get_pipe_config;
		dev_priv->display.crtc_enable = ilk_crtc_enable;
		dev_priv->display.crtc_disable = ilk_crtc_disable;
	} else if (IS_CHERRYVIEW(dev_priv) ||
		   IS_VALLEYVIEW(dev_priv)) {
		dev_priv->display.get_pipe_config = i9xx_get_pipe_config;
		dev_priv->display.crtc_enable = valleyview_crtc_enable;
		dev_priv->display.crtc_disable = i9xx_crtc_disable;
	} else {
		dev_priv->display.get_pipe_config = i9xx_get_pipe_config;
		dev_priv->display.crtc_enable = i9xx_crtc_enable;
		dev_priv->display.crtc_disable = i9xx_crtc_disable;
	}
}
