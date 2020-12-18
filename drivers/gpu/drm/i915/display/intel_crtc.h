/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2020 Intel Corporation
 */

#ifndef _INTEL_CRTC_H_
#define _INTEL_CRTC_H_

#include <linux/types.h>

enum pipe;
struct drm_i915_private;
struct intel_crtc;
struct intel_crtc_state;
struct intel_atomic_state;

u32 intel_crtc_max_vblank_count(const struct intel_crtc_state *crtc_state);
int intel_crtc_init(struct drm_i915_private *dev_priv, enum pipe pipe);
struct intel_crtc_state *intel_crtc_state_alloc(struct intel_crtc *crtc);
void intel_crtc_state_reset(struct intel_crtc_state *crtc_state,
			    struct intel_crtc *crtc);

void intel_crtc_update_active_timings(const struct intel_crtc_state *crtc_state);
void intel_enable_crtc(struct intel_atomic_state *state,
		       struct intel_crtc *crtc);
void intel_update_crtc(struct intel_atomic_state *state,
		       struct intel_crtc *crtc);
bool intel_crtc_get_pipe_config(struct intel_crtc_state *crtc_state);

void intel_crtc_init_hooks(struct drm_i915_private *dev_priv);

void intel_wait_for_pipe_scanline_moving(struct intel_crtc *crtc);
void intel_wait_for_pipe_scanline_stopped(struct intel_crtc *crtc);
#endif
