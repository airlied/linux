/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2020 Intel Corporation
 */

#ifndef _SKL_UNIVERSAL_PLANE_H_
#define _SKL_UNIVERSAL_PLANE_H_

struct drm_framebuffer;
struct intel_atomic_state;
struct intel_crtc;
struct intel_crtc_state;
struct intel_initial_plane_config;
struct intel_plane_state;

enum pipe;
enum plane_id;

struct intel_plane *
skl_universal_plane_create(struct drm_i915_private *dev_priv,
			   enum pipe pipe, enum plane_id plane_id);

void skl_get_initial_plane_config(struct intel_crtc *crtc,
				  struct intel_initial_plane_config *plane_config);

int skl_format_to_fourcc(int format, bool rgb_order, bool alpha);

int skl_ccs_to_main_plane(const struct drm_framebuffer *fb, int ccs_plane);

#endif
