/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2021 Intel Corporation
 */

#include "xe_pci.h"

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/vga_switcheroo.h>

#include <drm/drm_drv.h>
#include <drm/i915_pciids.h>
#include <drm/drm_color_mgmt.h>

#include "xe_drv.h"
#include "xe_device.h"

#include "../i915/i915_reg.h"

/* Keep in gen based order, and chronological order within a gen */
enum intel_platform {
	INTEL_PLATFORM_UNINITIALIZED = 0,
	/* gen2 */
	INTEL_I830,
	INTEL_I845G,
	INTEL_I85X,
	INTEL_I865G,
	/* gen3 */
	INTEL_I915G,
	INTEL_I915GM,
	INTEL_I945G,
	INTEL_I945GM,
	INTEL_G33,
	INTEL_PINEVIEW,
	/* gen4 */
	INTEL_I965G,
	INTEL_I965GM,
	INTEL_G45,
	INTEL_GM45,
	/* gen5 */
	INTEL_IRONLAKE,
	/* gen6 */
	INTEL_SANDYBRIDGE,
	/* gen7 */
	INTEL_IVYBRIDGE,
	INTEL_VALLEYVIEW,
	INTEL_HASWELL,
	/* gen8 */
	INTEL_BROADWELL,
	INTEL_CHERRYVIEW,
	/* gen9 */
	INTEL_SKYLAKE,
	INTEL_BROXTON,
	INTEL_KABYLAKE,
	INTEL_GEMINILAKE,
	INTEL_COFFEELAKE,
	INTEL_COMETLAKE,
	/* gen10 */
	INTEL_CANNONLAKE,
	/* gen11 */
	INTEL_ICELAKE,
	INTEL_ELKHARTLAKE,
	INTEL_JASPERLAKE,
	/* gen12 */
	INTEL_TIGERLAKE,
	INTEL_ROCKETLAKE,
	INTEL_DG1,
	INTEL_ALDERLAKE_S,
	INTEL_ALDERLAKE_P,
	INTEL_XEHPSDV,
	INTEL_DG2,
	INTEL_MAX_PLATFORMS
};

enum intel_ppgtt_type {
	INTEL_PPGTT_NONE = 0,
	INTEL_PPGTT_ALIASING = 1,
	INTEL_PPGTT_FULL = 2,
};

enum pipe {
	INVALID_PIPE = -1,

	PIPE_A = 0,
	PIPE_B,
	PIPE_C,
	PIPE_D,
	_PIPE_EDP,

	I915_MAX_PIPES = _PIPE_EDP
};

/*
 * FIXME: We should probably switch this to a 0-based scheme to be consistent
 * with how we now name/number DBUF_CTL instances.
 */
enum dbuf_slice {
	DBUF_S1,
	DBUF_S2,
	DBUF_S3,
	DBUF_S4,
	I915_MAX_DBUF_SLICES
};


#define I915_GTT_PAGE_SIZE_4K	BIT_ULL(12)
#define I915_GTT_PAGE_SIZE_64K	BIT_ULL(16)
#define I915_GTT_PAGE_SIZE_2M	BIT_ULL(21)


enum intel_region_id {
	INTEL_REGION_SMEM = 0,
	INTEL_REGION_LMEM,
	INTEL_REGION_STOLEN_SMEM,
	INTEL_REGION_STOLEN_LMEM,
	INTEL_REGION_UNKNOWN, /* Should be last */
};

#define REGION_SMEM     BIT(INTEL_REGION_SMEM)
#define REGION_LMEM     BIT(INTEL_REGION_LMEM)
#define REGION_STOLEN_SMEM   BIT(INTEL_REGION_STOLEN_SMEM)
#define REGION_STOLEN_LMEM   BIT(INTEL_REGION_STOLEN_LMEM)


/*
 * Engine IDs definitions.
 * Keep instances of the same type engine together.
 */
enum intel_engine_id {
	RCS0 = 0,
	BCS0,
	VCS0,
	VCS1,
	VCS2,
	VCS3,
	VCS4,
	VCS5,
	VCS6,
	VCS7,
#define _VCS(n) (VCS0 + (n))
	VECS0,
	VECS1,
	VECS2,
	VECS3,
#define _VECS(n) (VECS0 + (n))
	I915_NUM_ENGINES
#define INVALID_ENGINE ((enum intel_engine_id)-1)
};


enum transcoder {
	INVALID_TRANSCODER = -1,
	/*
	 * The following transcoders have a 1:1 transcoder -> pipe mapping,
	 * keep their values fixed: the code assumes that TRANSCODER_A=0, the
	 * rest have consecutive values and match the enum values of the pipes
	 * they map to.
	 */
	TRANSCODER_A = PIPE_A,
	TRANSCODER_B = PIPE_B,
	TRANSCODER_C = PIPE_C,
	TRANSCODER_D = PIPE_D,

	/*
	 * The following transcoders can map to any pipe, their enum value
	 * doesn't need to stay fixed.
	 */
	TRANSCODER_EDP,
	TRANSCODER_DSI_0,
	TRANSCODER_DSI_1,
	TRANSCODER_DSI_A = TRANSCODER_DSI_0,	/* legacy DSI */
	TRANSCODER_DSI_C = TRANSCODER_DSI_1,	/* legacy DSI */

	I915_MAX_TRANSCODERS
};

typedef u32 intel_engine_mask_t;

#define DEV_INFO_FOR_EACH_FLAG(func) \
	func(is_mobile); \
	func(is_lp); \
	func(require_force_probe); \
	func(is_dgfx); \
	/* Keep has_* in alphabetical order */ \
	func(has_64bit_reloc); \
	func(gpu_reset_clobbers_display); \
	func(has_reset_engine); \
	func(has_global_mocs); \
	func(has_gt_uc); \
	func(has_l3_dpf); \
	func(has_llc); \
	func(has_logical_ring_contexts); \
	func(has_logical_ring_elsq); \
	func(has_pooled_eu); \
	func(has_rc6); \
	func(has_rc6p); \
	func(has_rps); \
	func(has_runtime_pm); \
	func(has_snoop); \
	func(has_coherent_ggtt); \
	func(unfenced_needs_alignment); \
	func(hws_needs_physical);

#define DEV_INFO_DISPLAY_FOR_EACH_FLAG(func) \
	/* Keep in alphabetical order */ \
	func(cursor_needs_physical); \
	func(has_cdclk_crawl); \
	func(has_dmc); \
	func(has_ddi); \
	func(has_dp_mst); \
	func(has_dsb); \
	func(has_dsc); \
	func(has_fbc); \
	func(has_fpga_dbg); \
	func(has_gmch); \
	func(has_hdcp); \
	func(has_hotplug); \
	func(has_hti); \
	func(has_ipc); \
	func(has_modular_fia); \
	func(has_overlay); \
	func(has_psr); \
	func(has_psr_hw_tracking); \
	func(overlay_needs_physical); \
	func(supports_tv);

struct intel_device_info {
	u8 graphics_ver;
	u8 graphics_rel;
	u8 media_ver;
	u8 media_rel;

	intel_engine_mask_t platform_engine_mask; /* Engines supported by the HW */

	enum intel_platform platform;

	unsigned int dma_mask_size; /* available DMA address bits */

	enum intel_ppgtt_type ppgtt_type;
	unsigned int ppgtt_size; /* log2, e.g. 31/32/48 bits */

	unsigned int page_sizes; /* page sizes supported by the HW */

	u32 memory_regions; /* regions supported by the HW */

	u32 display_mmio_offset;

	u8 gt; /* GT number, 0 if undefined */

	u8 pipe_mask;
	u8 cpu_transcoder_mask;

	u8 abox_mask;

#define DEFINE_FLAG(name) u8 name:1
	DEV_INFO_FOR_EACH_FLAG(DEFINE_FLAG);
#undef DEFINE_FLAG

	struct {
		u8 ver;

#define DEFINE_FLAG(name) u8 name:1
		DEV_INFO_DISPLAY_FOR_EACH_FLAG(DEFINE_FLAG);
#undef DEFINE_FLAG
	} display;

	struct {
		u16 size; /* in blocks */
		u8 slice_mask;
	} dbuf;

	/* Register offsets for the various display pipes and transcoders */
	int pipe_offsets[I915_MAX_TRANSCODERS];
	int trans_offsets[I915_MAX_TRANSCODERS];
	int cursor_offsets[I915_MAX_PIPES];

	struct color_luts {
		u32 degamma_lut_size;
		u32 gamma_lut_size;
		u32 degamma_lut_tests;
		u32 gamma_lut_tests;
	} color;
};

#define PLATFORM(x) .platform = (x)
#define GEN(x) \
	.graphics_ver = (x), \
	.media_ver = (x), \
	.display.ver = (x)

#define I845_PIPE_OFFSETS \
	.pipe_offsets = { \
		[TRANSCODER_A] = PIPE_A_OFFSET,	\
	}, \
	.trans_offsets = { \
		[TRANSCODER_A] = TRANSCODER_A_OFFSET, \
	}

#define I9XX_PIPE_OFFSETS \
	.pipe_offsets = { \
		[TRANSCODER_A] = PIPE_A_OFFSET,	\
		[TRANSCODER_B] = PIPE_B_OFFSET, \
	}, \
	.trans_offsets = { \
		[TRANSCODER_A] = TRANSCODER_A_OFFSET, \
		[TRANSCODER_B] = TRANSCODER_B_OFFSET, \
	}

#define IVB_PIPE_OFFSETS \
	.pipe_offsets = { \
		[TRANSCODER_A] = PIPE_A_OFFSET,	\
		[TRANSCODER_B] = PIPE_B_OFFSET, \
		[TRANSCODER_C] = PIPE_C_OFFSET, \
	}, \
	.trans_offsets = { \
		[TRANSCODER_A] = TRANSCODER_A_OFFSET, \
		[TRANSCODER_B] = TRANSCODER_B_OFFSET, \
		[TRANSCODER_C] = TRANSCODER_C_OFFSET, \
	}

#define HSW_PIPE_OFFSETS \
	.pipe_offsets = { \
		[TRANSCODER_A] = PIPE_A_OFFSET,	\
		[TRANSCODER_B] = PIPE_B_OFFSET, \
		[TRANSCODER_C] = PIPE_C_OFFSET, \
		[TRANSCODER_EDP] = PIPE_EDP_OFFSET, \
	}, \
	.trans_offsets = { \
		[TRANSCODER_A] = TRANSCODER_A_OFFSET, \
		[TRANSCODER_B] = TRANSCODER_B_OFFSET, \
		[TRANSCODER_C] = TRANSCODER_C_OFFSET, \
		[TRANSCODER_EDP] = TRANSCODER_EDP_OFFSET, \
	}

#define CHV_PIPE_OFFSETS \
	.pipe_offsets = { \
		[TRANSCODER_A] = PIPE_A_OFFSET, \
		[TRANSCODER_B] = PIPE_B_OFFSET, \
		[TRANSCODER_C] = CHV_PIPE_C_OFFSET, \
	}, \
	.trans_offsets = { \
		[TRANSCODER_A] = TRANSCODER_A_OFFSET, \
		[TRANSCODER_B] = TRANSCODER_B_OFFSET, \
		[TRANSCODER_C] = CHV_TRANSCODER_C_OFFSET, \
	}

#define I845_CURSOR_OFFSETS \
	.cursor_offsets = { \
		[PIPE_A] = CURSOR_A_OFFSET, \
	}

#define I9XX_CURSOR_OFFSETS \
	.cursor_offsets = { \
		[PIPE_A] = CURSOR_A_OFFSET, \
		[PIPE_B] = CURSOR_B_OFFSET, \
	}

#define CHV_CURSOR_OFFSETS \
	.cursor_offsets = { \
		[PIPE_A] = CURSOR_A_OFFSET, \
		[PIPE_B] = CURSOR_B_OFFSET, \
		[PIPE_C] = CHV_CURSOR_C_OFFSET, \
	}

#define IVB_CURSOR_OFFSETS \
	.cursor_offsets = { \
		[PIPE_A] = CURSOR_A_OFFSET, \
		[PIPE_B] = IVB_CURSOR_B_OFFSET, \
		[PIPE_C] = IVB_CURSOR_C_OFFSET, \
	}

#define TGL_CURSOR_OFFSETS \
	.cursor_offsets = { \
		[PIPE_A] = CURSOR_A_OFFSET, \
		[PIPE_B] = IVB_CURSOR_B_OFFSET, \
		[PIPE_C] = IVB_CURSOR_C_OFFSET, \
		[PIPE_D] = TGL_CURSOR_D_OFFSET, \
	}

#define I9XX_COLORS \
	.color = { .gamma_lut_size = 256 }
#define I965_COLORS \
	.color = { .gamma_lut_size = 129, \
		   .gamma_lut_tests = DRM_COLOR_LUT_NON_DECREASING, \
	}
#define ILK_COLORS \
	.color = { .gamma_lut_size = 1024 }
#define IVB_COLORS \
	.color = { .degamma_lut_size = 1024, .gamma_lut_size = 1024 }
#define CHV_COLORS \
	.color = { .degamma_lut_size = 65, .gamma_lut_size = 257, \
		   .degamma_lut_tests = DRM_COLOR_LUT_NON_DECREASING, \
		   .gamma_lut_tests = DRM_COLOR_LUT_NON_DECREASING, \
	}
#define GLK_COLORS \
	.color = { .degamma_lut_size = 33, .gamma_lut_size = 1024, \
		   .degamma_lut_tests = DRM_COLOR_LUT_NON_DECREASING | \
					DRM_COLOR_LUT_EQUAL_CHANNELS, \
	}

/* Keep in gen based order, and chronological order within a gen */

#define GEN_DEFAULT_PAGE_SIZES \
	.page_sizes = I915_GTT_PAGE_SIZE_4K

#define GEN_DEFAULT_REGIONS \
	.memory_regions = REGION_SMEM | REGION_STOLEN_SMEM

#define I830_FEATURES \
	GEN(2), \
	.is_mobile = 1, \
	.pipe_mask = BIT(PIPE_A) | BIT(PIPE_B), \
	.cpu_transcoder_mask = BIT(TRANSCODER_A) | BIT(TRANSCODER_B), \
	.display.has_overlay = 1, \
	.display.cursor_needs_physical = 1, \
	.display.overlay_needs_physical = 1, \
	.display.has_gmch = 1, \
	.gpu_reset_clobbers_display = true, \
	.hws_needs_physical = 1, \
	.unfenced_needs_alignment = 1, \
	.platform_engine_mask = BIT(RCS0), \
	.has_snoop = true, \
	.has_coherent_ggtt = false, \
	.dma_mask_size = 32, \
	I9XX_PIPE_OFFSETS, \
	I9XX_CURSOR_OFFSETS, \
	I9XX_COLORS, \
	GEN_DEFAULT_PAGE_SIZES, \
	GEN_DEFAULT_REGIONS

#define I845_FEATURES \
	GEN(2), \
	.pipe_mask = BIT(PIPE_A), \
	.cpu_transcoder_mask = BIT(TRANSCODER_A), \
	.display.has_overlay = 1, \
	.display.overlay_needs_physical = 1, \
	.display.has_gmch = 1, \
	.gpu_reset_clobbers_display = true, \
	.hws_needs_physical = 1, \
	.unfenced_needs_alignment = 1, \
	.platform_engine_mask = BIT(RCS0), \
	.has_snoop = true, \
	.has_coherent_ggtt = false, \
	.dma_mask_size = 32, \
	I845_PIPE_OFFSETS, \
	I845_CURSOR_OFFSETS, \
	I9XX_COLORS, \
	GEN_DEFAULT_PAGE_SIZES, \
	GEN_DEFAULT_REGIONS

static const struct intel_device_info i830_info = {
	I830_FEATURES,
	PLATFORM(INTEL_I830),
};

static const struct intel_device_info i845g_info = {
	I845_FEATURES,
	PLATFORM(INTEL_I845G),
};

static const struct intel_device_info i85x_info = {
	I830_FEATURES,
	PLATFORM(INTEL_I85X),
	.display.has_fbc = 1,
};

static const struct intel_device_info i865g_info = {
	I845_FEATURES,
	PLATFORM(INTEL_I865G),
	.display.has_fbc = 1,
};

#define GEN3_FEATURES \
	GEN(3), \
	.pipe_mask = BIT(PIPE_A) | BIT(PIPE_B), \
	.cpu_transcoder_mask = BIT(TRANSCODER_A) | BIT(TRANSCODER_B), \
	.display.has_gmch = 1, \
	.gpu_reset_clobbers_display = true, \
	.platform_engine_mask = BIT(RCS0), \
	.has_snoop = true, \
	.has_coherent_ggtt = true, \
	.dma_mask_size = 32, \
	I9XX_PIPE_OFFSETS, \
	I9XX_CURSOR_OFFSETS, \
	I9XX_COLORS, \
	GEN_DEFAULT_PAGE_SIZES, \
	GEN_DEFAULT_REGIONS

static const struct intel_device_info i915g_info = {
	GEN3_FEATURES,
	PLATFORM(INTEL_I915G),
	.has_coherent_ggtt = false,
	.display.cursor_needs_physical = 1,
	.display.has_overlay = 1,
	.display.overlay_needs_physical = 1,
	.hws_needs_physical = 1,
	.unfenced_needs_alignment = 1,
};

static const struct intel_device_info i915gm_info = {
	GEN3_FEATURES,
	PLATFORM(INTEL_I915GM),
	.is_mobile = 1,
	.display.cursor_needs_physical = 1,
	.display.has_overlay = 1,
	.display.overlay_needs_physical = 1,
	.display.supports_tv = 1,
	.display.has_fbc = 1,
	.hws_needs_physical = 1,
	.unfenced_needs_alignment = 1,
};

static const struct intel_device_info i945g_info = {
	GEN3_FEATURES,
	PLATFORM(INTEL_I945G),
	.display.has_hotplug = 1,
	.display.cursor_needs_physical = 1,
	.display.has_overlay = 1,
	.display.overlay_needs_physical = 1,
	.hws_needs_physical = 1,
	.unfenced_needs_alignment = 1,
};

static const struct intel_device_info i945gm_info = {
	GEN3_FEATURES,
	PLATFORM(INTEL_I945GM),
	.is_mobile = 1,
	.display.has_hotplug = 1,
	.display.cursor_needs_physical = 1,
	.display.has_overlay = 1,
	.display.overlay_needs_physical = 1,
	.display.supports_tv = 1,
	.display.has_fbc = 1,
	.hws_needs_physical = 1,
	.unfenced_needs_alignment = 1,
};

static const struct intel_device_info g33_info = {
	GEN3_FEATURES,
	PLATFORM(INTEL_G33),
	.display.has_hotplug = 1,
	.display.has_overlay = 1,
	.dma_mask_size = 36,
};

static const struct intel_device_info pnv_g_info = {
	GEN3_FEATURES,
	PLATFORM(INTEL_PINEVIEW),
	.display.has_hotplug = 1,
	.display.has_overlay = 1,
	.dma_mask_size = 36,
};

static const struct intel_device_info pnv_m_info = {
	GEN3_FEATURES,
	PLATFORM(INTEL_PINEVIEW),
	.is_mobile = 1,
	.display.has_hotplug = 1,
	.display.has_overlay = 1,
	.dma_mask_size = 36,
};

#define GEN4_FEATURES \
	GEN(4), \
	.pipe_mask = BIT(PIPE_A) | BIT(PIPE_B), \
	.cpu_transcoder_mask = BIT(TRANSCODER_A) | BIT(TRANSCODER_B), \
	.display.has_hotplug = 1, \
	.display.has_gmch = 1, \
	.gpu_reset_clobbers_display = true, \
	.platform_engine_mask = BIT(RCS0), \
	.has_snoop = true, \
	.has_coherent_ggtt = true, \
	.dma_mask_size = 36, \
	I9XX_PIPE_OFFSETS, \
	I9XX_CURSOR_OFFSETS, \
	I965_COLORS, \
	GEN_DEFAULT_PAGE_SIZES, \
	GEN_DEFAULT_REGIONS

static const struct intel_device_info i965g_info = {
	GEN4_FEATURES,
	PLATFORM(INTEL_I965G),
	.display.has_overlay = 1,
	.hws_needs_physical = 1,
	.has_snoop = false,
};

static const struct intel_device_info i965gm_info = {
	GEN4_FEATURES,
	PLATFORM(INTEL_I965GM),
	.is_mobile = 1,
	.display.has_fbc = 1,
	.display.has_overlay = 1,
	.display.supports_tv = 1,
	.hws_needs_physical = 1,
	.has_snoop = false,
};

static const struct intel_device_info g45_info = {
	GEN4_FEATURES,
	PLATFORM(INTEL_G45),
	.platform_engine_mask = BIT(RCS0) | BIT(VCS0),
	.gpu_reset_clobbers_display = false,
};

static const struct intel_device_info gm45_info = {
	GEN4_FEATURES,
	PLATFORM(INTEL_GM45),
	.is_mobile = 1,
	.display.has_fbc = 1,
	.display.supports_tv = 1,
	.platform_engine_mask = BIT(RCS0) | BIT(VCS0),
	.gpu_reset_clobbers_display = false,
};

#define GEN5_FEATURES \
	GEN(5), \
	.pipe_mask = BIT(PIPE_A) | BIT(PIPE_B), \
	.cpu_transcoder_mask = BIT(TRANSCODER_A) | BIT(TRANSCODER_B), \
	.display.has_hotplug = 1, \
	.platform_engine_mask = BIT(RCS0) | BIT(VCS0), \
	.has_snoop = true, \
	.has_coherent_ggtt = true, \
	/* ilk does support rc6, but we do not implement [power] contexts */ \
	.has_rc6 = 0, \
	.dma_mask_size = 36, \
	I9XX_PIPE_OFFSETS, \
	I9XX_CURSOR_OFFSETS, \
	ILK_COLORS, \
	GEN_DEFAULT_PAGE_SIZES, \
	GEN_DEFAULT_REGIONS

static const struct intel_device_info ilk_d_info = {
	GEN5_FEATURES,
	PLATFORM(INTEL_IRONLAKE),
};

static const struct intel_device_info ilk_m_info = {
	GEN5_FEATURES,
	PLATFORM(INTEL_IRONLAKE),
	.is_mobile = 1,
	.has_rps = true,
	.display.has_fbc = 1,
};

#define GEN6_FEATURES \
	GEN(6), \
	.pipe_mask = BIT(PIPE_A) | BIT(PIPE_B), \
	.cpu_transcoder_mask = BIT(TRANSCODER_A) | BIT(TRANSCODER_B), \
	.display.has_hotplug = 1, \
	.display.has_fbc = 1, \
	.platform_engine_mask = BIT(RCS0) | BIT(VCS0) | BIT(BCS0), \
	.has_coherent_ggtt = true, \
	.has_llc = 1, \
	.has_rc6 = 1, \
	.has_rc6p = 1, \
	.has_rps = true, \
	.dma_mask_size = 40, \
	.ppgtt_type = INTEL_PPGTT_ALIASING, \
	.ppgtt_size = 31, \
	I9XX_PIPE_OFFSETS, \
	I9XX_CURSOR_OFFSETS, \
	ILK_COLORS, \
	GEN_DEFAULT_PAGE_SIZES, \
	GEN_DEFAULT_REGIONS

#define SNB_D_PLATFORM \
	GEN6_FEATURES, \
	PLATFORM(INTEL_SANDYBRIDGE)

static const struct intel_device_info snb_d_gt1_info = {
	SNB_D_PLATFORM,
	.gt = 1,
};

static const struct intel_device_info snb_d_gt2_info = {
	SNB_D_PLATFORM,
	.gt = 2,
};

#define SNB_M_PLATFORM \
	GEN6_FEATURES, \
	PLATFORM(INTEL_SANDYBRIDGE), \
	.is_mobile = 1


static const struct intel_device_info snb_m_gt1_info = {
	SNB_M_PLATFORM,
	.gt = 1,
};

static const struct intel_device_info snb_m_gt2_info = {
	SNB_M_PLATFORM,
	.gt = 2,
};

#define GEN7_FEATURES  \
	GEN(7), \
	.pipe_mask = BIT(PIPE_A) | BIT(PIPE_B) | BIT(PIPE_C), \
	.cpu_transcoder_mask = BIT(TRANSCODER_A) | BIT(TRANSCODER_B) | BIT(TRANSCODER_C), \
	.display.has_hotplug = 1, \
	.display.has_fbc = 1, \
	.platform_engine_mask = BIT(RCS0) | BIT(VCS0) | BIT(BCS0), \
	.has_coherent_ggtt = true, \
	.has_llc = 1, \
	.has_rc6 = 1, \
	.has_rc6p = 1, \
	.has_reset_engine = true, \
	.has_rps = true, \
	.dma_mask_size = 40, \
	.ppgtt_type = INTEL_PPGTT_ALIASING, \
	.ppgtt_size = 31, \
	IVB_PIPE_OFFSETS, \
	IVB_CURSOR_OFFSETS, \
	IVB_COLORS, \
	GEN_DEFAULT_PAGE_SIZES, \
	GEN_DEFAULT_REGIONS

#define IVB_D_PLATFORM \
	GEN7_FEATURES, \
	PLATFORM(INTEL_IVYBRIDGE), \
	.has_l3_dpf = 1

static const struct intel_device_info ivb_d_gt1_info = {
	IVB_D_PLATFORM,
	.gt = 1,
};

static const struct intel_device_info ivb_d_gt2_info = {
	IVB_D_PLATFORM,
	.gt = 2,
};

#define IVB_M_PLATFORM \
	GEN7_FEATURES, \
	PLATFORM(INTEL_IVYBRIDGE), \
	.is_mobile = 1, \
	.has_l3_dpf = 1

static const struct intel_device_info ivb_m_gt1_info = {
	IVB_M_PLATFORM,
	.gt = 1,
};

static const struct intel_device_info ivb_m_gt2_info = {
	IVB_M_PLATFORM,
	.gt = 2,
};

static const struct intel_device_info ivb_q_info = {
	GEN7_FEATURES,
	PLATFORM(INTEL_IVYBRIDGE),
	.gt = 2,
	.pipe_mask = 0, /* legal, last one wins */
	.cpu_transcoder_mask = 0,
	.has_l3_dpf = 1,
};

static const struct intel_device_info vlv_info = {
	PLATFORM(INTEL_VALLEYVIEW),
	GEN(7),
	.is_lp = 1,
	.pipe_mask = BIT(PIPE_A) | BIT(PIPE_B),
	.cpu_transcoder_mask = BIT(TRANSCODER_A) | BIT(TRANSCODER_B),
	.has_runtime_pm = 1,
	.has_rc6 = 1,
	.has_reset_engine = true,
	.has_rps = true,
	.display.has_gmch = 1,
	.display.has_hotplug = 1,
	.dma_mask_size = 40,
	.ppgtt_type = INTEL_PPGTT_ALIASING,
	.ppgtt_size = 31,
	.has_snoop = true,
	.has_coherent_ggtt = false,
	.platform_engine_mask = BIT(RCS0) | BIT(VCS0) | BIT(BCS0),
	.display_mmio_offset = VLV_DISPLAY_BASE,
	I9XX_PIPE_OFFSETS,
	I9XX_CURSOR_OFFSETS,
	I965_COLORS,
	GEN_DEFAULT_PAGE_SIZES,
	GEN_DEFAULT_REGIONS,
};

#define G75_FEATURES  \
	GEN7_FEATURES, \
	.platform_engine_mask = BIT(RCS0) | BIT(VCS0) | BIT(BCS0) | BIT(VECS0), \
	.cpu_transcoder_mask = BIT(TRANSCODER_A) | BIT(TRANSCODER_B) | \
		BIT(TRANSCODER_C) | BIT(TRANSCODER_EDP), \
	.display.has_ddi = 1, \
	.display.has_fpga_dbg = 1, \
	.display.has_psr = 1, \
	.display.has_psr_hw_tracking = 1, \
	.display.has_dp_mst = 1, \
	.has_rc6p = 0 /* RC6p removed-by HSW */, \
	HSW_PIPE_OFFSETS, \
	.has_runtime_pm = 1

#define HSW_PLATFORM \
	G75_FEATURES, \
	PLATFORM(INTEL_HASWELL), \
	.has_l3_dpf = 1

static const struct intel_device_info hsw_gt1_info = {
	HSW_PLATFORM,
	.gt = 1,
};

static const struct intel_device_info hsw_gt2_info = {
	HSW_PLATFORM,
	.gt = 2,
};

static const struct intel_device_info hsw_gt3_info = {
	HSW_PLATFORM,
	.gt = 3,
};

#define GEN8_FEATURES \
	G75_FEATURES, \
	GEN(8), \
	.has_logical_ring_contexts = 1, \
	.dma_mask_size = 39, \
	.ppgtt_type = INTEL_PPGTT_FULL, \
	.ppgtt_size = 48, \
	.has_64bit_reloc = 1

#define BDW_PLATFORM \
	GEN8_FEATURES, \
	PLATFORM(INTEL_BROADWELL)

static const struct intel_device_info bdw_gt1_info = {
	BDW_PLATFORM,
	.gt = 1,
};

static const struct intel_device_info bdw_gt2_info = {
	BDW_PLATFORM,
	.gt = 2,
};

static const struct intel_device_info bdw_rsvd_info = {
	BDW_PLATFORM,
	.gt = 3,
	/* According to the device ID those devices are GT3, they were
	 * previously treated as not GT3, keep it like that.
	 */
};

static const struct intel_device_info bdw_gt3_info = {
	BDW_PLATFORM,
	.gt = 3,
	.platform_engine_mask =
		BIT(RCS0) | BIT(VCS0) | BIT(BCS0) | BIT(VECS0) | BIT(VCS1),
};

static const struct intel_device_info chv_info = {
	PLATFORM(INTEL_CHERRYVIEW),
	GEN(8),
	.pipe_mask = BIT(PIPE_A) | BIT(PIPE_B) | BIT(PIPE_C),
	.cpu_transcoder_mask = BIT(TRANSCODER_A) | BIT(TRANSCODER_B) | BIT(TRANSCODER_C),
	.display.has_hotplug = 1,
	.is_lp = 1,
	.platform_engine_mask = BIT(RCS0) | BIT(VCS0) | BIT(BCS0) | BIT(VECS0),
	.has_64bit_reloc = 1,
	.has_runtime_pm = 1,
	.has_rc6 = 1,
	.has_rps = true,
	.has_logical_ring_contexts = 1,
	.display.has_gmch = 1,
	.dma_mask_size = 39,
	.ppgtt_type = INTEL_PPGTT_FULL,
	.ppgtt_size = 32,
	.has_reset_engine = 1,
	.has_snoop = true,
	.has_coherent_ggtt = false,
	.display_mmio_offset = VLV_DISPLAY_BASE,
	CHV_PIPE_OFFSETS,
	CHV_CURSOR_OFFSETS,
	CHV_COLORS,
	GEN_DEFAULT_PAGE_SIZES,
	GEN_DEFAULT_REGIONS,
};

#define GEN9_DEFAULT_PAGE_SIZES \
	.page_sizes = I915_GTT_PAGE_SIZE_4K | \
		      I915_GTT_PAGE_SIZE_64K

#define GEN9_FEATURES \
	GEN8_FEATURES, \
	GEN(9), \
	GEN9_DEFAULT_PAGE_SIZES, \
	.display.has_dmc = 1, \
	.has_gt_uc = 1, \
	.display.has_hdcp = 1, \
	.display.has_ipc = 1, \
	.dbuf.size = 896 - 4, /* 4 blocks for bypass path allocation */ \
	.dbuf.slice_mask = BIT(DBUF_S1)

#define SKL_PLATFORM \
	GEN9_FEATURES, \
	PLATFORM(INTEL_SKYLAKE)

static const struct intel_device_info skl_gt1_info = {
	SKL_PLATFORM,
	.gt = 1,
};

static const struct intel_device_info skl_gt2_info = {
	SKL_PLATFORM,
	.gt = 2,
};

#define SKL_GT3_PLUS_PLATFORM \
	SKL_PLATFORM, \
	.platform_engine_mask = \
		BIT(RCS0) | BIT(VCS0) | BIT(BCS0) | BIT(VECS0) | BIT(VCS1)


static const struct intel_device_info skl_gt3_info = {
	SKL_GT3_PLUS_PLATFORM,
	.gt = 3,
};

static const struct intel_device_info skl_gt4_info = {
	SKL_GT3_PLUS_PLATFORM,
	.gt = 4,
};

#define GEN9_LP_FEATURES \
	GEN(9), \
	.is_lp = 1, \
	.dbuf.slice_mask = BIT(DBUF_S1), \
	.display.has_hotplug = 1, \
	.platform_engine_mask = BIT(RCS0) | BIT(VCS0) | BIT(BCS0) | BIT(VECS0), \
	.pipe_mask = BIT(PIPE_A) | BIT(PIPE_B) | BIT(PIPE_C), \
	.cpu_transcoder_mask = BIT(TRANSCODER_A) | BIT(TRANSCODER_B) | \
		BIT(TRANSCODER_C) | BIT(TRANSCODER_EDP) | \
		BIT(TRANSCODER_DSI_A) | BIT(TRANSCODER_DSI_C), \
	.has_64bit_reloc = 1, \
	.display.has_ddi = 1, \
	.display.has_fpga_dbg = 1, \
	.display.has_fbc = 1, \
	.display.has_hdcp = 1, \
	.display.has_psr = 1, \
	.display.has_psr_hw_tracking = 1, \
	.has_runtime_pm = 1, \
	.display.has_dmc = 1, \
	.has_rc6 = 1, \
	.has_rps = true, \
	.display.has_dp_mst = 1, \
	.has_logical_ring_contexts = 1, \
	.has_gt_uc = 1, \
	.dma_mask_size = 39, \
	.ppgtt_type = INTEL_PPGTT_FULL, \
	.ppgtt_size = 48, \
	.has_reset_engine = 1, \
	.has_snoop = true, \
	.has_coherent_ggtt = false, \
	.display.has_ipc = 1, \
	HSW_PIPE_OFFSETS, \
	IVB_CURSOR_OFFSETS, \
	IVB_COLORS, \
	GEN9_DEFAULT_PAGE_SIZES, \
	GEN_DEFAULT_REGIONS

static const struct intel_device_info bxt_info = {
	GEN9_LP_FEATURES,
	PLATFORM(INTEL_BROXTON),
	.dbuf.size = 512 - 4, /* 4 blocks for bypass path allocation */
};

static const struct intel_device_info glk_info = {
	GEN9_LP_FEATURES,
	PLATFORM(INTEL_GEMINILAKE),
	.display.ver = 10,
	.dbuf.size = 1024 - 4, /* 4 blocks for bypass path allocation */
	GLK_COLORS,
};

#define KBL_PLATFORM \
	GEN9_FEATURES, \
	PLATFORM(INTEL_KABYLAKE)

static const struct intel_device_info kbl_gt1_info = {
	KBL_PLATFORM,
	.gt = 1,
};

static const struct intel_device_info kbl_gt2_info = {
	KBL_PLATFORM,
	.gt = 2,
};

static const struct intel_device_info kbl_gt3_info = {
	KBL_PLATFORM,
	.gt = 3,
	.platform_engine_mask =
		BIT(RCS0) | BIT(VCS0) | BIT(BCS0) | BIT(VECS0) | BIT(VCS1),
};

#define CFL_PLATFORM \
	GEN9_FEATURES, \
	PLATFORM(INTEL_COFFEELAKE)

static const struct intel_device_info cfl_gt1_info = {
	CFL_PLATFORM,
	.gt = 1,
};

static const struct intel_device_info cfl_gt2_info = {
	CFL_PLATFORM,
	.gt = 2,
};

static const struct intel_device_info cfl_gt3_info = {
	CFL_PLATFORM,
	.gt = 3,
	.platform_engine_mask =
		BIT(RCS0) | BIT(VCS0) | BIT(BCS0) | BIT(VECS0) | BIT(VCS1),
};

#define CML_PLATFORM \
	GEN9_FEATURES, \
	PLATFORM(INTEL_COMETLAKE)

static const struct intel_device_info cml_gt1_info = {
	CML_PLATFORM,
	.gt = 1,
};

static const struct intel_device_info cml_gt2_info = {
	CML_PLATFORM,
	.gt = 2,
};

#define GEN10_FEATURES \
	GEN9_FEATURES, \
	GEN(10), \
	.dbuf.size = 1024 - 4, /* 4 blocks for bypass path allocation */ \
	.display.has_dsc = 1, \
	.has_coherent_ggtt = false, \
	GLK_COLORS

static const struct intel_device_info cnl_info = {
	GEN10_FEATURES,
	PLATFORM(INTEL_CANNONLAKE),
	.gt = 2,
};

#define GEN11_DEFAULT_PAGE_SIZES \
	.page_sizes = I915_GTT_PAGE_SIZE_4K | \
		      I915_GTT_PAGE_SIZE_64K | \
		      I915_GTT_PAGE_SIZE_2M

#define GEN11_FEATURES \
	GEN10_FEATURES, \
	GEN11_DEFAULT_PAGE_SIZES, \
	.abox_mask = BIT(0), \
	.cpu_transcoder_mask = BIT(TRANSCODER_A) | BIT(TRANSCODER_B) | \
		BIT(TRANSCODER_C) | BIT(TRANSCODER_EDP) | \
		BIT(TRANSCODER_DSI_0) | BIT(TRANSCODER_DSI_1), \
	.pipe_offsets = { \
		[TRANSCODER_A] = PIPE_A_OFFSET, \
		[TRANSCODER_B] = PIPE_B_OFFSET, \
		[TRANSCODER_C] = PIPE_C_OFFSET, \
		[TRANSCODER_EDP] = PIPE_EDP_OFFSET, \
		[TRANSCODER_DSI_0] = PIPE_DSI0_OFFSET, \
		[TRANSCODER_DSI_1] = PIPE_DSI1_OFFSET, \
	}, \
	.trans_offsets = { \
		[TRANSCODER_A] = TRANSCODER_A_OFFSET, \
		[TRANSCODER_B] = TRANSCODER_B_OFFSET, \
		[TRANSCODER_C] = TRANSCODER_C_OFFSET, \
		[TRANSCODER_EDP] = TRANSCODER_EDP_OFFSET, \
		[TRANSCODER_DSI_0] = TRANSCODER_DSI0_OFFSET, \
		[TRANSCODER_DSI_1] = TRANSCODER_DSI1_OFFSET, \
	}, \
	GEN(11), \
	.dbuf.size = 2048, \
	.dbuf.slice_mask = BIT(DBUF_S1) | BIT(DBUF_S2), \
	.has_logical_ring_elsq = 1, \
	.color = { .degamma_lut_size = 33, .gamma_lut_size = 262145 }

static const struct intel_device_info icl_info = {
	GEN11_FEATURES,
	PLATFORM(INTEL_ICELAKE),
	.platform_engine_mask =
		BIT(RCS0) | BIT(BCS0) | BIT(VECS0) | BIT(VCS0) | BIT(VCS2),
};

static const struct intel_device_info ehl_info = {
	GEN11_FEATURES,
	PLATFORM(INTEL_ELKHARTLAKE),
	.platform_engine_mask = BIT(RCS0) | BIT(BCS0) | BIT(VCS0) | BIT(VECS0),
	.ppgtt_size = 36,
};

static const struct intel_device_info jsl_info = {
	GEN11_FEATURES,
	PLATFORM(INTEL_JASPERLAKE),
	.platform_engine_mask = BIT(RCS0) | BIT(BCS0) | BIT(VCS0) | BIT(VECS0),
	.ppgtt_size = 36,
};

#define GEN12_FEATURES \
	GEN11_FEATURES, \
	GEN(12), \
	.abox_mask = GENMASK(2, 1), \
	.pipe_mask = BIT(PIPE_A) | BIT(PIPE_B) | BIT(PIPE_C) | BIT(PIPE_D), \
	.cpu_transcoder_mask = BIT(TRANSCODER_A) | BIT(TRANSCODER_B) | \
		BIT(TRANSCODER_C) | BIT(TRANSCODER_D) | \
		BIT(TRANSCODER_DSI_0) | BIT(TRANSCODER_DSI_1), \
	.pipe_offsets = { \
		[TRANSCODER_A] = PIPE_A_OFFSET, \
		[TRANSCODER_B] = PIPE_B_OFFSET, \
		[TRANSCODER_C] = PIPE_C_OFFSET, \
		[TRANSCODER_D] = PIPE_D_OFFSET, \
		[TRANSCODER_DSI_0] = PIPE_DSI0_OFFSET, \
		[TRANSCODER_DSI_1] = PIPE_DSI1_OFFSET, \
	}, \
	.trans_offsets = { \
		[TRANSCODER_A] = TRANSCODER_A_OFFSET, \
		[TRANSCODER_B] = TRANSCODER_B_OFFSET, \
		[TRANSCODER_C] = TRANSCODER_C_OFFSET, \
		[TRANSCODER_D] = TRANSCODER_D_OFFSET, \
		[TRANSCODER_DSI_0] = TRANSCODER_DSI0_OFFSET, \
		[TRANSCODER_DSI_1] = TRANSCODER_DSI1_OFFSET, \
	}, \
	TGL_CURSOR_OFFSETS, \
	.has_global_mocs = 1, \
	.display.has_dsb = 1

static const struct intel_device_info tgl_info = {
	GEN12_FEATURES,
	PLATFORM(INTEL_TIGERLAKE),
	.display.has_modular_fia = 1,
	.platform_engine_mask =
		BIT(RCS0) | BIT(BCS0) | BIT(VECS0) | BIT(VCS0) | BIT(VCS2),
};

static const struct intel_device_info rkl_info = {
	GEN12_FEATURES,
	PLATFORM(INTEL_ROCKETLAKE),
	.abox_mask = BIT(0),
	.pipe_mask = BIT(PIPE_A) | BIT(PIPE_B) | BIT(PIPE_C),
	.cpu_transcoder_mask = BIT(TRANSCODER_A) | BIT(TRANSCODER_B) |
		BIT(TRANSCODER_C),
	.display.has_hti = 1,
	.display.has_psr_hw_tracking = 0,
	.platform_engine_mask =
		BIT(RCS0) | BIT(BCS0) | BIT(VECS0) | BIT(VCS0),
};

#define DGFX_FEATURES \
	.memory_regions = REGION_SMEM | REGION_LMEM | REGION_STOLEN_LMEM, \
	.has_llc = 0, \
	.has_snoop = 1, \
	.is_dgfx = 1

static const struct intel_device_info dg1_info __maybe_unused = {
	GEN12_FEATURES,
	DGFX_FEATURES,
	.graphics_rel = 10,
	PLATFORM(INTEL_DG1),
	.pipe_mask = BIT(PIPE_A) | BIT(PIPE_B) | BIT(PIPE_C) | BIT(PIPE_D),
	.require_force_probe = 1,
	.platform_engine_mask =
		BIT(RCS0) | BIT(BCS0) | BIT(VECS0) |
		BIT(VCS0) | BIT(VCS2),
	/* Wa_16011227922 */
	.ppgtt_size = 47,
};

static const struct intel_device_info adl_s_info = {
	GEN12_FEATURES,
	PLATFORM(INTEL_ALDERLAKE_S),
	.pipe_mask = BIT(PIPE_A) | BIT(PIPE_B) | BIT(PIPE_C) | BIT(PIPE_D),
	.require_force_probe = 1,
	.display.has_hti = 1,
	.display.has_psr_hw_tracking = 0,
	.platform_engine_mask =
		BIT(RCS0) | BIT(BCS0) | BIT(VECS0) | BIT(VCS0) | BIT(VCS2),
	.dma_mask_size = 39,
};

#define XE_LPD_CURSOR_OFFSETS \
	.cursor_offsets = { \
		[PIPE_A] = CURSOR_A_OFFSET, \
		[PIPE_B] = IVB_CURSOR_B_OFFSET, \
		[PIPE_C] = IVB_CURSOR_C_OFFSET, \
		[PIPE_D] = TGL_CURSOR_D_OFFSET, \
	}

#define XE_LPD_FEATURES \
	.abox_mask = GENMASK(1, 0),						\
	.color = { .degamma_lut_size = 0, .gamma_lut_size = 0 },		\
	.cpu_transcoder_mask = BIT(TRANSCODER_A) | BIT(TRANSCODER_B) |		\
		BIT(TRANSCODER_C) | BIT(TRANSCODER_D),				\
	.dbuf.size = 4096,							\
	.dbuf.slice_mask = BIT(DBUF_S1) | BIT(DBUF_S2) | BIT(DBUF_S3) |		\
		BIT(DBUF_S4),							\
	.display.has_ddi = 1,							\
	.display.has_dmc = 1,							\
	.display.has_dp_mst = 1,						\
	.display.has_dsb = 1,							\
	.display.has_dsc = 1,							\
	.display.has_fbc = 1,							\
	.display.has_fpga_dbg = 1,						\
	.display.has_hdcp = 1,							\
	.display.has_hotplug = 1,						\
	.display.has_ipc = 1,							\
	.display.has_psr = 1,							\
	.display.ver = 13,							\
	.pipe_mask = BIT(PIPE_A) | BIT(PIPE_B) | BIT(PIPE_C) | BIT(PIPE_D),	\
	.pipe_offsets = {							\
		[TRANSCODER_A] = PIPE_A_OFFSET,					\
		[TRANSCODER_B] = PIPE_B_OFFSET,					\
		[TRANSCODER_C] = PIPE_C_OFFSET,					\
		[TRANSCODER_D] = PIPE_D_OFFSET,					\
	},									\
	.trans_offsets = {							\
		[TRANSCODER_A] = TRANSCODER_A_OFFSET,				\
		[TRANSCODER_B] = TRANSCODER_B_OFFSET,				\
		[TRANSCODER_C] = TRANSCODER_C_OFFSET,				\
		[TRANSCODER_D] = TRANSCODER_D_OFFSET,				\
	},									\
	XE_LPD_CURSOR_OFFSETS

static const struct intel_device_info adl_p_info = {
	GEN12_FEATURES,
	XE_LPD_FEATURES,
	PLATFORM(INTEL_ALDERLAKE_P),
	.require_force_probe = 1,
	.display.has_cdclk_crawl = 1,
	.display.has_modular_fia = 1,
	.display.has_psr_hw_tracking = 0,
	.platform_engine_mask =
		BIT(RCS0) | BIT(BCS0) | BIT(VECS0) | BIT(VCS0) | BIT(VCS2),
	.ppgtt_size = 48,
	.dma_mask_size = 39,
};

#undef GEN

#define XE_HP_PAGE_SIZES \
	.page_sizes = I915_GTT_PAGE_SIZE_4K | \
		      I915_GTT_PAGE_SIZE_64K | \
		      I915_GTT_PAGE_SIZE_2M

#define XE_HP_FEATURES \
	.graphics_ver = 12, \
	.graphics_rel = 50, \
	XE_HP_PAGE_SIZES, \
	.dma_mask_size = 46, \
	.has_64bit_reloc = 1, \
	.has_global_mocs = 1, \
	.has_gt_uc = 1, \
	.has_llc = 1, \
	.has_logical_ring_contexts = 1, \
	.has_logical_ring_elsq = 1, \
	.has_rc6 = 1, \
	.has_reset_engine = 1, \
	.has_rps = 1, \
	.has_runtime_pm = 1, \
	.ppgtt_size = 48, \
	.ppgtt_type = INTEL_PPGTT_FULL

#define XE_HPM_FEATURES \
	.media_ver = 12, \
	.media_rel = 50

__maybe_unused
static const struct intel_device_info xehpsdv_info = {
	XE_HP_FEATURES,
	XE_HPM_FEATURES,
	DGFX_FEATURES,
	PLATFORM(INTEL_XEHPSDV),
	.display = { },
	.pipe_mask = 0,
	.platform_engine_mask =
		BIT(RCS0) | BIT(BCS0) |
		BIT(VECS0) | BIT(VECS1) | BIT(VECS2) | BIT(VECS3) |
		BIT(VCS0) | BIT(VCS1) | BIT(VCS2) | BIT(VCS3) |
		BIT(VCS4) | BIT(VCS5) | BIT(VCS6) | BIT(VCS7),
	.require_force_probe = 1,
};

__maybe_unused
static const struct intel_device_info dg2_info = {
	XE_HP_FEATURES,
	XE_HPM_FEATURES,
	XE_LPD_FEATURES,
	DGFX_FEATURES,
	.graphics_rel = 55,
	.media_rel = 55,
	PLATFORM(INTEL_DG2),
	.platform_engine_mask =
		BIT(RCS0) | BIT(BCS0) |
		BIT(VECS0) | BIT(VECS1) |
		BIT(VCS0) | BIT(VCS2),
	.require_force_probe = 1,
};

#undef PLATFORM

/*
 * Make sure any device matches here are from most specific to most
 * general.  For example, since the Quanta match is based on the subsystem
 * and subvendor IDs, we need it to come before the more general IVB
 * PCI ID matches, otherwise we'll use the wrong info struct above.
 */
static const struct pci_device_id pciidlist[] = {
	INTEL_TGL_12_GT2_IDS(&tgl_info),
	INTEL_DG1_IDS(&dg1_info),
	{0, 0, 0}
};
MODULE_DEVICE_TABLE(pci, pciidlist);

static void xe_pci_remove(struct pci_dev *pdev)
{
	struct xe_device *xe;

	xe = pci_get_drvdata(pdev);
	if (!xe) /* driver load aborted, nothing to cleanup */
		return;

	xe_device_remove(xe);
	pci_set_drvdata(pdev, NULL);
}

static int xe_pci_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	const struct intel_device_info *devinfo = (void *)ent->driver_data;
	struct xe_device *xe;
	int err;

	/* Only bind to function 0 of the device. Early generations
	 * used function 1 as a placeholder for multi-head. This causes
	 * us confusion instead, especially on the systems where both
	 * functions have the same PCI-ID!
	 */
	if (PCI_FUNC(pdev->devfn))
		return -ENODEV;

	/*
	 * apple-gmux is needed on dual GPU MacBook Pro
	 * to probe the panel if we're the inactive GPU.
	 */
	if (vga_switcheroo_client_probe_defer(pdev))
		return -EPROBE_DEFER;

	xe = xe_device_create(pdev, ent);
	if (IS_ERR(xe))
		return PTR_ERR(xe);

	/* TODO: Once we sort out includes, embed the intel_device_info
	 * directly in xe_device
	 */
	XE_BUG_ON(devinfo->graphics_rel % 10);
	xe->info.graphics_verx10 = devinfo->graphics_ver * 10 +
				   devinfo->graphics_rel / 10;
	xe->info.is_dgfx = devinfo->is_dgfx;

	pci_set_drvdata(pdev, xe);
	err = pci_enable_device(pdev);
	if (err) {
		drm_dev_put(&xe->drm);
		return err;
	}

	if (pci_enable_msi(pdev) < 0)
		drm_dbg(&xe->drm, "can't enable MSI");

	err = xe_device_probe(xe);
	if (err) {
		pci_disable_device(pdev);
		return err;
	}

	return 0;
}

static void xe_pci_shutdown(struct pci_dev *pdev)
{
	xe_device_shutdown(pdev_to_xe_device(pdev));
}

static struct pci_driver i915_pci_driver = {
	.name = DRIVER_NAME,
	.id_table = pciidlist,
	.probe = xe_pci_probe,
	.remove = xe_pci_remove,
	.shutdown = xe_pci_shutdown,
//	.driver.pm = &xe_pm_ops,
};

int i915_register_pci_driver(void)
{
	return pci_register_driver(&i915_pci_driver);
}

void i915_unregister_pci_driver(void)
{
	pci_unregister_driver(&i915_pci_driver);
}
