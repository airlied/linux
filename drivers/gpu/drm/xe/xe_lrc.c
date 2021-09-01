/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2021 Intel Corporation
 */

#include "xe_lrc.h"

#include "xe_bo.h"
#include "xe_device.h"

#include "../i915/i915_reg.h"
#include "../i915/gt/intel_gpu_commands.h"
#include "../i915/gt/intel_lrc_reg.h"

uint32_t lrc_size(struct xe_device *xe, enum xe_engine_class class)
{
	switch (class) {
	case XE_ENGINE_CLASS_RENDER:
		switch (GRAPHICS_VER(xe)) {
		case 12:
		case 11:
			return 14 * SZ_4K;
		case 9:
			return 22 * SZ_4K;
		case 8:
			return 20 * SZ_4K;
		default:
			WARN(1, "Unknown GFX version: %d", GRAPHICS_VER(xe));
			return 22 * SZ_4K;
		}
	default:
		WARN(1, "Unknown engine class: %d", class);
		fallthrough;
	case XE_ENGINE_CLASS_COPY:
	case XE_ENGINE_CLASS_VIDEO_DECODE:
	case XE_ENGINE_CLASS_VIDEO_ENHANCE:
		return 2 * SZ_4K;
	}
}

/* TODO: Shameless copy+paste from i915 */
static void set_offsets(u32 *regs,
			const u8 *data,
			const struct xe_hw_engine *hwe,
			bool close)
#define NOP(x) (BIT(7) | (x))
#define LRI(count, flags) ((flags) << 6 | (count) | BUILD_BUG_ON_ZERO(count >= BIT(6)))
#define POSTED BIT(0)
#define REG(x) (((x) >> 2) | BUILD_BUG_ON_ZERO(x >= 0x200))
#define REG16(x) \
	(((x) >> 9) | BIT(7) | BUILD_BUG_ON_ZERO(x >= 0x10000)), \
	(((x) >> 2) & 0x7f)
#define END 0
{
	const u32 base = hwe->mmio_base;

	while (*data) {
		u8 count, flags;

		if (*data & BIT(7)) { /* skip */
			count = *data++ & ~BIT(7);
			regs += count;
			continue;
		}

		count = *data & 0x3f;
		flags = *data >> 6;
		data++;

		*regs = MI_LOAD_REGISTER_IMM(count);
		if (flags & POSTED)
			*regs |= MI_LRI_FORCE_POSTED;
		if (GRAPHICS_VER(hwe->xe) >= 11)
			*regs |= MI_LRI_LRM_CS_MMIO;
		regs++;

		XE_BUG_ON(!count);
		do {
			u32 offset = 0;
			u8 v;

			do {
				v = *data++;
				offset <<= 7;
				offset |= v & ~BIT(7);
			} while (v & BIT(7));

			regs[0] = base + (offset << 2);
			regs += 2;
		} while (--count);
	}

	if (close) {
		/* Close the batch; used mainly by live_lrc_layout() */
		*regs = MI_BATCH_BUFFER_END;
		if (GRAPHICS_VER(hwe->xe) >= 11)
			*regs |= BIT(0);
	}
}

static const u8 gen8_xcs_offsets[] = {
	NOP(1),
	LRI(11, 0),
	REG16(0x244),
	REG(0x034),
	REG(0x030),
	REG(0x038),
	REG(0x03c),
	REG(0x168),
	REG(0x140),
	REG(0x110),
	REG(0x11c),
	REG(0x114),
	REG(0x118),

	NOP(9),
	LRI(9, 0),
	REG16(0x3a8),
	REG16(0x28c),
	REG16(0x288),
	REG16(0x284),
	REG16(0x280),
	REG16(0x27c),
	REG16(0x278),
	REG16(0x274),
	REG16(0x270),

	NOP(13),
	LRI(2, 0),
	REG16(0x200),
	REG(0x028),

	END
};

static const u8 gen9_xcs_offsets[] = {
	NOP(1),
	LRI(14, POSTED),
	REG16(0x244),
	REG(0x034),
	REG(0x030),
	REG(0x038),
	REG(0x03c),
	REG(0x168),
	REG(0x140),
	REG(0x110),
	REG(0x11c),
	REG(0x114),
	REG(0x118),
	REG(0x1c0),
	REG(0x1c4),
	REG(0x1c8),

	NOP(3),
	LRI(9, POSTED),
	REG16(0x3a8),
	REG16(0x28c),
	REG16(0x288),
	REG16(0x284),
	REG16(0x280),
	REG16(0x27c),
	REG16(0x278),
	REG16(0x274),
	REG16(0x270),

	NOP(13),
	LRI(1, POSTED),
	REG16(0x200),

	NOP(13),
	LRI(44, POSTED),
	REG(0x028),
	REG(0x09c),
	REG(0x0c0),
	REG(0x178),
	REG(0x17c),
	REG16(0x358),
	REG(0x170),
	REG(0x150),
	REG(0x154),
	REG(0x158),
	REG16(0x41c),
	REG16(0x600),
	REG16(0x604),
	REG16(0x608),
	REG16(0x60c),
	REG16(0x610),
	REG16(0x614),
	REG16(0x618),
	REG16(0x61c),
	REG16(0x620),
	REG16(0x624),
	REG16(0x628),
	REG16(0x62c),
	REG16(0x630),
	REG16(0x634),
	REG16(0x638),
	REG16(0x63c),
	REG16(0x640),
	REG16(0x644),
	REG16(0x648),
	REG16(0x64c),
	REG16(0x650),
	REG16(0x654),
	REG16(0x658),
	REG16(0x65c),
	REG16(0x660),
	REG16(0x664),
	REG16(0x668),
	REG16(0x66c),
	REG16(0x670),
	REG16(0x674),
	REG16(0x678),
	REG16(0x67c),
	REG(0x068),

	END
};

static const u8 gen12_xcs_offsets[] = {
	NOP(1),
	LRI(13, POSTED),
	REG16(0x244),
	REG(0x034),
	REG(0x030),
	REG(0x038),
	REG(0x03c),
	REG(0x168),
	REG(0x140),
	REG(0x110),
	REG(0x1c0),
	REG(0x1c4),
	REG(0x1c8),
	REG(0x180),
	REG16(0x2b4),

	NOP(5),
	LRI(9, POSTED),
	REG16(0x3a8),
	REG16(0x28c),
	REG16(0x288),
	REG16(0x284),
	REG16(0x280),
	REG16(0x27c),
	REG16(0x278),
	REG16(0x274),
	REG16(0x270),

	END
};

static const u8 gen8_rcs_offsets[] = {
	NOP(1),
	LRI(14, POSTED),
	REG16(0x244),
	REG(0x034),
	REG(0x030),
	REG(0x038),
	REG(0x03c),
	REG(0x168),
	REG(0x140),
	REG(0x110),
	REG(0x11c),
	REG(0x114),
	REG(0x118),
	REG(0x1c0),
	REG(0x1c4),
	REG(0x1c8),

	NOP(3),
	LRI(9, POSTED),
	REG16(0x3a8),
	REG16(0x28c),
	REG16(0x288),
	REG16(0x284),
	REG16(0x280),
	REG16(0x27c),
	REG16(0x278),
	REG16(0x274),
	REG16(0x270),

	NOP(13),
	LRI(1, 0),
	REG(0x0c8),

	END
};

static const u8 gen9_rcs_offsets[] = {
	NOP(1),
	LRI(14, POSTED),
	REG16(0x244),
	REG(0x34),
	REG(0x30),
	REG(0x38),
	REG(0x3c),
	REG(0x168),
	REG(0x140),
	REG(0x110),
	REG(0x11c),
	REG(0x114),
	REG(0x118),
	REG(0x1c0),
	REG(0x1c4),
	REG(0x1c8),

	NOP(3),
	LRI(9, POSTED),
	REG16(0x3a8),
	REG16(0x28c),
	REG16(0x288),
	REG16(0x284),
	REG16(0x280),
	REG16(0x27c),
	REG16(0x278),
	REG16(0x274),
	REG16(0x270),

	NOP(13),
	LRI(1, 0),
	REG(0xc8),

	NOP(13),
	LRI(44, POSTED),
	REG(0x28),
	REG(0x9c),
	REG(0xc0),
	REG(0x178),
	REG(0x17c),
	REG16(0x358),
	REG(0x170),
	REG(0x150),
	REG(0x154),
	REG(0x158),
	REG16(0x41c),
	REG16(0x600),
	REG16(0x604),
	REG16(0x608),
	REG16(0x60c),
	REG16(0x610),
	REG16(0x614),
	REG16(0x618),
	REG16(0x61c),
	REG16(0x620),
	REG16(0x624),
	REG16(0x628),
	REG16(0x62c),
	REG16(0x630),
	REG16(0x634),
	REG16(0x638),
	REG16(0x63c),
	REG16(0x640),
	REG16(0x644),
	REG16(0x648),
	REG16(0x64c),
	REG16(0x650),
	REG16(0x654),
	REG16(0x658),
	REG16(0x65c),
	REG16(0x660),
	REG16(0x664),
	REG16(0x668),
	REG16(0x66c),
	REG16(0x670),
	REG16(0x674),
	REG16(0x678),
	REG16(0x67c),
	REG(0x68),

	END
};

static const u8 gen11_rcs_offsets[] = {
	NOP(1),
	LRI(15, POSTED),
	REG16(0x244),
	REG(0x034),
	REG(0x030),
	REG(0x038),
	REG(0x03c),
	REG(0x168),
	REG(0x140),
	REG(0x110),
	REG(0x11c),
	REG(0x114),
	REG(0x118),
	REG(0x1c0),
	REG(0x1c4),
	REG(0x1c8),
	REG(0x180),

	NOP(1),
	LRI(9, POSTED),
	REG16(0x3a8),
	REG16(0x28c),
	REG16(0x288),
	REG16(0x284),
	REG16(0x280),
	REG16(0x27c),
	REG16(0x278),
	REG16(0x274),
	REG16(0x270),

	LRI(1, POSTED),
	REG(0x1b0),

	NOP(10),
	LRI(1, 0),
	REG(0x0c8),

	END
};

static const u8 gen12_rcs_offsets[] = {
	NOP(1),
	LRI(13, POSTED),
	REG16(0x244),
	REG(0x034),
	REG(0x030),
	REG(0x038),
	REG(0x03c),
	REG(0x168),
	REG(0x140),
	REG(0x110),
	REG(0x1c0),
	REG(0x1c4),
	REG(0x1c8),
	REG(0x180),
	REG16(0x2b4),

	NOP(5),
	LRI(9, POSTED),
	REG16(0x3a8),
	REG16(0x28c),
	REG16(0x288),
	REG16(0x284),
	REG16(0x280),
	REG16(0x27c),
	REG16(0x278),
	REG16(0x274),
	REG16(0x270),

	LRI(3, POSTED),
	REG(0x1b0),
	REG16(0x5a8),
	REG16(0x5ac),

	NOP(6),
	LRI(1, 0),
	REG(0x0c8),
	NOP(3 + 9 + 1),

	LRI(51, POSTED),
	REG16(0x588),
	REG16(0x588),
	REG16(0x588),
	REG16(0x588),
	REG16(0x588),
	REG16(0x588),
	REG(0x028),
	REG(0x09c),
	REG(0x0c0),
	REG(0x178),
	REG(0x17c),
	REG16(0x358),
	REG(0x170),
	REG(0x150),
	REG(0x154),
	REG(0x158),
	REG16(0x41c),
	REG16(0x600),
	REG16(0x604),
	REG16(0x608),
	REG16(0x60c),
	REG16(0x610),
	REG16(0x614),
	REG16(0x618),
	REG16(0x61c),
	REG16(0x620),
	REG16(0x624),
	REG16(0x628),
	REG16(0x62c),
	REG16(0x630),
	REG16(0x634),
	REG16(0x638),
	REG16(0x63c),
	REG16(0x640),
	REG16(0x644),
	REG16(0x648),
	REG16(0x64c),
	REG16(0x650),
	REG16(0x654),
	REG16(0x658),
	REG16(0x65c),
	REG16(0x660),
	REG16(0x664),
	REG16(0x668),
	REG16(0x66c),
	REG16(0x670),
	REG16(0x674),
	REG16(0x678),
	REG16(0x67c),
	REG(0x068),
	REG(0x084),
	NOP(1),

	END
};

static const u8 xehp_rcs_offsets[] = {
	NOP(1),
	LRI(13, POSTED),
	REG16(0x244),
	REG(0x034),
	REG(0x030),
	REG(0x038),
	REG(0x03c),
	REG(0x168),
	REG(0x140),
	REG(0x110),
	REG(0x1c0),
	REG(0x1c4),
	REG(0x1c8),
	REG(0x180),
	REG16(0x2b4),

	NOP(5),
	LRI(9, POSTED),
	REG16(0x3a8),
	REG16(0x28c),
	REG16(0x288),
	REG16(0x284),
	REG16(0x280),
	REG16(0x27c),
	REG16(0x278),
	REG16(0x274),
	REG16(0x270),

	LRI(3, POSTED),
	REG(0x1b0),
	REG16(0x5a8),
	REG16(0x5ac),

	NOP(6),
	LRI(1, 0),
	REG(0x0c8),

	END
};

#undef END
#undef REG16
#undef REG
#undef LRI
#undef NOP

static const u8 *reg_offsets(struct xe_device *xe, enum xe_engine_class class)
{
	if (class == XE_ENGINE_CLASS_RENDER) {
		if (GRAPHICS_VERx10(xe) >= 125)
			return xehp_rcs_offsets;
		else if (GRAPHICS_VER(xe) >= 12)
			return gen12_rcs_offsets;
		else if (GRAPHICS_VER(xe) >= 11)
			return gen11_rcs_offsets;
		else if (GRAPHICS_VER(xe) >= 9)
			return gen9_rcs_offsets;
		else
			return gen8_rcs_offsets;
	} else {
		if (GRAPHICS_VER(xe) >= 12)
			return gen12_xcs_offsets;
		else if (GRAPHICS_VER(xe) >= 9)
			return gen9_xcs_offsets;
		else
			return gen8_xcs_offsets;
	}
}

static void set_context_control(uint32_t * regs, struct xe_hw_engine *hwe,
				bool inhibit)
{
	u32 ctl = 0;

	ctl |= _MASKED_BIT_ENABLE(CTX_CTRL_INHIBIT_SYN_CTX_SWITCH);
	ctl |= _MASKED_BIT_DISABLE(CTX_CTRL_ENGINE_CTX_RESTORE_INHIBIT);
	if (inhibit)
		ctl |= CTX_CTRL_ENGINE_CTX_RESTORE_INHIBIT;
	if (GRAPHICS_VER(hwe->xe) < 11)
		ctl |= _MASKED_BIT_DISABLE(CTX_CTRL_ENGINE_CTX_SAVE_INHIBIT |
					   CTX_CTRL_RS_CTX_ENABLE);
	regs[CTX_CONTEXT_CONTROL] = ctl;

	/* TODO: Timestamp */
}

static int lrc_ring_mi_mode(struct xe_hw_engine *hwe)
{
	if (GRAPHICS_VERx10(hwe->xe) >= 125)
		return 0x70;
	else if (GRAPHICS_VER(hwe->xe) >= 12)
		return 0x60;
	else if (GRAPHICS_VER(hwe->xe) >= 9)
		return 0x54;
	else if (hwe->class == XE_ENGINE_CLASS_RENDER)
		return 0x58;
	else
		return -1;
}

static void reset_stop_ring(uint32_t *regs, struct xe_hw_engine *hwe)
{
	int x;

	x = lrc_ring_mi_mode(hwe);
	if (x != -1) {
		regs[x + 1] &= ~STOP_RING;
		regs[x + 1] |= STOP_RING << 16;
	}
}

static void *__xe_lrc_get_map(struct xe_lrc *lrc)
{
	XE_BUG_ON(dma_buf_map_is_null(&lrc->bo->vmap));
	WARN_ON_ONCE(lrc->bo->vmap.is_iomem);
	return lrc->bo->vmap.vaddr;
}

static inline uint32_t __xe_lrc_ring_offset(struct xe_lrc *lrc)
{
	return 0;
}

static inline uint32_t __xe_lrc_pphwsp_offset(struct xe_lrc *lrc)
{
	return lrc->ring_size;
}

#define LRC_SEQNO_PPHWSP_OFFSET 512
#define LRC_PPHWSP_SIZE SZ_4K

static inline uint32_t __xe_lrc_seqno_offset(struct xe_lrc *lrc)
{
	/* The seqno is stored in the driver-defined portion of PPHWSP */
	return __xe_lrc_pphwsp_offset(lrc) + LRC_SEQNO_PPHWSP_OFFSET;
}

static inline uint32_t __xe_lrc_regs_offset(struct xe_lrc *lrc)
{
	return __xe_lrc_pphwsp_offset(lrc) + LRC_PPHWSP_SIZE;
}

#define DECL_MAP_ADDR_HELPERS(elem) \
static inline void *__xe_lrc_##elem##_map(struct xe_lrc *lrc) \
{ \
	return __xe_lrc_get_map(lrc) + __xe_lrc_##elem##_offset(lrc); \
} \
static inline uint32_t __xe_lrc_##elem##_ggtt_addr(struct xe_lrc *lrc) \
{ \
	return xe_bo_ggtt_addr(lrc->bo) + __xe_lrc_##elem##_offset(lrc); \
} \

DECL_MAP_ADDR_HELPERS(ring)
DECL_MAP_ADDR_HELPERS(pphwsp)
DECL_MAP_ADDR_HELPERS(seqno)
DECL_MAP_ADDR_HELPERS(regs)

#undef DECL_MAP_ADDR_HELPERS

uint32_t xe_lrc_ggtt_addr(struct xe_lrc *lrc)
{
	return __xe_lrc_pphwsp_ggtt_addr(lrc);
}

uint32_t xe_lrc_last_seqno(struct xe_lrc *lrc)
{
	return *(uint32_t *)__xe_lrc_seqno_map(lrc);
}

uint32_t xe_lrc_seqno_ggtt_addr(struct xe_lrc *lrc)
{
	return __xe_lrc_seqno_ggtt_addr(lrc);
}

uint32_t xe_lrc_read_ctx_reg(struct xe_lrc *lrc, int reg_nr)
{
	uint32_t *regs = __xe_lrc_regs_map(lrc);

	return regs[reg_nr];
}

void xe_lrc_write_ctx_reg(struct xe_lrc *lrc, int reg_nr, uint32_t val)
{
	uint32_t *regs = __xe_lrc_regs_map(lrc);

	regs[reg_nr] = val;
}

static void *empty_lrc_data(struct xe_hw_engine *hwe)
{
	void *data;
	uint32_t *regs;

	data = kzalloc(lrc_size(hwe->xe, hwe->class), GFP_KERNEL);
	if (!data)
		return NULL;

	/* Per-Process of HW status Page */
	memset(data, 0, LRC_PPHWSP_SIZE);

	regs = data + LRC_PPHWSP_SIZE;
	memset(regs, 0, SZ_4K);
	set_offsets(regs, reg_offsets(hwe->xe, hwe->class), hwe, true);
	set_context_control(regs, hwe, true);
	reset_stop_ring(regs, hwe);

	return data;
}

static void xe_lrc_set_ppgtt(struct xe_lrc *lrc, struct xe_vm *vm)
{
	uint64_t desc = xe_vm_pdp4_descriptor(vm);

	xe_lrc_write_ctx_reg(lrc, CTX_PDP0_UDW, upper_32_bits(desc));
	xe_lrc_write_ctx_reg(lrc, CTX_PDP0_LDW, lower_32_bits(desc));
}

int xe_lrc_init(struct xe_lrc *lrc, struct xe_hw_engine *hwe,
		struct xe_vm *vm, uint32_t ring_size)
{
	struct xe_device *xe = hwe->xe;
	void *init_data;
	uint32_t arb_enable;
	int err;

	lrc->bo = xe_bo_create(xe, vm, ring_size + lrc_size(xe, hwe->class),
			       ttm_bo_type_kernel,
			       XE_BO_CREATE_LOCAL_BIT | XE_BO_CREATE_GGTT_BIT);
	if (IS_ERR(lrc->bo))
		return PTR_ERR(lrc->bo);

	err = xe_bo_vmap(lrc->bo);
	if (err) {
		xe_bo_put(lrc->bo);
		return err;
	}

	lrc->ring_size = ring_size;
	lrc->ring_tail = 0;

	init_data = empty_lrc_data(hwe);
	if (!init_data) {
		xe_bo_put(lrc->bo);
		return -ENOMEM;
	}

	/* Per-Process of HW status Page */
	memcpy(__xe_lrc_pphwsp_map(lrc), init_data, lrc_size(xe, hwe->class));
	kfree(init_data);

	if (vm)
		xe_lrc_set_ppgtt(lrc, vm);

	xe_lrc_write_ctx_reg(lrc, CTX_RING_START, __xe_lrc_ring_ggtt_addr(lrc));
	xe_lrc_write_ctx_reg(lrc, CTX_RING_HEAD, 0);
	xe_lrc_write_ctx_reg(lrc, CTX_RING_TAIL, lrc->ring_tail);
	xe_lrc_write_ctx_reg(lrc, CTX_RING_CTL,
			     RING_CTL_SIZE(lrc->ring_size) | RING_VALID);

	lrc->desc = GEN8_CTX_VALID;
	lrc->desc |= INTEL_LEGACY_64B_CONTEXT << GEN8_CTX_ADDRESSING_MODE_SHIFT;
	/* TODO: Priority */

	/* While this appears to have something about privileged batches or
	 * some such, it really just means PPGTT mode.
	 */
	if (vm)
		lrc->desc |= GEN8_CTX_PRIVILEGE;
	if (GRAPHICS_VER(xe) == 8)
		lrc->desc |= GEN8_CTX_L3LLC_COHERENT;

	if (GRAPHICS_VER(xe) >= 11) {
		lrc->desc |= (uint64_t)hwe->instance << GEN11_ENGINE_INSTANCE_SHIFT;
		lrc->desc |= (uint64_t)hwe->class << GEN11_ENGINE_CLASS_SHIFT;
	}

	arb_enable = MI_ARB_ON_OFF | MI_ARB_ENABLE;
	xe_lrc_write_ring(lrc, &arb_enable, sizeof(arb_enable));

	return 0;
}

void xe_lrc_finish(struct xe_lrc *lrc)
{
	xe_bo_put(lrc->bo);
}

uint32_t xe_lrc_ring_head(struct xe_lrc *lrc)
{
	return xe_lrc_read_ctx_reg(lrc, CTX_RING_HEAD);
}

uint32_t xe_lrc_ring_space(struct xe_lrc *lrc)
{
	const uint32_t head = xe_lrc_ring_head(lrc);
	const uint32_t tail = lrc->ring_tail;
	const uint32_t size = lrc->ring_size;

	return ((head - tail - 1) & (size - 1)) + 1;
}

static void xe_lrc_assert_ring_space(struct xe_lrc *lrc, size_t size)
{
#if XE_EXTRA_DEBUG
	uint32_t space = xe_lrc_ring_space(lrc);

	BUG_ON(size > lrc->ring_size);
	WARN(size > space, "Insufficient ring space: %lu > %u", size, space);
#endif
}

static void __xe_lrc_write_ring(struct xe_lrc *lrc, void *ring,
				const void *data, size_t size)
{
	memcpy(ring + lrc->ring_tail, data, size);
	lrc->ring_tail = (lrc->ring_tail + size) & (lrc->ring_size - 1);
}

void xe_lrc_write_ring(struct xe_lrc *lrc, const void *data, size_t size)
{
	void *ring;
	uint32_t rhs;
	size_t aligned_size;

	XE_BUG_ON(!IS_ALIGNED(size, 4));
	aligned_size = ALIGN(size, 8);

	xe_lrc_assert_ring_space(lrc, aligned_size);

	ring = __xe_lrc_ring_map(lrc);

	XE_BUG_ON(lrc->ring_tail >= lrc->ring_size);
	rhs = lrc->ring_size - lrc->ring_tail;
	if (size > rhs) {
		__xe_lrc_write_ring(lrc, ring, data, rhs);
		__xe_lrc_write_ring(lrc, ring, data + rhs, size - rhs);
	} else {
		__xe_lrc_write_ring(lrc, ring, data, size);
	}

	if (aligned_size > size) {
		uint32_t noop = MI_NOOP;

		__xe_lrc_write_ring(lrc, ring, &noop, sizeof(noop));
	}
}

uint64_t xe_lrc_descriptor(struct xe_lrc *lrc)
{
	return lrc->desc | xe_lrc_ggtt_addr(lrc);
}
