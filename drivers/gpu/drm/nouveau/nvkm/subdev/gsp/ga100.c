/*
 * Copyright 2022 Red Hat Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */
#include "priv.h"

#include "fw/booter/load/ga100.h"
#include "fw/booter/unload/ga100.h"
#include "fw/gsp-rm/boot/ga100.h"

static const struct nvkm_gsp_func
ga100_gsp_r515_48_07 = {
	.flcn = &tu102_gsp_flcn,
	.fwsec = &tu102_gsp_fwsec,

	.booter.ctor = tu102_gsp_booter_ctor,
	.booter.load = &ga100_gsp_booter_load_fw,
	.booter.unload = &ga100_gsp_booter_unload_fw,

	.boot = &ga100_gsp_gsp_rm_boot_fw,

	.dtor = r515_gsp_dtor,
	.oneinit = tu102_gsp_oneinit,
	.init = r515_gsp_init,
	.fini = r515_gsp_fini,
	.reset = tu102_gsp_reset,

	.rpc = &r515_gsp_rpc,
};

static struct nvkm_gsp_fwif
ga100_gsps[] = {
	{ 5154807,  r515_gsp_load, &ga100_gsp_r515_48_07, ".fwsignature_ga100" },
	{      -1, gv100_gsp_nofw, &gv100_gsp },
	{}
};

int
ga100_gsp_new(struct nvkm_device *device, enum nvkm_subdev_type type, int inst,
	      struct nvkm_gsp **pgsp)
{
	return nvkm_gsp_new_(ga100_gsps, device, type, inst, pgsp);
}
