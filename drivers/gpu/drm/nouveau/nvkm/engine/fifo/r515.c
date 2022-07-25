#include "priv.h"
#include "chan.h"
#include "chid.h"
#include "runl.h"
#include "nvkm/subdev/gsp.h"

#include <nvhw/class/cl2080.h>
#include <nvif/class.h>

const struct nvkm_chan_func_inst
r515_chan_inst = {
	.size = 0x1000,
	.zero = true,
	.vmm = true,
};

static void
r515_chan_stop(struct nvkm_chan *chan)
{
}

static void
r515_chan_start(struct nvkm_chan *chan)
{
}

const struct nvkm_chan_func
r515_chan = {
	.inst = &r515_chan_inst,
	.userd = &gv100_chan_userd,
	.ramfc = &gv100_chan_ramfc,
	.start = &r515_chan_start,
	.stop = &r515_chan_stop,
};

static const struct nvkm_engn_func
r515_engn = {
};

const struct nvkm_engn_func
r515_engn_sw = {
};

static int conv_nv2080_to_nvkm(unsigned nv2080, int *idx)
{
	*idx = 0;

	if (NV2080_ENGINE_TYPE_IS_GR(nv2080)) {
		*idx = NV2080_ENGINE_TYPE_GR_IDX(nv2080);
		return NVKM_ENGINE_GR;
	} else if (NV2080_ENGINE_TYPE_IS_COPY(nv2080)) {
		*idx = NV2080_ENGINE_TYPE_COPY_IDX(nv2080);
		return NVKM_ENGINE_CE;
	}
	//TODO
	return -1;
}

static int
r515_fifo_runl_ctor(struct nvkm_fifo *fifo)
{
  	struct nvkm_device *device = fifo->engine.subdev.device;
	struct nvkm_gsp *gsp = device->gsp;
	struct nvkm_runl *runl;
	int idx = 0;

	runl = nvkm_runl_new(fifo, 0, 0, 0);
	if (IS_ERR(runl))
		return PTR_ERR(runl);

	for (unsigned i = 0; i < gsp->num_device_entries; i++) {
		if (gsp->device_entries[i].vals.runlist == 0) {
			int eng_idx;
			int eng_type = conv_nv2080_to_nvkm(gsp->device_entries[i].vals.nv2080, &eng_idx);
			if (eng_type >= 0) {
				printk(KERN_ERR "Adding runl %d %d/%d %s\n", idx, eng_type, eng_idx, gsp->device_entries[i].engine_name);
				nvkm_runl_add(runl, idx, fifo->func->engn, eng_type, eng_idx);
				idx++;
			}
		}
	}
	return 0;
}

int
r515_fifo_chid_ctor(struct nvkm_fifo *fifo, int nr)
{
	int ret;
	ret = nvkm_chid_new(&nvkm_chan_event, &fifo->engine.subdev, nr, 0, nr, &fifo->cgid);
	if (ret)
		return ret;	
	return nvkm_chid_new(&nvkm_chan_event, &fifo->engine.subdev, nr, 0, nr, &fifo->chid);
}

static const struct nvkm_runl_func
r515_runl = {
	.size = 8,
};

static const struct nvkm_fifo_func
r515_fifo = {
	.chid_nr = nv50_fifo_chid_nr,
	.chid_ctor = r515_fifo_chid_ctor,
	.runl = &r515_runl,	
	.runl_ctor = r515_fifo_runl_ctor,
	.chan = {{ 0, 0, TURING_CHANNEL_GPFIFO_A }, &r515_chan },
	.engn = &r515_engn,
};

int
r515_fifo_new(struct nvkm_device *device, enum nvkm_subdev_type type, int inst,
	      struct nvkm_fifo **pfifo)
{
	return nvkm_fifo_new_(&r515_fifo, device, type, inst, pfifo);
}

  
