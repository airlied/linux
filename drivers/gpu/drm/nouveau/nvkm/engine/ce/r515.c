
#include "priv.h"

static const struct nvkm_engine_func
r515_ce = {

};

int
r515_ce_new(struct nvkm_device *device, enum nvkm_subdev_type type, int inst,
	    struct nvkm_engine **pengine)
{
	return nvkm_engine_new_(&r515_ce, device, type, inst, true, pengine);
}
  
