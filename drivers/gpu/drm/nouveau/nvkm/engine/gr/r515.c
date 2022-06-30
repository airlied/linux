#include "priv.h"
#include "nv50.h"

static int
r515_gr_init(struct nvkm_gr *base)
{
	return 0;
}
	
static const struct nvkm_gr_func
r515_gr = {
	.init = r515_gr_init,
};

int
r515_gr_new(struct nvkm_device *device, enum nvkm_subdev_type type, int inst,
	    struct nvkm_gr **pgr)
{
	return nv50_gr_new_(&r515_gr, device, type, inst, pgr);
}
