#ifndef __NVKM_GSP_CLIENT_H__
#define __NVKM_GSP_CLIENT_H__
#include <subdev/gsp.h>

struct nvkm_gsp_client {
	const struct nvkm_gsp_client_func {
		int (*ctor)(struct nvkm_gsp_client *);
	} *func;
	struct nvkm_gsp *gsp;

	u32 handle;
};

int nvkm_gsp_client_new(struct nvkm_gsp *, struct nvkm_gsp_client **);
int nvkm_gsp_client_new_(const struct nvkm_gsp_client_func *, struct nvkm_gsp *,
			 struct nvkm_gsp_client **);
void nvkm_gsp_client_del(struct nvkm_gsp_client **);
#endif
