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
#include <subdev/gsp/client.h>
#include "priv.h"

void
nvkm_gsp_client_del(struct nvkm_gsp_client **pclient)
{
	kfree(*pclient);
	*pclient = NULL;
}

int
nvkm_gsp_client_new(struct nvkm_gsp *gsp, struct nvkm_gsp_client **pclient)
{
	struct nvkm_gsp_client *client;
	int ret;

	if (!gsp->func->client)
		return -ENODEV;

	if (!(client = *pclient = kmalloc(sizeof(*client), GFP_KERNEL)))
		return -ENOMEM;

	client->func = gsp->func->client;
	client->gsp = gsp;
	client->handle = atomic_inc_return(&gsp->client_id);

	ret = gsp->func->client->ctor(client);
	if (ret)
		nvkm_gsp_client_del(pclient);

	return ret;
}
